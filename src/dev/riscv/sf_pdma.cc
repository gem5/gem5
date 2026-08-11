/*
 * Copyright (c) 2025 Nikita Proshkin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dev/riscv/sf_pdma.hh"

#include "base/bitfield.hh"
#include "debug/Checkpoint.hh"
#include "debug/Drain.hh"
#include "debug/SfPDMA.hh"

namespace gem5
{

PDMAChannel::PDMAReqPort::PDMAReqPort(const std::string &name,
        PDMAChannel &chan) : RequestPort(name), chan(chan)
{}

bool
PDMAChannel::PDMAReqPort::recvTimingResp(PacketPtr pkt)
{
    DPRINTF(SfPDMA, "Got response addr: 0x%x cmd: %s\n", pkt->getAddr(),
            pkt->cmdString());
    PDMAPkt *payload = safe_cast<PDMAPkt *>(pkt->popSenderState());

    if (pkt->isError())
        chan.processRespError(payload);
    else
        chan.processRespSuccess(payload);
    delete pkt;
    return true;
}

void
PDMAChannel::PDMAReqPort::recvReqRetry()
{
    DPRINTF(SfPDMA, "Got request retry\n");

    chan.waitingRetry = false;
    if (!chan.sendPktsEvent.scheduled() && chan.ctrl.run) {
        chan.processTiming();
    }
}

PDMAChannel::PDMAChannel(SfPDMA *pdma, const SfPDMAParams &p, int cid) :
    pdma(pdma), system(p.system), chanId(cid),
    dma(csprintf("%s.dma", name()), *this),
    requestorId(p.system->getRequestorId(pdma)), streamId(p.sid),
    doneIrq(p.done_irq[cid]), errIrq(p.error_irq[cid]), platform(p.platform),
    pktsN(p.pkts_each_dir * 2), dmaWidth(p.dma_ports_width),
    dataFifo(FIFO_SIZE), sendPktsEvent(*this), sendComplEvent(*this),
    stats(*this)
{
    delays.chanRegRdWrDelay = p.chan_reg_rd_wr_delay;
    delays.latBeforeBegin = p.lat_before_begin;
    delays.latBeforeCompletion = p.lat_before_completion;

    // We can transfer a maximum of Cache Line Size bytes in one packet not
    // to trigger cache asserts.
    size_t buf_size = system->cacheLineSize();

    for (int i = 0; i < pktsN; i++)
        idlePkts.push(std::make_unique<PDMAPkt>(buf_size));
}

Port &
PDMAChannel::getPort()
{
    return dma;
}

std::string
PDMAChannel::name() const
{
    assert(pdma);
    return csprintf("%s-chan%d", pdma->name(), chanId);
}

Tick
PDMAChannel::read(PacketPtr pkt, Addr addr, int size)
{
    DPRINTF(SfPDMA, "Read register 0x%x size: %d\n", addr, size);

    pkt->makeResponse();

    if (addr == PDMA_CTRL_OFFSET && size == 4) {
        pkt->setLE<uint32_t>(ctrl);
        return delays.chanRegRdWrDelay * pdma->clockPeriod();
    }

    PDMARegs *regs;
    // 'Next' regs have offsets 0x0**, 'Exec' regs - 0x1**
    switch (addr & 0xf00) {
    case 0x0:
        regs = &next;
        break;
    case 0x100:
        regs = &exec;
        break;
    default:
        return 0;
    }

    addr &= 0xff;
    switch (addr) {
    case PDMA_CONF_OFFSET:
        if (size == 4)
            pkt->setLE<uint32_t>(regs->config);
        break;
    case PDMA_SIZE_OFFSET:
    case PDMA_DEST_OFFSET:
    case PDMA_SRC_OFFSET:
        if (size == 8) {
            pkt->setLE<uint64_t>(
                    *reinterpret_cast<uint64_t *>(regs->data + addr));
        }
        break;
    default:
        break;
    }

    return delays.chanRegRdWrDelay * pdma->clockPeriod();
}

Tick
PDMAChannel::write(PacketPtr pkt, Addr addr, int size)
{
    DPRINTF(SfPDMA, "Write register 0x%x size: %d data: 0x%x cmd: %s\n", addr,
            size, size == 4 ? pkt->getLE<uint32_t>() : pkt->getLE<uint64_t>(),
            pkt->cmdString().c_str());

    pkt->makeResponse();

    switch (addr) {
    case PDMA_CTRL_OFFSET: {
        if (size != 4)
            return 0;
        PDMACtrl data = pkt->getLE<uint32_t>();
        ctrl.done_ie = data.done_ie;
        ctrl.error_ie = data.error_ie;

        if (ctrl.run != data.run) {
            // Start transfer
            if (data.run) {
                run(false);
            }
            // Stop transfer.
            // Transfer processing routines will do all the necessary work,
            // do not touch state here.
            else {
                shutdownDescr = true;
            }
        }

        // Setting 'claim' bit clears all 'Next' registers.
        // 'claim' can only be cleared when run is low.
        if (ctrl.claim != data.claim) {
            if (data.claim) {
                ctrl.claim = 1;
                memset(next.data, 0, sizeof(next));
            } else if (!ctrl.run) {
                ctrl.claim = 0;
            }
        }

        if (!data.done) {
            ctrl.done = 0;
            platform->clearPciInt(doneIrq);
        }

        if (!data.error) {
            ctrl.error = 0;
            platform->clearPciInt(errIrq);
            busErr = false;
        }
    } break;
    case PDMA_CONF_OFFSET: {
        if (size != 4)
            return 0;
        PDMAConf data = pkt->getLE<uint32_t>();
        int log_max_size = findMsbSet(system->cacheLineSize());
        if (data.wsize > log_max_size)
            warn("pdma: wsize is limited to %d, got: %d\n", log_max_size,
                    data.wsize);
        data.wsize = std::min(static_cast<int>(data.wsize), log_max_size);
        if (data.rsize > log_max_size)
            warn("pdma: rsize is limited to %d, got: %d\n", log_max_size,
                    data.rsize);
        data.rsize = std::min(static_cast<int>(data.rsize), log_max_size);

        next.config = data;
    } break;
    case PDMA_SIZE_OFFSET:
        if (size == 8)
            next.size = pkt->getLE<uint64_t>();
        break;
    case PDMA_DEST_OFFSET:
        if (size == 8)
            next.dest = pkt->getLE<uint64_t>();
        break;
    case PDMA_SRC_OFFSET:
        if (size == 8)
            next.src = pkt->getLE<uint64_t>();
        break;
    default:
        break;
    }

    return delays.chanRegRdWrDelay * pdma->clockPeriod();
}

DrainState
PDMAChannel::drain()
{
    if (readyForDrain()) {
        DPRINTF(Drain, "drained\n");
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "not drained. Waiting for in flight pkts\n");
        return DrainState::Draining;
    }
}

void
PDMAChannel::drainResume()
{
    if (ctrl.run && !sendPktsEvent.scheduled() &&
            !sendComplEvent.scheduled()) {
        if (system->isAtomicMode())
            sendAtomicPkts();
        else
            processTiming();
    }
}

bool
PDMAChannel::readyForDrain()
{
    return inFlightR == 0 && inFlightW == 0 && pendingPkts.empty() &&
           !nextTimingPkt;
}

void
PDMAChannel::serialize(CheckpointOut &cp) const
{
    DPRINTF(Checkpoint, "Serializing Sf-PDMA\n");
    SERIALIZE_SCALAR(ctrl.__storage);
    SERIALIZE_SCALAR(shutdownDescr);
    SERIALIZE_SCALAR(busErr);
    SERIALIZE_ARRAY(next.data, sizeof(next.data) / sizeof(next.data[0]));
    SERIALIZE_ARRAY(exec.data, sizeof(exec.data) / sizeof(exec.data[0]));
    arrayParamOut(cp, "dataFifo", dataFifo);
}

void
PDMAChannel::unserialize(CheckpointIn &cp)
{
    DPRINTF(Checkpoint, "Unserializing Sf-PDMA\n");
    UNSERIALIZE_SCALAR(ctrl.__storage);
    UNSERIALIZE_SCALAR(shutdownDescr);
    UNSERIALIZE_SCALAR(busErr);
    UNSERIALIZE_ARRAY(next.data, sizeof(next.data) / sizeof(next.data[0]));
    UNSERIALIZE_ARRAY(exec.data, sizeof(exec.data) / sizeof(exec.data[0]));
    arrayParamIn(cp, "dataFifo", dataFifo);
}

void
PDMAChannel::run(bool repeated)
{
    // - Copy transfer params set by driver from 'Next' regs to
    //  'Exec' set;
    // - Reset channel state according to new transfer params;
    // - In RO 'Exec' regs channel keeps in-progress transfer state
    //   (for example, the driver can read from them how much more
    //   data is left to transfer).

    if (!repeated) {
        ctrl.run = 1;
        ctrl.done = 0;
        ctrl.error = 0;
        platform->clearPciInt(doneIrq);
        shutdownDescr = false;
        busErr = false;
        platform->clearPciInt(errIrq);
    }

    exec = next;

    PDMAConf conf = static_cast<PDMAConf>(exec.config);

    // User can set max number of in-flight packets in each
    // direction through Params. Channel constructor set this
    // value multiplied by 2 to the pktsN.
    // So, the default value for inFlightMax (meaning packets
    // in each direction) is pktsN / 2. This is the case when
    // out-of-order responses to read requests are possible.
    // Also, transfer config has 'order' bit using which driver
    // can allow to exist only one packet in each direction
    // in-flight at a time.
    inFlightMax = conf.order ? 1 : pktsN / 2;
    rChunks.reset(new ChunkGenerator(exec.src, exec.size, 1 << conf.rsize));
    wrChunks.reset(new ChunkGenerator(exec.dest, exec.size, 1 << conf.wsize));
    startTime = curTick();
    pendingId = 0;
    idsForReads = 0;
    dataFifo.flush();
    nextTimingPkt = nullptr;
    pdma->schedule(sendPktsEvent, pdma->clockEdge(delays.latBeforeBegin));
}

void
PDMAChannel::processCompletion()
{
    ctrl.done = (exec.size == 0);
    ctrl.error = busErr;
    stats.workTime += curTick() - startTime;

    bool repeat = static_cast<PDMAConf>(exec.config).repeat;

    if (ctrl.done && !shutdownDescr && repeat) {
        run(true);
    } else {
        ctrl.claim = 0;
        ctrl.run = 0;
    }

    if (ctrl.done && ctrl.done_ie) {
        DPRINTF(SfPDMA, "raising done irq\n");
        platform->postPciInt(doneIrq);
    }

    if (ctrl.error && ctrl.error_ie) {
        DPRINTF(SfPDMA, "raising err irq\n");
        platform->postPciInt(errIrq);
    }
}

PacketPtr
PDMAChannel::buildPkt(PDMAPkt *pkt, MemCmd::Command cmd)
{
    RequestPtr req;
    PacketPtr pkt_to_send;

    if (cmd == MemCmd::WriteReq) {
        if ((dataFifo.size() < wrChunks->size()) || wrChunks->done())
            return nullptr;
        pkt->cmd = MemCmd::WriteReq;
        pkt->pktSize = wrChunks->size();
        dataFifo.read(pkt->buf.get(), wrChunks->size());

        req = std::make_shared<Request>(
                wrChunks->addr(), wrChunks->size(), 0, requestorId);

        wrChunks->next();
    } else if (cmd == MemCmd::ReadReq) {
        if (rChunks->done())
            return nullptr;
        pkt->cmd = MemCmd::ReadReq;
        pkt->pktSize = rChunks->size();
        pkt->id = idsForReads;
        idsForReads++;

        req = std::make_shared<Request>(
                rChunks->addr(), rChunks->size(), 0, requestorId);

        rChunks->next();
    } else
        panic("Invalid pkt command");

    pkt_to_send = new Packet(req, pkt->cmd);
    pkt_to_send->dataStatic(pkt->buf.get());
    pkt_to_send->pushSenderState(pkt);
    pkt_to_send->req->taskId(context_switch_task_id::DMA);
    pkt_to_send->req->setStreamId(streamId);
    return pkt_to_send;
}

void
PDMAChannel::sendPkts()
{
    if (system->isTimingMode()) {
        processTiming();
    } else if (system->isAtomicMode()) {
        sendAtomicPkts();
    } else
        panic("Not in timing or atomic mode!");
}

void
PDMAChannel::sendAtomicPkts()
{
    if (drainState() == DrainState::Draining)
        return;

    Tick total_delay = 0;
    Tick delay;

    std::unique_ptr<Packet> pkt_to_send;
    PDMAPkt *pkt = idlePkts.front().get();

    while (!wrChunks->done() && !shutdownDescr) {
        if ((dataFifo.size() + rChunks->size()) < dataFifo.capacity()) {
            pkt_to_send.reset(buildPkt(pkt, MemCmd::ReadReq));
            if (pkt_to_send) {
                DPRINTF(SfPDMA, "Starting read [a] src: 0x%x size: %d\n",
                        pkt_to_send->getAddr(), pkt_to_send->getSize());
                // '+1' to simulate transmission delay for packet with no data
                delay = dma.sendAtomic(pkt_to_send.get()) + 1;
                stats.turnaroundRdPkts.sample(delay);
                total_delay += delay;
                if (!pkt_to_send->isError()) {
                    dataFifo.write(pkt->buf.get(), pkt->pktSize);
                } else {
                    DPRINTF(SfPDMA, "Abort\n");
                    shutdownDescr = true;
                    busErr = true;
                    break;
                }
            }
        }
        // Move new Packet to the unique_ptr.
        // Previous Packet assembled by buildPkt() was already used by
        // sendAtomic(), so we can free its memory.
        pkt_to_send.reset(buildPkt(pkt, MemCmd::WriteReq));
        if (pkt_to_send) {
            DPRINTF(SfPDMA, "Starting write [a] dst: 0x%x size: %d\n",
                    pkt_to_send->getAddr(), pkt_to_send->getSize());
            delay = dma.sendAtomic(pkt_to_send.get()) +
                    (pkt_to_send->getSize() + (dmaWidth - 1)) / dmaWidth;
            stats.turnaroundWrPkts.sample(delay);
            total_delay += delay;
            if (!pkt_to_send->isError()) {
                stats.bytesTransmitted += pkt->pktSize;
                exec.src += pkt->pktSize;
                exec.dest += pkt->pktSize;
                exec.size -= pkt->pktSize;
            } else {
                DPRINTF(SfPDMA, "Abort\n");
                shutdownDescr = true;
                busErr = true;
                break;
            }
        }
    }

    pdma->schedule(sendComplEvent,
            pdma->clockEdge(delays.latBeforeCompletion) + total_delay);
}

void
PDMAChannel::processTiming()
{
    if (!sendPktsEvent.scheduled())
        sendTimingPkts();
    if ((drainState() == DrainState::Draining) && readyForDrain()) {
        DPRINTF(Drain, "Drained. Signaling to DrainManager\n");
        signalDrainDone();
    } else if (!shutdownDescr) {
        if (exec.size == 0)
            pdma->schedule(sendComplEvent,
                    pdma->clockEdge(delays.latBeforeCompletion));
    } else if (inFlightR == 0 && inFlightW == 0)
        pdma->schedule(
                sendComplEvent, pdma->clockEdge(delays.latBeforeCompletion));
}

void
PDMAChannel::sendTimingPkts()
{
    fillFifo();

    if (waitingRetry)
        return;

    // Nothing to resend and no available buffers to build a new Packet
    if (!nextTimingPkt && idlePkts.empty())
        return;

    if (!nextTimingPkt && inFlightW < inFlightMax && !shutdownDescr) {
        nextTimingPkt = buildPkt(idlePkts.front().get(), MemCmd::WriteReq);
    }

    if (!nextTimingPkt && inFlightR < inFlightMax && !shutdownDescr &&
            drainState() != DrainState::Draining) {
        nextTimingPkt = buildPkt(idlePkts.front().get(), MemCmd::ReadReq);
    }

    if (nextTimingPkt) {
        idlePkts.front()->pktSendTime = curTick();

        if (dma.sendTimingReq(nextTimingPkt)) {
            uint64_t nbeats;

            if (nextTimingPkt->isWrite()) {
                inFlightW++;
                nbeats =
                        (nextTimingPkt->getSize() + (dmaWidth - 1)) / dmaWidth;
                DPRINTF(SfPDMA, "Starting write [t] dst: 0x%x size: %d\n",
                        nextTimingPkt->getAddr(), nextTimingPkt->getSize());
            } else {
                inFlightR++;
                // to simulate transmission delay for packet with no data
                nbeats = 1;
                DPRINTF(SfPDMA, "Starting read [t] src: 0x%x size: %d\n",
                        nextTimingPkt->getAddr(), nextTimingPkt->getSize());
            }
            idlePkts.front().release();
            idlePkts.pop();
            nextTimingPkt = nullptr;
            pdma->schedule(sendPktsEvent, pdma->clockEdge(Cycles(nbeats)));
        } else
            waitingRetry = true;
    }

    fillFifo();
}

void
PDMAChannel::fillFifo()
{
    while (!pendingPkts.empty() && pendingPkts.top()->id == pendingId &&
            ((dataFifo.size() + pendingPkts.top()->pktSize) <
                    dataFifo.capacity())) {
        dataFifo.write(
                pendingPkts.top()->buf.get(), pendingPkts.top()->pktSize);
        idlePkts.push(std::unique_ptr<PDMAPkt>(pendingPkts.top()));
        pendingPkts.pop();
        pendingId++;
    }
}

void
PDMAChannel::processRespSuccess(PDMAPkt *pkt)
{
    if (pkt->cmd == MemCmd::ReadReq) {
        inFlightR--;
        stats.turnaroundRdPkts.sample(curTick() - pkt->pktSendTime);
        if (!shutdownDescr)
            pendingPkts.push(pkt);
        else
            idlePkts.push(std::unique_ptr<PDMAPkt>(pkt));
    } else {
        inFlightW--;
        stats.turnaroundWrPkts.sample(curTick() - pkt->pktSendTime);

        exec.src += pkt->pktSize;
        exec.dest += pkt->pktSize;
        exec.size -= pkt->pktSize;
        stats.bytesTransmitted += pkt->pktSize;
        idlePkts.push(std::unique_ptr<PDMAPkt>(pkt));
    }
    processTiming();
}

void
PDMAChannel::processRespError(PDMAPkt *pkt)
{
    DPRINTF(SfPDMA, "Abort\n");

    shutdownDescr = true;
    busErr = true;
    while (!pendingPkts.empty()) {
        idlePkts.push(std::unique_ptr<PDMAPkt>(pendingPkts.top()));
        pendingPkts.pop();
    }

    if (pkt->cmd == MemCmd::WriteReq)
        inFlightW--;
    else
        inFlightR--;
    idlePkts.push(std::unique_ptr<PDMAPkt>(pkt));
    processTiming();
}

PDMAChannel::PDMAStats::PDMAStats(PDMAChannel &chan) :
    statistics::Group(chan.pdma, csprintf("chan%d", chan.chanId).c_str()),
    ADD_STAT(turnaroundRdPkts, statistics::units::Tick::get(),
            "Turnaround delay for dma rd pkts"),
    ADD_STAT(turnaroundWrPkts, statistics::units::Tick::get(),
            "Turnaround delay for dma wr pkts"),
    ADD_STAT(bytesTransmitted, statistics::units::Byte::get(),
            "Number of bytes copied by channel"),
    ADD_STAT(workTime, statistics::units::Tick::get(),
            "Sum of intervals between receiving dma command and raising irq"),
    ADD_STAT(averageSpeed,
            statistics::units::Rate<statistics::units::Byte,
                    statistics::units::Second>::get(),
            "Channel average transmission speed")
{
    using namespace statistics;

    turnaroundRdPkts.init(4).flags(nozero | nonan);
    turnaroundWrPkts.init(4).flags(nozero | nonan);
    bytesTransmitted.flags(nozero | nonan);
    workTime.flags(nozero | nonan);
    averageSpeed.precision(0).prereq(bytesTransmitted).flags(nozero | nonan);
    averageSpeed = bytesTransmitted / (workTime / simFreq);
}

SfPDMA::SfPDMA(const SfPDMAParams &p) :
    PioDevice(p), pioAddr(p.pio_addr), chanCnt(p.chan_cnt)
{
    if (p.done_irq.size() != chanCnt || p.error_irq.size() != chanCnt)
        fatal("pdma: need to provide all irqs\n");

    for (int i = 0; i < p.chan_cnt; i++)
        chans.push_back(std::make_unique<PDMAChannel>(this, p, i));
}

PDMAChannel *
SfPDMA::chanByAddr(Addr addr)
{
    addr -= pioAddr;
    uint8_t chan = bits(addr, 15, 12);
    fatal_if(chan >= chanCnt, "pdma: access to nonexistent channel\n");
    return chans[chan].get();
}

Tick
SfPDMA::read(PacketPtr pkt)
{
    Addr addr = pkt->getAddr() & 0xfff;
    int size = pkt->getSize();
    return chanByAddr(pkt->getAddr())->read(pkt, addr, size);
}

Tick
SfPDMA::write(PacketPtr pkt)
{
    Addr addr = pkt->getAddr() & 0xfff;
    int size = pkt->getSize();
    return chanByAddr(pkt->getAddr())->write(pkt, addr, size);
}

AddrRangeList
SfPDMA::getAddrRanges() const
{
    return AddrRangeList({RangeSize(pioAddr, 0x1000 * chanCnt)});
}

Port &
SfPDMA::getPort(const std::string &if_name, PortID idx)
{
    if (if_name != "dma") {
        // pass it along to our super class
        return PioDevice::getPort(if_name, idx);
    } else {
        fatal_if(idx >= chanCnt, "pdma::getPort: unknown index %d\n", idx);

        return chans[idx]->getPort();
    }
}

void
SfPDMA::serialize(CheckpointOut &cp) const
{
    PioDevice::serialize(cp);
    for (int i = 0; i < chanCnt; i++)
        chans[i]->serializeSection(cp, csprintf("channel%d", i));
}

void
SfPDMA::unserialize(CheckpointIn &cp)
{
    PioDevice::unserialize(cp);
    for (int i = 0; i < chanCnt; i++)
        chans[i]->unserializeSection(cp, csprintf("channel%d", i));
}

} // namespace gem5
