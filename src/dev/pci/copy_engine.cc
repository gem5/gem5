/*
 * Copyright (c) 2012 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2008 The Regents of The University of Michigan
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
 *
 * Authors: Ali Saidi
 */

/* @file
 * Device model for Intel's I/O AT DMA copy engine.
 */

#include "dev/pci/copy_engine.hh"

#include <algorithm>

#include "base/cp_annotate.hh"
#include "base/trace.hh"
#include "debug/DMACopyEngine.hh"
#include "debug/Drain.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/CopyEngine.hh"
#include "sim/stats.hh"
#include "sim/system.hh"

using namespace CopyEngineReg;

CopyEngine::CopyEngine(const Params *p)
    : PciDevice(p)
{
    // All Reg regs are initialized to 0 by default
    regs.chanCount = p->ChanCnt;
    regs.xferCap = findMsbSet(p->XferCap);
    regs.attnStatus = 0;

    if (regs.chanCount > 64)
        fatal("CopyEngine interface doesn't support more than 64 DMA engines\n");

    for (int x = 0; x < regs.chanCount; x++) {
        CopyEngineChannel *ch = new CopyEngineChannel(this, x);
        chan.push_back(ch);
    }
}


CopyEngine::CopyEngineChannel::CopyEngineChannel(CopyEngine *_ce, int cid)
    : cePort(_ce, _ce->sys),
      ce(_ce), channelId(cid), busy(false), underReset(false),
    refreshNext(false), latBeforeBegin(ce->params()->latBeforeBegin),
    latAfterCompletion(ce->params()->latAfterCompletion),
    completionDataReg(0), nextState(Idle),
    fetchCompleteEvent(this), addrCompleteEvent(this),
    readCompleteEvent(this), writeCompleteEvent(this),
    statusCompleteEvent(this)

{
        cr.status.dma_transfer_status(3);
        cr.descChainAddr = 0;
        cr.completionAddr = 0;

        curDmaDesc = new DmaDesc;
        memset(curDmaDesc, 0, sizeof(DmaDesc));
        copyBuffer = new uint8_t[ce->params()->XferCap];
}

CopyEngine::~CopyEngine()
{
    for (int x = 0; x < chan.size(); x++) {
        delete chan[x];
    }
}

CopyEngine::CopyEngineChannel::~CopyEngineChannel()
{
    delete curDmaDesc;
    delete [] copyBuffer;
}

BaseMasterPort &
CopyEngine::getMasterPort(const std::string &if_name, PortID idx)
{
    if (if_name != "dma") {
        // pass it along to our super class
        return PciDevice::getMasterPort(if_name, idx);
    } else {
        if (idx >= static_cast<int>(chan.size())) {
            panic("CopyEngine::getMasterPort: unknown index %d\n", idx);
        }

        return chan[idx]->getMasterPort();
    }
}


BaseMasterPort &
CopyEngine::CopyEngineChannel::getMasterPort()
{
    return cePort;
}

void
CopyEngine::CopyEngineChannel::recvCommand()
{
    if (cr.command.start_dma()) {
        assert(!busy);
        cr.status.dma_transfer_status(0);
        nextState = DescriptorFetch;
        fetchAddress = cr.descChainAddr;
        if (ce->drainState() == DrainState::Running)
            fetchDescriptor(cr.descChainAddr);
    } else if (cr.command.append_dma()) {
        if (!busy) {
            nextState = AddressFetch;
            if (ce->drainState() == DrainState::Running)
                fetchNextAddr(lastDescriptorAddr);
        } else
            refreshNext = true;
    } else if (cr.command.reset_dma()) {
        if (busy)
            underReset = true;
        else {
            cr.status.dma_transfer_status(3);
            nextState = Idle;
        }
    } else if (cr.command.resume_dma() || cr.command.abort_dma() ||
            cr.command.suspend_dma())
        panic("Resume, Abort, and Suspend are not supported\n");
    cr.command(0);
}

Tick
CopyEngine::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    int size = pkt->getSize();
    if (size != sizeof(uint64_t) && size != sizeof(uint32_t) &&
        size != sizeof(uint16_t) && size != sizeof(uint8_t)) {
        panic("Unknown size for MMIO access: %d\n", pkt->getSize());
    }

    DPRINTF(DMACopyEngine, "Read device register %#X size: %d\n", daddr, size);

    ///
    /// Handle read of register here
    ///

    if (daddr < 0x80) {
        switch (daddr) {
          case GEN_CHANCOUNT:
            assert(size == sizeof(regs.chanCount));
            pkt->set<uint8_t>(regs.chanCount);
            break;
          case GEN_XFERCAP:
            assert(size == sizeof(regs.xferCap));
            pkt->set<uint8_t>(regs.xferCap);
            break;
          case GEN_INTRCTRL:
            assert(size == sizeof(uint8_t));
            pkt->set<uint8_t>(regs.intrctrl());
            regs.intrctrl.master_int_enable(0);
            break;
          case GEN_ATTNSTATUS:
            assert(size == sizeof(regs.attnStatus));
            pkt->set<uint32_t>(regs.attnStatus);
            regs.attnStatus = 0;
            break;
          default:
            panic("Read request to unknown register number: %#x\n", daddr);
        }
        pkt->makeAtomicResponse();
        return pioDelay;
    }


    // Find which channel we're accessing
    int chanid = 0;
    daddr -= 0x80;
    while (daddr >= 0x80) {
        chanid++;
        daddr -= 0x80;
    }

    if (chanid >= regs.chanCount)
        panic("Access to channel %d (device only configured for %d channels)",
                chanid, regs.chanCount);

    ///
    /// Channel registers are handled here
    ///
    chan[chanid]->channelRead(pkt, daddr, size);

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
CopyEngine::CopyEngineChannel::channelRead(Packet *pkt, Addr daddr, int size)
{
    switch (daddr) {
      case CHAN_CONTROL:
        assert(size == sizeof(uint16_t));
        pkt->set<uint16_t>(cr.ctrl());
        cr.ctrl.in_use(1);
        break;
      case CHAN_STATUS:
        assert(size == sizeof(uint64_t));
        pkt->set<uint64_t>(cr.status() | ~busy);
        break;
      case CHAN_CHAINADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            pkt->set<uint64_t>(cr.descChainAddr);
        else
            pkt->set<uint32_t>(bits(cr.descChainAddr,0,31));
        break;
      case CHAN_CHAINADDR_HIGH:
        assert(size == sizeof(uint32_t));
        pkt->set<uint32_t>(bits(cr.descChainAddr,32,63));
        break;
      case CHAN_COMMAND:
        assert(size == sizeof(uint8_t));
        pkt->set<uint32_t>(cr.command());
        break;
      case CHAN_CMPLNADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            pkt->set<uint64_t>(cr.completionAddr);
        else
            pkt->set<uint32_t>(bits(cr.completionAddr,0,31));
        break;
      case CHAN_CMPLNADDR_HIGH:
        assert(size == sizeof(uint32_t));
        pkt->set<uint32_t>(bits(cr.completionAddr,32,63));
        break;
      case CHAN_ERROR:
        assert(size == sizeof(uint32_t));
        pkt->set<uint32_t>(cr.error());
        break;
      default:
        panic("Read request to unknown channel register number: (%d)%#x\n",
                channelId, daddr);
    }
}


Tick
CopyEngine::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;


    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    int size = pkt->getSize();

    ///
    /// Handle write of register here
    ///

    if (size == sizeof(uint64_t)) {
        uint64_t val M5_VAR_USED = pkt->get<uint64_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n", daddr, val);
    } else if (size == sizeof(uint32_t)) {
        uint32_t val M5_VAR_USED = pkt->get<uint32_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n", daddr, val);
    } else if (size == sizeof(uint16_t)) {
        uint16_t val M5_VAR_USED = pkt->get<uint16_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n", daddr, val);
    } else if (size == sizeof(uint8_t)) {
        uint8_t val M5_VAR_USED = pkt->get<uint8_t>();
        DPRINTF(DMACopyEngine, "Wrote device register %#X value %#X\n", daddr, val);
    } else {
        panic("Unknown size for MMIO access: %d\n", size);
    }

    if (daddr < 0x80) {
        switch (daddr) {
          case GEN_CHANCOUNT:
          case GEN_XFERCAP:
          case GEN_ATTNSTATUS:
            DPRINTF(DMACopyEngine, "Warning, ignorning write to register %x\n",
                    daddr);
            break;
          case GEN_INTRCTRL:
            regs.intrctrl.master_int_enable(bits(pkt->get<uint8_t>(),0,1));
            break;
          default:
            panic("Read request to unknown register number: %#x\n", daddr);
        }
        pkt->makeAtomicResponse();
        return pioDelay;
    }

    // Find which channel we're accessing
    int chanid = 0;
    daddr -= 0x80;
    while (daddr >= 0x80) {
        chanid++;
        daddr -= 0x80;
    }

    if (chanid >= regs.chanCount)
        panic("Access to channel %d (device only configured for %d channels)",
                chanid, regs.chanCount);

    ///
    /// Channel registers are handled here
    ///
    chan[chanid]->channelWrite(pkt, daddr, size);

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
CopyEngine::CopyEngineChannel::channelWrite(Packet *pkt, Addr daddr, int size)
{
    switch (daddr) {
      case CHAN_CONTROL:
        assert(size == sizeof(uint16_t));
        int old_int_disable;
        old_int_disable = cr.ctrl.interrupt_disable();
        cr.ctrl(pkt->get<uint16_t>());
        if (cr.ctrl.interrupt_disable())
            cr.ctrl.interrupt_disable(0);
        else
            cr.ctrl.interrupt_disable(old_int_disable);
        break;
      case CHAN_STATUS:
        assert(size == sizeof(uint64_t));
        DPRINTF(DMACopyEngine, "Warning, ignorning write to register %x\n",
                    daddr);
        break;
      case CHAN_CHAINADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            cr.descChainAddr = pkt->get<uint64_t>();
        else
            cr.descChainAddr =  (uint64_t)pkt->get<uint32_t>() |
                (cr.descChainAddr & ~mask(32));
        DPRINTF(DMACopyEngine, "Chain Address %x\n", cr.descChainAddr);
        break;
      case CHAN_CHAINADDR_HIGH:
        assert(size == sizeof(uint32_t));
        cr.descChainAddr =  ((uint64_t)pkt->get<uint32_t>() <<32) |
            (cr.descChainAddr & mask(32));
        DPRINTF(DMACopyEngine, "Chain Address %x\n", cr.descChainAddr);
        break;
      case CHAN_COMMAND:
        assert(size == sizeof(uint8_t));
        cr.command(pkt->get<uint8_t>());
        recvCommand();
        break;
      case CHAN_CMPLNADDR:
        assert(size == sizeof(uint64_t) || size == sizeof(uint32_t));
        if (size == sizeof(uint64_t))
            cr.completionAddr = pkt->get<uint64_t>();
        else
            cr.completionAddr =  pkt->get<uint32_t>() |
                (cr.completionAddr & ~mask(32));
        break;
      case CHAN_CMPLNADDR_HIGH:
        assert(size == sizeof(uint32_t));
        cr.completionAddr =  ((uint64_t)pkt->get<uint32_t>() <<32) |
            (cr.completionAddr & mask(32));
        break;
      case CHAN_ERROR:
        assert(size == sizeof(uint32_t));
        cr.error(~pkt->get<uint32_t>() & cr.error());
        break;
      default:
        panic("Read request to unknown channel register number: (%d)%#x\n",
                channelId, daddr);
    }
}

void
CopyEngine::regStats()
{
    using namespace Stats;
    bytesCopied
        .init(regs.chanCount)
        .name(name() + ".bytes_copied")
        .desc("Number of bytes copied by each engine")
        .flags(total)
        ;
    copiesProcessed
        .init(regs.chanCount)
        .name(name() + ".copies_processed")
        .desc("Number of copies processed by each engine")
        .flags(total)
        ;
}

void
CopyEngine::CopyEngineChannel::fetchDescriptor(Addr address)
{
    anDq();
    anBegin("FetchDescriptor");
    DPRINTF(DMACopyEngine, "Reading descriptor from at memory location %#x(%#x)\n",
           address, ce->pciToDma(address));
    assert(address);
    busy = true;

    DPRINTF(DMACopyEngine, "dmaAction: %#x, %d bytes, to addr %#x\n",
            ce->pciToDma(address), sizeof(DmaDesc), curDmaDesc);

    cePort.dmaAction(MemCmd::ReadReq, ce->pciToDma(address),
                     sizeof(DmaDesc), &fetchCompleteEvent,
                     (uint8_t*)curDmaDesc, latBeforeBegin);
    lastDescriptorAddr = address;
}

void
CopyEngine::CopyEngineChannel::fetchDescComplete()
{
    DPRINTF(DMACopyEngine, "Read of descriptor complete\n");

    if ((curDmaDesc->command & DESC_CTRL_NULL)) {
        DPRINTF(DMACopyEngine, "Got NULL descriptor, skipping\n");
        assert(!(curDmaDesc->command & DESC_CTRL_CP_STS));
        if (curDmaDesc->command & DESC_CTRL_CP_STS) {
            panic("Shouldn't be able to get here\n");
            nextState = CompletionWrite;
            if (inDrain()) return;
            writeCompletionStatus();
        } else {
            anBegin("Idle");
            anWait();
            busy = false;
            nextState = Idle;
            inDrain();
        }
        return;
    }

    if (curDmaDesc->command & ~DESC_CTRL_CP_STS)
        panic("Descriptor has flag other that completion status set\n");

    nextState = DMARead;
    if (inDrain()) return;
    readCopyBytes();
}

void
CopyEngine::CopyEngineChannel::readCopyBytes()
{
    anBegin("ReadCopyBytes");
    DPRINTF(DMACopyEngine, "Reading %d bytes from buffer to memory location %#x(%#x)\n",
           curDmaDesc->len, curDmaDesc->dest,
           ce->pciToDma(curDmaDesc->src));
    cePort.dmaAction(MemCmd::ReadReq, ce->pciToDma(curDmaDesc->src),
                     curDmaDesc->len, &readCompleteEvent, copyBuffer, 0);
}

void
CopyEngine::CopyEngineChannel::readCopyBytesComplete()
{
    DPRINTF(DMACopyEngine, "Read of bytes to copy complete\n");

    nextState = DMAWrite;
    if (inDrain()) return;
    writeCopyBytes();
}

void
CopyEngine::CopyEngineChannel::writeCopyBytes()
{
    anBegin("WriteCopyBytes");
    DPRINTF(DMACopyEngine, "Writing %d bytes from buffer to memory location %#x(%#x)\n",
           curDmaDesc->len, curDmaDesc->dest,
           ce->pciToDma(curDmaDesc->dest));

    cePort.dmaAction(MemCmd::WriteReq, ce->pciToDma(curDmaDesc->dest),
                     curDmaDesc->len, &writeCompleteEvent, copyBuffer, 0);

    ce->bytesCopied[channelId] += curDmaDesc->len;
    ce->copiesProcessed[channelId]++;
}

void
CopyEngine::CopyEngineChannel::writeCopyBytesComplete()
{
    DPRINTF(DMACopyEngine, "Write of bytes to copy complete user1: %#x\n",
            curDmaDesc->user1);

    cr.status.compl_desc_addr(lastDescriptorAddr >> 6);
    completionDataReg = cr.status() | 1;

    anQ("DMAUsedDescQ", channelId, 1);
    anQ("AppRecvQ", curDmaDesc->user1, curDmaDesc->len);
    if (curDmaDesc->command & DESC_CTRL_CP_STS) {
        nextState = CompletionWrite;
        if (inDrain()) return;
        writeCompletionStatus();
        return;
    }

    continueProcessing();
}

void
CopyEngine::CopyEngineChannel::continueProcessing()
{
    busy = false;

    if (underReset) {
        anBegin("Reset");
        anWait();
        underReset = false;
        refreshNext = false;
        busy = false;
        nextState = Idle;
        return;
    }

    if (curDmaDesc->next) {
        nextState = DescriptorFetch;
        fetchAddress = curDmaDesc->next;
        if (inDrain()) return;
        fetchDescriptor(curDmaDesc->next);
    } else if (refreshNext) {
        nextState = AddressFetch;
        refreshNext = false;
        if (inDrain()) return;
        fetchNextAddr(lastDescriptorAddr);
    } else {
        inDrain();
        nextState = Idle;
        anWait();
        anBegin("Idle");
    }
}

void
CopyEngine::CopyEngineChannel::writeCompletionStatus()
{
    anBegin("WriteCompletionStatus");
    DPRINTF(DMACopyEngine, "Writing completion status %#x to address %#x(%#x)\n",
            completionDataReg, cr.completionAddr,
            ce->pciToDma(cr.completionAddr));

    cePort.dmaAction(MemCmd::WriteReq,
                     ce->pciToDma(cr.completionAddr),
                     sizeof(completionDataReg), &statusCompleteEvent,
                     (uint8_t*)&completionDataReg, latAfterCompletion);
}

void
CopyEngine::CopyEngineChannel::writeStatusComplete()
{
    DPRINTF(DMACopyEngine, "Writing completion status complete\n");
    continueProcessing();
}

void
CopyEngine::CopyEngineChannel::fetchNextAddr(Addr address)
{
    anBegin("FetchNextAddr");
    DPRINTF(DMACopyEngine, "Fetching next address...\n");
    busy = true;
    cePort.dmaAction(MemCmd::ReadReq,
                     ce->pciToDma(address + offsetof(DmaDesc, next)),
                     sizeof(Addr), &addrCompleteEvent,
                     (uint8_t*)curDmaDesc + offsetof(DmaDesc, next), 0);
}

void
CopyEngine::CopyEngineChannel::fetchAddrComplete()
{
    DPRINTF(DMACopyEngine, "Fetching next address complete: %#x\n",
            curDmaDesc->next);
    if (!curDmaDesc->next) {
        DPRINTF(DMACopyEngine, "Got NULL descriptor, nothing more to do\n");
        busy = false;
        nextState = Idle;
        anWait();
        anBegin("Idle");
        inDrain();
        return;
    }
    nextState = DescriptorFetch;
    fetchAddress = curDmaDesc->next;
    if (inDrain()) return;
    fetchDescriptor(curDmaDesc->next);
}

bool
CopyEngine::CopyEngineChannel::inDrain()
{
    if (drainState() == DrainState::Draining) {
        DPRINTF(Drain, "CopyEngine done draining, processing drain event\n");
        signalDrainDone();
    }

    return ce->drainState() != DrainState::Running;
}

DrainState
CopyEngine::CopyEngineChannel::drain()
{
    if (nextState == Idle || ce->drainState() != DrainState::Running) {
        return DrainState::Drained;
    } else {
        DPRINTF(Drain, "CopyEngineChannel not drained\n");
        return DrainState::Draining;
    }
}

void
CopyEngine::serialize(CheckpointOut &cp) const
{
    PciDevice::serialize(cp);
    regs.serialize(cp);
    for (int x =0; x < chan.size(); x++)
        chan[x]->serializeSection(cp, csprintf("channel%d", x));
}

void
CopyEngine::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);
    regs.unserialize(cp);
    for (int x = 0; x < chan.size(); x++)
        chan[x]->unserializeSection(cp, csprintf("channel%d", x));
}

void
CopyEngine::CopyEngineChannel::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(channelId);
    SERIALIZE_SCALAR(busy);
    SERIALIZE_SCALAR(underReset);
    SERIALIZE_SCALAR(refreshNext);
    SERIALIZE_SCALAR(lastDescriptorAddr);
    SERIALIZE_SCALAR(completionDataReg);
    SERIALIZE_SCALAR(fetchAddress);
    int nextState = this->nextState;
    SERIALIZE_SCALAR(nextState);
    arrayParamOut(cp, "curDmaDesc", (uint8_t*)curDmaDesc, sizeof(DmaDesc));
    SERIALIZE_ARRAY(copyBuffer, ce->params()->XferCap);
    cr.serialize(cp);

}
void
CopyEngine::CopyEngineChannel::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(channelId);
    UNSERIALIZE_SCALAR(busy);
    UNSERIALIZE_SCALAR(underReset);
    UNSERIALIZE_SCALAR(refreshNext);
    UNSERIALIZE_SCALAR(lastDescriptorAddr);
    UNSERIALIZE_SCALAR(completionDataReg);
    UNSERIALIZE_SCALAR(fetchAddress);
    int nextState;
    UNSERIALIZE_SCALAR(nextState);
    this->nextState = (ChannelState)nextState;
    arrayParamIn(cp, "curDmaDesc", (uint8_t*)curDmaDesc, sizeof(DmaDesc));
    UNSERIALIZE_ARRAY(copyBuffer, ce->params()->XferCap);
    cr.unserialize(cp);

}

void
CopyEngine::CopyEngineChannel::restartStateMachine()
{
    switch(nextState) {
      case AddressFetch:
        fetchNextAddr(lastDescriptorAddr);
        break;
      case DescriptorFetch:
        fetchDescriptor(fetchAddress);
        break;
      case DMARead:
        readCopyBytes();
        break;
      case DMAWrite:
        writeCopyBytes();
        break;
      case CompletionWrite:
        writeCompletionStatus();
        break;
      case Idle:
        break;
      default:
        panic("Unknown state for CopyEngineChannel\n");
    }
}

void
CopyEngine::CopyEngineChannel::drainResume()
{
    DPRINTF(DMACopyEngine, "Restarting state machine at state %d\n", nextState);
    restartStateMachine();
}

CopyEngine *
CopyEngineParams::create()
{
    return new CopyEngine(this);
}
