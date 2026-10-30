/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#include "salam/noncoherent_dma.hh"

#include <algorithm>

namespace
{

constexpr uint8_t DmaStart = 0x01;
constexpr uint8_t DmaBusy = 0x02;
constexpr uint8_t DmaDone = 0x04;
constexpr Addr SourceOffset = 1;
constexpr Addr DestinationOffset = 9;
constexpr Addr LengthOffset = 17;
constexpr Addr RegisterBytes = 21;

} // anonymous namespace

NoncoherentDma::DmaStats::DmaStats(NoncoherentDma *dma)
    : statistics::Group(dma),
      ADD_STAT(transfers, statistics::units::Count::get(),
               "Transfers started"),
      ADD_STAT(bytes, statistics::units::Byte::get(),
               "Bytes of the transfers started"),
      ADD_STAT(beats, statistics::units::Count::get(),
               "Moves of up to max_req_size bytes from the read buffer to "
               "the write buffer"),
      ADD_STAT(writePackets, statistics::units::Count::get(),
               "Write FIFO requests issued (may split into packets)"),
      ADD_STAT(active, statistics::units::Ratio::get(),
               "Fraction of the window with a transfer in progress")
{}

NoncoherentDma::NoncoherentDma(const NoncoherentDmaParams &p)
    : DmaDevice(p),
      devname(p.devicename),
      pioAddr(p.pio_addr),
      pioDelay(p.pio_delay),
      pioSize(p.pio_size),
      bufferSize(p.buffer_size),
      maxPending(p.max_pending),
      maxReqSize(p.max_req_size),
      gic(p.gic),
      intNum(p.int_num),
      processCycles(p.process_cycles),
      stats(this),
      tickEvent([this] { tick(); }, name()),
      accPort(this, sys, p.sid, p.ssid)
{
    fatal_if(pioSize < RegisterBytes,
             "%s: DMA PIO size must cover %u register bytes", name(),
             RegisterBytes);
    memSideReadFifo = new DmaReadFifo(dmaPort, size_t(bufferSize / 2),
                                      maxReqSize, maxPending);
    memSideWriteFifo = new DmaWriteFifo(dmaPort, size_t(bufferSize / 2),
                                        maxReqSize, maxPending);
    accSideReadFifo = new DmaReadFifo(accPort, size_t(bufferSize / 2),
                                      maxReqSize, maxPending);
    accSideWriteFifo = new DmaWriteFifo(accPort, size_t(bufferSize / 2),
                                        maxReqSize, maxPending);
    memSideWriteFifo->countPacketsIn(&stats.writePackets);
    accSideWriteFifo->countPacketsIn(&stats.writePackets);
    dmaPort.enablePacketStats(this, "memoryPackets");
    accPort.enablePacketStats(this, "clusterPackets");
    readFifo = nullptr;
    writeFifo = nullptr;
    mmreg = new uint8_t[pioSize];
    for (int i = 0; i < pioSize; i++) {
        mmreg[i] = 0;
    }
    FLAGS = mmreg;
    last_flag = 0;
    running = false;
}

AddrRangeList
NoncoherentDma::getAddrRanges() const
{
    assert(pioSize != 0);
    AddrRangeList ranges;
    DPRINTF(AddrRanges, "registering range: %#x-%#x\n", pioAddr, pioSize);
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

// Select the appropriate DmaReadFifo based on which port holds
// the active read address
DmaReadFifo *
NoncoherentDma::getActiveReadFifo()
{
    AddrRangeList accPortRanges = accPort.getAddrRanges();
    for (auto range : accPortRanges) {
        if (range.contains(activeSrc)) {
            return accSideReadFifo;
        }
    }
    return memSideReadFifo;
}

// Select the appropriate DmaWriteFifo based on which port holds
// the active write address
DmaWriteFifo *
NoncoherentDma::getActiveWriteFifo()
{
    AddrRangeList accPortRanges = accPort.getAddrRanges();
    for (auto range : accPortRanges) {
        if (range.contains(activeDst)) {
            return accSideWriteFifo;
        }
    }
    return memSideWriteFifo;
}

void
NoncoherentDma::tick()
{
    if (!running && (*FLAGS & DmaStart)) {
        activeSrc = readRegister<uint64_t>(SourceOffset);
        activeDst = readRegister<uint64_t>(DestinationOffset);
        writesLeft = readRegister<int32_t>(LengthOffset);
        readFifo = getActiveReadFifo();
        writeFifo = getActiveWriteFifo();
        // The controller moves a whole chunk between FIFOs in every
        // memory mode, even when the DMA port completes synchronously.
        const size_t request_size = std::min<size_t>(maxReqSize, writesLeft);
        fatal_if(request_size > readFifo->capacity(),
                 "%s: DMA transfer length %d requires a %u-byte chunk "
                 "exceeding the selected read FIFO capacity of %u bytes",
                 name(), writesLeft, request_size, readFifo->capacity());
        fatal_if(request_size > writeFifo->capacity(),
                 "%s: DMA transfer length %d requires a %u-byte chunk "
                 "exceeding the selected write FIFO capacity of %u bytes",
                 name(), writesLeft, request_size, writeFifo->capacity());
        running = true;
        *FLAGS &= ~DmaStart;
        *FLAGS |= DmaBusy;
        DPRINTF(NoncoherentDma, "SRC:0x%016x, DST:0x%016x, LEN:%d\n",
                activeSrc, activeDst, writesLeft);
        start_time = curTick();
        stats.transfers++;
        stats.bytes += writesLeft;
        stats.active = 1;
        readFifo->startFill(activeSrc, writesLeft);
        writeFifo->startEmpty(activeDst, writesLeft);
    }
    if ((last_flag & DmaDone) && !(*FLAGS & DmaDone)) {
        // clear interrupts
        gic->clearInt(intNum);
    }
    if (running) {
        if (writesLeft > 0) {
            int toWrite = MIN(maxReqSize, writesLeft);
            if (writeFifo->canFill(toWrite)) {
                uint8_t *data = new uint8_t[toWrite];
                if (readFifo->tryGet(data, toWrite)) {
                    writeFifo->fill(data, toWrite);
                    writesLeft -= toWrite;
                    stats.beats++;
                }
                delete[] data;
            }
        } else {
            if (!writeFifo->isActive()) {
                running = false;
                stats.active = 0;
                *FLAGS &= ~DmaBusy;
                *FLAGS |= DmaDone;
                // raise interrupts
                gic->sendInt(intNum);
                double xfer_time =
                    (curTick() - start_time) / sim_clock::as_float::us;
                DPRINTF(NoncoherentDma, "Transfer completed in %f us\n",
                        xfer_time);
            }
        }
    }
    last_flag = *FLAGS;
    if (!tickEvent.scheduled() && running) {
        schedule(tickEvent, clockEdge(processCycles));
    }
}

Addr
NoncoherentDma::pioOffset(PacketPtr pkt) const
{
    const Addr addr = pkt->req->getPaddr();
    panic_if(addr < pioAddr, "DMA PIO access precedes register range");
    const Addr offset = addr - pioAddr;
    panic_if(offset >= pioSize || pkt->getSize() == 0 ||
                 pkt->getSize() > pioSize - offset,
             "DMA PIO access exceeds register range");
    return offset;
}

Tick
NoncoherentDma::read(PacketPtr pkt)
{
    DPRINTF(DeviceMMR,
            "The address range associated with this DMA was read!\n");

    const Addr offset = pioOffset(pkt);
    const auto size = pkt->getSize();
    panic_if(size != 1 && size != 2 && size != 4 && size != 8,
             "Unsupported DMA register read size %u", size);
    pkt->setData(mmreg + offset);

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
NoncoherentDma::write(PacketPtr pkt)
{
    DPRINTF(DeviceMMR,
            "The address range associated with this DMA was written to!\n");
    const Addr offset = pioOffset(pkt);
    DPRINTF(DeviceMMR, "LEN Reg:0x%08x\n",
            readRegister<int32_t>(LengthOffset));
    DPRINTF(DeviceMMR, "SRC Reg:0x%016x\n",
            readRegister<uint64_t>(SourceOffset));
    DPRINTF(DeviceMMR, "DST Reg:0x%016x\n",
            readRegister<uint64_t>(DestinationOffset));
    DPRINTF(DeviceMMR, "FLAGS Reg:0x%02x\n", *FLAGS);

    pkt->writeData(mmreg + offset);

    if (!tickEvent.scheduled()) {
        schedule(tickEvent, clockEdge(processCycles));
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

Port &
NoncoherentDma::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "cluster_dma") {
        return accPort;
    }
    return DmaDevice::getPort(if_name, idx);
}
