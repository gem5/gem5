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

namespace
{

constexpr uint8_t DmaStart = 0x01;
constexpr uint8_t DmaBusy = 0x02;
constexpr uint8_t DmaDone = 0x04;

} // anonymous namespace

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
      clock_period(p.clock_period),
      tickEvent([this] { tick(); }, name()),
      accPort(this, sys, p.sid, p.ssid)
{
    memSideReadFifo = new DmaReadFifo(dmaPort, size_t(bufferSize / 2),
                                      maxReqSize, maxPending);
    memSideWriteFifo = new DmaWriteFifo(dmaPort, size_t(bufferSize / 2),
                                        maxReqSize, maxPending);
    accSideReadFifo = new DmaReadFifo(accPort, size_t(bufferSize / 2),
                                      maxReqSize, maxPending);
    accSideWriteFifo = new DmaWriteFifo(accPort, size_t(bufferSize / 2),
                                        maxReqSize, maxPending);
    readFifo = nullptr;
    writeFifo = nullptr;
    mmreg = new uint8_t[pioSize];
    for (int i = 0; i < pioSize; i++) {
        mmreg[i] = 0;
    }
    FLAGS = mmreg;
    last_flag = 0;
    SRC = (uint64_t *)(mmreg + 1);
    DST = (uint64_t *)(mmreg + 9);
    LEN = (int *)(mmreg + 17);
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
        running = true;
        *FLAGS &= ~DmaStart;
        *FLAGS |= DmaBusy;
        activeSrc = *SRC;
        activeDst = *DST;
        writesLeft = *LEN;
        DPRINTF(NoncoherentDma, "SRC:0x%016x, DST:0x%016x, LEN:%d\n",
                activeSrc, activeDst, writesLeft);
        start_time = curTick();
        readFifo = getActiveReadFifo();
        writeFifo = getActiveWriteFifo();
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
                }
                delete[] data;
            }
        } else {
            if (!writeFifo->isActive()) {
                running = false;
                *FLAGS &= ~DmaBusy;
                *FLAGS |= DmaDone;
                // raise interrupts
                gic->sendInt(intNum);
                double xfer_time = (double)(curTick() - start_time) * (1e-6);
                DPRINTF(NoncoherentDma, "Transfer completed in %f us\n",
                        xfer_time);
            }
        }
    }
    last_flag = *FLAGS;
    if (!tickEvent.scheduled() && running) {
        schedule(tickEvent, curTick() + clock_period * 1000);
    }
}

Tick
NoncoherentDma::read(PacketPtr pkt)
{
    DPRINTF(DeviceMMR,
            "The address range associated with this DMA was read!\n");

    Addr offset = pkt->req->getPaddr() - pioAddr;

    uint64_t data;

    data = *(uint64_t *)(mmreg + offset);

    switch (pkt->getSize()) {
        case 1:
            pkt->setLE<uint8_t>(data);
            break;
        case 2:
            pkt->setLE<uint16_t>(data);
            break;
        case 4:
            pkt->setLE<uint32_t>(data);
            break;
        case 8:
            pkt->setLE<uint64_t>(data);
            break;
        default:
            panic("Read size too big?\n");
            break;
    }

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
NoncoherentDma::write(PacketPtr pkt)
{
    DPRINTF(DeviceMMR,
            "The address range associated with this DMA was written to!\n");
    DPRINTF(DeviceMMR, "LEN Reg:0x%08x\n", *LEN);
    DPRINTF(DeviceMMR, "SRC Reg:0x%016x\n", *SRC);
    DPRINTF(DeviceMMR, "DST Reg:0x%016x\n", *DST);
    DPRINTF(DeviceMMR, "FLAGS Reg:0x%02x\n", *FLAGS);

    pkt->writeData(mmreg + (pkt->req->getPaddr() - pioAddr));

    if (!tickEvent.scheduled()) {
        schedule(tickEvent, curTick() + clock_period * 1000);
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
