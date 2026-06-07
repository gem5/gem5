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

#include "salam/stream_dma.hh"

StreamDma::StreamDma(const StreamDmaParams &p)
    : DmaDevice(p),
      devname(p.devicename),
      streamIn(this),
      streamOut(this),
      statusIn(this, false),
      statusOut(this, true),
      pioAddr(p.pio_addr),
      pioDelay(p.pio_delay),
      pioSize(p.pio_size),
      streamAddr(p.stream_addr),
      streamSize(p.stream_size),
      statusAddr(p.status_addr),
      statusSize(p.status_size),
      memDelay(p.mem_delay),
      rdBufferSize(p.read_buffer_size),
      wrBufferSize(p.write_buffer_size),
      maxPending(p.max_pending),
      maxReqSize(p.max_req_size),
      gic(p.gic),
      rdInt(p.rd_int),
      wrInt(p.wr_int),
      tickEvent(this),
      bandwidth(p.bandwidth)
{
    readFifo = new DmaReadFifo(dmaPort, rdBufferSize, maxReqSize, maxPending);
    writeFifo =
        new DmaWriteFifo(dmaPort, wrBufferSize, maxReqSize, maxPending);
    mmreg = new uint8_t[32];
    for (int i = 0; i < pioSize; i++) {
        mmreg[i] = 0;
    }
    FLAGS = mmreg;
    CONFIG = (uint16_t *)(mmreg + CONFIG_OFF);
    RD_ADDR = (uint64_t *)(mmreg + RD_ADDR_OFF);
    WR_ADDR = (uint64_t *)(mmreg + WR_ADDR_OFF);
    RD_FRAME_SIZE = (uint32_t *)(mmreg + RD_FRAME_SIZE_OFF);
    NUM_RD_FRAMES = (uint8_t *)(mmreg + NUM_RD_FRAMES_OFF);
    RD_FRAME_BUFF_SIZE = (uint8_t *)(mmreg + RD_FRAME_BUFF_SIZE_OFF);
    WR_FRAME_SIZE = (uint32_t *)(mmreg + WR_FRAME_SIZE_OFF);
    NUM_WR_FRAMES = (uint8_t *)(mmreg + NUM_WR_FRAMES_OFF);
    WR_FRAME_BUFF_SIZE = (uint8_t *)(mmreg + WR_FRAME_BUFF_SIZE_OFF);

    rdRunning = false;
    wrRunning = false;
    running = false;

    endian = sys->getGuestByteOrder();
}

AddrRangeList
StreamDma::getAddrRanges() const
{
    assert(pioSize != 0);
    AddrRangeList ranges;
    DPRINTF(AddrRanges, "Valid pio range: %#x-%#x\n", pioAddr,
            pioAddr + pioSize);
    ranges.push_back(RangeSize(pioAddr, pioSize));
    return ranges;
}

AddrRangeList
StreamDma::getStreamAddrRanges() const
{
    assert(streamSize != 0);
    AddrRangeList streamRanges;
    DPRINTF(AddrRanges, "Valid stream range: %#x-%#x\n", streamAddr,
            streamAddr + streamSize);
    streamRanges.push_back(RangeSize(streamAddr, streamSize));
    return streamRanges;
}

AddrRangeList
StreamDma::getStatusAddrRanges() const
{
    assert(statusSize != 0);
    AddrRangeList statusRanges;
    DPRINTF(AddrRanges, "registering range: %#x-%#x\n", statusAddr,
            statusSize);
    statusRanges.push_back(RangeSize(statusAddr, statusSize));
    return statusRanges;
}

void
StreamDma::tick()
{

    if (!rdRunning && ((*FLAGS & RD_START_MASK) == RD_START_MASK)) {
        rdRunning = true;
        *FLAGS &= ~RD_START_MASK;
        *FLAGS |= RD_RUNNING_MASK;
        readAddr = *RD_ADDR;
        readPtr = readAddr;
        readFrameSize = *RD_FRAME_SIZE;
        framesToRead = *NUM_RD_FRAMES;
        readFrameBuffSize = *RD_FRAME_BUFF_SIZE;
        framesRead = 0;
        readIntFrames = *(uint8_t *)CONFIG;
        DPRINTF(StreamDma,
                "Init frame read from 0x%016x with frame size of %d Bytes\n",
                readPtr, readFrameSize);
        readFifo->startFill(readPtr, readFrameSize);
    }

    if (!wrRunning && ((*FLAGS & WR_START_MASK) == WR_START_MASK)) {
        wrRunning = true;
        *FLAGS &= ~WR_START_MASK;
        *FLAGS |= WR_RUNNING_MASK;
        writeAddr = *WR_ADDR;
        writePtr = writeAddr;
        writeFrameSize = *WR_FRAME_SIZE;
        framesToWrite = *NUM_WR_FRAMES;
        writeFrameBuffSize = *WR_FRAME_BUFF_SIZE;
        framesWritten = 0;
        writeIntFrames = *CONFIG >> 8;
        DPRINTF(StreamDma, "MMR After Write: %08x\n", *FLAGS);
        DPRINTF(StreamDma,
                "Init frame write to 0x%016x with frame size of %d Bytes\n",
                writePtr, writeFrameSize);
        writeFifo->startEmpty(writePtr, writeFrameSize);
    }

    if ((*FLAGS & RD_INT_MASK) != RD_INT_MASK) {
        gic->clearInt(rdInt);
    }

    if ((*FLAGS & WR_INT_MASK) != WR_INT_MASK) {
        gic->clearInt(wrInt);
    }

    if (rdRunning && !readFifo->isActive()) {
        framesRead++;
        DPRINTF(StreamDma, "Frame %d of %d read\n", framesRead, framesToRead);
        if (readIntFrames != 0) {
            if (framesRead % readIntFrames == 0) {
                gic->sendInt(rdInt);
                *FLAGS |= RD_INT_MASK;
            }
        }
        if ((framesToRead != 0) && (framesRead >= framesToRead)) {
            rdRunning = false;
            *FLAGS &= ~RD_RUNNING_MASK;
        } else {
            assert(readFrameBuffSize != 0);
            readPtr =
                readAddr + ((framesRead % readFrameBuffSize) * readFrameSize);
            DPRINTF(
                StreamDma,
                "Init frame read from 0x%016x with frame size of %d Bytes\n",
                readPtr, readFrameSize);
            readFifo->startFill(readPtr, readFrameSize);
        }
    }

    if (wrRunning && !writeFifo->isActive()) {
        framesWritten++;
        DPRINTF(StreamDma, "Frame %d of %d written\n", framesWritten,
                framesToWrite);
        if (writeIntFrames != 0) {
            if (framesWritten % writeIntFrames == 0) {
                gic->sendInt(wrInt);
                *FLAGS |= WR_INT_MASK;
            }
        }
        if ((framesToWrite != 0) && (framesWritten >= framesToWrite)) {
            wrRunning = false;
            *FLAGS &= ~WR_RUNNING_MASK;
        } else {
            assert(writeFrameBuffSize != 0);
            writePtr = writeAddr +
                       ((framesWritten % writeFrameBuffSize) * writeFrameSize);
            DPRINTF(
                StreamDma,
                "Init frame write to 0x%016x with frame size of %d Bytes\n",
                writePtr, writeFrameSize);
            writeFifo->startEmpty(writePtr, writeFrameSize);
        }
    }

    running = rdRunning || wrRunning;
    if (!tickEvent.scheduled() && running) {
        schedule(tickEvent, nextCycle());
    }
}

Tick
StreamDma::read(PacketPtr pkt)
{

    Addr offset = pkt->req->getPaddr() - pioAddr;

    if (offset < BUFFER_ACCESS_OFF) {
        DPRINTF(DeviceMMR,
                "The MMR associated with this DMA was read from!\n");

        uint32_t data;
        data = *(uint32_t *)(mmreg + offset);

        switch (pkt->getSize()) {
            case 1:
                pkt->set<uint8_t>(data, endian);
                break;
            case 2:
                pkt->set<uint16_t>(data, endian);
                break;
            case 4:
                pkt->set<uint32_t>(data, endian);
                break;
            default:
                panic("Read size too big?\n");
                break;
        }
    } else {
        DPRINTF(DeviceMMR,
                "The data buffer associated with this DMA was read from!\n");

        uint8_t *buff = new uint8_t[pkt->getSize()];
        readFifo->get(buff, pkt->getSize());
        uint64_t data = *(uint64_t *)buff;
        delete[] buff;

        switch (pkt->getSize()) {
            case 1:
                pkt->set<uint8_t>(data, endian);
                break;
            case 2:
                pkt->set<uint16_t>(data, endian);
                break;
            case 4:
                pkt->set<uint32_t>(data, endian);
                break;
            case 8:
                pkt->set<uint64_t>(data, endian);
                break;
            default:
                panic("Read size too big?\n");
                break;
        }
    }

    if (!tickEvent.scheduled()) {
        schedule(tickEvent, nextCycle());
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
StreamDma::write(PacketPtr pkt)
{

    Addr offset = pkt->req->getPaddr() - pioAddr;

    if (offset < BUFFER_ACCESS_OFF) {
        DPRINTF(DeviceMMR,
                "The MMR associated with this DMA was written to!\n");
        pkt->writeData(mmreg + offset);
    } else {
        DPRINTF(DeviceMMR,
                "The data buffer associated with this DMA was written to!\n");
        uint8_t *data = new uint8_t[pkt->getSize()];
        pkt->writeData(data);
        writeFifo->fill(data, pkt->getSize());
        delete[] data;
    }

    if (!tickEvent.scheduled()) {
        schedule(tickEvent, nextCycle());
    }
    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
StreamDma::streamRead(PacketPtr pkt)
{
    DPRINTF(DeviceMMR,
            "The data buffer associated with this DMA was read from!\n");
    uint8_t *buff = new uint8_t[pkt->getSize()];
    readFifo->get(buff, pkt->getSize());
    uint64_t data = *(uint64_t *)buff;
    delete[] buff;

    switch (pkt->getSize()) {
        case 1:
            pkt->set<uint8_t>(data, endian);
            break;
        case 2:
            pkt->set<uint16_t>(data, endian);
            break;
        case 4:
            pkt->set<uint32_t>(data, endian);
            break;
        case 8:
            pkt->set<uint64_t>(data, endian);
            break;
        default:
            panic("Read size too big?\n");
            break;
    }
    Tick duration = pkt->getSize() * bandwidth;
    pkt->makeAtomicResponse();
    return duration;
}

Tick
StreamDma::streamWrite(PacketPtr pkt)
{
    DPRINTF(DeviceMMR,
            "The data buffer associated with this DMA was written to!\n");
    uint8_t *data = new uint8_t[pkt->getSize()];
    pkt->writeData(data);
    writeFifo->fill(data, pkt->getSize());
    delete[] data;

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
StreamDma::status(PacketPtr pkt, bool readStatus)
{
    // Provide a means of reading the current buffer capacity of the stream
    // Writes to this register do nothing
    if (pkt->isRead()) {
        uint64_t data;
        if (readStatus) {
            DPRINTF(StreamDma,
                    "Read MM2S buffer status. Current capacity: %d/%d bytes\n",
                    readFifo->size(), rdBufferSize);
            data = readFifo->size();
        } else {
            DPRINTF(StreamDma,
                    "Read S2MM buffer status. Current capacity: %d/%d bytes\n",
                    writeFifo->size(), wrBufferSize);
            data = writeFifo->size();
        }

        switch (pkt->getSize()) {
            case 1:
                pkt->set<uint8_t>(data, endian);
                break;
            case 2:
                pkt->set<uint16_t>(data, endian);
                break;
            case 4:
                pkt->set<uint32_t>(data, endian);
                break;
            case 8:
                pkt->set<uint64_t>(data, endian);
                break;
            default:
                panic("Read size too big?\n");
                break;
        }
    }
    Tick duration = pkt->getSize() * bandwidth;
    pkt->makeAtomicResponse();
    return duration;
}

bool
StreamDma::tvalid(PacketPtr pkt)
{
    return tvalid(pkt->getSize(), pkt->isRead());
}

bool
StreamDma::tvalid(size_t len, bool isRead)
{
    if (isRead) {
        return (readFifo->size() >= len) ? true : false;
    } else {
        return writeFifo->canFill(len);
    }
}

Port &
StreamDma::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "stream_in") {
        return streamIn;
    } else if (if_name == "stream_out") {
        return streamOut;
    } else if (if_name == "status_in") {
        return statusIn;
    } else if (if_name == "status_out") {
        return statusOut;
    }
    return DmaDevice::getPort(if_name, idx);
}
