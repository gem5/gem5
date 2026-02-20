/*
 * Copyright (c) 2025 Akanksha Chaudhari, Matt Sinclair
 * All rights reserved.
 *
 * This file contains modifications and/or code derived from:
 * gem5-SALAM: https://github.com/TeCSAR-UNCC/gem5-SALAM
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "salam/stream_buffer.hh"

#include "debug/AddrRanges.hh"
#include "debug/StreamBuffer.hh"
#include "mem/packet_access.hh"
#include "sim/system.hh"

using namespace std;

StreamBuffer::StreamBuffer(const StreamBufferParams &p)
    : ClockedObject(p),
      streamIn(this),
      streamOut(this),
      statusIn(this),
      statusOut(this),
      buffer(p.buffer_size),
      fifoSize(p.buffer_size),
      endian(p.system->getGuestByteOrder()),
      streamAddr(p.stream_address),
      streamSize(p.stream_size),
      statusAddr(p.status_address),
      statusSize(p.status_size),
      streamDelay(p.stream_latency),
      bandwidth(p.bandwidth)
{}

bool
StreamBuffer::canReadStream(size_t len)
{
    if (buffer.size() >= len) {
        return true;
    } else {
        return false;
    }
}

bool
StreamBuffer::canWriteStream(size_t len)
{
    if ((buffer.size() + len) <= fifoSize) {
        return true;
    } else {
        return false;
    }
}

bool
StreamBuffer::tryReadStream(uint8_t *dst, size_t len)
{
    if (buffer.size() >= len) {
        buffer.read(dst, len);
        return true;
    } else {
        return false;
    }
}

bool
StreamBuffer::tryWriteStream(uint8_t *src, size_t len)
{
    if ((buffer.size() + len) <= fifoSize) {
        buffer.write(src, len);
        return true;
    } else {
        return false;
    }
}

void
StreamBuffer::readStream(uint8_t *dst, size_t len)
{
    const bool success(tryReadStream(dst, len));
    panic_if(!success, "Buffer underrun in StreamBuffer::readStream()\n");
}

void
StreamBuffer::writeStream(uint8_t *src, size_t len)
{
    const bool success(tryWriteStream(src, len));
    panic_if(!success, "Buffer overrun in StreamBuffer::writeStream()\n");
}

bool
StreamBuffer::tvalid(PacketPtr pkt)
{
    return tvalid(pkt->getSize(), pkt->isRead());
}

bool
StreamBuffer::tvalid(size_t len, bool isRead)
{
    return isRead ? canReadStream(len) : canWriteStream(len);
}

Tick
StreamBuffer::streamRead(PacketPtr pkt)
{
    DPRINTF(StreamBuffer,
            "A read request of size %d was received by this"
            " stream buffer\n",
            pkt->getSize());
    uint8_t *buff = new uint8_t[pkt->getSize()];
    readStream(buff, pkt->getSize());
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
StreamBuffer::streamWrite(PacketPtr pkt)
{
    DPRINTF(StreamBuffer,
            "A write request of size %d was received by this"
            " stream buffer\n",
            pkt->getSize());
    uint8_t *data = new uint8_t[pkt->getSize()];
    pkt->writeData(data);
    writeStream(data, pkt->getSize());
    delete[] data;
    pkt->makeAtomicResponse();
    return streamDelay;
}

Tick
StreamBuffer::status(PacketPtr pkt, bool readStatus)
{
    // Provide a means of reading the current buffer capacity of the stream
    // Writes to this register do nothing
    if (pkt->isRead()) {
        DPRINTF(StreamBuffer,
                "The status of the buffer has been read."
                "Current capacity is %d of %d bytes\n",
                buffer.size(), fifoSize);
        uint64_t data = buffer.size();
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

AddrRangeList
StreamBuffer::getStreamAddrRanges() const
{
    assert(streamSize != 0);
    AddrRangeList streamRanges;
    DPRINTF(AddrRanges, "registering range: %#x-%#x\n", streamAddr,
            streamSize);
    streamRanges.push_back(RangeSize(streamAddr, streamSize));
    return streamRanges;
}

AddrRangeList
StreamBuffer::getStatusAddrRanges() const
{
    assert(statusSize != 0);
    AddrRangeList statusRanges;
    DPRINTF(AddrRanges, "registering range: %#x-%#x\n", statusAddr,
            statusSize);
    statusRanges.push_back(RangeSize(statusAddr, statusSize));
    return statusRanges;
}

Port &
StreamBuffer::getPort(const std::string &if_name, PortID idx)
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
    return ClockedObject::getPort(if_name, idx);
}

void
StreamBuffer::serialize(CheckpointOut &cp) const
{
    SERIALIZE_CONTAINER(buffer);
}

void
StreamBuffer::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_CONTAINER(buffer);
}
