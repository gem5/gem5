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

#ifndef __SALAM_STREAM_BUFFER_HH__
#define __SALAM_STREAM_BUFFER_HH__

#include "base/circlebuf.hh"
#include "params/StreamBuffer.hh"
#include "salam/stream_port.hh"
#include "sim/clocked_object.hh"

class StreamBuffer : public ClockedObject
{
  private:
    StreamResponsePortT<StreamBuffer> streamIn;
    StreamResponsePortT<StreamBuffer> streamOut;
    StatusPort<StreamBuffer> statusIn;
    StatusPort<StreamBuffer> statusOut;
    Fifo<uint8_t> buffer;
    size_t const fifoSize;
    ByteOrder endian;
    Addr streamAddr;
    Addr streamSize;
    Addr statusAddr;
    Addr statusSize;
    Tick streamDelay;
    const double bandwidth;

  public:
    PARAMS(StreamBuffer);
    StreamBuffer(const StreamBufferParams &p);

    size_t
    size() const
    {
        return buffer.size();
    }
    void
    flush()
    {
        buffer.flush();
    }
    bool canReadStream(size_t len);
    bool canWriteStream(size_t len);
    void readStream(uint8_t *dst, size_t len);
    void writeStream(uint8_t *src, size_t len);
    bool tryReadStream(uint8_t *dst, size_t len);
    bool tryWriteStream(uint8_t *src, size_t len);

    bool tvalid(PacketPtr pkt);
    bool tvalid(size_t len, bool isRead);

    virtual Tick streamRead(PacketPtr pkt);
    virtual Tick streamWrite(PacketPtr pkt);
    Tick status(PacketPtr pkt, bool readStatus);

    AddrRangeList getStreamAddrRanges() const;
    AddrRangeList getStatusAddrRanges() const;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    double
    getBandwidth()
    {
        return bandwidth;
    };
};

#endif // __SALAM_STREAM_BUFFER_HH__
