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

#ifndef __SALAM_STREAM_DMA_HH__
#define __SALAM_STREAM_DMA_HH__

#include "dev/arm/base_gic.hh"
#include "dev/dma_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/StreamDma.hh"
#include "salam/LLVMRead/debug_flags.hh"
#include "salam/dma_write_fifo.hh"
#include "salam/stream_port.hh"

/*
    Steaming DMA device with 2 modes.
    1. Memory to Stream (MM2S)
    2. Stream to Memory (S2MM)
    Data read from memory is stored in a FIFO buffer.
    Reads from the Buffer Access Range will read the FIFO, enabling use with
    devices that make "addressable" memory requests. Writes to the Buffer
    Access Range will store into the write FIFO to be writen back to memory
    later. Supports interrupts based on the number of Frames transmitted.


    Memory Mapped Register 32+N Bytes

    | Buffer Access Range | WrFrameBuffSize | NumWrFrames | WrFrameSize |
    |---------------------|-----------------|-------------|-------------|
    |       N Bytes       |     1 Bytes     |   1 Bytes   |   4 Bytes   |

    | RdFrameBuffSize | NumRdFrames | RdFrameSize | Wr_Addr | Rd_Addr |
    |-----------------|-------------|-------------|---------|---------|
    |     1 Bytes     |   1 Bytes   |   4 Bytes   | 8 Bytes | 8 Bytes |

    | Configs | Flags  |
    |---------|--------|
    | 3 Bytes | 1 Byte |

    Buffer Access Range - Reads/Writes from/to this address range access the
        internal Rd and Wr FIFOs. This is to enable access by devices using
        "addressable" memory requests. The actual memory offset does not
        matter however.
    WrFrameBuffSize - Number of Write frames actually stored in memory.
        Allows for ping-pong buffering in memory.
    NumWrFrames - Total number of frames to write. Use '0' to enable unlimited
        frames.
    WrFrameSize - The size of a Write frame in Bytes.
    RdFrameBuffSize - Number of Read frames actually stored in memory.
        Allows for ping-pong buffering in memory.
    NumRdFrames - Total number of frames to read. Use '0' to enable unlimited
        frames.
    RdFrameSize - The size of a Read frame in Bytes.
    Wr_Addr - The base address to which we are writing the first frame.
    Rd_Addr - The base address from which the first frame is read.

    Config Register
    | Unused  | WrIntFrame | RdIntFrame |
    |---------|------------|------------|
    | 1 Byte  |   1 Byte   |   1 Byte   |
    23        15           7            0

    RdIntFrame - Number of frames to read before raising interrupt '0'
                 for never
    WrIntFrame - Number of frames to write before raising interrupt '0'
                 for never

    Flags Register
    | Unused | WrInt | RdInt | WrRunning | RdRunning | WrStart | RdStart |
    |--------|-------|-------|-----------|-----------|---------|---------|
    | 2 Bits | 1 Bit | 1 Bit |   1 Bit   |   1 Bit   |  1 Bit  |  1 Bit  |
    7        6       5       4           3           2         1         0

*/

#define FLAGS_OFF 0
#define CONFIG_OFF FLAGS_OFF + 1
#define RD_ADDR_OFF CONFIG_OFF + 3
#define WR_ADDR_OFF RD_ADDR_OFF + 8
#define RD_FRAME_SIZE_OFF WR_ADDR_OFF + 8
#define NUM_RD_FRAMES_OFF RD_FRAME_SIZE_OFF + 4
#define RD_FRAME_BUFF_SIZE_OFF NUM_RD_FRAMES_OFF + 1
#define WR_FRAME_SIZE_OFF RD_FRAME_BUFF_SIZE_OFF + 1
#define NUM_WR_FRAMES_OFF WR_FRAME_SIZE_OFF + 4
#define WR_FRAME_BUFF_SIZE_OFF NUM_WR_FRAMES_OFF + 1
#define BUFFER_ACCESS_OFF WR_FRAME_BUFF_SIZE_OFF + 1

#define START_MASK 0x03
#define RD_START_MASK 0x01
#define WR_START_MASK 0x02
#define RUNNING_MASK 0x0C
#define RD_RUNNING_MASK 0x04
#define WR_RUNNING_MASK 0x08
#define INT_MASK 0x30
#define RD_INT_MASK 0x10
#define WR_INT_MASK 0x20

class StreamDma : public DmaDevice
{
  private:
    std::string devname;
    StreamResponsePortT<StreamDma> streamIn;
    StreamResponsePortT<StreamDma> streamOut;
    StatusPort<StreamDma> statusIn;
    StatusPort<StreamDma> statusOut;
    DmaReadFifo *readFifo;
    DmaWriteFifo *writeFifo;
    Addr pioAddr;
    Tick pioDelay;
    Addr pioSize;
    Addr streamAddr;
    Addr streamSize;
    Addr statusAddr;
    Addr statusSize;
    Tick memDelay;
    size_t rdBufferSize;
    size_t wrBufferSize;
    unsigned maxPending;
    unsigned maxReqSize;
    BaseGic *gic;
    uint32_t rdInt;
    uint32_t wrInt;

    bool rdRunning;
    bool wrRunning;
    bool running;

    ByteOrder endian;

    class TickEvent : public Event
    {
      private:
        StreamDma *dma;

      public:
        TickEvent(StreamDma *_dma) : Event(CPU_Tick_Pri), dma(_dma) {}
        void
        process()
        {
            dma->tick();
        }
        virtual const char *
        description() const
        {
            return "StreamDma tick";
        }
    };

    TickEvent tickEvent;

    const double bandwidth;

    uint8_t *mmreg;
    uint8_t *FLAGS;
    uint16_t *CONFIG;
    uint64_t *RD_ADDR;
    uint64_t *WR_ADDR;
    uint32_t *RD_FRAME_SIZE;
    uint8_t *NUM_RD_FRAMES;
    uint8_t *RD_FRAME_BUFF_SIZE;
    uint32_t *WR_FRAME_SIZE;
    uint8_t *NUM_WR_FRAMES;
    uint8_t *WR_FRAME_BUFF_SIZE;

    uint64_t readAddr;
    uint32_t readFrameSize;
    uint8_t framesToRead;
    uint8_t readFrameBuffSize;
    uint64_t framesRead;
    uint8_t readIntFrames;
    uint64_t readPtr;

    uint64_t writeAddr;
    uint32_t writeFrameSize;
    uint8_t framesToWrite;
    uint8_t writeFrameBuffSize;
    uint64_t framesWritten;
    uint8_t writeIntFrames;
    uint64_t writePtr;

  protected:
  public:
    PARAMS(StreamDma);
    StreamDma(const StreamDmaParams &p);
    ~StreamDma() {}

    AddrRangeList getAddrRanges() const;
    AddrRangeList getStreamAddrRanges() const;
    AddrRangeList getStatusAddrRanges() const;

    void tick();

    Tick read(PacketPtr pkt);
    Tick write(PacketPtr pkt);

    Tick streamRead(PacketPtr pkt);
    Tick streamWrite(PacketPtr pkt);
    Tick status(PacketPtr pkt, bool readStatus);

    bool tvalid(PacketPtr pkt);
    bool tvalid(size_t len, bool isRead);

    double
    getBandwidth()
    {
        return bandwidth;
    };

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

#endif //__SALAM_STREAM_DMA_HH__
