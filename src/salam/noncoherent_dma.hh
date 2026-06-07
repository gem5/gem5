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

#ifndef __SALAM_NONCOHERENT_DMA_HH__
#define __SALAM_NONCOHERENT_DMA_HH__

#include "dev/arm/base_gic.hh"
#include "dev/dma_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/NoncoherentDma.hh"
#include "salam/LLVMRead/debug_flags.hh"
#include "salam/dma_write_fifo.hh"

//------------------------------------------
//    Memory Map
//    |  Length  | Dst Addr | Src Addr | Flags  |
//    |----------|----------|----------|--------|
//    |  4 Bytes | 8 Bytes  | 8 Bytes  | 1 Byte |
//------------------------------------------//

class NoncoherentDma : public DmaDevice
{
  private:
    std::string devname;
    DmaReadFifo *memSideReadFifo;
    DmaReadFifo *accSideReadFifo;
    DmaReadFifo *readFifo;
    DmaWriteFifo *memSideWriteFifo;
    DmaWriteFifo *accSideWriteFifo;
    DmaWriteFifo *writeFifo;
    Addr pioAddr;
    Addr pioDelay;
    Addr pioSize;
    size_t bufferSize;
    unsigned maxPending;
    unsigned maxReqSize;
    BaseGic *gic;
    uint32_t intNum;
    int clock_period;

    uint8_t *mmreg;
    uint8_t *FLAGS;
    uint64_t *SRC;
    uint64_t *DST;
    int *LEN;

    uint8_t last_flag;

    Addr activeSrc;
    Addr activeDst;
    int writesLeft;
    bool running;

    Tick start_time;

    EventFunctionWrapper tickEvent;

  protected:
    DmaPort accPort;
    DmaReadFifo *getActiveReadFifo();
    DmaWriteFifo *getActiveWriteFifo();

  public:
    PARAMS(NoncoherentDma);
    NoncoherentDma(const NoncoherentDmaParams &p);
    ~NoncoherentDma() {}

    AddrRangeList getAddrRanges() const;

    void tick();

    Tick read(PacketPtr pkt);
    Tick write(PacketPtr pkt);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

#endif //_SALAM_NONCOHERENT_DMA_HH__
