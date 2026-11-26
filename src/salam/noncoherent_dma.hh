/*
 * Copyright (c) 2025-2026 Akanksha Chaudhari, Matt Sinclair
 * (University of Wisconsin-Madison)
 * All rights reserved.
 *
 * This file contains code derived from gem5-SALAM:
 * https://github.com/TeCSAR-UNCC/gem5-SALAM
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

#include <cstring>

#include "base/statistics.hh"
#include "dev/arm/base_gic.hh"
#include "dev/dma_device.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/NoncoherentDma.hh"
#include "salam/LLVMRead/debug_flags.hh"
#include "salam/dma_write_fifo.hh"
#include "sim/byteswap.hh"

//------------------------------------------
//    Little-endian packed register map
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
    // Clock cycles between transfer steps.
    const Cycles processCycles;

    struct DmaStats : public statistics::Group
    {
        DmaStats(NoncoherentDma *dma);

        statistics::Scalar transfers;
        statistics::Scalar bytes;
        statistics::Scalar beats;
        statistics::Scalar writePackets;
        statistics::Average active;
    } stats;

    uint8_t *mmreg;
    uint8_t *FLAGS;

    Addr pioOffset(PacketPtr pkt) const;

    template <typename T>
    T
    readRegister(Addr offset) const
    {
        T value;
        std::memcpy(&value, mmreg + offset, sizeof(value));
        return letoh(value);
    }

    uint8_t last_flag;

    Addr activeSrc = 0;
    Addr activeDst = 0;
    int writesLeft = 0;
    bool running;

    Tick start_time = 0;

    EventFunctionWrapper tickEvent;

  protected:
    DmaPort accPort;
    DmaReadFifo *getActiveReadFifo();
    DmaWriteFifo *getActiveWriteFifo();

  public:
    PARAMS(NoncoherentDma);
    NoncoherentDma(const NoncoherentDmaParams &p);
    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
    void loadState(CheckpointIn &cp) override;
    ~NoncoherentDma() {}

    AddrRangeList getAddrRanges() const;

    void tick();

    Tick read(PacketPtr pkt);
    Tick write(PacketPtr pkt);

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;
};

#endif //_SALAM_NONCOHERENT_DMA_HH__
