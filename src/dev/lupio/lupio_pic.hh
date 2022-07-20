/*
 * Copyright (c) 2021 The Regents of the University of California
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

#ifndef __DEV_LUPIO_LUPIO_PIC_HH__
#define __DEV_LUPIO_LUPIO_PIC_HH__

#include "arch/riscv/interrupts.hh"
#include "dev/io_device.hh"
#include "dev/platform.hh"
#include "params/LupioPIC.hh"
#include "sim/system.hh"

#define LUPIO_PIC_NSRC 32

namespace gem5
{

/**
 * LupioPIC:
 * A programmable interrupt controller virtual device that can
 *  manage input IRQs coming from up to 32 sources
 */

class LupioPIC : public BasicPioDevice
{
  protected:
    System *system;
    int nSrc;
    int nThread;
    // Type of interrupt
    int intType;

    const ByteOrder byteOrder = ByteOrder::little;

  // Register map
  private:
    enum
    {
        LUPIO_PIC_PRIO = 0x0,
        LUPIO_PIC_MASK = 0x4,
        LUPIO_PIC_PEND = 0x8,
        LUPIO_PIC_ENAB = 0xC,

        // Max offset
        LUPIO_PIC_MAX = 0x10,
    };

    uint32_t pending = 0;
    // Register for masking or unmasking up to 32 sources
    uint32_t mask[LUPIO_PIC_NSRC];
    // Regitser to determine which input IRQ is routed to the
    // corresponding processor
    uint32_t enable[LUPIO_PIC_NSRC];

  protected:
    /**
     * Function to return information about interrupt requests to the LupIO-PIC
     */
    uint64_t lupioPicRead(const uint8_t addr);
    /**
     * Function to update interrupt requests
     */
    void lupioPicWrite(const uint8_t addr, uint64_t val64);
    /**
     * Function to post and clear interrupts
     **/
    void lupioPicUpdateIRQ();

  public:
    PARAMS(LupioPIC);
    LupioPIC(const Params &params);

    void post(int src_id);
    void clear(int src_id);
    /**
     * Implement BasicPioDevice virtual functions
     */
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;
};

} // namespace gem5

#endif // __DEV_LUPIO_LUPIO_PIC_HH_
