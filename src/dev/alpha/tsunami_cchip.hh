/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/** @file
 * Emulation of the Tsunami CChip CSRs
 */

#ifndef __TSUNAMI_CCHIP_HH__
#define __TSUNAMI_CCHIP_HH__

#include "dev/alpha/tsunami.hh"
#include "dev/io_device.hh"
#include "params/TsunamiCChip.hh"

/**
 * Tsunami CChip CSR Emulation. This device includes all the interrupt
 * handling code for the chipset.
 */
class TsunamiCChip : public BasicPioDevice
{
  protected:
    /**
     * pointer to the tsunami object.
     * This is our access to all the other tsunami
     * devices.
     */
    Tsunami *tsunami;

    /**
     * The dims are device interrupt mask registers.
     * One exists for each CPU, the DRIR X DIM = DIR
     */
    uint64_t dim[Tsunami::Max_CPUs];

    /**
     * The dirs are device interrupt registers.
     * One exists for each CPU, the DRIR X DIM = DIR
     */
    uint64_t dir[Tsunami::Max_CPUs];

    /**
     * This register contains bits for each PCI interrupt
     * that can occur.
     */
    uint64_t drir;

    /** Indicator of which CPUs have an IPI interrupt */
    uint64_t ipint;

    /** Indicator of which CPUs have an RTC interrupt */
    uint64_t itint;

  public:
    typedef TsunamiCChipParams Params;
    /**
     * Initialize the Tsunami CChip by setting all of the
     * device register to 0.
     * @param p params struct
     */
    TsunamiCChip(const Params *p);

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }

    Tick read(PacketPtr pkt) override;

    Tick write(PacketPtr pkt) override;

    /**
     * post an RTC interrupt to the CPU
     */
    void postRTC();

    /**
     * post an interrupt to the CPU.
     * @param interrupt the interrupt number to post (0-64)
     */
    void postDRIR(uint32_t interrupt);

    /**
     * clear an interrupt previously posted to the CPU.
     * @param interrupt the interrupt number to post (0-64)
     */
    void clearDRIR(uint32_t interrupt);

    /**
     * post an ipi interrupt  to the CPU.
     * @param ipintr the cpu number to clear(bitvector)
     */
    void clearIPI(uint64_t ipintr);

    /**
     * clear a timer interrupt previously posted to the CPU.
     * @param itintr the cpu number to clear(bitvector)
     */
    void clearITI(uint64_t itintr);

    /**
     * request an interrupt be posted to the CPU.
     * @param ipreq the cpu number to interrupt(bitvector)
     */
    void reqIPI(uint64_t ipreq);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif // __TSUNAMI_CCHIP_HH__
