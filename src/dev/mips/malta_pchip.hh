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
 * Malta PCI interface CSRs
 */

#ifndef __MALTA_PCHIP_HH__
#define __MALTA_PCHIP_HH__

#include "dev/mips/malta.hh"
#include "dev/io_device.hh"
#include "params/MaltaPChip.hh"

/**
 * A very simple implementation of the Malta PCI interface chips.
 */
class MaltaPChip : public BasicPioDevice
{
  protected:

    static const Addr MaltaPciBus0Config = ULL(0x801fe000000);

    /** Pchip control register */
    uint64_t pctl;

    /** Window Base addresses */
    uint64_t wsba[4];

    /** Window masks */
    uint64_t wsm[4];

    /** Translated Base Addresses */
    uint64_t tba[4];

  public:
    typedef MaltaPChipParams Params;

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }
  public:
    /**
     * Register the PChip with the mmu and init all wsba, wsm, and tba to 0
     * @param p pointer to the parameters struct
     */
    MaltaPChip(const Params *p);

    /**
     * Translate a PCI bus address to a memory address for DMA.
     * @todo Andrew says this needs to be fixed. What's wrong with it?
     * @param busAddr PCI address to translate.
     * @return memory system address
     */
    Addr translatePciToDma(Addr busAddr);

    Addr calcConfigAddr(int bus, int dev, int func);

    virtual Tick read(PacketPtr pkt);
    virtual Tick write(PacketPtr pkt);

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

#endif // __TSUNAMI_PCHIP_HH__
