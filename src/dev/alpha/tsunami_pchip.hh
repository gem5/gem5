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
 * Tsunami PCI interface CSRs
 */

#ifndef __TSUNAMI_PCHIP_HH__
#define __TSUNAMI_PCHIP_HH__

#include "dev/alpha/tsunami.hh"
#include "dev/pci/host.hh"
#include "params/TsunamiPChip.hh"

/**
 * A very simple implementation of the Tsunami PCI interface chips.
 */
class TsunamiPChip : public GenericPciHost
{
  protected:

    /** Pchip control register */
    uint64_t pctl;

    /** Window Base addresses */
    uint64_t wsba[4];

    /** Window masks */
    uint64_t wsm[4];

    /** Translated Base Addresses */
    uint64_t tba[4];

  public:
    typedef TsunamiPChipParams Params;
    /**
     * Register the PChip with the mmu and init all wsba, wsm, and tba to 0
     * @param p pointer to the parameters struct
     */
    TsunamiPChip(const Params *p);

    const Params *
    params() const
    {
        return dynamic_cast<const Params *>(_params);
    }


    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

  public:
    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    AddrRangeList getAddrRanges() const override;

    /**
     * Translate a PCI bus address to a memory address for DMA.
     * @todo Andrew says this needs to be fixed. What's wrong with it?
     * @param pci_addr PCI address to translate.
     * @return memory system address
     */
    Addr dmaAddr(const PciBusAddr &addr, Addr pci_addr) const override;

  protected:
    const AddrRange pioRange;
    const Tick pioDelay;
};

#endif // __TSUNAMI_PCHIP_HH__
