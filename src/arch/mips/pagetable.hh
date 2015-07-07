/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Jaidev Patwardhan
 */

#ifndef __ARCH_MIPS_PAGETABLE_H__
#define __ARCH_MIPS_PAGETABLE_H__

#include "base/misc.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace MipsISA {

struct VAddr
{
};

// ITB/DTB page table entry
struct PTE
{
    Addr Mask;
    Addr VPN;
    uint8_t asid;

    bool G;

    /* Contents of Entry Lo0 */
    Addr PFN0;  // Physical Frame Number - Even
    bool D0;    // Even entry Dirty Bit
    bool V0;    // Even entry Valid Bit
    uint8_t C0; // Cache Coherency Bits - Even

    /* Contents of Entry Lo1 */
    Addr PFN1;  // Physical Frame Number - Odd
    bool D1;    // Odd entry Dirty Bit
    bool V1;    // Odd entry Valid Bit
    uint8_t C1; // Cache Coherency Bits (3 bits)

    /* 
     * The next few variables are put in as optimizations to reduce
     * TLB lookup overheads. For a given Mask, what is the address shift
     * amount, and what is the OffsetMask
     */
    int AddrShiftAmount;
    int OffsetMask;

    bool Valid() { return (V0 | V1); };
    void serialize(CheckpointOut &cp) const;
    void unserialize(CheckpointIn &cp);
};

// WARN: This particular TLB entry is not necessarily conformed to MIPS ISA
struct TlbEntry
{
    Addr _pageStart;
    TlbEntry() {}
    TlbEntry(Addr asn, Addr vaddr, Addr paddr,
             bool uncacheable, bool read_only)
        : _pageStart(paddr)
    {
        if (uncacheable || read_only)
            warn("MIPS TlbEntry does not support uncacheable"
                 " or read-only mappings\n");
    }

    Addr pageStart()
    {
        return _pageStart;
    }

    void
    updateVaddr(Addr new_vaddr) {}

    void serialize(CheckpointOut &cp) const
    {
        SERIALIZE_SCALAR(_pageStart);
    }

    void unserialize(CheckpointIn &cp)
    {
        UNSERIALIZE_SCALAR(_pageStart);
    }

};

};
#endif // __ARCH_MIPS_PAGETABLE_H__

