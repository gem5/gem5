/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

#ifndef __ARCH_ARM_PAGETABLE_H__
#define __ARCH_ARM_PAGETABLE_H__

#include "arch/arm/isa_traits.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/vtophys.hh"
#include "config/full_system.hh"

namespace ArmISA {

struct VAddr
{
    VAddr(Addr a) { panic("not implemented yet."); }
};


// ITB/DTB page table entry
struct PTE
{
    void serialize(std::ostream &os)
    {
        panic("Need to implement PTE serialization\n");
    }

    void unserialize(Checkpoint *cp, const std::string &section)
    {
        panic("Need to implement PTE serialization\n");
    }

};

struct TlbRange
{
    Addr va;
    Addr size;
    int contextId;
    bool global;

    inline bool
    operator<(const TlbRange &r2) const
    {
        if (!(global || r2.global)) {
            if (contextId < r2.contextId)
                return true;
            else if (contextId > r2.contextId)
                return false;
        }

        if (va < r2.va)
            return true;
        return false;
    }

    inline bool
    operator==(const TlbRange &r2) const
    {
        return va == r2.va &&
               size == r2.size &&
               contextId == r2.contextId &&
               global == r2.global;
    }
};


// ITB/DTB table entry
struct TlbEntry
{
  public:
    enum MemoryType {
        StronglyOrdered,
        Device,
        Normal
    };
    enum DomainType {
        DomainNoAccess = 0,
        DomainClient,
        DomainReserved,
        DomainManager
    };

    // Matching variables
    Addr pfn;
    Addr size;              // Size of this entry, == Type of TLB Rec
    Addr vpn;               // Virtual Page Number
    uint32_t asid;          // Address Space Identifier
    uint8_t N;              // Number of bits in pagesize
    bool global;
    bool valid;

    // Type of memory
    bool nonCacheable;     // Can we wrap this in mtype?
    bool sNp;      // Section descriptor

    // Memory Attributes
    MemoryType mtype;
    uint8_t innerAttrs;
    uint8_t outerAttrs;
    bool shareable;
    uint32_t attributes;    // Memory attributes formatted for PAR


    // Access permissions
    bool xn;                // Execute Never
    uint8_t ap:3;           // Access permissions bits
    uint8_t domain:4;       // Access Domain

    TlbRange range;         // For fast TLB searching

    //Construct an entry that maps to physical address addr for SE mode
    TlbEntry(Addr _asn, Addr _vaddr, Addr _paddr)
    {
        pfn = _paddr >> PageShift;
        size = PageBytes - 1;
        asid = _asn;
        global = false;
        valid = true;

        vpn = _vaddr >> PageShift;

        nonCacheable = sNp = false;

        xn = 0;
        ap = 0; // ???
        domain = DomainClient; //???
    }

    TlbEntry()
    {}

    void
    updateVaddr(Addr new_vaddr)
    {
        vpn = new_vaddr >> PageShift;
    }

    Addr
    pageStart()
    {
        return pfn << PageShift;
    }

    bool
    match(Addr va, uint8_t cid)
    {
        Addr v = vpn << N;
        if (valid && va >= v && va <= v + size && (global || cid == asid))
            return true;
        return false;
    }

    Addr
    pAddr(Addr va)
    {
        return (pfn << N) | (va & size);
    }

    void serialize(std::ostream &os) { panic("Need to Implement\n"); }
    void unserialize(Checkpoint *cp, const std::string &section)
                   { panic("Need to Implement\n");}
};



};
#endif // __ARCH_ARM_PAGETABLE_H__

