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
 * Authors: Nathan Binkert
 *          Steve Reinhardt
 *          Ali Saidi
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

// ITB/DTB table entry
struct TlbEntry
{
    Addr tag;               // virtual page number tag
    Addr ppn;               // physical page number
    uint8_t asn;            // address space number
    bool valid;             // valid page table entry


    //Construct an entry that maps to physical address addr.
    TlbEntry(Addr _asn, Addr _vaddr, Addr _paddr)
    {
        tag = _vaddr >> PageShift;
        ppn = _paddr >> PageShift;
        asn = _asn;
        valid = true;
    }

    TlbEntry()
    {}

    void
    updateVaddr(Addr new_vaddr)
    {
        tag = new_vaddr >> PageShift;
    }

    Addr
    pageStart()
    {
        return ppn << PageShift;
    }

    void serialize(std::ostream &os);
    void unserialize(Checkpoint *cp, const std::string &section);
};



};
#endif // __ARCH_ARM_PAGETABLE_H__

