/*
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
 */

#ifndef __ARCH_ALPHA_PAGETABLE_H__
#define __ARCH_ALPHA_PAGETABLE_H__

#include "arch/alpha/isa_traits.hh"
#include "arch/alpha/utility.hh"

namespace AlphaISA {

struct VAddr
{
    static const int ImplBits = 43;
    static const Addr ImplMask = (ULL(1) << ImplBits) - 1;
    static const Addr UnImplMask = ~ImplMask;

    Addr addr;

    VAddr(Addr a) : addr(a) {}
    operator Addr() const { return addr; }
    const VAddr &operator=(Addr a) { addr = a; return *this; }

    Addr vpn() const { return (addr & ImplMask) >> PageShift; }
    Addr page() const { return addr & PageMask; }
    Addr offset() const { return addr & PageOffset; }

    Addr level3() const
    { return PteAddr(addr >> PageShift); }
    Addr level2() const
    { return PteAddr(addr >> (NPtePageShift + PageShift)); }
    Addr level1() const
    { return PteAddr(addr >> (2 * NPtePageShift + PageShift)); }
};

struct PageTableEntry
{
    PageTableEntry(uint64_t e) : entry(e) {}
    uint64_t entry;
    operator uint64_t() const { return entry; }
    const PageTableEntry &operator=(uint64_t e) { entry = e; return *this; }
    const PageTableEntry &operator=(const PageTableEntry &e)
    { entry = e.entry; return *this; }

    Addr _pfn()  const { return (entry >> 32) & 0xffffffff; }
    Addr _sw()   const { return (entry >> 16) & 0xffff; }
    int  _rsv0() const { return (entry >> 14) & 0x3; }
    bool _uwe()  const { return (entry >> 13) & 0x1; }
    bool _kwe()  const { return (entry >> 12) & 0x1; }
    int  _rsv1() const { return (entry >> 10) & 0x3; }
    bool _ure()  const { return (entry >>  9) & 0x1; }
    bool _kre()  const { return (entry >>  8) & 0x1; }
    bool _nomb() const { return (entry >>  7) & 0x1; }
    int  _gh()   const { return (entry >>  5) & 0x3; }
    bool _asm_()  const { return (entry >>  4) & 0x1; }
    bool _foe()  const { return (entry >>  3) & 0x1; }
    bool _fow()  const { return (entry >>  2) & 0x1; }
    bool _for()  const { return (entry >>  1) & 0x1; }
    bool valid() const { return (entry >>  0) & 0x1; }

    Addr paddr() const { return _pfn() << PageShift; }
};

// ITB/DTB table entry
struct TlbEntry : public Serializable
{
    Addr tag;               // virtual page number tag
    Addr ppn;               // physical page number
    uint8_t xre;            // read permissions - VMEM_PERM_* mask
    uint8_t xwe;            // write permissions - VMEM_PERM_* mask
    uint8_t asn;            // address space number
    bool asma;              // address space match
    bool fonr;              // fault on read
    bool fonw;              // fault on write
    bool valid;             // valid page table entry


    //Construct an entry that maps to physical address addr.
    TlbEntry(Addr _asn, Addr _vaddr, Addr _paddr,
             bool uncacheable, bool read_only)
    {
        VAddr vaddr(_vaddr);
        VAddr paddr(_paddr);
        tag = vaddr.vpn();
        ppn = paddr.vpn();
        xre = 15;
        xwe = 15;
        asn = _asn;
        asma = false;
        fonr = false;
        fonw = false;
        valid = true;
        if (uncacheable || read_only)
            warn("Alpha TlbEntry does not support uncacheable"
                 " or read-only mappings\n");
    }

    TlbEntry()
        : tag(0), ppn(0), xre(0), xwe(0), asn(0),
          asma(false), fonr(0), fonw(0), valid(0)
    {
    }

    void
    updateVaddr(Addr new_vaddr)
    {
        VAddr vaddr(new_vaddr);
        tag = vaddr.vpn();
    }

    Addr
    pageStart()
    {
        return ppn << PageShift;
    }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace AlphaISA

#endif // __ARCH_ALPHA_PAGETABLE_H__

