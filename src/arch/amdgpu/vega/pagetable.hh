/*
 * Copyright (c) 2021 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __ARCH_AMDGPU_VEGA_PAGETABLE_H__
#define __ARCH_AMDGPU_VEGA_PAGETABLE_H__

#include "arch/amdgpu/vega/page_size.hh"
#include "base/bitunion.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{
namespace VegaISA
{

/**
 * The page table entry is reverse engineered from the macros here:
 *
 * https://github.com/RadeonOpenCompute/ROCK-Kernel-Driver/blob/roc-4.3.x/
 *     drivers/gpu/drm/amd/amdgpu/amdgpu_vm.h#L53
 */
BitUnion64(PageTableEntry)
    Bitfield<58, 57> m;
    Bitfield<56> f;
    Bitfield<55> l;
    Bitfield<53, 52> sw;
    Bitfield<51> t;
    Bitfield<47, 12> ppn;
    Bitfield<11, 7> fragment;
    Bitfield<6> w;
    Bitfield<5> r;
    Bitfield<4> x;
    Bitfield<3> z;
    Bitfield<2> c;
    Bitfield<1> s;
    Bitfield<0> v;
EndBitUnion(PageTableEntry)

BitUnion64(PageDirectoryEntry)
    Bitfield<63, 59> blockFragmentSize;
    Bitfield<54> p;
    Bitfield<47, 6> baseAddr;
    Bitfield<2> c;
    Bitfield<1> s;
    Bitfield<0> v;
EndBitUnion(PageDirectoryEntry)

struct VegaTlbEntry : public Serializable
{
    uint16_t vmid;

    // The base of the physical page.
    Addr paddr;

    // The beginning of the virtual page this entry maps.
    Addr vaddr;
    // The size of the page this represents, in address bits.
    unsigned logBytes;

    PageTableEntry pte;

    // Read permission is always available, assuming it isn't blocked by
    // other mechanisms.
    bool
    writable()
    {
        return pte.w;
    };

    // Whether the page is cacheable or not.
    bool
    uncacheable()
    {
        return !pte.c;
    };

    // Whether or not memory on this page can be executed.
    bool
    noExec()
    {
        return !pte.x;
    };

    // A sequence number to keep track of LRU.
    uint64_t lruSeq;

    VegaTlbEntry()
        : vmid(0), paddr(0), vaddr(0), logBytes(PageShift), pte(), lruSeq(0)
    {}

    VegaTlbEntry(Addr _vmid, Addr _vaddr, Addr _paddr, unsigned _logBytes,
                 PageTableEntry _pte)
        : vmid(_vmid),
          paddr(_paddr),
          vaddr(_vaddr),
          logBytes(_logBytes),
          pte(_pte),
          lruSeq(0)
    {}

    // Return the page size in bytes
    Addr
    size() const
    {
        return (static_cast<Addr>(1) << logBytes);
    }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_AMDGPU_VEGA_PAGETABLE_H__
