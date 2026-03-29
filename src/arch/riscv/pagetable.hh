/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2020 Barkhausen Institut
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

#ifndef __ARCH_RISCV_PAGETABLE_H__
#define __ARCH_RISCV_PAGETABLE_H__

#include "base/bitunion.hh"
#include "base/logging.hh"
#include "base/trie.hh"
#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{

namespace RiscvISA
{

BitUnion64(SATP)
    Bitfield<63, 60> mode;
    Bitfield<59, 44> asid;
    Bitfield<43, 0> ppn;
EndBitUnion(SATP)

enum AddrXlateMode
{
    BARE = 0,
    SV39 = 8,
    SV48 = 9,
    SV57 = 10,
};

// Sv39 paging
const Addr SV39_XLEN = 64;
const Addr SV39_VADDR_BITS = 39;
const Addr SV39_LEVELS = 3;
const Addr SV39_LEVEL_BITS = 9;
const Addr SV39X4_WIDENED_BITS = 2;

const Addr SV48_XLEN = 64;
const Addr SV48_VADDR_BITS = 48;
const Addr SV48_LEVELS = 4;
const Addr SV48_LEVEL_BITS = 9;
const Addr SV48X4_WIDENED_BITS = 2;

BitUnion64(PTESv39) // совпадает с sv48
    Bitfield<63> n;
    Bitfield<62, 54> reserved;
    Bitfield<53, 10> ppn;
    Bitfield<53, 28> ppn2;
    Bitfield<27, 19> ppn1;
    Bitfield<18, 10> ppn0;
    Bitfield<7> d;
    Bitfield<6> a;
    Bitfield<5> g;
    Bitfield<4> u;
    Bitfield<3, 1> perm;
    Bitfield<3> x;
    Bitfield<2> w;
    Bitfield<1> r;
    Bitfield<0> v;
EndBitUnion(PTESv39)

BitUnion64(PTESv48)
    Bitfield<63> n; // бит 63: зарезервирован (аналогично Sv39)
    Bitfield<62, 54> reserved; // биты 62–54: зарезервированы (должны быть 0)
    Bitfield<53, 10> ppn;  // биты 53–10: 44-битный PPN
    Bitfield<53, 37> ppn3; // биты 53–37 (17 бит) – уровень 3 (самый старший)
    Bitfield<36, 28> ppn2; // биты 36–28 (9 бит)  – уровень 2
    Bitfield<27, 19> ppn1; // биты 27–19 (9 бит)  – уровень 1
    Bitfield<18, 10> ppn0; // биты 18–10 (9 бит)  – уровень 0 (младший)

    Bitfield<7> d; // Dirty
    Bitfield<6> a; // Accessed
    Bitfield<5> g; // Global
    Bitfield<4> u; // User

    Bitfield<3, 1> perm; // биты 3–1: объединённое поле r/w/x
    Bitfield<3> x;       // Execute
    Bitfield<2> w;       // Write
    Bitfield<1> r;       // Read
    Bitfield<0> v;       // Valid
EndBitUnion(PTESv48)

Addr
getXLEN(Addr mode)
{
    switch (mode) {
        case 8: // Sv39
            return SV39_XLEN;
            break;
        case 9: // Sv48
            return SV48_XLEN;
            break;
        default:
            return 0; // недопустимый режим
    }
}

Addr
getVADDR_BITS(Addr mode)
{
    switch (mode) {
        case 8:
            return SV39_VADDR_BITS;
            break;
        case 9:
            return SV48_VADDR_BITS;
            break;
        default:
            return 0;
    }
}

Addr
getLEVELS(Addr mode)
{
    switch (mode) {
        case 8:
            return SV39_LEVELS;
            break;
        case 9:
            return SV48_LEVELS;
            break;
        default:
            return 0;
    }
}

Addr
getLEVEL_BITS(Addr mode)
{
    switch (mode) {
        case 8:
            return SV39_LEVEL_BITS;
            break;
        case 9:
            return SV48_LEVEL_BITS;
            break;
        default:
            return 0;
    }
}

Addr
getWIDENED_BITS(Addr mode)
{
    switch (mode) {
        case 8:
            return SV39X4_WIDENED_BITS;
            break;
        case 9:
            return SV48X4_WIDENED_BITS;
            break;
        default:
            return 0;
    }
}

size_t
getPTESize(Addr mode)
{
    switch (mode) {
        case 8:
            return sizeof(PTESv39);
            break;
        case 9:
            return sizeof(PTESv48);
            break;
        default:
            return 0;
    }
}

struct PTES
{
    uint64_t raw;

    PTES(uint64_t val = 0) : raw(val) {}
    PTES(const PTESv39 &pte) : raw(static_cast<uint64_t>(pte)) {}
    PTES(const PTESv48 &pte) : raw(static_cast<uint64_t>(pte)) {}

    PTESv39
    getSV39() const
    {
        return PTESv39(raw);
    }

    PTESv48
    getSV48() const
    {
        return PTESv48(raw);
    }

    operator uint64_t() const { return raw; }
    PTES &
    operator=(const PTESv39 &pte)
    {
        raw = static_cast<uint64_t>(pte);
        return *this;
    }
    PTES &
    operator=(const PTESv48 &pte)
    {
        raw = static_cast<uint64_t>(pte);
        return *this;
    }
};

/**
 * Remove the page offset and the upper bits that are
 * not part of the VPN from the address.
 * Note that this must assume the smallest page size
 */
Addr getVPNFromVAddr(Addr vaddr, Addr mode);

struct TlbEntry;
typedef Trie<Addr, TlbEntry> TlbEntryTrie;

struct TlbEntry : public Serializable
{
    // The base of the physical page.
    Addr paddr;

    // The beginning of the virtual page this entry maps.
    Addr vaddr;
    // The size of the page this represents, in address bits.
    unsigned logBytes;

    uint16_t asid;

    PTES pte;

    PTES gpte;

    TlbEntryTrie::Handle trieHandle;

    // A sequence number to keep track of LRU.
    uint64_t lruSeq;

    TlbEntry() : paddr(0), vaddr(0), logBytes(0), pte(), gpte(), lruSeq(0) {}

    // Return the page size in bytes
    Addr
    size() const
    {
        return (static_cast<Addr>(1) << logBytes);
    }

    void
    reset()
    {
        paddr = vaddr = logBytes = pte.raw = gpte.raw = lruSeq = 0;
    }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;
};

} // namespace RiscvISA

template <> struct ParseParam<RiscvISA::PTES>
{
    static bool
    parse(const std::string &s, RiscvISA::PTES &val)
    {
        uint64_t tmp;

        if (!to_number(s, tmp)) {
            return false;
        }

        val = RiscvISA::PTES(tmp);
        return true;
    }
};

inline void
paramOut(CheckpointOut &cp, const std::string &name, const RiscvISA::PTES &val)
{
    paramOut(cp, name, static_cast<uint64_t>(val));
}

} // namespace gem5

#endif // __ARCH_RISCV_PAGETABLE_H__
