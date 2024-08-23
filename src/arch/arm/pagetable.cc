/*
 * Copyright (c) 2013, 2018-2019, 2021, 2024 Arm Limited
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

#include "arch/arm/pagetable.hh"

#include "base/bitfield.hh"
#include "base/logging.hh"

namespace gem5
{

namespace ArmISA
{

const GrainSize GrainMap_tg0[] =
  { Grain4KB, Grain64KB, Grain16KB, ReservedGrain };
const GrainSize GrainMap_tg1[] =
  { ReservedGrain, Grain16KB, Grain4KB, Grain64KB };

Addr
PageTableOps::walkMask(unsigned level) const
{
    return ~mask(walkBits(level));
}

bool
V7LPageTableOps::isValid(pte_t pte, unsigned level) const
{
    switch (level) {
        case 1: return  pte & 0x1;
        case 2: return  pte & 0x1;
        case 3: return (pte & 0x1) && (pte & 0x2);
        default: panic("bad level %d", level);
    }
}

bool
V7LPageTableOps::isLeaf(pte_t pte, unsigned level) const
{
    switch (level) {
        case 1: return !(pte & 0x2);
        case 2: return !(pte & 0x2);
        case 3: return true;
        default: panic("bad level %d", level);
    }
}

bool
V7LPageTableOps::isWritable(pte_t pte, unsigned level, bool stage2) const
{
    return stage2 ? bits(pte, 7, 6)==3 : bits(pte, 7)==0;
}

Addr
V7LPageTableOps::nextLevelPointer(pte_t pte, unsigned level) const
{
    if (isLeaf(pte, level)) {
        switch (level) {
            case 1: return mbits(pte, 39, 30);
            case 2: return mbits(pte, 39, 21);
            case 3: return mbits(pte, 39, 12);
            default: panic("bad level %d", level);
        }
    } else {
        return mbits(pte, 39, 12);
    }
}

Addr
V7LPageTableOps::index(Addr va, unsigned level, int tsz) const
{
    // In theory this should be configurable...
    const int n = 12;

    switch (level) {
        case 1: return bits(va, std::min(26+n, tsz - 1), 30) << 3; break;
        case 2: return bits(va, std::min(29, tsz - 1), 21) << 3; break;
        case 3: return bits(va, std::min(20, tsz - 1), 12) << 3; break;
        default: panic("bad level %d", level);
    }
}

Addr
V7LPageTableOps::pageMask(pte_t pte, unsigned level) const
{
    switch (level) {
        case 1: return ~mask(30);
        case 2: return ~mask(21);
        case 3: return bits(pte, 52) ? ~mask(16) : ~mask(12);
        default: panic("bad level %d", level);
    }
}

unsigned
V7LPageTableOps::walkBits(unsigned level) const
{
    switch (level) {
      case 1: return 30;
      case 2: return 21;
      case 3: return 12;
      default: panic("bad level %d", level);
    }
}

V7LPageTableOps::LookupLevel
V7LPageTableOps::firstLevel(uint8_t tsz) const
{
    return LookupLevel::L1;
}

V7LPageTableOps::LookupLevel
V7LPageTableOps::lastLevel() const
{
    return LookupLevel::L3;
}

bool
V8PageTableOps4k::isValid(pte_t pte, unsigned level) const
{
    switch (level) {
        case 0: return  pte & 0x1;
        case 1: return  pte & 0x1;
        case 2: return  pte & 0x1;
        case 3: return (pte & 0x1) && (pte & 0x2);
        default: panic("bad level %d", level);
    }
}

bool
V8PageTableOps4k::isLeaf(pte_t pte, unsigned level) const
{
    switch (level) {
        case 0: return false;
        case 1: return !(pte & 0x2);
        case 2: return !(pte & 0x2);
        case 3: return true;
        default: panic("bad level %d", level);
    }
}

bool
V8PageTableOps4k::isWritable(pte_t pte, unsigned level, bool stage2) const
{
    return stage2 ? bits(pte, 7, 6)==3 : bits(pte, 7)==0;
}

Addr
V8PageTableOps4k::nextLevelPointer(pte_t pte, unsigned level) const
{
    if (isLeaf(pte, level)) {
        switch (level) {
            // no level 0 here
            case 1: return mbits(pte, 47, 30);
            case 2: return mbits(pte, 47, 21);
            case 3: return mbits(pte, 47, 12);
            default: panic("bad level %d", level);
        }
    } else {
        return mbits(pte, 47, 12);
    }
}

Addr
V8PageTableOps4k::index(Addr va, unsigned level, int tsz) const
{
    switch (level) {
        case 0: return bits(va, std::min(47, tsz - 1), 39) << 3; break;
        case 1: return bits(va, std::min(38, tsz - 1), 30) << 3; break;
        case 2: return bits(va, std::min(29, tsz - 1), 21) << 3; break;
        case 3: return bits(va, std::min(20, tsz - 1), 12) << 3; break;
        default: panic("bad level %d", level);
    }
}

Addr
V8PageTableOps4k::pageMask(pte_t pte, unsigned level) const
{
    switch (level) {
        // no level 0 here
        case 1: return ~mask(30);
        case 2: return ~mask(21);
        case 3: return bits(pte, 52) ? ~mask(16) : ~mask(12);
        default: panic("bad level %d", level);
    }
}

unsigned
V8PageTableOps4k::walkBits(unsigned level) const
{
    switch (level) {
      case 0: return 39;
      case 1: return 30;
      case 2: return 21;
      case 3: return 12;
      default: panic("bad level %d", level);
    }
}

V8PageTableOps4k::LookupLevel
V8PageTableOps4k::firstLevel(uint8_t tsz) const
{
    if (tsz >= 16 && tsz <= 24) return LookupLevel::L0;
    if (tsz >= 25 && tsz <= 33) return LookupLevel::L1;
    if (tsz >= 34 && tsz <= 39) return LookupLevel::L2;

    panic("Unsupported TnSZ: %d\n", tsz);
}

V8PageTableOps4k::LookupLevel
V8PageTableOps4k::firstS2Level(uint8_t sl0) const
{
    switch (sl0) {
      case 0: return LookupLevel::L2;
      case 1: return LookupLevel::L1;
      case 2: return LookupLevel::L0;
      case 3: return LookupLevel::L3; // Only when FEAT_TTST is implemented
      default: panic("Unsupported VTCR_EL2.SL0: %d", sl0);
    }
}

V8PageTableOps4k::LookupLevel
V8PageTableOps4k::lastLevel() const
{
    return LookupLevel::L3;
}

bool
V8PageTableOps16k::isValid(pte_t pte, unsigned level) const
{
    switch (level) {
        case 0: return  pte & 0x1;
        case 1: return  pte & 0x1;
        case 2: return  pte & 0x1;
        case 3: return (pte & 0x1) && (pte & 0x2);
        default: panic("bad level %d", level);
    }
}

bool
V8PageTableOps16k::isLeaf(pte_t pte, unsigned level) const
{
    switch (level) {
        case 0: return false;
        case 1: return false;
        case 2: return !(pte & 0x2);
        case 3: return true;
        default: panic("bad level %d", level);
    }
}

bool
V8PageTableOps16k::isWritable(pte_t pte, unsigned level, bool stage2) const
{
    return stage2 ? bits(pte, 7, 6) == 3 : bits(pte, 7) == 0;
}

Addr
V8PageTableOps16k::nextLevelPointer(pte_t pte, unsigned level) const
{
    if (isLeaf(pte, level)) {
        switch (level) {
            // no level 0 here
            case 1: return mbits(pte, 47, 36);
            case 2: return mbits(pte, 47, 25);
            case 3: return mbits(pte, 47, 14);
            default: panic("bad level %d", level);
        }
    } else {
        return mbits(pte, 47, 14);
    }
}

Addr
V8PageTableOps16k::index(Addr va, unsigned level, int tsz) const
{
    switch (level) {
        case 0: return bits(va, std::min(47, tsz - 1), 47) << 3; break;
        case 1: return bits(va, std::min(46, tsz - 1), 36) << 3; break;
        case 2: return bits(va, std::min(35, tsz - 1), 25) << 3; break;
        case 3: return bits(va, std::min(24, tsz - 1), 14) << 3; break;
        default: panic("bad level %d", level);
    }
}

Addr
V8PageTableOps16k::pageMask(pte_t pte, unsigned level) const
{
    switch (level) {
        // no level 0 here
        case 1: return ~mask(36);
        // 16K granule supports contiguous entries also at L2; - 1G
        case 2: return bits(pte, 52) ? ~mask(30) : ~mask(25);
        // as well as at L3; - 2M
        case 3: return bits(pte, 52) ? ~mask(21) : ~mask(14);
        default: panic("bad level %d", level);
    }
}

unsigned
V8PageTableOps16k::walkBits(unsigned level) const
{
    switch (level) {
      case 0: return 47;
      case 1: return 36;
      case 2: return 25;
      case 3: return 14;
      default: panic("bad level %d", level);
    }
}

V8PageTableOps16k::LookupLevel
V8PageTableOps16k::firstLevel(uint8_t tsz) const
{
    if (tsz == 16) return LookupLevel::L0;
    if (tsz >= 17 && tsz <= 27) return LookupLevel::L1;
    if (tsz >= 28 && tsz <= 38) return LookupLevel::L2;
    if (tsz == 39) return LookupLevel::L3;

    panic("Unsupported TnSZ: %d\n", tsz);
}

V8PageTableOps16k::LookupLevel
V8PageTableOps16k::firstS2Level(uint8_t sl0) const
{
    switch (sl0) {
      case 0: return LookupLevel::L3;
      case 1: return LookupLevel::L2;
      case 2: return LookupLevel::L1;
      default: panic("Unsupported VTCR_EL2.SL0: %d", sl0);
    }
}

V8PageTableOps16k::LookupLevel
V8PageTableOps16k::lastLevel() const
{
    return LookupLevel::L3;
}

bool
V8PageTableOps64k::isValid(pte_t pte, unsigned level) const
{
    switch (level) {
        case 1: return  pte & 0x1;
        case 2: return  pte & 0x1;
        case 3: return (pte & 0x1) && (pte & 0x2);
        default: panic("bad level %d", level);
    }
}

bool
V8PageTableOps64k::isLeaf(pte_t pte, unsigned level) const
{
    switch (level) {
        case 1: return false;
        case 2: return !(pte & 0x2);
        case 3: return true;
        default: panic("bad level %d", level);
    }
}

bool
V8PageTableOps64k::isWritable(pte_t pte, unsigned level, bool stage2) const
{
    return stage2 ? bits(pte, 7, 6)==3 : bits(pte, 7)==0;
}

Addr
V8PageTableOps64k::nextLevelPointer(pte_t pte, unsigned level) const
{
    if (isLeaf(pte, level)) {
        switch (level) {
            // no level 1 here
            case 2: return mbits(pte, 47, 29);
            case 3: return mbits(pte, 47, 16);
            default: panic("bad level %d", level);
        }
    } else {
        return mbits(pte, 47, 16);
    }
}

Addr
V8PageTableOps64k::index(Addr va, unsigned level, int tsz) const
{
    switch (level) {
        case 1: return bits(va, std::min(47, tsz - 1), 42) << 3; break;
        case 2: return bits(va, std::min(41, tsz - 1), 29) << 3; break;
        case 3: return bits(va, std::min(28, tsz - 1), 16) << 3; break;
        default: panic("bad level %d", level);
    }
}

Addr
V8PageTableOps64k::pageMask(pte_t pte, unsigned level) const
{
    switch (level) {
        // no level 1 here
        case 2: return ~mask(29);
        case 3: return bits(pte, 52) ? ~mask(21) : ~mask(16);
        default: panic("bad level %d", level);
    }
}

unsigned
V8PageTableOps64k::walkBits(unsigned level) const
{
    switch (level) {
      case 1: return 42;
      case 2: return 29;
      case 3: return 16;
      default: panic("bad level %d", level);
    }
}

V8PageTableOps64k::LookupLevel
V8PageTableOps64k::firstLevel(uint8_t tsz) const
{
    if (tsz >= 12 && tsz <= 21) return LookupLevel::L1;
    if (tsz >= 22 && tsz <= 34) return LookupLevel::L2;
    if (tsz >= 35 && tsz <= 39) return LookupLevel::L3;

    panic("Unsupported TnSZ: %d\n", tsz);
}

V8PageTableOps64k::LookupLevel
V8PageTableOps64k::firstS2Level(uint8_t sl0) const
{
    switch (sl0) {
      case 0: return LookupLevel::L3;
      case 1: return LookupLevel::L2;
      case 2: return LookupLevel::L1;
      default: panic("Unsupported VTCR_EL2.SL0: %d", sl0);
    }
}

V8PageTableOps64k::LookupLevel
V8PageTableOps64k::lastLevel() const
{
    return LookupLevel::L3;
}

const PageTableOps *
getPageTableOps(GrainSize trans_granule)
{
    static V8PageTableOps4k  ptOps4k;
    static V8PageTableOps16k ptOps16k;
    static V8PageTableOps64k ptOps64k;

    switch (trans_granule) {
    case Grain4KB:  return &ptOps4k;
    case Grain16KB: return &ptOps16k;
    case Grain64KB: return &ptOps64k;
    default:
        panic("Unknown translation granule size %d", trans_granule);
    }
}

} // namespace ArmISA
} // namespace gem5
