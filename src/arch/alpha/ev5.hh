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
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 *          Ali Saidi
 */

#ifndef __ARCH_ALPHA_EV5_HH__
#define __ARCH_ALPHA_EV5_HH__

#include "arch/alpha/isa_traits.hh"

class ThreadContext;

namespace AlphaISA {

const uint64_t AsnMask = ULL(0xff);
const int VAddrImplBits = 43;
const Addr VAddrImplMask = (ULL(1) << VAddrImplBits) - 1;
const Addr VAddrUnImplMask = ~VAddrImplMask;
inline Addr VAddrImpl(Addr a) { return a & VAddrImplMask; }
inline Addr VAddrVPN(Addr a) { return a >> PageShift; }
inline Addr VAddrOffset(Addr a) { return a & PageOffset; }
inline Addr VAddrSpaceEV5(Addr a) { return a >> 41 & 0x3; }
inline Addr VAddrSpaceEV6(Addr a) { return a >> 41 & 0x7f; }

inline bool PAddrIprSpace(Addr a) { return a >= ULL(0xFFFFFF00000); }
const int PAddrImplBits = 44; // for Tsunami
const Addr PAddrImplMask = (ULL(1) << PAddrImplBits) - 1;
const Addr PAddrUncachedBit39 = ULL(0x8000000000);
const Addr PAddrUncachedBit40 = ULL(0x10000000000);
const Addr PAddrUncachedBit43 = ULL(0x80000000000);
const Addr PAddrUncachedMask = ULL(0x807ffffffff); // Clear PA<42:35>

inline Addr
Phys2K0Seg(Addr addr)
{
    if (addr & PAddrUncachedBit43) {
        addr &= PAddrUncachedMask;
        addr |= PAddrUncachedBit40;
    }
    return addr | K0SegBase;
}

inline int DTB_ASN_ASN(uint64_t reg) { return reg >> 57 & AsnMask; }
inline Addr DTB_PTE_PPN(uint64_t reg)
{ return reg >> 32 & ((ULL(1) << (PAddrImplBits - PageShift)) - 1); }
inline int DTB_PTE_XRE(uint64_t reg) { return reg >> 8 & 0xf; }
inline int DTB_PTE_XWE(uint64_t reg) { return reg >> 12 & 0xf; }
inline int DTB_PTE_FONR(uint64_t reg) { return reg >> 1 & 0x1; }
inline int DTB_PTE_FONW(uint64_t reg) { return reg >> 2 & 0x1; }
inline int DTB_PTE_GH(uint64_t reg) { return reg >> 5 & 0x3; }
inline int DTB_PTE_ASMA(uint64_t reg) { return reg >> 4 & 0x1; }

inline int ITB_ASN_ASN(uint64_t reg) { return reg >> 4 & AsnMask; }
inline Addr ITB_PTE_PPN(uint64_t reg)
{ return reg >> 32 & ((ULL(1) << (PAddrImplBits - PageShift)) - 1); }
inline int ITB_PTE_XRE(uint64_t reg) { return reg >> 8 & 0xf; }
inline bool ITB_PTE_FONR(uint64_t reg) { return reg >> 1 & 0x1; }
inline bool ITB_PTE_FONW(uint64_t reg) { return reg >> 2 & 0x1; }
inline int ITB_PTE_GH(uint64_t reg) { return reg >> 5 & 0x3; }
inline bool ITB_PTE_ASMA(uint64_t reg) { return reg >> 4 & 0x1; }

inline uint64_t MCSR_SP(uint64_t reg) { return reg >> 1 & 0x3; }

inline bool ICSR_SDE(uint64_t reg) { return reg >> 30 & 0x1; }
inline int ICSR_SPE(uint64_t reg) { return reg >> 28 & 0x3; }
inline bool ICSR_FPE(uint64_t reg) { return reg >> 26 & 0x1; }

inline uint64_t ALT_MODE_AM(uint64_t reg) { return reg >> 3 & 0x3; }
inline uint64_t DTB_CM_CM(uint64_t reg) { return reg >> 3 & 0x3; }
inline uint64_t ICM_CM(uint64_t reg) { return reg >> 3 & 0x3; }

const uint64_t MM_STAT_BAD_VA_MASK = ULL(0x0020);
const uint64_t MM_STAT_DTB_MISS_MASK = ULL(0x0010);
const uint64_t MM_STAT_FONW_MASK = ULL(0x0008);
const uint64_t MM_STAT_FONR_MASK = ULL(0x0004);
const uint64_t MM_STAT_ACV_MASK = ULL(0x0002);
const uint64_t MM_STAT_WR_MASK = ULL(0x0001);
inline int Opcode(MachInst inst) { return inst >> 26 & 0x3f; }
inline int Ra(MachInst inst) { return inst >> 21 & 0x1f; }

const Addr PalBase = 0x4000;
const Addr PalMax = 0x10000;

void copyIprs(ThreadContext *src, ThreadContext *dest);

} // namespace AlphaISA

#endif // __ARCH_ALPHA_EV5_HH__
