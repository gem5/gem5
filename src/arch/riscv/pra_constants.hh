/*
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
 */

#ifndef __ARCH_RISCV_PRA_CONSTANTS_HH__
#define __ARCH_RISCV_PRA_CONSTANTS_HH__

#include "arch/riscv/types.hh"
#include "base/bitunion.hh"

namespace gem5
{

namespace RiscvISA
{

BitUnion32(IndexReg)
    Bitfield<31> p;
    // Need to figure out how to put in the TLB specific bits here
    // For now, we assume that the entire length is used by the index
    // field In reality, Index_HI = N-1, where
    // N = Ceiling(log2(TLB Entries))
    Bitfield<30, 0> index;
EndBitUnion(IndexReg)

BitUnion32(RandomReg)
    // This has a problem similar to the IndexReg index field. We'll keep
    // both consistent at 30 for now
    Bitfield<30, 0> random;
EndBitUnion(RandomReg)

BitUnion64(EntryLoReg)
    Bitfield<63, 30> fill;
    Bitfield<29, 6> pfn; // Page frame number
    Bitfield<5, 3> c;    // Coherency attribute
    Bitfield<2> d;       // Dirty Bit
    Bitfield<1> v;       // Valid Bit
    Bitfield<0> g;       // Global Bit
EndBitUnion(EntryLoReg)

BitUnion64(ContextReg)
    Bitfield<63, 23> pteBase;
    Bitfield<22, 4> badVPN2;
    // Bits 3-0 are 0
EndBitUnion(ContextReg)

BitUnion32(PageMaskReg)
    // Bits 31-29 are 0
    Bitfield<28, 13> mask;
    Bitfield<12, 11> maskx;
    // Bits 10-0 are zero
EndBitUnion(PageMaskReg)

BitUnion32(PageGrainReg)
    Bitfield<31, 30> aseUp;
    Bitfield<29> elpa;
    Bitfield<28> esp;
    // Bits 27-13 are zeros
    Bitfield<12, 8> aseDn;
    // Bits 7-0 are zeros
EndBitUnion(PageGrainReg)

BitUnion32(WiredReg)
    // See note on Index register above
    Bitfield<30, 0> wired;
EndBitUnion(WiredReg)

BitUnion32(HWREnaReg)
    Bitfield<31, 30> impl;
    Bitfield<3, 0> mask;
EndBitUnion(HWREnaReg)

BitUnion64(EntryHiReg)
    Bitfield<63, 62> r;
    Bitfield<61, 40> fill;
    Bitfield<39, 13> vpn2;
    Bitfield<12, 11> vpn2x;
    Bitfield<7, 0> asid;
EndBitUnion(EntryHiReg)

BitUnion32(StatusReg)
    SubBitUnion(cu, 31, 28)
        Bitfield<31> cu3;
        Bitfield<30> cu2;
        Bitfield<29> cu1;
        Bitfield<28> cu0;
    EndSubBitUnion(cu)
    Bitfield<27> rp;
    Bitfield<26> fr;
    Bitfield<25> re;
    Bitfield<24> mx;
    Bitfield<23> px;
    Bitfield<22> bev;
    Bitfield<21> ts;
    Bitfield<20> sr;
    Bitfield<19> nmi;
    // Bit 18 is zero
    Bitfield<17, 16> impl;
    Bitfield<15, 10> ipl;
    SubBitUnion(im, 15, 8)
        Bitfield<15> im7;
        Bitfield<14> im6;
        Bitfield<13> im5;
        Bitfield<12> im4;
        Bitfield<11> im3;
        Bitfield<10> im2;
        Bitfield<9> im1;
        Bitfield<8> im0;
    EndSubBitUnion(im)
    Bitfield<7> kx;
    Bitfield<6> sx;
    Bitfield<5> ux;
    Bitfield<4, 3> ksu;
    Bitfield<4> um;
    Bitfield<3> r0;
    Bitfield<2> erl;
    Bitfield<1> exl;
    Bitfield<0> ie;
EndBitUnion(StatusReg)

BitUnion32(IntCtlReg)
    Bitfield<31, 29> ipti;
    Bitfield<28, 26> ippci;
    // Bits 26-10 are zeros
    Bitfield<9, 5> vs;
    // Bits 4-0 are zeros
EndBitUnion(IntCtlReg)

BitUnion32(SRSCtlReg)
    // Bits 31-30 are zeros
    Bitfield<29, 26> hss;
    // Bits 25-22 are zeros
    Bitfield<21, 18> eicss;
    // Bits 17-16 are zeros
    Bitfield<15, 12> ess;
    // Bits 11-10 are zeros
    Bitfield<9, 6> pss;
    // Bits 5-4 are zeros
    Bitfield<3, 0> css;
EndBitUnion(SRSCtlReg)

BitUnion32(SRSMapReg)
    Bitfield<31, 28> ssv7;
    Bitfield<27, 24> ssv6;
    Bitfield<23, 20> ssv5;
    Bitfield<19, 16> ssv4;
    Bitfield<15, 12> ssv3;
    Bitfield<11, 8> ssv2;
    Bitfield<7, 4> ssv1;
    Bitfield<3, 0> ssv0;
EndBitUnion(SRSMapReg)

BitUnion32(CauseReg)
    Bitfield<31> bd;
    Bitfield<30> ti;
    Bitfield<29, 28> ce;
    Bitfield<27> dc;
    Bitfield<26> pci;
    // Bits 25-24 are zeros
    Bitfield<23> iv;
    Bitfield<22> wp;
    // Bits 21-16 are zeros
    Bitfield<15, 10> ripl;
    SubBitUnion(ip, 15, 8)
        Bitfield<15> ip7;
        Bitfield<14> ip6;
        Bitfield<13> ip5;
        Bitfield<12> ip4;
        Bitfield<11> ip3;
        Bitfield<10> ip2;
        Bitfield<9> ip1;
        Bitfield<8> ip0;
    EndSubBitUnion(ip);
    // Bit 7 is zero
    Bitfield<6, 2> excCode;
    // Bits 1-0 are zeros
EndBitUnion(CauseReg)

BitUnion32(PRIdReg)
    Bitfield<31, 24> coOp;
    Bitfield<23, 16> coId;
    Bitfield<15, 8> procId;
    Bitfield<7, 0> rev;
EndBitUnion(PRIdReg)

BitUnion32(EBaseReg)
    // Bit 31 is one
    // Bit 30 is zero
    Bitfield<29, 12> exceptionBase;
    // Bits 11-10 are zeros
    Bitfield<9, 9> cpuNum;
EndBitUnion(EBaseReg)

BitUnion32(ConfigReg)
    Bitfield<31> m;
    Bitfield<30, 28> k23;
    Bitfield<27, 25> ku;
    Bitfield<24, 16> impl;
    Bitfield<15> be;
    Bitfield<14, 13> at;
    Bitfield<12, 10> ar;
    Bitfield<9, 7> mt;
    // Bits 6-4 are zeros
    Bitfield<3> vi;
    Bitfield<2, 0> k0;
EndBitUnion(ConfigReg)

BitUnion32(Config1Reg)
    Bitfield<31> m;
    Bitfield<30, 25> mmuSize;
    Bitfield<24, 22> is;
    Bitfield<21, 19> il;
    Bitfield<18, 16> ia;
    Bitfield<15, 13> ds;
    Bitfield<12, 10> dl;
    Bitfield<9, 7> da;
    Bitfield<6> c2;
    Bitfield<5> md;
    Bitfield<4> pc;
    Bitfield<3> wr;
    Bitfield<2> ca;
    Bitfield<1> ep;
    Bitfield<0> fp;
EndBitUnion(Config1Reg)

BitUnion32(Config2Reg)
    Bitfield<31> m;
    Bitfield<30, 28> tu;
    Bitfield<27, 24> ts;
    Bitfield<23, 20> tl;
    Bitfield<19, 16> ta;
    Bitfield<15, 12> su;
    Bitfield<11, 8> ss;
    Bitfield<7, 4> sl;
    Bitfield<3, 0> sa;
EndBitUnion(Config2Reg)

BitUnion32(Config3Reg)
    Bitfield<31> m;
    // Bits 30-11 are zeros
    Bitfield<10> dspp;
    // Bits 9-8 are zeros
    Bitfield<7> lpa;
    Bitfield<6> veic;
    Bitfield<5> vint;
    Bitfield<4> sp;
    // Bit 3 is zero
    Bitfield<2> mt;
    Bitfield<1> sm;
    Bitfield<0> tl;
EndBitUnion(Config3Reg)

BitUnion64(WatchLoReg)
    Bitfield<63, 3> vaddr;
    Bitfield<2> i;
    Bitfield<1> r;
    Bitfield<0> w;
EndBitUnion(WatchLoReg)

BitUnion32(WatchHiReg)
    Bitfield<31> m;
    Bitfield<30> g;
    // Bits 29-24 are zeros
    Bitfield<23, 16> asid;
    // Bits 15-12 are zeros
    Bitfield<11, 3> mask;
    Bitfield<2> i;
    Bitfield<1> r;
    Bitfield<0> w;
EndBitUnion(WatchHiReg)

BitUnion32(PerfCntCtlReg)
    Bitfield<31> m;
    Bitfield<30> w;
    // Bits 29-11 are zeros
    Bitfield<10, 5> event;
    Bitfield<4> ie;
    Bitfield<3> u;
    Bitfield<2> s;
    Bitfield<1> k;
    Bitfield<0> exl;
EndBitUnion(PerfCntCtlReg)

BitUnion32(CacheErrReg)
    Bitfield<31> er;
    Bitfield<30> ec;
    Bitfield<29> ed;
    Bitfield<28> et;
    Bitfield<27> es;
    Bitfield<26> ee;
    Bitfield<25> eb;
    Bitfield<24, 22> impl;
    Bitfield<22, 0> index;
EndBitUnion(CacheErrReg)

BitUnion32(TagLoReg)
    Bitfield<31, 8> pTagLo;
    Bitfield<7, 6> pState;
    Bitfield<5> l;
    Bitfield<4, 3> impl;
    // Bits 2-1 are zeros
    Bitfield<0> p;
EndBitUnion(TagLoReg)

} // namespace RiscvISA
} // namespace gem5

#endif
