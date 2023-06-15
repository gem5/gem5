/*
 * Copyright (c) 2010-2022 Arm Limited
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
 * Copyright (c) 2009 The Regents of The University of Michigan
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

#ifndef __ARCH_ARM_REGS_MISC_TYPES_HH__
#define __ARCH_ARM_REGS_MISC_TYPES_HH__

#include "base/bitunion.hh"

namespace gem5
{

namespace ArmISA
{
    BitUnion32(CPSR)
        Bitfield<31, 30> nz;
        Bitfield<29> c;
        Bitfield<28> v;
        Bitfield<27> q;
        Bitfield<26, 25> it1;
        Bitfield<24> j;
        Bitfield<23> uao;       // AArch64
        Bitfield<22> pan;
        Bitfield<21> ss;        // AArch64
        Bitfield<20> il;        // AArch64
        Bitfield<19, 16> ge;
        Bitfield<15, 10> it2;
        Bitfield<9> d;          // AArch64
        Bitfield<9> e;
        Bitfield<8> a;
        Bitfield<7> i;
        Bitfield<6> f;
        Bitfield<8, 6> aif;
        Bitfield<9, 6> daif;    // AArch64
        Bitfield<5> t;
        Bitfield<4> width;      // AArch64
        Bitfield<3, 2> el;      // AArch64
        Bitfield<4, 0> mode;
        Bitfield<0> sp;         // AArch64
    EndBitUnion(CPSR)

    BitUnion32(ISAR5)
        Bitfield<31, 28> vcma;
        Bitfield<27, 24> rdm;
        Bitfield<19, 16> crc32;
        Bitfield<15, 12> sha2;
        Bitfield<11, 8> sha1;
        Bitfield<7, 4> aes;
        Bitfield<3, 0> sevl;
    EndBitUnion(ISAR5)

    BitUnion32(ISAR6)
        Bitfield<31, 28> clrbhb;
        Bitfield<27, 24> i8mm;
        Bitfield<23, 20> bf16;
        Bitfield<19, 16> specres;
        Bitfield<15, 12> sb;
        Bitfield<11, 8> fhm;
        Bitfield<7, 4> dp;
        Bitfield<3, 0> jscvt;
    EndBitUnion(ISAR6)

    BitUnion64(AA64DFR0)
        Bitfield<43, 40> tracefilt;
        Bitfield<39, 36> doublelock;
        Bitfield<35, 32> pmsver;
        Bitfield<31, 28> ctx_cmps;
        Bitfield<23, 20> wrps;
        Bitfield<15, 12> brps;
        Bitfield<11, 8> pmuver;
        Bitfield<7, 4> tracever;
        Bitfield<3, 0> debugver;
    EndBitUnion(AA64DFR0)

    BitUnion64(AA64ISAR0)
        Bitfield<63, 60> rndr;
        Bitfield<59, 56> tlb;
        Bitfield<55, 52> ts;
        Bitfield<51, 48> fhm;
        Bitfield<47, 44> dp;
        Bitfield<43, 40> sm4;
        Bitfield<39, 36> sm3;
        Bitfield<35, 32> sha3;
        Bitfield<31, 28> rdm;
        Bitfield<27, 24> tme;
        Bitfield<23, 20> atomic;
        Bitfield<19, 16> crc32;
        Bitfield<15, 12> sha2;
        Bitfield<11, 8> sha1;
        Bitfield<7, 4> aes;
    EndBitUnion(AA64ISAR0)

    BitUnion64(AA64ISAR1)
        Bitfield<55, 52> i8mm;
        Bitfield<43, 40> specres;
        Bitfield<39, 36> sb;
        Bitfield<35, 32> frintts;
        Bitfield<31, 28> gpi;
        Bitfield<27, 24> gpa;
        Bitfield<23, 20> lrcpc;
        Bitfield<19, 16> fcma;
        Bitfield<15, 12> jscvt;
        Bitfield<11, 8> api;
        Bitfield<7, 4> apa;
        Bitfield<3, 0> dpb;
    EndBitUnion(AA64ISAR1)

    BitUnion64(AA64MMFR0)
        Bitfield<63, 60> ecv;
        Bitfield<47, 44> exs;
        Bitfield<43, 40> tgran4_2;
        Bitfield<39, 36> tgran64_2;
        Bitfield<35, 32> tgran16_2;
        Bitfield<31, 28> tgran4;
        Bitfield<27, 24> tgran64;
        Bitfield<23, 20> tgran16;
        Bitfield<19, 16> bigendEL0;
        Bitfield<15, 12> snsmem;
        Bitfield<11, 8> bigend;
        Bitfield<7, 4> asidbits;
        Bitfield<3, 0> parange;
    EndBitUnion(AA64MMFR0)

    BitUnion64(AA64MMFR1)
        Bitfield<43, 40> hcx;
        Bitfield<31, 28> xnx;
        Bitfield<27, 24> specsei;
        Bitfield<23, 20> pan;
        Bitfield<19, 16> lo;
        Bitfield<15, 12> hpds;
        Bitfield<11, 8> vh;
        Bitfield<7, 4> vmidbits;
        Bitfield<3, 0> hafdbs;
    EndBitUnion(AA64MMFR1)

    BitUnion64(AA64MMFR2)
        Bitfield<63, 60> e0pd;
        Bitfield<59, 56> evt;
        Bitfield<55, 52> bbm;
        Bitfield<51, 48> ttl;
        Bitfield<43, 40> fwb;
        Bitfield<39, 36> ids;
        Bitfield<35, 32> at;
        Bitfield<31, 28> st;
        Bitfield<27, 24> nv;
        Bitfield<23, 20> ccidx;
        Bitfield<19, 16> varange;
        Bitfield<15, 12> iesb;
        Bitfield<11, 8> lsm;
        Bitfield<7, 4> uao;
        Bitfield<3, 0> cnp;
    EndBitUnion(AA64MMFR2)

    BitUnion64(AA64PFR0)
        Bitfield<63, 60> csv3;
        Bitfield<59, 56> csv2;
        Bitfield<51, 48> dit;
        Bitfield<47, 44> amu;
        Bitfield<43, 40> mpam;
        Bitfield<39, 36> sel2;
        Bitfield<35, 32> sve;
        Bitfield<31, 28> ras;
        Bitfield<27, 24> gic;
        Bitfield<23, 20> advsimd;
        Bitfield<19, 16> fp;
        Bitfield<15, 12> el3;
        Bitfield<11, 8> el2;
        Bitfield<7, 4> el1;
        Bitfield<3, 0> el0;
    EndBitUnion(AA64PFR0)

    BitUnion64(AA64ZFR0)
        Bitfield<59, 56> f64mm;
        Bitfield<55, 52> f32mm;
        Bitfield<47, 44> i8mm;
        Bitfield<43, 40> sm4;
        Bitfield<35, 32> sha3;
        Bitfield<27, 24> b16b16;
        Bitfield<23, 20> bf16;
        Bitfield<19, 16> bitPerm;
        Bitfield<7, 4> aes;
        Bitfield<3, 0> sveVer;
    EndBitUnion(AA64ZFR0)

    BitUnion64(AA64SMFR0)
        Bitfield<63> fa64;
        Bitfield<59, 56> smEver;
        Bitfield<55, 52> i16i64;
        Bitfield<48> f64f64;
        Bitfield<39, 36> i8i32;
        Bitfield<35> f16f32;
        Bitfield<34> b16f32;
        Bitfield<32> f32f32;
    EndBitUnion(AA64SMFR0)

    BitUnion32(HDCR)
        Bitfield<27>   tdcc;
        Bitfield<11>   tdra;
        Bitfield<10>   tdosa;
        Bitfield<9>    tda;
        Bitfield<8>    tde;
        Bitfield<7>    hpme;
        Bitfield<6>    tpm;
        Bitfield<5>    tpmcr;
        Bitfield<4, 0> hpmn;
    EndBitUnion(HDCR)

    BitUnion32(HCPTR)
        Bitfield<31> tcpac;
        Bitfield<20> tta;
        Bitfield<15> tase;
        Bitfield<13> tcp13;
        Bitfield<12> tcp12;
        Bitfield<11> tcp11;
        Bitfield<10> tcp10;
        Bitfield<10> tfp;  // AArch64
        Bitfield<9>  tcp9;
        Bitfield<8>  tcp8;
        Bitfield<8>  tz;  // SVE
        Bitfield<7>  tcp7;
        Bitfield<6>  tcp6;
        Bitfield<5>  tcp5;
        Bitfield<4>  tcp4;
        Bitfield<3>  tcp3;
        Bitfield<2>  tcp2;
        Bitfield<1>  tcp1;
        Bitfield<0>  tcp0;
    EndBitUnion(HCPTR)

    BitUnion32(HSTR)
        Bitfield<17> tjdbx;
        Bitfield<16> ttee;
        Bitfield<15> t15;
        Bitfield<13> t13;
        Bitfield<12> t12;
        Bitfield<11> t11;
        Bitfield<10> t10;
        Bitfield<9>  t9;
        Bitfield<8>  t8;
        Bitfield<7>  t7;
        Bitfield<6>  t6;
        Bitfield<5>  t5;
        Bitfield<4>  t4;
        Bitfield<3>  t3;
        Bitfield<2>  t2;
        Bitfield<1>  t1;
        Bitfield<0>  t0;
    EndBitUnion(HSTR)

    BitUnion64(HCR)
        Bitfield<55>     ttlbos;
        Bitfield<54>     ttlbis;
        Bitfield<52>     tocu;
        Bitfield<50>     ticab;
        Bitfield<49>     tid4;
        Bitfield<47>     fien;
        Bitfield<46>     fwb;
        Bitfield<45>     nv2;
        Bitfield<44>     at;
        Bitfield<43>     nv1;
        Bitfield<42>     nv;
        Bitfield<41>     api;
        Bitfield<40>     apk;
        Bitfield<38>     miocnce;
        Bitfield<37>     tea;
        Bitfield<36>     terr;
        Bitfield<35>     tlor;
        Bitfield<34>     e2h;   // AArch64
        Bitfield<33>     id;
        Bitfield<32>     cd;
        Bitfield<31>     rw;    // AArch64
        Bitfield<30>     trvm;  // AArch64
        Bitfield<29>     hcd;   // AArch64
        Bitfield<28>     tdz;   // AArch64
        Bitfield<27>     tge;
        Bitfield<26>     tvm;
        Bitfield<25>     ttlb;
        Bitfield<24>     tpu;
        Bitfield<23>     tpc;
        Bitfield<22>     tsw;
        Bitfield<21>     tac;
        Bitfield<21>     tacr;  // AArch64
        Bitfield<20>     tidcp;
        Bitfield<19>     tsc;
        Bitfield<18>     tid3;
        Bitfield<17>     tid2;
        Bitfield<16>     tid1;
        Bitfield<15>     tid0;
        Bitfield<14>     twe;
        Bitfield<13>     twi;
        Bitfield<12>     dc;
        Bitfield<11, 10> bsu;
        Bitfield<9>      fb;
        Bitfield<8>      va;
        Bitfield<8>      vse;   // AArch64
        Bitfield<7>      vi;
        Bitfield<6>      vf;
        Bitfield<5>      amo;
        Bitfield<4>      imo;
        Bitfield<3>      fmo;
        Bitfield<2>      ptw;
        Bitfield<1>      swio;
        Bitfield<0>      vm;
    EndBitUnion(HCR)

    BitUnion32(NSACR)
        Bitfield<20> nstrcdis;
        Bitfield<19> rfr;
        Bitfield<15> nsasedis;
        Bitfield<14> nsd32dis;
        Bitfield<13> cp13;
        Bitfield<12> cp12;
        Bitfield<11> cp11;
        Bitfield<10> cp10;
        Bitfield<9>  cp9;
        Bitfield<8>  cp8;
        Bitfield<7>  cp7;
        Bitfield<6>  cp6;
        Bitfield<5>  cp5;
        Bitfield<4>  cp4;
        Bitfield<3>  cp3;
        Bitfield<2>  cp2;
        Bitfield<1>  cp1;
        Bitfield<0>  cp0;
    EndBitUnion(NSACR)

    BitUnion64(SCR)
        Bitfield<40> trndr;
        Bitfield<38> hxen;
        Bitfield<21> fien;
        Bitfield<20> nmea;
        Bitfield<19> ease;
        Bitfield<18> eel2; // AArch64 (Armv8.4-SecEL2)
        Bitfield<17> api;
        Bitfield<16> apk;
        Bitfield<15> teer;
        Bitfield<14> tlor;
        Bitfield<13> twe;
        Bitfield<12> twi;
        Bitfield<11> st;   // AArch64
        Bitfield<10> rw;   // AArch64
        Bitfield<9> sif;
        Bitfield<8> hce;
        Bitfield<7> scd;
        Bitfield<7> smd;   // AArch64
        Bitfield<6> nEt;
        Bitfield<5> aw;
        Bitfield<4> fw;
        Bitfield<3> ea;
        Bitfield<2> fiq;
        Bitfield<1> irq;
        Bitfield<0> ns;
    EndBitUnion(SCR)

    BitUnion64(SCTLR)
        Bitfield<31>   enia;    // ARMv8.3 PAuth
        Bitfield<30>   enib;    // ARMv8.3 PAuth
        Bitfield<30>   te;      // Thumb Exception Enable (AArch32 only)
        Bitfield<29>   afe;     // Access flag enable (AArch32 only)
        Bitfield<28>   tre;     // TEX remap enable (AArch32 only)
        Bitfield<27>   nmfi;    // Non-maskable FIQ support (ARMv7 only)
        Bitfield<27>   enda;    // ARMv8.3 PAuth
        Bitfield<26>   uci;     // Enable EL0 access to DC CVAU, DC CIVAC,
                                // DC CVAC and IC IVAU instructions
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<25>   ee;      // Exception Endianness
        Bitfield<24>   e0e;     // Endianness of explicit data accesses at EL0
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<23>   span;    // Set Priviledge Access Never on taking
                                // an exception
        Bitfield<23>   xp;      // Extended page table enable
                                // (dropped in ARMv7)
        Bitfield<22>   u;       // Alignment (dropped in ARMv7)
        Bitfield<21>   fi;      // Fast interrupts configuration enable
                                // (ARMv7 only)
        Bitfield<20>   uwxn;    // Unprivileged write permission implies EL1 XN
                                // (AArch32 only)
        Bitfield<19>   dz;      // Divide by Zero fault enable
                                // (dropped in ARMv7)
        Bitfield<19>   wxn;     // Write permission implies XN
        Bitfield<18>   ntwe;    // Not trap WFE
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<18>   rao2;    // Read as one
        Bitfield<16>   ntwi;    // Not trap WFI
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<16>   rao3;    // Read as one
        Bitfield<15>   uct;     // Enable EL0 access to CTR_EL0
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<14>   rr;      // Round Robin select (ARMv7 only)
        Bitfield<14>   dze;     // Enable EL0 access to DC ZVA
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<13>   v;       // Vectors bit (AArch32 only)
        Bitfield<13>   endb;    // ARMv8.3 PAuth
        Bitfield<12>   i;       // Instruction cache enable
        Bitfield<11>   z;       // Branch prediction enable (ARMv7 only)
        Bitfield<10>   sw;      // SWP/SWPB enable (ARMv7 only)
        Bitfield<9, 8> rs;      // Deprecated protection bits
                                // (dropped in ARMv7)
        Bitfield<9>    uma;     // User mask access (AArch64 SCTLR_EL1 only)
        Bitfield<8>    sed;     // SETEND disable
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<7>    b;       // Endianness support (dropped in ARMv7)
        Bitfield<7>    itd;     // IT disable
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<6, 3> rao4;    // Read as one
        Bitfield<6>    thee;    // ThumbEE enable
                                // (ARMv8 AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<5>    cp15ben; // CP15 barrier enable
                                // (AArch32 and AArch64 SCTLR_EL1 only)
        Bitfield<4>    sa0;     // Stack Alignment Check Enable for EL0
                                // (AArch64 SCTLR_EL1 only)
        Bitfield<3>    sa;      // Stack Alignment Check Enable (AArch64 only)
        Bitfield<2>    c;       // Cache enable
        Bitfield<1>    a;       // Alignment check enable
        Bitfield<0>    m;       // MMU enable
    EndBitUnion(SCTLR)

    BitUnion32(CPACR)
        Bitfield<1, 0> cp0;
        Bitfield<3, 2> cp1;
        Bitfield<5, 4> cp2;
        Bitfield<7, 6> cp3;
        Bitfield<9, 8> cp4;
        Bitfield<11, 10> cp5;
        Bitfield<13, 12> cp6;
        Bitfield<15, 14> cp7;
        Bitfield<17, 16> cp8;
        Bitfield<17, 16> zen;  // SVE
        Bitfield<19, 18> cp9;
        Bitfield<21, 20> cp10;
        Bitfield<21, 20> fpen;  // AArch64
        Bitfield<23, 22> cp11;
        Bitfield<25, 24> smen; // SME
        Bitfield<25, 24> cp12;
        Bitfield<27, 26> cp13;
        Bitfield<29, 28> rsvd;
        Bitfield<28> tta;  // AArch64
        Bitfield<30> d32dis;
        Bitfield<31> asedis;
    EndBitUnion(CPACR)

    BitUnion32(FSR)
        Bitfield<3, 0> fsLow;
        Bitfield<5, 0> status;  // LPAE
        Bitfield<7, 4> domain;
        Bitfield<9> lpae;
        Bitfield<10> fsHigh;
        Bitfield<11> wnr;
        Bitfield<12> ext;
        Bitfield<13> cm;  // LPAE
    EndBitUnion(FSR)

    BitUnion32(FPSCR)
        Bitfield<0> ioc;
        Bitfield<1> dzc;
        Bitfield<2> ofc;
        Bitfield<3> ufc;
        Bitfield<4> ixc;
        Bitfield<7> idc;
        Bitfield<8> ioe;
        Bitfield<9> dze;
        Bitfield<10> ofe;
        Bitfield<11> ufe;
        Bitfield<12> ixe;
        Bitfield<15> ide;
        Bitfield<18, 16> len;
        Bitfield<19> fz16;
        Bitfield<21, 20> stride;
        Bitfield<23, 22> rMode;
        Bitfield<24> fz;
        Bitfield<25> dn;
        Bitfield<26> ahp;
        Bitfield<27> qc;
        Bitfield<28> v;
        Bitfield<29> c;
        Bitfield<30> z;
        Bitfield<31> n;
    EndBitUnion(FPSCR)

    BitUnion32(FPEXC)
        Bitfield<31> ex;
        Bitfield<30> en;
        Bitfield<29, 0> subArchDefined;
    EndBitUnion(FPEXC)

    BitUnion32(MVFR0)
        Bitfield<3, 0> advSimdRegisters;
        Bitfield<7, 4> singlePrecision;
        Bitfield<11, 8> doublePrecision;
        Bitfield<15, 12> vfpExceptionTrapping;
        Bitfield<19, 16> divide;
        Bitfield<23, 20> squareRoot;
        Bitfield<27, 24> shortVectors;
        Bitfield<31, 28> roundingModes;
    EndBitUnion(MVFR0)

    BitUnion32(MVFR1)
        Bitfield<3, 0> flushToZero;
        Bitfield<7, 4> defaultNaN;
        Bitfield<11, 8> advSimdLoadStore;
        Bitfield<15, 12> advSimdInteger;
        Bitfield<19, 16> advSimdSinglePrecision;
        Bitfield<23, 20> advSimdHalfPrecision;
        Bitfield<27, 24> vfpHalfPrecision;
        Bitfield<31, 28> raz;
    EndBitUnion(MVFR1)

    BitUnion64(TTBCR)
        // Short-descriptor translation table format
        Bitfield<2, 0> n;
        Bitfield<4> pd0;
        Bitfield<5> pd1;
        // Long-descriptor translation table format
        Bitfield<2, 0> t0sz;
        Bitfield<6> t2e;
        Bitfield<7> epd0;
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
        Bitfield<14> tg0;
        Bitfield<18, 16> t1sz;
        Bitfield<22> a1;
        Bitfield<23> epd1;
        Bitfield<25, 24> irgn1;
        Bitfield<27, 26> orgn1;
        Bitfield<29, 28> sh1;
        Bitfield<30> tg1;
        Bitfield<34, 32> ips;
        Bitfield<36> as;
        Bitfield<37> tbi0;
        Bitfield<38> tbi1;
        // Common
        Bitfield<31> eae;
        // TCR_EL2/3 (AArch64)
        Bitfield<18, 16> ps;
        Bitfield<20> tbi;
        Bitfield<41> hpd0;
        Bitfield<42> hpd1;
    EndBitUnion(TTBCR)

    // Fields of TCR_EL{1,2,3} (mostly overlapping)
    // TCR_EL1 is natively 64 bits, the others are 32 bits
    BitUnion64(TCR)
        Bitfield<5, 0> t0sz;
        Bitfield<7> epd0; // EL1
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
        Bitfield<15, 14> tg0;
        Bitfield<18, 16> ps;
        Bitfield<20> tbi; // EL2/EL3
        Bitfield<21, 16> t1sz; // EL1
        Bitfield<22> a1; // EL1
        Bitfield<23> epd1; // EL1
        Bitfield<24> hpd; // EL2/EL3, E2H=0
        Bitfield<25, 24> irgn1; // EL1
        Bitfield<27, 26> orgn1; // EL1
        Bitfield<29, 28> sh1; // EL1
        Bitfield<29> tbid; // EL2
        Bitfield<31, 30> tg1; // EL1
        Bitfield<34, 32> ips; // EL1
        Bitfield<36> as; // EL1
        Bitfield<37> tbi0; // EL1
        Bitfield<38> tbi1; // EL1
        Bitfield<39> ha;
        Bitfield<40> hd;
        Bitfield<41> hpd0;
        Bitfield<42> hpd1;
        Bitfield<51> tbid0; // EL1
        Bitfield<52> tbid1; // EL1
    EndBitUnion(TCR)

    BitUnion32(HTCR)
        Bitfield<2, 0> t0sz;
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
        Bitfield<24> hpd;
    EndBitUnion(HTCR)

    BitUnion32(VTCR_t)
        Bitfield<3, 0> t0sz;
        Bitfield<4> s;
        Bitfield<5, 0> t0sz64;
        Bitfield<7, 6> sl0;
        Bitfield<9, 8> irgn0;
        Bitfield<11, 10> orgn0;
        Bitfield<13, 12> sh0;
        Bitfield<15, 14> tg0;
        Bitfield<18, 16> ps; // Only defined for VTCR_EL2
        Bitfield<19> vs;     // Only defined for VTCR_EL2
        Bitfield<21> ha;     // Only defined for VTCR_EL2
        Bitfield<22> hd;     // Only defined for VTCR_EL2
    EndBitUnion(VTCR_t)

    BitUnion32(PRRR)
       Bitfield<1,0> tr0;
       Bitfield<3,2> tr1;
       Bitfield<5,4> tr2;
       Bitfield<7,6> tr3;
       Bitfield<9,8> tr4;
       Bitfield<11,10> tr5;
       Bitfield<13,12> tr6;
       Bitfield<15,14> tr7;
       Bitfield<16> ds0;
       Bitfield<17> ds1;
       Bitfield<18> ns0;
       Bitfield<19> ns1;
       Bitfield<24> nos0;
       Bitfield<25> nos1;
       Bitfield<26> nos2;
       Bitfield<27> nos3;
       Bitfield<28> nos4;
       Bitfield<29> nos5;
       Bitfield<30> nos6;
       Bitfield<31> nos7;
   EndBitUnion(PRRR)

   BitUnion32(NMRR)
       Bitfield<1,0> ir0;
       Bitfield<3,2> ir1;
       Bitfield<5,4> ir2;
       Bitfield<7,6> ir3;
       Bitfield<9,8> ir4;
       Bitfield<11,10> ir5;
       Bitfield<13,12> ir6;
       Bitfield<15,14> ir7;
       Bitfield<17,16> or0;
       Bitfield<19,18> or1;
       Bitfield<21,20> or2;
       Bitfield<23,22> or3;
       Bitfield<25,24> or4;
       Bitfield<27,26> or5;
       Bitfield<29,28> or6;
       Bitfield<31,30> or7;
   EndBitUnion(NMRR)

   BitUnion32(CONTEXTIDR)
      Bitfield<7,0>  asid;
      Bitfield<31,8> procid;
   EndBitUnion(CONTEXTIDR)

   BitUnion32(L2CTLR)
      Bitfield<2,0>   sataRAMLatency;
      Bitfield<4,3>   reserved_4_3;
      Bitfield<5>     dataRAMSetup;
      Bitfield<8,6>   tagRAMLatency;
      Bitfield<9>     tagRAMSetup;
      Bitfield<11,10> dataRAMSlice;
      Bitfield<12>    tagRAMSlice;
      Bitfield<20,13> reserved_20_13;
      Bitfield<21>    eccandParityEnable;
      Bitfield<22>    reserved_22;
      Bitfield<23>    interptCtrlPresent;
      Bitfield<25,24> numCPUs;
      Bitfield<30,26> reserved_30_26;
      Bitfield<31>    l2rstDISABLE_monitor;
   EndBitUnion(L2CTLR)

   BitUnion32(CTR)
      Bitfield<3,0>   iCacheLineSize;
      Bitfield<13,4>  raz_13_4;
      Bitfield<15,14> l1IndexPolicy;
      Bitfield<19,16> dCacheLineSize;
      Bitfield<23,20> erg;
      Bitfield<27,24> cwg;
      Bitfield<28>    raz_28;
      Bitfield<31,29> format;
   EndBitUnion(CTR)

   BitUnion32(PMSELR)
      Bitfield<4, 0> sel;
   EndBitUnion(PMSELR)

    BitUnion64(PAR)
        // 64-bit format
        Bitfield<63, 56> attr;
        Bitfield<39, 12> pa;
        Bitfield<11>     lpae;
        Bitfield<9>      ns;
        Bitfield<9>      s;
        Bitfield<8, 7>   sh;
        Bitfield<8>      ptw;
        Bitfield<6, 1>   fst;
        Bitfield<6>      fs5;
        Bitfield<5, 1>   fs4_0;
        Bitfield<0>      f;
   EndBitUnion(PAR)

   BitUnion32(ESR)
        Bitfield<31, 26> ec;
        Bitfield<25> il;
        Bitfield<24, 0> iss;

        // Generic Condition ISS
        // Used by exception syndromes holding the condition code of
        // the trapped instruction. TODO: We should really have a
        // different SubBitUnion per exception type and avoid
        // such generic sub-fields
        SubBitUnion(cond_iss, 24, 0)
            Bitfield<24> cv;
            Bitfield<23, 20> cond;
            Bitfield<19, 0> iss;
        EndSubBitUnion(cond_iss)

        // Data Abort ISS
        SubBitUnion(data_abort_iss, 24, 0)
            Bitfield<24> isv;
            Bitfield<23, 22> sas;
            Bitfield<21> sse;
            Bitfield<20, 16> srt;
            Bitfield<15> sf;
            Bitfield<14> ar;
            Bitfield<13> vncr;
            Bitfield<10> fnv;
            Bitfield<9> ea;
            Bitfield<8> cm;
            Bitfield<7> s1ptw;
            Bitfield<6> wnr;
            Bitfield<5, 0> dfsc;
        EndSubBitUnion(data_abort_iss)

        // Instruction Abort ISS
        SubBitUnion(instruction_abort_iss, 24, 0)
            Bitfield<12, 11> set;
            Bitfield<10> fnv;
            Bitfield<9> ea;
            Bitfield<7> s1ptw;
            Bitfield<5, 0> ifsc;
        EndSubBitUnion(instruction_abort_iss)

        // Software Step ISS
        SubBitUnion(software_step_iss, 24, 0)
            Bitfield<24> isv;
            Bitfield<6> ex;
            Bitfield<5, 0> ifsc;
        EndSubBitUnion(software_step_iss)

        // Watchpoint ISS
        SubBitUnion(watchpoint_iss, 24, 0)
            Bitfield<13> vncr;
            Bitfield<8> cm;
            Bitfield<6> wnr;
            Bitfield<5, 0> dfsc;
        EndSubBitUnion(watchpoint_iss)
   EndBitUnion(ESR)

   BitUnion32(CPTR)
        Bitfield<31> tcpac;
        Bitfield<30> tam;
        Bitfield<28> tta_e2h;
        Bitfield<25, 24> smen;
        Bitfield<21, 20> fpen;
        Bitfield<20> tta;
        Bitfield<17, 16> zen;
        Bitfield<13, 13> res1_13_el2;
        Bitfield<12, 12> res1_12_el2;
        Bitfield<12> esm;  // SME (CPTR_EL3)
        Bitfield<12> tsm;  // SME (CPTR_EL2)
        Bitfield<10> tfp;
        Bitfield<9> res1_9_el2;
        Bitfield<8> res1_8_el2;
        Bitfield<8> ez;  // SVE (CPTR_EL3)
        Bitfield<8> tz;  // SVE (CPTR_EL2)
        Bitfield<7, 0> res1_7_0_el2;
   EndBitUnion(CPTR)

    BitUnion64(ZCR)
        Bitfield<3, 0> len;
    EndBitUnion(ZCR)

    BitUnion64(SMCR)
        Bitfield<63, 32> res0_63_32;
        Bitfield<31, 31> fa64;
        Bitfield<30, 9> res0_30_9;
        Bitfield<8, 4> razwi_8_4;
        Bitfield<3, 0> len;
    EndBitUnion(SMCR)

    BitUnion64(SVCR)
        Bitfield<63, 2> res0_63_2;
        Bitfield<1, 1> za;
        Bitfield<0, 0> sm;
    EndBitUnion(SVCR)

    BitUnion64(SMIDR)
        Bitfield<63, 32> res0_63_32;
        Bitfield<31, 24> implementer;
        Bitfield<23, 16> revision;
        Bitfield<15, 15> smps;
        Bitfield<14, 12> res0_14_12;
        Bitfield<11, 0>  affinity;
    EndBitUnion(SMIDR)

    BitUnion64(SMPRI)
        Bitfield<63, 4> res0_63_4;
        Bitfield<3, 0> priority;
    EndBitUnion(SMPRI)

   BitUnion32(OSL)
        Bitfield<64, 4> res0;
        Bitfield<3> oslm_3;
        Bitfield<2> nTT;
        Bitfield<1> oslk;
        Bitfield<0> oslm_0;
   EndBitUnion(OSL)

   BitUnion64(DBGBCR)
        Bitfield<63, 24> res0_2;
        Bitfield<23, 20> bt;
        Bitfield<19, 16> lbn;
        Bitfield<15, 14> ssc;
        Bitfield<13> hmc;
        Bitfield<12, 9> res0_1;
        Bitfield<8, 5> bas;
        Bitfield<4, 3> res0_0;
        Bitfield<2, 1> pmc;
        Bitfield<0> e;
   EndBitUnion(DBGBCR)

    BitUnion64(DBGWCR)
        Bitfield<63, 29> res0_2;
        Bitfield<28, 24> mask;
        Bitfield<23, 21> res0_1;
        Bitfield<20> wt;
        Bitfield<19, 16> lbn;
        Bitfield<15, 14> ssc;
        Bitfield<13> hmc;
        Bitfield<12, 5> bas;
        Bitfield<4, 3> lsv;
        Bitfield<2, 1> pac;
        Bitfield<0> e;
   EndBitUnion(DBGWCR)

   BitUnion32(DBGDS32)
        Bitfield<31> tfo;
        Bitfield<30> rxfull;
        Bitfield<29> txfull;
        Bitfield<28> res0_5;
        Bitfield<27> rxo;
        Bitfield<26> txu;
        Bitfield<25, 24> res0_4;
        Bitfield<23, 22> intdis;
        Bitfield<21> tda;
        Bitfield<20> res0_3;
        Bitfield<19> sc2;
        Bitfield<18> ns;
        Bitfield<17> spniddis;
        Bitfield<16> spiddis;
        Bitfield<15> mdbgen;
        Bitfield<14> hde;
        Bitfield<13> res0_;
        Bitfield<12> udccdis;
        Bitfield<12> tdcc;
        Bitfield<11, 7> res0_2;
        Bitfield<6> err;
        Bitfield<5, 2> moe;
        Bitfield<1, 0> res0_1;
   EndBitUnion(DBGDS32)

   BitUnion32(DBGVCR)
        Bitfield<31> nsf;
        Bitfield<30> nsi;
        Bitfield<29> res0_5;
        Bitfield<28> nsd;
        Bitfield<27> nsp;
        Bitfield<26> nss;
        Bitfield<25> nsu;
        Bitfield<24, 16> res0_4;
        Bitfield<15> mf;
        Bitfield<14> mi;
        Bitfield<13> res0_3;
        Bitfield<12> md;
        Bitfield<11> mp;
        Bitfield<10> ms;
        Bitfield<9,8> res0_2;
        Bitfield<7> sf;
        Bitfield<6> si;
        Bitfield<5> res0_1;
        Bitfield<4> sd;
        Bitfield<3> sp;
        Bitfield<2> ss;
        Bitfield<1> su;
        Bitfield<0> res0_0;
   EndBitUnion(DBGVCR)

   BitUnion32(DEVID)
        Bitfield<31,28> cidmask;
        Bitfield<27,24> auxregs;
        Bitfield<23,20> doublelock;
        Bitfield<19,16> virtextns;
        Bitfield<15,12> vectorcatch;
        Bitfield<11,8>  bpaddremask;
        Bitfield<7,4>   wpaddrmask;
        Bitfield<3,0>   pcsample;
   EndBitUnion(DEVID)

} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_REGS_MISC_TYPES_HH__
