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
 *
 * Authors: Jaidev Patwardhan
 */

#ifndef __ARCH_MIPS_DT_CONSTANTS_HH__
#define __ARCH_MIPS_DT_CONSTANTS_HH__

#include "arch/mips/types.hh"
#include "base/bitunion.hh"

namespace MipsISA
{

BitUnion32(DebugReg)
    Bitfield<31>     dbd;
    Bitfield<30>     dm;
    Bitfield<29>     nodcr;
    Bitfield<28>     lsnm;
    Bitfield<27>     doze;
    Bitfield<26>     halt;
    Bitfield<25>     conutdm;
    Bitfield<24>     ibusep;
    Bitfield<23>     mcheckep;
    Bitfield<22>     cacheep;
    Bitfield<21>     dbusep;
    Bitfield<20, 19> iexi;
    Bitfield<19>     ddbsImpr;
    Bitfield<18>     ddblImpr;
    SubBitUnion(ejtagVer, 17, 15)
        Bitfield<17> ejtagVer2;
        Bitfield<16> ejtagVer1;
        Bitfield<15> ejtagVer0;
    EndSubBitUnion(ejtagVer)
    Bitfield<14, 10> dexcCode;
    Bitfield<9>      nosst;
    Bitfield<8>      sst;
    Bitfield<7>      offline;
    Bitfield<6>      dibimpr;
    Bitfield<5>      dint;
    Bitfield<4>      dib;
    Bitfield<3>      ddbs;
    Bitfield<2>      ddbl;
    Bitfield<1>      dbp;
    Bitfield<0>      dss;
EndBitUnion(DebugReg)

BitUnion32(TraceControlReg)
    Bitfield<31>     ts;
    Bitfield<30>     ut;
    Bitfield<27>     tb;
    Bitfield<26>     io;
    Bitfield<25>     d;
    Bitfield<24>     e;
    Bitfield<23>     k;
    Bitfield<22>     s;
    Bitfield<21>     u;
    Bitfield<20, 13> asidM;
    Bitfield<12, 5>  asid;
    Bitfield<4>      g;
    Bitfield<3>      tfcr;
    Bitfield<2>      tlsm;
    Bitfield<1>      tim;
    Bitfield<0>      on;
EndBitUnion(TraceControlReg)

BitUnion32(TraceControl2Reg)
    Bitfield<29>     cpuidv;
    Bitfield<28, 21> cpuid;
    Bitfield<20>     tcv;
    Bitfield<19, 12> tcnum;
    Bitfield<11, 7>  mode;
    Bitfield<6,  5>  validModes;
    Bitfield<4>      tbi;
    Bitfield<3>      tbu;
    Bitfield<2,  0>  syp;
EndBitUnion(TraceControl2Reg)

BitUnion32(TraceBPCReg)
    Bitfield<31>     mb;
    Bitfield<28>     e;
    Bitfield<27>     ate;
    Bitfield<26, 24> bpc8;
    Bitfield<23, 21> bpc7;
    Bitfield<20, 18> bpc6;
    Bitfield<17, 15> bpc5;
    Bitfield<14, 12> bpc4;
    Bitfield<11, 9>  bpc3;
    Bitfield<8,  6>  bpc2;
    Bitfield<5,  3>  bpc1;
    Bitfield<2,  0>  bpc0;
EndBitUnion(TraceBPCReg)

BitUnion32(TraceBPC2Reg)
    Bitfield<17, 15> bpc14;
    Bitfield<14, 12> bpc13;
    Bitfield<11, 9>  bpc12;
    Bitfield<8,  6>  bpc11;
    Bitfield<5,  3>  bpc10;
    Bitfield<2,  0>  bpc9;
EndBitUnion(TraceBPC2Reg)

BitUnion32(Debug2Reg)
    Bitfield<3> prm;
    Bitfield<2> dq;
    Bitfield<1> tup;
    Bitfield<0> paco;
EndBitUnion(Debug2Reg)
} // namespace MipsISA

#endif
