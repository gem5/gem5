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

#ifndef __ARCH_MIPS_MT_CONSTANTS_HH__
#define __ARCH_MIPS_MT_CONSTANTS_HH__

#include "arch/mips/types.hh"
#include "base/bitunion.hh"

namespace gem5
{

namespace MipsISA
{

BitUnion32(MVPControlReg)
    Bitfield<3> cpa;
    Bitfield<2> stlb;
    Bitfield<1> vpc;
    Bitfield<0> evp;
EndBitUnion(MVPControlReg)

BitUnion32(MVPConf0Reg)
    Bitfield<31> m;
    Bitfield<29> tlbs;
    Bitfield<28> gs;
    Bitfield<27> pcp;
    Bitfield<25, 16> ptlbe;
    Bitfield<15> tca;
    Bitfield<13, 10> pvpe;
    Bitfield<7, 0> ptc;
EndBitUnion(MVPConf0Reg)

BitUnion32(VPEControlReg)
    Bitfield<21> ysi;
    Bitfield<18, 16> excpt;
    Bitfield<15> te;
    Bitfield<7, 0> targTC;
EndBitUnion(VPEControlReg)

BitUnion32(VPEConf0Reg)
    Bitfield<31> m;
    Bitfield<28, 21> xtc;
    Bitfield<19> tcs;
    Bitfield<18> scs;
    Bitfield<17> dcs;
    Bitfield<16> ics;
    Bitfield<1> mvp;
    Bitfield<0> vpa;
EndBitUnion(VPEConf0Reg)

BitUnion32(TCBindReg)
    Bitfield<28, 21> curTC;
    Bitfield<20, 18> a0;
    Bitfield<17> tbe;
    Bitfield<3, 0> curVPE;
EndBitUnion(TCBindReg)

BitUnion32(TCStatusReg)
    Bitfield<31, 28> tcu;
    Bitfield<27> tmx;
    Bitfield<24, 23> rnst;
    Bitfield<21> tds;
    Bitfield<20> dt;
    Bitfield<19, 16> impl;
    Bitfield<15> da;
    Bitfield<13> a;
    Bitfield<12, 11> tksu;
    Bitfield<10> ixmt;
    Bitfield<7, 0> asid;
EndBitUnion(TCStatusReg)

BitUnion32(TCHaltReg)
    Bitfield<0> h;
EndBitUnion(TCHaltReg)

} // namespace MipsISA
} // namespace gem5

#endif
