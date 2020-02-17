/*
 * Copyright (c) 2009 The University of Edinburgh
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

#ifndef __ARCH_POWER_MISCREGS_HH__
#define __ARCH_POWER_MISCREGS_HH__

#include "base/bitunion.hh"

namespace PowerISA
{

enum MiscRegIndex {
    NUM_MISCREGS = 0
};

const char * const miscRegName[NUM_MISCREGS] = {
};

BitUnion32(Cr)
    SubBitUnion(cr0, 31, 28)
        Bitfield<31> lt;
        Bitfield<30> gt;
        Bitfield<29> eq;
        Bitfield<28> so;
    EndSubBitUnion(cr0)
    Bitfield<27,24> cr1;
EndBitUnion(Cr)

BitUnion32(Xer)
    Bitfield<31> so;
    Bitfield<30> ov;
    Bitfield<29> ca;
EndBitUnion(Xer)

BitUnion32(Fpscr)
    Bitfield<31> fx;
    Bitfield<30> fex;
    Bitfield<29> vx;
    Bitfield<28> ox;
    Bitfield<27> ux;
    Bitfield<26> zx;
    Bitfield<25> xx;
    Bitfield<24> vxsnan;
    Bitfield<23> vxisi;
    Bitfield<22> vxidi;
    Bitfield<21> vxzdz;
    Bitfield<20> vximz;
    Bitfield<19> vxvc;
    Bitfield<18> fr;
    Bitfield<17> fi;
    SubBitUnion(fprf, 16, 12)
        Bitfield<16> c;
        SubBitUnion(fpcc, 15, 12)
            Bitfield<15> fl;
            Bitfield<14> fg;
            Bitfield<13> fe;
            Bitfield<12> fu;
        EndSubBitUnion(fpcc)
    EndSubBitUnion(fprf)
    Bitfield<10> vxsqrt;
    Bitfield<9> vxcvi;
    Bitfield<8> ve;
    Bitfield<7> oe;
    Bitfield<6> ue;
    Bitfield<5> ze;
    Bitfield<4> xe;
    Bitfield<3> ni;
    Bitfield<2,1> rn;
EndBitUnion(Fpscr)

} // namespace PowerISA

#endif // __ARCH_POWER_MISCREGS_HH__
