/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#ifndef __ARCH_MIPS_ISA_TRAITS_HH__
#define __ARCH_MIPS_ISA_TRAITS_HH__

#include "arch/mips/types.hh"
#include "base/types.hh"
#include "cpu/static_inst_fwd.hh"

namespace MipsISA
{

const ByteOrder GuestByteOrder = LittleEndianByteOrder;

StaticInstPtr decodeInst(ExtMachInst);

const Addr PageShift = 13;
const Addr PageBytes = ULL(1) << PageShift;
const Addr Page_Mask = ~(PageBytes - 1);
const Addr PageOffset = PageBytes - 1;


////////////////////////////////////////////////////////////////////////
//
//  Translation stuff
//

const Addr PteShift = 3;
const Addr NPtePageShift = PageShift - PteShift;
const Addr NPtePage = ULL(1) << NPtePageShift;
const Addr PteMask = NPtePage - 1;

//// All 'Mapped' segments go through the TLB
//// All other segments are translated by dropping the MSB, to give
//// the corresponding physical address
// User Segment - Mapped
const Addr USegBase = ULL(0x0);
const Addr USegEnd = ULL(0x7FFFFFFF);

// Kernel Segment 0 - Unmapped
const Addr KSeg0End = ULL(0x9FFFFFFF);
const Addr KSeg0Base =  ULL(0x80000000);
const Addr KSeg0Mask = ULL(0x1FFFFFFF);

// Kernel Segment 1 - Unmapped, Uncached
const Addr KSeg1End = ULL(0xBFFFFFFF);
const Addr KSeg1Base = ULL(0xA0000000);
const Addr KSeg1Mask = ULL(0x1FFFFFFF);

// Kernel/Supervisor Segment - Mapped
const Addr KSSegEnd = ULL(0xDFFFFFFF);
const Addr KSSegBase = ULL(0xC0000000);

// Kernel Segment 3 - Mapped
const Addr KSeg3End = ULL(0xFFFFFFFF);
const Addr KSeg3Base = ULL(0xE0000000);


inline Addr Phys2K0Seg(Addr addr)
{
    return addr | KSeg0Base;
}


const unsigned VABits = 32;
const unsigned PABits = 32; // Is this correct?
const Addr VAddrImplMask = (ULL(1) << VABits) - 1;
const Addr VAddrUnImplMask = ~VAddrImplMask;
inline Addr VAddrImpl(Addr a) { return a & VAddrImplMask; }
inline Addr VAddrVPN(Addr a) { return a >> MipsISA::PageShift; }
inline Addr VAddrOffset(Addr a) { return a & MipsISA::PageOffset; }

const Addr PAddrImplMask = (ULL(1) << PABits) - 1;

////////////////////////////////////////////////////////////////////////
//
//  Interrupt levels
//
enum InterruptLevels
{
    INTLEVEL_SOFTWARE_MIN = 4,
    INTLEVEL_SOFTWARE_MAX = 19,

    INTLEVEL_EXTERNAL_MIN = 20,
    INTLEVEL_EXTERNAL_MAX = 34,

    INTLEVEL_IRQ0 = 20,
    INTLEVEL_IRQ1 = 21,
    INTINDEX_ETHERNET = 0,
    INTINDEX_SCSI = 1,
    INTLEVEL_IRQ2 = 22,
    INTLEVEL_IRQ3 = 23,

    INTLEVEL_SERIAL = 33,

    NumInterruptLevels = INTLEVEL_EXTERNAL_MAX
};

// MIPS modes
enum mode_type
{
    mode_kernel = 0,        // kernel
    mode_supervisor = 1,    // supervisor
    mode_user = 2,          // user mode
    mode_debug = 3,         // debug mode
    mode_number             // number of modes
};

const int ANNOTE_NONE = 0;
const uint32_t ITOUCH_ANNOTE = 0xffffffff;

const bool HasUnalignedMemAcc = true;

const bool CurThreadInfoImplemented = false;
const int CurThreadInfoReg = -1;

} // namespace MipsISA

#endif // __ARCH_MIPS_ISA_TRAITS_HH__
