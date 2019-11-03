/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *          Gabe Black
 */

#ifndef __ARCH_ALPHA_ISA_TRAITS_HH__
#define __ARCH_ALPHA_ISA_TRAITS_HH__

namespace LittleEndianGuest {}

#include "arch/alpha/ipr.hh"
#include "arch/alpha/types.hh"
#include "base/types.hh"
#include "cpu/static_inst_fwd.hh"

namespace AlphaISA {

using namespace LittleEndianGuest;

StaticInstPtr decodeInst(ExtMachInst);

const Addr PageShift = 13;
const Addr PageBytes = ULL(1) << PageShift;
const Addr PageMask = ~(PageBytes - 1);
const Addr PageOffset = PageBytes - 1;

////////////////////////////////////////////////////////////////////////
//
//  Translation stuff
//

const Addr PteShift = 3;
const Addr NPtePageShift = PageShift - PteShift;
const Addr NPtePage = ULL(1) << NPtePageShift;
const Addr PteMask = NPtePage - 1;

// User Virtual
const Addr USegBase = ULL(0x0);
const Addr USegEnd = ULL(0x000003ffffffffff);

// Kernel Direct Mapped
const Addr K0SegBase = ULL(0xfffffc0000000000);
const Addr K0SegEnd = ULL(0xfffffdffffffffff);

// Kernel Virtual
const Addr K1SegBase = ULL(0xfffffe0000000000);
const Addr K1SegEnd = ULL(0xffffffffffffffff);

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

// EV5 modes
enum mode_type
{
    mode_kernel = 0,        // kernel
    mode_executive = 1,     // executive (unused by unix)
    mode_supervisor = 2,    // supervisor (unused by unix)
    mode_user = 3,          // user mode
    mode_number             // number of modes
};

const int MachineBytes = 8;

// Memory accesses cannot be unaligned
const bool HasUnalignedMemAcc = false;

const bool CurThreadInfoImplemented = true;
const int CurThreadInfoReg = AlphaISA::IPR_PALtemp23;

} // namespace AlphaISA

#endif // __ARCH_ALPHA_ISA_TRAITS_HH__
