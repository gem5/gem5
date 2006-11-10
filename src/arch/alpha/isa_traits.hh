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
#include "config/full_system.hh"
#include "sim/host.hh"

class StaticInstPtr;

namespace AlphaISA
{
    using namespace LittleEndianGuest;

    // These enumerate all the registers for dependence tracking.
    enum DependenceTags {
        // 0..31 are the integer regs 0..31
        // 32..63 are the FP regs 0..31, i.e. use (reg + FP_Base_DepTag)
        FP_Base_DepTag = 40,
        Ctrl_Base_DepTag = 72
    };

    StaticInstPtr decodeInst(ExtMachInst);

    // Alpha Does NOT have a delay slot
    #define ISA_HAS_DELAY_SLOT 0

    const Addr PageShift = 13;
    const Addr PageBytes = ULL(1) << PageShift;
    const Addr PageMask = ~(PageBytes - 1);
    const Addr PageOffset = PageBytes - 1;

#if FULL_SYSTEM

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

    // For loading... XXX This maybe could be USegEnd?? --ali
    const Addr LoadAddrMask = ULL(0xffffffffff);

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
        mode_kernel = 0,		// kernel
        mode_executive = 1,		// executive (unused by unix)
        mode_supervisor = 2,	// supervisor (unused by unix)
        mode_user = 3,		// user mode
        mode_number			// number of modes
    };

#endif

    // Constants Related to the number of registers

    const int NumIntArchRegs = 32;
    const int NumPALShadowRegs = 8;
    const int NumFloatArchRegs = 32;
    // @todo: Figure out what this number really should be.
    const int NumMiscArchRegs = 32;

    const int NumIntRegs = NumIntArchRegs + NumPALShadowRegs;
    const int NumFloatRegs = NumFloatArchRegs;
    const int NumMiscRegs = NumMiscArchRegs;

    const int TotalNumRegs = NumIntRegs + NumFloatRegs +
        NumMiscRegs + NumInternalProcRegs;

    const int TotalDataRegs = NumIntRegs + NumFloatRegs;

    // Static instruction parameters
    const int MaxInstSrcRegs = 3;
    const int MaxInstDestRegs = 2;

    // semantically meaningful register indices
    const int ZeroReg = 31;	// architecturally meaningful
    // the rest of these depend on the ABI
    const int StackPointerReg = 30;
    const int GlobalPointerReg = 29;
    const int ProcedureValueReg = 27;
    const int ReturnAddressReg = 26;
    const int ReturnValueReg = 0;
    const int FramePointerReg = 15;
    const int ArgumentReg0 = 16;
    const int ArgumentReg1 = 17;
    const int ArgumentReg2 = 18;
    const int ArgumentReg3 = 19;
    const int ArgumentReg4 = 20;
    const int ArgumentReg5 = 21;
    const int SyscallNumReg = ReturnValueReg;
    const int SyscallPseudoReturnReg = ArgumentReg4;
    const int SyscallSuccessReg = 19;

    const int LogVMPageSize = 13;	// 8K bytes
    const int VMPageSize = (1 << LogVMPageSize);

    const int BranchPredAddrShiftAmt = 2; // instructions are 4-byte aligned

    const int MachineBytes = 8;
    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;

    // return a no-op instruction... used for instruction fetch faults
    // Alpha UNOP (ldq_u r31,0(r0))
    const ExtMachInst NoopMachInst = 0x2ffe0000;

};

#endif // __ARCH_ALPHA_ISA_TRAITS_HH__
