/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
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
 * Authors: Gabe Black
 *          Stephen Hines
 */

#ifndef __ARCH_ARM_ISA_TRAITS_HH__
#define __ARCH_ARM_ISA_TRAITS_HH__

#include "arch/arm/types.hh"
#include "sim/host.hh"

namespace LittleEndianGuest {};

#define TARGET_ARM

class StaticInstPtr;

namespace ArmISA
{
    using namespace LittleEndianGuest;

    StaticInstPtr decodeInst(ExtMachInst);

    // ARM DOES NOT have a delay slot
    #define ISA_HAS_DELAY_SLOT 0

    const Addr PageShift = 12;
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

    // For loading... XXX This maybe could be USegEnd?? --ali
    const Addr LoadAddrMask = ULL(0xffffffffff);

    const unsigned VABits = 32;
    const unsigned PABits = 32; // Is this correct?
    const Addr VAddrImplMask = (ULL(1) << VABits) - 1;
    const Addr VAddrUnImplMask = ~VAddrImplMask;
    inline Addr VAddrImpl(Addr a) { return a & VAddrImplMask; }
    inline Addr VAddrVPN(Addr a) { return a >> ArmISA::PageShift; }
    inline Addr VAddrOffset(Addr a) { return a & ArmISA::PageOffset; }

    const Addr PAddrImplMask = (ULL(1) << PABits) - 1;

    // return a no-op instruction... used for instruction fetch faults
    const ExtMachInst NoopMachInst = 0x00000000;

    // Constants Related to the number of registers
    const int NumIntArchRegs = 16;
    const int NumIntSpecialRegs = 19;
    const int NumFloatArchRegs = 16;
    const int NumFloatSpecialRegs = 5;
    const int NumControlRegs = 7;
    const int NumInternalProcRegs = 0;

    const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;
    const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;
    const int NumMiscRegs = NumControlRegs;

    const int TotalNumRegs = NumIntRegs + NumFloatRegs + NumMiscRegs;

    const int TotalDataRegs = NumIntRegs + NumFloatRegs;

    // Static instruction parameters
    const int MaxInstSrcRegs = 5;
    const int MaxInstDestRegs = 3;

    // semantically meaningful register indices
    const int ReturnValueReg = 0;
    const int ReturnValueReg1 = 1;
    const int ReturnValueReg2 = 2;
    const int ArgumentReg0 = 0;
    const int ArgumentReg1 = 1;
    const int ArgumentReg2 = 2;
    const int ArgumentReg3 = 3;
    const int FramePointerReg = 11;
    const int StackPointerReg = 13;
    const int ReturnAddressReg = 14;
    const int PCReg = 15;

    const int ZeroReg = NumIntArchRegs;
    const int AddrReg = ZeroReg + 1; // Used to generate address for uops

    const int SyscallNumReg = ReturnValueReg;
    const int SyscallPseudoReturnReg = ReturnValueReg;
    const int SyscallSuccessReg = ReturnValueReg;

    const int LogVMPageSize = 12;	// 4K bytes
    const int VMPageSize = (1 << LogVMPageSize);

    const int BranchPredAddrShiftAmt = 2; // instructions are 4-byte aligned

    const int MachineBytes = 4;
    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;

    // These help enumerate all the registers for dependence tracking.
    const int FP_Base_DepTag = NumIntRegs;
    const int Ctrl_Base_DepTag = FP_Base_DepTag + NumFloatRegs;
};

using namespace ArmISA;

#endif // __ARCH_ARM_ISA_TRAITS_HH__
