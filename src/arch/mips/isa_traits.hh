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
 * Authors: Gabe Black
 *          Korey Sewell
 */

#ifndef __ARCH_MIPS_ISA_TRAITS_HH__
#define __ARCH_MIPS_ISA_TRAITS_HH__

#include "arch/mips/types.hh"
#include "sim/host.hh"

namespace LittleEndianGuest {};

#define TARGET_MIPS

class StaticInstPtr;

namespace MipsISA
{
    using namespace LittleEndianGuest;

    StaticInstPtr decodeInst(ExtMachInst);

    // MIPS DOES a delay slot
    #define ISA_HAS_DELAY_SLOT 1

    const Addr PageShift = 13;
    const Addr PageBytes = ULL(1) << PageShift;
    const Addr PageMask = ~(PageBytes - 1);
    const Addr PageOffset = PageBytes - 1;

    // return a no-op instruction... used for instruction fetch faults
    const ExtMachInst NoopMachInst = 0x00000000;

    // Constants Related to the number of registers
    const int NumIntArchRegs = 32;
    const int NumIntSpecialRegs = 2;
    const int NumFloatArchRegs = 32;
    const int NumFloatSpecialRegs = 5;
    const int NumControlRegs = 265;
    const int NumInternalProcRegs = 0;

    const int NumIntRegs = NumIntArchRegs + NumIntSpecialRegs;        //HI & LO Regs
    const int NumFloatRegs = NumFloatArchRegs + NumFloatSpecialRegs;//
    const int NumMiscRegs = NumControlRegs;

    const int TotalNumRegs = NumIntRegs + NumFloatRegs +
    NumMiscRegs + 0/*NumInternalProcRegs*/;

    const int TotalDataRegs = NumIntRegs + NumFloatRegs;

    // Static instruction parameters
    const int MaxInstSrcRegs = 3;
    const int MaxInstDestRegs = 2;

    // semantically meaningful register indices
    const int ZeroReg = 0;
    const int AssemblerReg = 1;
    const int ReturnValueReg = 2;
    const int ReturnValueReg1 = 2;
    const int ReturnValueReg2 = 3;
    const int ArgumentReg0 = 4;
    const int ArgumentReg1 = 5;
    const int ArgumentReg2 = 6;
    const int ArgumentReg3 = 7;
    const int KernelReg0 = 26;
    const int KernelReg1 = 27;
    const int GlobalPointerReg = 28;
    const int StackPointerReg = 29;
    const int FramePointerReg = 30;
    const int ReturnAddressReg = 31;

    const int SyscallNumReg = ReturnValueReg1;
    const int SyscallPseudoReturnReg = ReturnValueReg1;
    const int SyscallSuccessReg = ArgumentReg3;

    const int LogVMPageSize = 13;	// 8K bytes
    const int VMPageSize = (1 << LogVMPageSize);

    const int BranchPredAddrShiftAmt = 2; // instructions are 4-byte aligned

    const int MachineBytes = 4;
    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;

    // These help enumerate all the registers for dependence tracking.
    const int FP_Base_DepTag = 34;
    const int Ctrl_Base_DepTag = 257;

    const int ANNOTE_NONE = 0;
    const uint32_t ITOUCH_ANNOTE = 0xffffffff;

};

using namespace MipsISA;

#endif // __ARCH_MIPS_ISA_TRAITS_HH__
