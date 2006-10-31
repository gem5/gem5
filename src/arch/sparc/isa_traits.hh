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
 */

#ifndef __ARCH_SPARC_ISA_TRAITS_HH__
#define __ARCH_SPARC_ISA_TRAITS_HH__

#include "arch/sparc/types.hh"
#include "base/misc.hh"
#include "config/full_system.hh"
#include "sim/host.hh"

class ThreadContext;
class FastCPU;
//class FullCPU;
class Checkpoint;

class StaticInst;
class StaticInstPtr;

namespace BigEndianGuest {}

#if FULL_SYSTEM
#include "arch/sparc/isa_fullsys_traits.hh"
#endif

namespace SparcISA
{
    class RegFile;

    //This makes sure the big endian versions of certain functions are used.
    using namespace BigEndianGuest;

    // SPARC has a delay slot
    #define ISA_HAS_DELAY_SLOT 1

    // SPARC NOP (sethi %(hi(0), g0)
    const MachInst NoopMachInst = 0x01000000;

    const int NumRegularIntRegs = 32;
    const int NumMicroIntRegs = 1;
    const int NumIntRegs =
        NumRegularIntRegs +
        NumMicroIntRegs;
    const int NumFloatRegs = 64;
    const int NumMiscRegs = 40;

    // These enumerate all the registers for dependence tracking.
    enum DependenceTags {
        // 0..31 are the integer regs 0..31
        // 32..95 are the FP regs 0..31, i.e. use (reg + FP_Base_DepTag)
        FP_Base_DepTag = NumIntRegs,
        Ctrl_Base_DepTag = NumIntRegs + NumFloatRegs,
        //XXX These are here solely to get compilation and won't work
        Fpcr_DepTag = 0,
        Uniq_DepTag = 0
    };


    // MAXTL - maximum trap level
    const int MaxPTL = 2;
    const int MaxTL  = 6;
    const int MaxGL  = 3;
    const int MaxPGL = 2;

    // NWINDOWS - number of register windows, can be 3 to 32
    const int NWindows = 8;

    // semantically meaningful register indices
    const int ZeroReg = 0;	// architecturally meaningful
    // the rest of these depend on the ABI
    const int StackPointerReg = 14;
    const int ReturnAddressReg = 31; // post call, precall is 15
    const int ReturnValueReg = 8; // Post return, 24 is pre-return.
    const int FramePointerReg = 30;
    const int ArgumentReg0 = 8;
    const int ArgumentReg1 = 9;
    const int ArgumentReg2 = 10;
    const int ArgumentReg3 = 11;
    const int ArgumentReg4 = 12;
    const int ArgumentReg5 = 13;
    // Some OS syscall use a second register (o1) to return a second value
    const int SyscallPseudoReturnReg = ArgumentReg1;

    //XXX These numbers are bogus
    const int MaxInstSrcRegs = 8;
    const int MaxInstDestRegs = 9;

    //8K. This value is implmentation specific; and should probably
    //be somewhere else.
    const int LogVMPageSize = 13;
    const int VMPageSize = (1 << LogVMPageSize);

    //Why does both the previous set of constants and this one exist?
    const int PageShift = 13;
    const int PageBytes = ULL(1) << PageShift;

    const int BranchPredAddrShiftAmt = 2;

    const int MachineBytes = 8;
    const int WordBytes = 4;
    const int HalfwordBytes = 2;
    const int ByteBytes = 1;

    void serialize(std::ostream & os);

    void unserialize(Checkpoint *cp, const std::string &section);

    StaticInstPtr decodeInst(ExtMachInst);

    // return a no-op instruction... used for instruction fetch faults
    extern const MachInst NoopMachInst;
}

#endif // __ARCH_SPARC_ISA_TRAITS_HH__
