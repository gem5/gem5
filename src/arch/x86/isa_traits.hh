/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#ifndef __ARCH_X86_ISATRAITS_HH__
#define __ARCH_X86_ISATRAITS_HH__

#include "arch/x86/x86_traits.hh"

namespace LittleEndianGuest {}

namespace X86ISA
{
    //This makes sure the little endian version of certain functions
    //are used.
    using namespace LittleEndianGuest;

    // X86 does not have a delay slot
#define ISA_HAS_DELAY_SLOT 0

    // X86 NOP (XCHG rAX, rAX)
    //XXX This needs to be set to an intermediate instruction struct
    //which encodes this instruction

    // These enumerate all the registers for dependence tracking.
    enum DependenceTags {
        //The number of microcode registers needs to be added to this
        FP_Base_DepTag = 16,
        Ctrl_Base_DepTag =
            FP_Base_DepTag +
            //mmx/x87 registers
            8 +
            //xmm registers
            16
    };

    // semantically meaningful register indices
    //There is no such register in X86
    const int ZeroReg = 0;
    const int StackPointerReg = 4; //RSP
    //X86 doesn't seem to have a link register
    const int ReturnAddressReg = 0;
    const int ReturnValueReg = 0; //RAX
    const int FramePointerReg = 5; //RBP
    const int ArgumentReg0 = 7; //RDI
    const int ArgumentReg1 = 6; //RSI
    const int ArgumentReg2 = 2; //RDX
    const int ArgumentReg3 = 1; //RCX
    const int ArgumentReg4 = 8; //R8W
    const int ArgumentReg5 = 9; //R9W

    // Some OS syscalls use a second register (rdx) to return a second
    // value
    const int SyscallPseudoReturnReg = 2; //RDX

    //XXX These numbers are bogus
    const int MaxInstSrcRegs = 10;
    const int MaxInstDestRegs = 10;

    //4k. This value is not constant on x86.
    const int LogVmPageSize = 12;
    const int VMPageSize = (1 << LogVmPageSize);

    const int BranchPredAddrShiftAmt = 0;
};

#endif // __ARCH_X86_ISATRAITS_HH__
