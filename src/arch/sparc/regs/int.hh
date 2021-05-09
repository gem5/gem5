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
 */

#ifndef __ARCH_SPARC_REGS_INT_HH__
#define __ARCH_SPARC_REGS_INT_HH__

#include "arch/sparc/sparc_traits.hh"

namespace gem5
{

namespace SparcISA
{

// semantically meaningful register indices
enum {
    // Globals
    INTREG_G0, INTREG_G1, INTREG_G2, INTREG_G3,
    INTREG_G4, INTREG_G5, INTREG_G6, INTREG_G7,
    // Outputs
    INTREG_O0, INTREG_O1, INTREG_O2, INTREG_O3,
    INTREG_O4, INTREG_O5, INTREG_O6, INTREG_O7,
    // Locals
    INTREG_L0, INTREG_L1, INTREG_L2, INTREG_L3,
    INTREG_L4, INTREG_L5, INTREG_L6, INTREG_L7,
    // Inputs
    INTREG_I0, INTREG_I1, INTREG_I2, INTREG_I3,
    INTREG_I4, INTREG_I5, INTREG_I6, INTREG_I7,

    NumIntArchRegs,

    INTREG_UREG0 = NumIntArchRegs,
    INTREG_Y,
    INTREG_CCR,
    INTREG_CANSAVE,
    INTREG_CANRESTORE,
    INTREG_CLEANWIN,
    INTREG_OTHERWIN,
    INTREG_WSTATE,
    INTREG_GSR,

    NumMicroIntRegs = INTREG_GSR - INTREG_UREG0 + 1
};

// the rest of these depend on the ABI
const int ReturnAddressReg = INTREG_I7; // post call, precall is 15
const int ReturnValueReg = INTREG_O0;  // Post return, 24 is pre-return.
const int StackPointerReg = INTREG_O6;
const int FramePointerReg = INTREG_I6;

// Some OS syscall use a second register to return a second value
const int SyscallPseudoReturnReg = INTREG_O1;

const int NumIntRegs = (MaxGL + 1) * 8 + NWindows * 16 + NumMicroIntRegs;

} // namespace SparcISA
} // namespace gem5

#endif
