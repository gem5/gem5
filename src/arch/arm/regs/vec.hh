/*
 * Copyright (c) 2010-2011, 2014, 2016-2019 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
 */

#ifndef __ARCH_ARM_REGS_VEC_HH__
#define __ARCH_ARM_REGS_VEC_HH__

#include "arch/arm/types.hh"
#include "arch/generic/vec_pred_reg.hh"
#include "arch/generic/vec_reg.hh"
#include "cpu/reg_class.hh"
#include "debug/VecPredRegs.hh"
#include "debug/VecRegs.hh"

namespace gem5
{

namespace ArmISA
{

// Number of VecElem per Vector Register considering only pre-SVE
// Advanced SIMD registers.
constexpr unsigned NumVecElemPerNeonVecReg = 4;
// Number of VecElem per Vector Register, computed based on the vector length
constexpr unsigned NumVecElemPerVecReg = MaxSveVecLenInWords;

using VecElem = uint32_t;
using VecRegContainer =
    gem5::VecRegContainer<NumVecElemPerVecReg * sizeof(VecElem)>;

using VecPredReg =
    gem5::VecPredRegT<VecElem, NumVecElemPerVecReg, false, false>;
using ConstVecPredReg =
    gem5::VecPredRegT<VecElem, NumVecElemPerVecReg, false, true>;
using VecPredRegContainer = VecPredReg::Container;

// Vec, PredVec
// NumFloatV7ArchRegs: This in theory should be 32.
// However in A32 gem5 is splitting double register accesses in two
// subsequent single register ones. This means we would use a index
// bigger than 31 when accessing D16-D31.
const int NumFloatV7ArchRegs = 64; // S0-S31, D0-D31
const int NumVecV7ArchRegs  = 16; // Q0-Q15
const int NumVecV8ArchRegs  = 32; // V0-V31
const int NumVecSpecialRegs = 8;
const int NumVecIntrlvRegs = 4;
const int NumVecRegs = NumVecV8ArchRegs + NumVecSpecialRegs + NumVecIntrlvRegs;
const int NumVecPredRegs = 18;  // P0-P15, FFR, UREG0

// Vec, PredVec indices
const int VecSpecialElem = NumVecV8ArchRegs * NumVecElemPerNeonVecReg;
const int INTRLVREG0 = NumVecV8ArchRegs + NumVecSpecialRegs;
const int INTRLVREG1 = INTRLVREG0 + 1;
const int INTRLVREG2 = INTRLVREG0 + 2;
const int INTRLVREG3 = INTRLVREG0 + 3;
const int VECREG_UREG0 = 32;
const int PREDREG_FFR = 16;
const int PREDREG_UREG0 = 17;

static inline VecElemRegClassOps<ArmISA::VecElem>
    vecRegElemClassOps(NumVecElemPerVecReg);
static inline TypedRegClassOps<ArmISA::VecRegContainer> vecRegClassOps;
static inline TypedRegClassOps<ArmISA::VecPredRegContainer> vecPredRegClassOps;

inline constexpr RegClass vecRegClass =
    RegClass(VecRegClass, VecRegClassName, NumVecRegs, debug::VecRegs).
        ops(vecRegClassOps).
        regType<VecRegContainer>();
inline constexpr RegClass vecElemClass =
    RegClass(VecElemClass, VecElemClassName, NumVecRegs * NumVecElemPerVecReg,
            debug::VecRegs).
        ops(vecRegElemClassOps);
inline constexpr RegClass vecPredRegClass =
    RegClass(VecPredRegClass, VecPredRegClassName, NumVecPredRegs,
            debug::VecPredRegs).
        ops(vecPredRegClassOps).
        regType<VecPredRegContainer>();

} // namespace ArmISA
} // namespace gem5

#endif
