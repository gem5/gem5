/*
 * Copyright (c) 2014 ARM Limited
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

#ifndef __ARCH_ARM_REGS_CC_HH__
#define __ARCH_ARM_REGS_CC_HH__

#include "cpu/reg_class.hh"
#include "debug/CCRegs.hh"

namespace gem5
{

namespace ArmISA
{

namespace cc_reg
{

enum : RegIndex
{
    _NzIdx,
    _CIdx,
    _VIdx,
    _GeIdx,
    _FpIdx,
    _ZeroIdx,
    NumRegs
};

const char * const RegName[NumRegs] = {
    "nz",
    "c",
    "v",
    "ge",
    "fp",
    "zero"
};

} // namespace cc_reg

class CCRegClassOps : public RegClassOps
{
  public:
    std::string
    regName(const RegId &id) const override
    {
        return cc_reg::RegName[id.index()];
    }
};

static inline CCRegClassOps ccRegClassOps;

inline constexpr RegClass ccRegClass = RegClass(CCRegClass, CCRegClassName,
        cc_reg::NumRegs, debug::CCRegs).ops(ccRegClassOps);

namespace cc_reg
{

inline constexpr RegId
    Nz = ccRegClass[_NzIdx],
    C = ccRegClass[_CIdx],
    V = ccRegClass[_VIdx],
    Ge = ccRegClass[_GeIdx],
    Fp = ccRegClass[_FpIdx],
    Zero = ccRegClass[_ZeroIdx];

} // namespace cc_reg

enum ConditionCode
{
    COND_EQ  =   0,
    COND_NE, //  1
    COND_CS, //  2
    COND_CC, //  3
    COND_MI, //  4
    COND_PL, //  5
    COND_VS, //  6
    COND_VC, //  7
    COND_HI, //  8
    COND_LS, //  9
    COND_GE, // 10
    COND_LT, // 11
    COND_GT, // 12
    COND_LE, // 13
    COND_AL, // 14
    COND_UC  // 15
};

} // namespace ArmISA
} // namespace gem5

#endif // __ARCH_ARM_REGS_CC_HH__
