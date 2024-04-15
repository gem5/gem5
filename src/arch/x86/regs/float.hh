/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
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

#ifndef __ARCH_X86_REGS_FLOAT_HH__
#define __ARCH_X86_REGS_FLOAT_HH__

#include "arch/x86/x86_traits.hh"
#include "base/bitunion.hh"
#include "cpu/reg_class.hh"
#include "debug/FloatRegs.hh"

namespace gem5
{
namespace X86ISA
{
namespace float_reg
{
enum FloatRegIndex
{
    // MMX/X87 registers
    MmxBase,
    FprBase = MmxBase,
    _Mmx0Idx = MmxBase,
    _Mmx1Idx,
    _Mmx2Idx,
    _Mmx3Idx,
    _Mmx4Idx,
    _Mmx5Idx,
    _Mmx6Idx,
    _Mmx7Idx,

    _Fpr0Idx = FprBase,
    _Fpr1Idx,
    _Fpr2Idx,
    _Fpr3Idx,
    _Fpr4Idx,
    _Fpr5Idx,
    _Fpr6Idx,
    _Fpr7Idx,

    XmmBase = MmxBase + NumMMXRegs,
    _Xmm0LowIdx = XmmBase,
    _Xmm0HighIdx,
    _Xmm1LowIdx,
    _Xmm1HighIdx,
    _Xmm2LowIdx,
    _Xmm2HighIdx,
    _Xmm3LowIdx,
    _Xmm3HighIdx,
    _Xmm4LowIdx,
    _Xmm4HighIdx,
    _Xmm5LowIdx,
    _Xmm5HighIdx,
    _Xmm6LowIdx,
    _Xmm6HighIdx,
    _Xmm7LowIdx,
    _Xmm7HighIdx,
    _Xmm8LowIdx,
    _Xmm8HighIdx,
    _Xmm9LowIdx,
    _Xmm9HighIdx,
    _Xmm10LowIdx,
    _Xmm10HighIdx,
    _Xmm11LowIdx,
    _Xmm11HighIdx,
    _Xmm12LowIdx,
    _Xmm12HighIdx,
    _Xmm13LowIdx,
    _Xmm13HighIdx,
    _Xmm14LowIdx,
    _Xmm14HighIdx,
    _Xmm15LowIdx,
    _Xmm15HighIdx,

    MicrofpBase = XmmBase + 2 * NumXMMRegs,
    _Microfp0Idx = MicrofpBase,
    _Microfp1Idx,
    _Microfp2Idx,
    _Microfp3Idx,
    _Microfp4Idx,
    _Microfp5Idx,
    _Microfp6Idx,
    _Microfp7Idx,

    NumRegs = MicrofpBase + NumMicroFpRegs
};

} // namespace float_reg

class FlatFloatRegClassOps : public RegClassOps
{
    std::string regName(const RegId &id) const override;
};

inline constexpr FlatFloatRegClassOps flatFloatRegClassOps;

inline constexpr RegClass flatFloatRegClass =
    RegClass(FloatRegClass, FloatRegClassName, float_reg::NumRegs,
             debug::FloatRegs)
        .ops(flatFloatRegClassOps);

class FloatRegClassOps : public FlatFloatRegClassOps
{
    RegId flatten(const BaseISA &isa, const RegId &id) const override;
};

inline constexpr FloatRegClassOps floatRegClassOps;

inline constexpr RegClass floatRegClass =
    RegClass(FloatRegClass, FloatRegClassName, float_reg::NumRegs,
             debug::FloatRegs)
        .ops(floatRegClassOps)
        .needsFlattening();

namespace float_reg
{

static inline RegId
mmx(int index)
{
    return floatRegClass[MmxBase + index];
}

static inline RegId
fpr(int index)
{
    return floatRegClass[FprBase + index];
}

static inline RegId
xmm(int index)
{
    return floatRegClass[XmmBase + index];
}

static inline RegId
xmmLow(int index)
{
    return floatRegClass[XmmBase + 2 * index];
}

static inline RegId
xmmHigh(int index)
{
    return floatRegClass[XmmBase + 2 * index + 1];
}

static inline RegId
microfp(int index)
{
    return floatRegClass[MicrofpBase + index];
}

static inline RegId
stack(int index, int top)
{
    return fpr((top + index + 8) % 8);
}

} // namespace float_reg

} // namespace X86ISA
} // namespace gem5

#endif // __ARCH_X86_REGS_FLOAT_HH__
