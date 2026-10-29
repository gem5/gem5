/*
 * Copyright (c) 2026 Mao Weiming
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

#ifndef __ARCH_RISCV_REGS_RENAMEABLE_MISC_HH__
#define __ARCH_RISCV_REGS_RENAMEABLE_MISC_HH__

#include "cpu/reg_class.hh"
#include "debug/IntRegs.hh"

namespace gem5
{

namespace RiscvISA
{

namespace rmisc_reg
{

enum : RegIndex
{
    _VlIdx,
    _VtypeIdx,
    NumRegs
};

} // namespace rmisc_reg

inline constexpr RegClass rmiscRegClass(RMiscRegClass, RMiscRegClassName,
                                        rmisc_reg::NumRegs, debug::IntRegs);

inline constexpr RegId VecRenamedVLReg(rmiscRegClass, rmisc_reg::_VlIdx);
inline constexpr RegId VecRenamedVTYPEReg(rmiscRegClass, rmisc_reg::_VtypeIdx);

// vtype.vill at reset.
inline constexpr RegVal VtypeReset = 1ULL << 63;

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_REGS_RENAMEABLE_MISC_HH__
