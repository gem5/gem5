/*
 * Copyright 2020 Google Inc.
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

#ifndef __ARCH_RISCV_REG_ABI_HH__
#define __ARCH_RISCV_REG_ABI_HH__

#include <vector>

#include "sim/syscall_abi.hh"

namespace gem5
{

namespace RiscvISA
{

//FIXME RISCV needs to handle 64 bit arguments in its 32 bit ISA.
struct RegABI64 : public GenericSyscallABI64
{
    static const std::vector<RegId> ArgumentRegs;
};

struct RegABI32 : public GenericSyscallABI32
{
    static const std::vector<RegId> ArgumentRegs;
};

} // namespace RiscvISA

namespace guest_abi
{

using namespace pseudo_inst;

// This method will be used if the size of argument type of function is
// greater than 4 byte for Riscv 32.
template <typename ABI, typename Arg>
struct Argument<ABI, Arg,
    typename std::enable_if_t<
        std::is_base_of_v<RiscvISA::RegABI32, ABI> &&
        std::is_integral_v<Arg> &&
        ABI::template IsWideV<Arg>>>
{
    static Arg
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state >= ABI::ArgumentRegs.size(),
                "Ran out of syscall argument registers.");

        auto low = ABI::ArgumentRegs[state++];
        auto high = ABI::ArgumentRegs[state++];
        return (Arg)ABI::mergeRegs(tc, low, high);
    }
};

// This method will be used for RV32 pointers.
template <>
struct Argument<RiscvISA::RegABI32, GuestAddr>
{
    using ABI = RiscvISA::RegABI32;

    static GuestAddr
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state >= ABI::ArgumentRegs.size(),
                "Ran out of syscall argument registers.");

        auto arg = bits(tc->getReg(ABI::ArgumentRegs[state++]), 31, 0);
        return *reinterpret_cast<GuestAddr*>(&arg);
    }
};

}

} // namespace gem5

#endif // __ARCH_RISCV_REG_ABI_HH__
