/*
 * Copyright (c) 2020 The Regents of the University of California.
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

#include "arch/x86/regs/int.hh"
#include "sim/guest_abi.hh"
#include "sim/pseudo_inst.hh"

namespace gem5
{

struct X86PseudoInstABI
{
    using State = int;
};

namespace guest_abi
{

using namespace pseudo_inst;

template <typename T>
struct Result<X86PseudoInstABI, T>
{
    static void
    store(ThreadContext *tc, const T &ret)
    {
        // This assumes that all pseudo ops have their return value set
        // by the pseudo op instruction. This may need to be revisited if we
        // modify the pseudo op ABI in util/m5/m5op_x86.S
        tc->setReg(X86ISA::int_reg::Rax, ret);
    }
};

template <>
struct Argument<X86PseudoInstABI, uint64_t>
{
    static uint64_t
    get(ThreadContext *tc, X86PseudoInstABI::State &state)
    {
        // The first 6 integer arguments are passed in registers, the rest
        // are passed on the stack.

        panic_if(state >= 6, "Too many psuedo inst arguments.");

        using namespace X86ISA;

        constexpr RegId int_reg_map[] = {
            int_reg::Rdi, int_reg::Rsi, int_reg::Rdx,
            int_reg::Rcx, int_reg::R8, int_reg::R9
        };

        return tc->getReg(int_reg_map[state++]);
    }
};

template <>
struct Argument<X86PseudoInstABI, GuestAddr>
{
    static GuestAddr
    get(ThreadContext *tc, X86PseudoInstABI::State &state)
    {
        // The first 6 integer arguments are passed in registers, the rest
        // are passed on the stack.

        panic_if(state >= 6, "Too many psuedo inst arguments.");

        using namespace X86ISA;

        constexpr RegId int_reg_map[] = {
            int_reg::Rdi, int_reg::Rsi, int_reg::Rdx,
            int_reg::Rcx, int_reg::R8, int_reg::R9
        };

        auto arg = tc->getReg(int_reg_map[state++]);
        return *reinterpret_cast<GuestAddr*>(&arg);
    }
};

} // namespace guest_abi
} // namespace gem5
