/*
 * Copyright 2019 Google, Inc.
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

#ifndef __SIM_SYSCALL_ABI_HH__
#define __SIM_SYSCALL_ABI_HH__

#include "base/bitfield.hh"
#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "sim/guest_abi.hh"
#include "sim/syscall_return.hh"

namespace gem5
{

class SyscallDesc;

struct GenericSyscallABI
{
    using State = int;
};

struct GenericSyscallABI64 : public GenericSyscallABI
{
    using UintPtr = uint64_t;
};

struct GenericSyscallABI32 : public GenericSyscallABI
{
    using UintPtr = uint32_t;

    // Is this argument too big for a single register?
    template <typename T, typename Enabled = void>
    struct IsWide : public std::false_type
    {
    };

    template <typename T>
    struct IsWide<T, std::enable_if_t<(sizeof(T) > sizeof(UintPtr))>> :
        public std::true_type
    {
    };

    template <typename T>
    static constexpr bool IsWideV = IsWide<T>::value;

    // Read two registers and merge them into one value.
    static uint64_t
    mergeRegs(ThreadContext *tc, const RegId &low_id, const RegId &high_id)
    {
        RegVal low = tc->getReg(low_id);
        RegVal high = tc->getReg(high_id);
        return insertBits(low, 63, 32, high);
    }
};

namespace guest_abi
{

// For 64 bit systems, return syscall args directly.
template <typename ABI, typename Arg>
struct Argument<
    ABI, Arg,
    typename std::enable_if_t<std::is_base_of_v<GenericSyscallABI64, ABI> &&
                              std::is_integral_v<Arg>>>
{
    static Arg
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state >= ABI::ArgumentRegs.size(),
                 "Ran out of syscall argument registers.");
        return tc->getReg(ABI::ArgumentRegs[state++]);
    }
};

// For 32 bit systems, return small enough syscall args directly. Large
// arguments aren't handled generically.
template <typename ABI, typename Arg>
struct Argument<ABI, Arg,
                typename std::enable_if_t<std::is_integral_v<Arg> &&
                                          !ABI::template IsWideV<Arg>>>
{
    static Arg
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state >= ABI::ArgumentRegs.size(),
                 "Ran out of syscall argument registers.");
        return bits(tc->getReg(ABI::ArgumentRegs[state++]), 31, 0);
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __SIM_SYSCALL_ABI_HH__
