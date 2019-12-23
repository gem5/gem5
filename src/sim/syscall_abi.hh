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

#include "base/types.hh"
#include "cpu/thread_context.hh"
#include "sim/guest_abi.hh"
#include "sim/syscall_return.hh"

class SyscallDesc;

namespace GuestABI
{

// Does this normally 64 bit data type shrink down to 32 bits for 32 bit ABIs?
template <typename T, typename Enabled=void>
struct IsConforming : public std::false_type {};

template <>
struct IsConforming<Addr> : public std::true_type {};

} // namespace GuestABI

struct GenericSyscallABI
{
    using State = int;
};

struct GenericSyscallABI64 : public GenericSyscallABI
{};

struct GenericSyscallABI32 : public GenericSyscallABI
{
    // Is this argument too big for a single register?
    template <typename T, typename Enabled=void>
    struct IsWide;

    template <typename T>
    struct IsWide<T, typename std::enable_if<
        std::is_integral<T>::value &&
        (sizeof(T) < sizeof(uint64_t) ||
         GuestABI::IsConforming<T>::value)>::type>
    {
        static const bool value = false;
    };

    template <typename T>
    struct IsWide<T, typename std::enable_if<
        std::is_integral<T>::value &&
        sizeof(T) == sizeof(uint64_t) &&
        !GuestABI::IsConforming<T>::value>::type>
    {
        static const bool value = true;
    };

    // Read two registers and merge them into one value.
    static uint64_t
    mergeRegs(ThreadContext *tc, RegIndex low_idx, RegIndex high_idx)
    {
        RegVal low = tc->readIntReg(low_idx);
        RegVal high = tc->readIntReg(high_idx);
        return insertBits(low, 63, 32, high);
    }
};

namespace GuestABI
{

// For 64 bit systems, return syscall args directly.
template <typename ABI, typename Arg>
struct Argument<ABI, Arg,
    typename std::enable_if<
        std::is_base_of<GenericSyscallABI64, ABI>::value &&
        std::is_integral<Arg>::value>::type>
{
    static Arg
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state >= ABI::ArgumentRegs.size(),
                "Ran out of syscall argument registers.");
        return tc->readIntReg(ABI::ArgumentRegs[state++]);
    }
};

// For 32 bit systems, return small enough syscall args directly. Large
// arguments aren't handled generically.
template <typename ABI, typename Arg>
struct Argument<ABI, Arg,
    typename std::enable_if<!ABI::template IsWide<Arg>::value>::type>
{
    static Arg
    get(ThreadContext *tc, typename ABI::State &state)
    {
        panic_if(state >= ABI::ArgumentRegs.size(),
                "Ran out of syscall argument registers.");
        return bits(tc->readIntReg(ABI::ArgumentRegs[state++]), 31, 0);
    }
};

} // namespace GuestABI

#endif // __SIM_SYSCALL_ABI_HH__
