/*
 * Copyright (c) 2018, 2019 ARM Limited
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

#ifndef __ARCH_ARM_SEMIHOSTING_HH__
#define __ARCH_ARM_SEMIHOSTING_HH__

#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <utility>
#include <vector>

#include "arch/arm/regs/int.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/semihosting.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "sim/core.hh"
#include "sim/guest_abi.hh"
#include "sim/pseudo_inst.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct ArmSemihostingParams;
class SerialDevice;

/** Semihosting for AArch32 and AArch64. */
class ArmSemihosting : public BaseSemihosting
{
  public:
    enum
    {
        // Standard ARM immediate values which trigger semihosting.
        T32Imm = 0xAB,
        A32Imm = 0x123456,
        A64Imm = 0xF000,

        // The immediate value which enables gem5 semihosting calls. Use the
        // standard value for thumb.
        Gem5Imm = 0x5D57
    };

    static PortProxy &portProxyImpl(ThreadContext *tc);
    PortProxy &portProxy(ThreadContext *tc) const override
    {
        return portProxyImpl(tc);
    }
    ByteOrder byteOrder(ThreadContext *tc) const override
    {
        return ArmISA::byteOrder(tc);
    }

    struct Abi64 : public AbiBase
    {
        using UintPtr = uint64_t;

        class State : public StateBase<uint64_t, ArmSemihosting>
        {
          public:
            // For 64 bit semihosting, the params are pointer to by X1.
            explicit
            State(const ThreadContext *tc) :
                StateBase<uint64_t, ArmSemihosting>(tc,
                        tc->getReg(ArmISA::int_reg::X1), &ArmISA::byteOrder)
            {}
        };
    };

    struct Abi32 : public AbiBase
    {
        using UintPtr = uint32_t;

        class State : public StateBase<uint64_t, ArmSemihosting>
        {
          public:
            // For 32 bit semihosting, the params are pointer to by R1.
            explicit
            State(const ThreadContext *tc) :
                StateBase<uint64_t, ArmSemihosting>(tc,
                        tc->getReg(ArmISA::int_reg::R1), &ArmISA::byteOrder)
            {}
        };
    };

    enum Operation
    {
        SYS_OPEN = 0x01,
        SYS_CLOSE = 0x02,
        SYS_WRITEC = 0x03,
        SYS_WRITE0 = 0x04,
        SYS_WRITE = 0x05,
        SYS_READ = 0x06,
        SYS_READC = 0x07,
        SYS_ISERROR = 0x08,
        SYS_ISTTY = 0x09,
        SYS_SEEK = 0x0A,
        SYS_FLEN = 0x0C,
        SYS_TMPNAM = 0x0D,
        SYS_REMOVE = 0x0E,
        SYS_RENAME = 0x0F,
        SYS_CLOCK = 0x10,
        SYS_TIME = 0x11,
        SYS_SYSTEM = 0x12,
        SYS_ERRNO = 0x13,
        SYS_GET_CMDLINE = 0x15,
        SYS_HEAPINFO = 0x16,
        SYS_EXIT = 0x18,
        SYS_EXIT_EXTENDED = 0x20,
        SYS_ELAPSED = 0x30,
        SYS_TICKFREQ = 0x31,

        MaxStandardOp = 0xFF,

        SYS_GEM5_PSEUDO_OP = 0x100
    };

    using SemiCall = SemiCallBase<ArmSemihosting, Abi32, Abi64>;

    explicit ArmSemihosting(const ArmSemihostingParams &p);

    /** Perform an Arm Semihosting call from aarch64 code. */
    bool call64(ThreadContext *tc, bool gem5_ops);
    /** Perform an Arm Semihosting call from aarch32 code. */
    bool call32(ThreadContext *tc, bool gem5_ops);

  protected:
    RetErrno callGem5PseudoOp32(ThreadContext *tc, uint32_t encoded_func);
    RetErrno callGem5PseudoOp64(ThreadContext *tc, uint64_t encoded_func);

    static const std::map<uint32_t, SemiCall> calls;
};

namespace guest_abi
{

template <typename Arg>
struct Argument<ArmSemihosting::Abi64, Arg,
    typename std::enable_if_t<
        (std::is_integral_v<Arg> ||
         std::is_same<Arg,pseudo_inst::GuestAddr>::value)>>
{
    static Arg
    get(ThreadContext *tc, ArmSemihosting::Abi64::State &state)
    {
        return (Arg)state.get(tc);
    }
};

template <typename Arg>
struct Argument<ArmSemihosting::Abi32, Arg,
    typename std::enable_if_t<
        (std::is_integral_v<Arg> ||
         std::is_same<Arg,pseudo_inst::GuestAddr>::value)>>
{
    static Arg
    get(ThreadContext *tc, ArmSemihosting::Abi32::State &state)
    {
        if (std::is_signed_v<Arg>) {
            return (Arg)sext<32>(state.get(tc));
        }
        else {
            return (Arg)state.get(tc);
        }
    }
};

template <typename Abi>
struct Argument<Abi, ArmSemihosting::InPlaceArg, typename std::enable_if_t<
    std::is_base_of_v<ArmSemihosting::AbiBase, Abi>>>
{
    static ArmSemihosting::InPlaceArg
    get(ThreadContext *tc, typename Abi::State &state)
    {
        return ArmSemihosting::InPlaceArg(
                state.getAddr(), sizeof(typename Abi::State::ArgType));
    }
};

template <>
struct Result<ArmSemihosting::Abi32, ArmSemihosting::RetErrno>
{
    static void
    store(ThreadContext *tc, const ArmSemihosting::RetErrno &err)
    {
        tc->setReg(ArmISA::int_reg::R0, err.first);
    }
};

template <>
struct Result<ArmSemihosting::Abi64, ArmSemihosting::RetErrno>
{
    static void
    store(ThreadContext *tc, const ArmSemihosting::RetErrno &err)
    {
        tc->setReg(ArmISA::int_reg::X0, err.first);
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_ARM_SEMIHOSTING_HH__
