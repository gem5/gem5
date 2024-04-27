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

#ifndef __ARCH_RISCV_SEMIHOSTING_HH__
#define __ARCH_RISCV_SEMIHOSTING_HH__

#include "arch/generic/semihosting.hh"
#include "arch/riscv/isa.hh"
#include "arch/riscv/regs/int.hh"
#include "cpu/thread_context.hh"
#include "sim/guest_abi.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct RiscvSemihostingParams;
class SerialDevice;

/** Semihosting for RV32 and RV64. */
class RiscvSemihosting : public BaseSemihosting
{
  public:
    enum class Opcode : uint32_t
    {
        // https://github.com/riscv-software-src/riscv-semihosting/blob/main/
        // riscv-semihosting-spec.adoc#21-semihosting-trap-instruction-sequence
        Prefix = 0x01f01013, // slli x0, x0, 0x1f Entry NOP
        EBreak = 0x00100073, // ebreak            Break to debugger
        Suffix = 0x40705013, // srai x0, x0, 7    NOP encoding semihosting
    };
    static PortProxy &portProxyImpl(ThreadContext *tc);
    PortProxy &
    portProxy(ThreadContext *tc) const override
    {
        return portProxyImpl(tc);
    }
    ByteOrder
    byteOrder(ThreadContext *tc) const override
    {
        return ByteOrder::little;
    }

    template <typename ArgType>
    struct RiscvSemihostingAbi : public AbiBase
    {
        using UintPtr = ArgType;

        class State : public StateBase<ArgType, RiscvSemihosting>
        {
          public:
            explicit
            State(const ThreadContext *tc) :
                StateBase<ArgType, RiscvSemihosting>(tc,
                        tc->getReg(RiscvISA::ArgumentRegs[1]),
                        [](const ThreadContext *) {
                            return ByteOrder::little;
                        })
            {}
        };
    };

    struct Abi64 : public RiscvSemihostingAbi<uint64_t>
    {};
    struct Abi32 : public RiscvSemihostingAbi<uint32_t>
    {};

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

    using SemiCall = SemiCallBase<RiscvSemihosting, Abi32, Abi64>;

    explicit RiscvSemihosting(const RiscvSemihostingParams &p);

    /** Perform a RISC-V Semihosting call */
    bool isSemihostingEBreak(ExecContext *xc);
    bool call(ThreadContext *tc);
  protected:
    bool call64(ThreadContext *tc);
    bool call32(ThreadContext *tc);
    static const std::map<uint32_t, SemiCall> calls;
};

namespace guest_abi
{

template <typename Arg>
struct Argument<RiscvSemihosting::Abi64, Arg,
        typename std::enable_if_t<std::is_integral_v<Arg>>>
{
    static Arg
    get(ThreadContext *tc, RiscvSemihosting::Abi64::State &state)
    {
        return state.get(tc);
    }
};

template <typename Arg>
struct Argument<RiscvSemihosting::Abi32, Arg,
        typename std::enable_if_t<std::is_integral_v<Arg>>>
{
    static Arg
    get(ThreadContext *tc, RiscvSemihosting::Abi32::State &state)
    {
        if (std::is_signed_v<Arg>)
            return sext<32>(state.get(tc));
        else
            return state.get(tc);
    }
};

template <typename Abi>
struct Argument<Abi, RiscvSemihosting::InPlaceArg,
        typename std::enable_if_t<
                std::is_base_of_v<RiscvSemihosting::AbiBase, Abi>>>
{
    static RiscvSemihosting::InPlaceArg
    get(ThreadContext *tc, typename Abi::State &state)
    {
        return RiscvSemihosting::InPlaceArg(
                state.getAddr(), sizeof(typename Abi::State::ArgType));
    }
};

template <typename Abi>
struct Result<Abi, RiscvSemihosting::RetErrno>
{
    static void
    store(ThreadContext *tc, const RiscvSemihosting::RetErrno &err)
    {
        tc->setReg(RiscvISA::ReturnValueReg, err.first);
    }
};

} // namespace guest_abi
} // namespace gem5

#endif // __ARCH_RISCV_SEMIHOSTING_HH__
