/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
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
 *
 * Authors: Alec Roelke
 */

#ifndef __ARCH_RISCV_FAULTS_HH__
#define __ARCH_RISCV_FAULTS_HH__

#include <string>

#include "cpu/thread_context.hh"
#include "sim/faults.hh"

namespace RiscvISA
{

const uint32_t FloatInexact = 1 << 0;
const uint32_t FloatUnderflow = 1 << 1;
const uint32_t FloatOverflow = 1 << 2;
const uint32_t FloatDivZero = 1 << 3;
const uint32_t FloatInvalid = 1 << 4;

enum ExceptionCode {
    INST_ADDR_MISALIGNED = 0,
    INST_ACCESS = 1,
    INST_ILLEGAL = 2,
    BREAKPOINT = 3,
    LOAD_ADDR_MISALIGNED = 4,
    LOAD_ACCESS = 5,
    STORE_ADDR_MISALIGNED = 6,
    AMO_ADDR_MISALIGNED = 6,
    STORE_ACCESS = 7,
    AMO_ACCESS = 7,
    ECALL_USER = 8,
    ECALL_SUPER = 9,
    ECALL_HYPER = 10,
    ECALL_MACH = 11
};

enum InterruptCode {
    SOFTWARE,
    TIMER
};

class RiscvFault : public FaultBase
{
  protected:
    const FaultName _name;
    const ExceptionCode _code;
    const InterruptCode _int;

    RiscvFault(FaultName n, ExceptionCode c, InterruptCode i)
        : _name(n), _code(c), _int(i)
    {}

    FaultName
    name() const
    {
        return _name;
    }

    ExceptionCode
    exception() const
    {
        return _code;
    }

    InterruptCode
    interrupt() const
    {
        return _int;
    }

    virtual void
    invoke_se(ThreadContext *tc, const StaticInstPtr &inst);

    void
    invoke(ThreadContext *tc, const StaticInstPtr &inst);
};


class UnknownInstFault : public RiscvFault
{
  public:
    UnknownInstFault() : RiscvFault("Unknown instruction", INST_ILLEGAL,
            SOFTWARE)
    {}

    void
    invoke_se(ThreadContext *tc, const StaticInstPtr &inst);
};

class IllegalInstFault : public RiscvFault
{
  private:
    const std::string reason;
  public:
    IllegalInstFault(std::string r)
        : RiscvFault("Illegal instruction", INST_ILLEGAL, SOFTWARE),
          reason(r)
    {}

    void invoke_se(ThreadContext *tc, const StaticInstPtr &inst);
};

class UnimplementedFault : public RiscvFault
{
  private:
    const std::string instName;
  public:
    UnimplementedFault(std::string name)
        : RiscvFault("Unimplemented instruction", INST_ILLEGAL, SOFTWARE),
        instName(name)
    {}

    void
    invoke_se(ThreadContext *tc, const StaticInstPtr &inst);
};

class IllegalFrmFault: public RiscvFault
{
  private:
    const uint8_t frm;
  public:
    IllegalFrmFault(uint8_t r)
        : RiscvFault("Illegal floating-point rounding mode", INST_ILLEGAL,
                SOFTWARE),
        frm(r)
    {}

    void invoke_se(ThreadContext *tc, const StaticInstPtr &inst);
};

class BreakpointFault : public RiscvFault
{
  public:
    BreakpointFault() : RiscvFault("Breakpoint", BREAKPOINT, SOFTWARE)
    {}

    void
    invoke_se(ThreadContext *tc, const StaticInstPtr &inst);
};

class SyscallFault : public RiscvFault
{
  public:
    // TODO: replace ECALL_USER with the appropriate privilege level of the
    // caller
    SyscallFault() : RiscvFault("System call", ECALL_USER, SOFTWARE)
    {}

    void
    invoke_se(ThreadContext *tc, const StaticInstPtr &inst);
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_FAULTS_HH__
