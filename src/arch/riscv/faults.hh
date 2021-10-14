/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2018 TU Dresden
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

#ifndef __ARCH_RISCV_FAULTS_HH__
#define __ARCH_RISCV_FAULTS_HH__

#include <cstdint>
#include <string>

#include "arch/riscv/isa.hh"
#include "cpu/null_static_inst.hh"
#include "sim/faults.hh"

namespace gem5
{

class ThreadContext;

namespace RiscvISA
{

enum FloatException : uint64_t
{
    FloatInexact = 0x1,
    FloatUnderflow = 0x2,
    FloatOverflow = 0x4,
    FloatDivZero = 0x8,
    FloatInvalid = 0x10
};

/*
 * In RISC-V, exception and interrupt codes share some values. They can be
 * differentiated by an 'Interrupt' flag that is enabled for interrupt faults
 * but not exceptions. The full fault cause can be computed by placing the
 * exception (or interrupt) code in the least significant bits of the CAUSE
 * CSR and then setting the highest bit of CAUSE with the 'Interrupt' flag.
 * For more details on exception causes, see Chapter 3.1.20 of the RISC-V
 * privileged specification v 1.10. Codes are enumerated in Table 3.6.
 */
enum ExceptionCode : uint64_t
{
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
    ECALL_MACHINE = 11,
    INST_PAGE = 12,
    LOAD_PAGE = 13,
    STORE_PAGE = 15,
    AMO_PAGE = 15,

    INT_SOFTWARE_USER = 0,
    INT_SOFTWARE_SUPER = 1,
    INT_SOFTWARE_MACHINE = 3,
    INT_TIMER_USER = 4,
    INT_TIMER_SUPER = 5,
    INT_TIMER_MACHINE = 7,
    INT_EXT_USER = 8,
    INT_EXT_SUPER = 9,
    INT_EXT_MACHINE = 11,
    NumInterruptTypes
};

enum class FaultType
{
    INTERRUPT,
    NON_MASKABLE_INTERRUPT,
    OTHERS,
};

class RiscvFault : public FaultBase
{
  protected:
    const FaultName _name;
    const FaultType _fault_type;
    ExceptionCode _code;

    RiscvFault(FaultName n, FaultType ft, ExceptionCode c)
        : _name(n), _fault_type(ft), _code(c)
    {}

    FaultName name() const override { return _name; }
    bool isInterrupt() const { return _fault_type == FaultType::INTERRUPT; }
    bool isNonMaskableInterrupt() const
    {
        return _fault_type == FaultType::NON_MASKABLE_INTERRUPT;
    }
    ExceptionCode exception() const { return _code; }
    virtual RegVal trap_value() const { return 0; }

    virtual void invokeSE(ThreadContext *tc, const StaticInstPtr &inst);
    void invoke(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class Reset : public FaultBase
{
  private:
    const FaultName _name;

  public:
    Reset() : _name("reset") {}
    FaultName name() const override { return _name; }

    void invoke(ThreadContext *tc, const StaticInstPtr &inst =
        nullStaticInstPtr) override;
};

class InterruptFault : public RiscvFault
{
  public:
    InterruptFault(ExceptionCode c)
        : RiscvFault("interrupt", FaultType::INTERRUPT, c)
    {}
    InterruptFault(int c) : InterruptFault(static_cast<ExceptionCode>(c)) {}
};

class NonMaskableInterruptFault : public RiscvFault
{
  public:
    NonMaskableInterruptFault()
        : RiscvFault("non_maskable_interrupt",
                     FaultType::NON_MASKABLE_INTERRUPT,
                     static_cast<ExceptionCode>(0))
    {}
};

class InstFault : public RiscvFault
{
  protected:
    const ExtMachInst _inst;

  public:
    InstFault(FaultName n, const ExtMachInst inst)
        : RiscvFault(n, FaultType::OTHERS, INST_ILLEGAL), _inst(inst)
    {}

    RegVal trap_value() const override { return _inst; }
};

class UnknownInstFault : public InstFault
{
  public:
    UnknownInstFault(const ExtMachInst inst)
        : InstFault("Unknown instruction", inst)
    {}

    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class IllegalInstFault : public InstFault
{
  private:
    const std::string reason;

  public:
    IllegalInstFault(std::string r, const ExtMachInst inst)
        : InstFault("Illegal instruction", inst),
          reason(r)
    {}

    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class UnimplementedFault : public InstFault
{
  private:
    const std::string instName;

  public:
    UnimplementedFault(std::string name, const ExtMachInst inst)
        : InstFault("Unimplemented instruction", inst),
          instName(name)
    {}

    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class IllegalFrmFault: public InstFault
{
  private:
    const uint8_t frm;

  public:
    IllegalFrmFault(uint8_t r, const ExtMachInst inst)
        : InstFault("Illegal floating-point rounding mode", inst),
          frm(r)
    {}

    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class AddressFault : public RiscvFault
{
  private:
    const Addr _addr;

  public:
    AddressFault(const Addr addr, ExceptionCode code)
        : RiscvFault("Address", FaultType::OTHERS, code), _addr(addr)
    {}

    RegVal trap_value() const override { return _addr; }
};

class BreakpointFault : public RiscvFault
{
  private:
    const PCState pcState;

  public:
    BreakpointFault(const PCStateBase &pc)
        : RiscvFault("Breakpoint", FaultType::OTHERS, BREAKPOINT),
        pcState(pc.as<PCState>())
    {}

    RegVal trap_value() const override { return pcState.pc(); }
    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class SyscallFault : public RiscvFault
{
  public:
    SyscallFault(PrivilegeMode prv)
        : RiscvFault("System call", FaultType::OTHERS, ECALL_USER)
    {
        switch (prv) {
          case PRV_U:
            _code = ECALL_USER;
            break;
          case PRV_S:
            _code = ECALL_SUPER;
            break;
          case PRV_M:
            _code = ECALL_MACHINE;
            break;
          default:
            panic("Unknown privilege mode %d.", prv);
            break;
        }
    }

    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_FAULTS_HH__
