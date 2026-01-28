/*
 * Copyright (c) 2016 RISC-V Foundation
 * Copyright (c) 2016 The University of Virginia
 * Copyright (c) 2018 TU Dresden
 * Copyright (c) 2024 University of Rostock
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

#include "arch/riscv/fault_codes.hh"
#include "arch/riscv/isa.hh"
#include "cpu/null_static_inst.hh"
#include "sim/faults.hh"

namespace gem5
{

class ThreadContext;

namespace RiscvISA
{

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
    bool isNonMaskableInterrupt() const {
        return _fault_type == FaultType::NON_MASKABLE_INTERRUPT;
    }

    bool isResumableNonMaskableInterrupt(ISA* isa) const
    {
        return isa->enableSmrnmi() && isNonMaskableInterrupt();
    }

    bool isPlainException() const {
        return _fault_type == FaultType::OTHERS;
    }
    bool isGuestPageFault() const {
      return _code == INST_GUEST_PAGE  || _code == LOAD_GUEST_PAGE ||
             _code == STORE_GUEST_PAGE || _code == AMO_GUEST_PAGE;
    }

    ExceptionCode exception() const { return _code; }
    virtual RegVal trap_value() const { return 0; }
    virtual RegVal trap_value2() const { return 0; }
    virtual bool mustSetGva() const { return false; }

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
        : NonMaskableInterruptFault(0) {}
    NonMaskableInterruptFault(ExceptionCode c)
        : RiscvFault("non_maskable_interrupt",
                     FaultType::NON_MASKABLE_INTERRUPT,
                     c)
    {}
    NonMaskableInterruptFault(int c)
        : NonMaskableInterruptFault(static_cast<ExceptionCode>(c)) {}
};

class InstFault : public RiscvFault
{
  protected:
    const ExtMachInst _inst;

  public:
    InstFault(FaultName n, const ExtMachInst inst)
        : RiscvFault(n, FaultType::OTHERS, INST_ILLEGAL), _inst(inst)
    {}

    RegVal trap_value() const override { return _inst.instBits; }
};

class VirtualInstFault : public RiscvFault
{
  protected:
    const ExtMachInst _inst;
    std::string _reason;

  public:
    VirtualInstFault(std::string reason, const ExtMachInst inst)
        : RiscvFault("VirtualInst", FaultType::OTHERS, VIRTUAL_INST),
        _inst(inst), _reason(reason)
    {}

    RegVal trap_value() const override { return _inst.instBits; }
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
    const Addr _vaddr;
    const Addr _gpaddr;
    const bool _two_stage;

  public:
    AddressFault(
        const Addr vaddr, ExceptionCode code,
        const Addr gpaddr = 0x0, const bool two_stage = false)
        : RiscvFault("Address", FaultType::OTHERS, code),
        _vaddr(vaddr), _gpaddr(gpaddr), _two_stage(two_stage)
    {}

    bool mustSetGva() const override {
        switch(_code) {
            case INST_ADDR_MISALIGNED:
            case LOAD_ADDR_MISALIGNED:
            case STORE_ADDR_MISALIGNED:

            case INST_ACCESS: case LOAD_ACCESS: case STORE_ACCESS:
            case INST_PAGE: case LOAD_PAGE: case STORE_PAGE:
            case INST_GUEST_PAGE: case LOAD_GUEST_PAGE: case STORE_GUEST_PAGE:
                return _two_stage;
            default:
                return false;
        }
    }
    RegVal trap_value() const override { return _vaddr; }
    RegVal trap_value2() const override { return _gpaddr >> 2; }
};

class BreakpointFault : public RiscvFault
{
  private:
    const PCState pcState;
    const bool _virtualized;

  public:
    BreakpointFault(const PCStateBase &pc, const bool virtualized = false)
        : RiscvFault("Breakpoint", FaultType::OTHERS, BREAKPOINT),
        pcState(pc.as<PCState>()), _virtualized(virtualized)
    {}

    bool mustSetGva() const override { return _virtualized; }
    RegVal trap_value() const override { return pcState.pc(); }
    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class SyscallFault : public RiscvFault
{
  public:
    SyscallFault(PrivilegeMode prv, const bool virtualized = false)
        : RiscvFault("System call", FaultType::OTHERS, ECALL_USER)
    {
        switch (prv) {
          case PRV_U:
            _code = ECALL_USER;
            break;
          case PRV_S:
            _code = virtualized ? ECALL_VIRTUAL_SUPER : ECALL_SUPER;
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

/**
 * Returns true if the fault passed as a first argument was triggered
 * by a memory access, false otherwise.
 * If true it is storing the faulting address in the va argument
 *
 * @param fault generated fault
 * @param va function will modify this passed-by-reference parameter
 *           with the correct faulting virtual address
 * @return true if va contains a valid value, false otherwise
 */
bool getFaultVAddr(Fault fault, Addr &va);

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_FAULTS_HH__
