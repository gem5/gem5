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
 *
 * Authors: Alec Roelke
 *          Robert Scheffel
 */

#ifndef __ARCH_RISCV_FAULTS_HH__
#define __ARCH_RISCV_FAULTS_HH__

#include <string>

#include "arch/riscv/registers.hh"
#include "cpu/thread_context.hh"
#include "sim/faults.hh"

namespace RiscvISA
{

enum FloatException : MiscReg {
    FloatInexact = 0x1,
    FloatUnderflow = 0x2,
    FloatOverflow = 0x4,
    FloatDivZero = 0x8,
    FloatInvalid = 0x10
};

enum ExceptionCode : MiscReg {
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
    AMO_PAGE = 15
};

/**
 * These fields are specified in the RISC-V Instruction Set Manual, Volume II,
 * v1.10, accessible at www.riscv.org. in Figure 3.7. The main register that
 * uses these fields is the MSTATUS register, which is shadowed by two others
 * accessible at lower privilege levels (SSTATUS and USTATUS) that can't see
 * the fields for higher privileges.
 */
BitUnion64(STATUS)
    Bitfield<63> sd;
    Bitfield<35, 34> sxl;
    Bitfield<33, 32> uxl;
    Bitfield<22> tsr;
    Bitfield<21> tw;
    Bitfield<20> tvm;
    Bitfield<19> mxr;
    Bitfield<18> sum;
    Bitfield<17> mprv;
    Bitfield<16, 15> xs;
    Bitfield<14, 13> fs;
    Bitfield<12, 11> mpp;
    Bitfield<8> spp;
    Bitfield<7> mpie;
    Bitfield<5> spie;
    Bitfield<4> upie;
    Bitfield<3> mie;
    Bitfield<1> sie;
    Bitfield<0> uie;
EndBitUnion(STATUS)

/**
 * These fields are specified in the RISC-V Instruction Set Manual, Volume II,
 * v1.10 in Figures 3.11 and 3.12, accessible at www.riscv.org. Both the MIP
 * and MIE registers have the same fields, so accesses to either should use
 * this bit union.
 */
BitUnion64(INTERRUPT)
    Bitfield<11> mei;
    Bitfield<9> sei;
    Bitfield<8> uei;
    Bitfield<7> mti;
    Bitfield<5> sti;
    Bitfield<4> uti;
    Bitfield<3> msi;
    Bitfield<1> ssi;
    Bitfield<0> usi;
EndBitUnion(INTERRUPT)

class RiscvFault : public FaultBase
{
  protected:
    const FaultName _name;
    bool _interrupt;
    const ExceptionCode _code;

    RiscvFault(FaultName n, bool i, ExceptionCode c)
        : _name(n), _interrupt(i), _code(c)
    {}

    FaultName name() const override { return _name; }
    bool isInterrupt() const { return _interrupt; }
    ExceptionCode exception() const { return _code; }
    virtual MiscReg trap_value() const { return 0; }

    virtual void invokeSE(ThreadContext *tc, const StaticInstPtr &inst);
    void invoke(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class Reset : public FaultBase
{

    public:
        Reset()
            : _name("reset")
        {}

        FaultName
        name() const override
        {
            return _name;
        }

        void
        invoke(ThreadContext *tc, const StaticInstPtr &inst =
            StaticInst::nullStaticInstPtr) override;

    private:
        const FaultName _name;
};

class InstFault : public RiscvFault
{
  protected:
    const ExtMachInst _inst;

  public:
    InstFault(FaultName n, const ExtMachInst inst)
        : RiscvFault(n, false, INST_ILLEGAL), _inst(inst)
    {}

    MiscReg trap_value() const override { return _inst; }
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
        : InstFault("Illegal instruction", inst)
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
        : RiscvFault("Address", false, code), _addr(addr)
    {}

    MiscReg trap_value() const override { return _addr; }
};

class BreakpointFault : public RiscvFault
{
  private:
    const PCState pcState;

  public:
    BreakpointFault(const PCState &pc)
        : RiscvFault("Breakpoint", false, BREAKPOINT), pcState(pc)
    {}

    MiscReg trap_value() const override { return pcState.pc(); }
    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

class SyscallFault : public RiscvFault
{
  public:
    // TODO: replace ECALL_USER with the appropriate privilege level of the
    // caller
    SyscallFault() : RiscvFault("System call", false, ECALL_USER) {}
    void invokeSE(ThreadContext *tc, const StaticInstPtr &inst) override;
};

} // namespace RiscvISA

#endif // __ARCH_RISCV_FAULTS_HH__