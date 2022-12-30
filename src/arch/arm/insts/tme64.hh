/*
 * Copyright (c) 2020-2021 ARM Limited
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

#ifndef __ARCH_ARM_INSTS_TME64_HH__
#define __ARCH_ARM_INSTS_TME64_HH__

#include "arch/arm/insts/macromem.hh"
#include "arch/arm/insts/pred_inst.hh"
#include "arch/arm/insts/static_inst.hh"

namespace gem5
{

namespace ArmISAInst {

class MicroTmeOp : public ArmISA::MicroOp
{
  protected:
    MicroTmeOp(const char *mnem, ArmISA::ExtMachInst machInst,
               OpClass __opClass)
      : ArmISA::MicroOp(mnem, machInst, __opClass)
    {}
};

class MicroTmeBasic64 : public MicroTmeOp
{
  protected:
    MicroTmeBasic64(const char *mnem, ArmISA::ExtMachInst machInst,
                    OpClass __opClass) :
                    MicroTmeOp(mnem, machInst, __opClass)
    {}

    std::string generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const;
};

class TmeImmOp64 : public ArmISA::ArmStaticInst
{
  protected:
    uint64_t imm;

    TmeImmOp64(const char *mnem, ArmISA::ExtMachInst machInst,
               OpClass __opClass, uint64_t _imm)
      : ArmISA::ArmStaticInst(mnem, machInst, __opClass),
        imm(_imm)
    {}

    std::string generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const;
};

class TmeRegNone64 : public ArmISA::ArmStaticInst
{
  protected:
    RegIndex dest;

    TmeRegNone64(const char *mnem, ArmISA::ExtMachInst machInst,
                 OpClass __opClass, RegIndex _dest)
      : ArmISA::ArmStaticInst(mnem, machInst, __opClass),
        dest(_dest)
    {}

    std::string generateDisassembly(Addr pc,
                                    const loader::SymbolTable *symtab) const;
};

class Tstart64 : public TmeRegNone64
{
  private:
    RegId destRegIdxArr[1];

  public:
    Tstart64(ArmISA::ExtMachInst, RegIndex);

    Fault execute(ExecContext *, trace::InstRecord *) const;
    Fault initiateAcc(ExecContext *, trace::InstRecord *) const;
    Fault completeAcc(PacketPtr, ExecContext *, trace::InstRecord *) const;
};

class Ttest64 : public TmeRegNone64
{
  private:
    RegId destRegIdxArr[1];

  public:
    Ttest64(ArmISA::ExtMachInst, RegIndex);

    Fault execute(ExecContext *, trace::InstRecord *) const;
};

class Tcancel64 : public TmeImmOp64
{
  public:
    Tcancel64(ArmISA::ExtMachInst, uint64_t);

    Fault execute(ExecContext *, trace::InstRecord *) const;
    Fault initiateAcc(ExecContext *, trace::InstRecord *) const;
    Fault completeAcc(PacketPtr, ExecContext *, trace::InstRecord *) const;
};

class MicroTfence64 : public MicroTmeBasic64
{
  public:
    MicroTfence64(ArmISA::ExtMachInst);

    Fault execute(ExecContext *, trace::InstRecord *) const;
    Fault initiateAcc(ExecContext *, trace::InstRecord *) const;
    Fault completeAcc(PacketPtr, ExecContext *, trace::InstRecord *) const;
};

class MicroTcommit64 : public MicroTmeBasic64
{
  public:
    MicroTcommit64(ArmISA::ExtMachInst);

    Fault execute(ExecContext *, trace::InstRecord *) const;
    Fault initiateAcc(ExecContext *, trace::InstRecord *) const;
    Fault completeAcc(PacketPtr, ExecContext *, trace::InstRecord *) const;
};


class MacroTmeOp : public ArmISA::PredMacroOp
{
  protected:
    MacroTmeOp(const char *mnem, ArmISA::ExtMachInst _machInst,
               OpClass __opClass);
};

class Tcommit64 : public MacroTmeOp
{
  public:
    Tcommit64(ArmISA::ExtMachInst _machInst);
};

} // namespace ArmISAInst
} // namespace gem5

#endif
