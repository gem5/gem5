/*
 * Copyright (c) 2014,2016,2018 ARM Limited
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
 * Copyright (c) 2007-2008 The Florida State University
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

#ifndef __ARCH_ARM_INSTS_PSEUDO_HH__
#define __ARCH_ARM_INSTS_PSEUDO_HH__

#include "arch/arm/insts/static_inst.hh"

class DecoderFaultInst : public ArmStaticInst
{
  protected:
    DecoderFault faultId;

    const char *faultName() const;

  public:
    DecoderFaultInst(ExtMachInst _machInst);

    Fault execute(ExecContext *xc,
                  Trace::InstRecord *traceData) const override;

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

/**
 * Static instruction class for unimplemented instructions that
 * cause simulator termination.  Note that these are recognized
 * (legal) instructions that the simulator does not support; the
 * 'Unknown' class is used for unrecognized/illegal instructions.
 * This is a leaf class.
 */
class FailUnimplemented : public ArmStaticInst
{
  private:
    /// Full mnemonic for MRC and MCR instructions including the
    /// coproc. register name
    std::string fullMnemonic;

  public:
    FailUnimplemented(const char *_mnemonic, ExtMachInst _machInst);
    FailUnimplemented(const char *_mnemonic, ExtMachInst _machInst,
                      const std::string& _fullMnemonic);

    Fault execute(ExecContext *xc,
                  Trace::InstRecord *traceData) const override;

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

/**
 * Base class for unimplemented instructions that cause a warning
 * to be printed (but do not terminate simulation).  This
 * implementation is a little screwy in that it will print a
 * warning for each instance of a particular unimplemented machine
 * instruction, not just for each unimplemented opcode.  Should
 * probably make the 'warned' flag a static member of the derived
 * class.
 */
class WarnUnimplemented : public ArmStaticInst
{
  private:
    /// Have we warned on this instruction yet?
    mutable bool warned;
    /// Full mnemonic for MRC and MCR instructions including the
    /// coproc. register name
    std::string fullMnemonic;

  public:
    WarnUnimplemented(const char *_mnemonic, ExtMachInst _machInst);
    WarnUnimplemented(const char *_mnemonic, ExtMachInst _machInst,
                      const std::string& _fullMnemonic);

    Fault execute(ExecContext *xc,
                  Trace::InstRecord *traceData) const override;

    std::string generateDisassembly(
            Addr pc, const Loader::SymbolTable *symtab) const override;
};

/**
 * This class is modelling instructions which are not going to be
 * executed since they are flagged as Illegal Execution Instructions
 * (PSTATE.IL = 1 or CPSR.IL = 1).
 * The sole purpose of this instruction is to generate an appropriate
 * fault when executed.
 */
class IllegalExecInst : public ArmStaticInst
{
  public:
    IllegalExecInst(ExtMachInst _machInst);

    Fault execute(ExecContext *xc, Trace::InstRecord *traceData) const;
};

#endif
