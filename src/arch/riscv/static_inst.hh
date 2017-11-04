// -*- mode:c++ -*-

// Copyright (c) 2015 RISC-V Foundation
// Copyright (c) 2016 The University of Virginia
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Authors: Maxwell Walter
//          Alec Roelke

#ifndef __ARCH_RISCV_STATIC_INST_HH__
#define __ARCH_RISCV_STATIC_INST_HH__

////////////////////////////////////////////////////////////////////
//
// Base class for Riscv instructions, and some support functions
//

namespace RiscvISA {

/**
 * Base class for all RISC-V static instructions.
 */
class RiscvStaticInst : public StaticInst
{
  protected:
    // Constructor
    RiscvStaticInst(const char *mnem, MachInst _machInst,
        OpClass __opClass) : StaticInst(mnem, _machInst, __opClass)
    {}

    virtual std::string
    generateDisassembly(Addr pc, const SymbolTable *symtab) const = 0;

  public:
    void
    advancePC(RiscvISA::PCState &pc) const
    {
        pc.advance();
    }
};

/**
 * Base class for all RISC-V Macroops
 */
class RiscvMacroInst : public RiscvStaticInst
{
  protected:
    std::vector<StaticInstPtr> microops;

    // Constructor
    RiscvMacroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass) :
            RiscvStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMacroop] = true;
    }

    ~RiscvMacroInst()
    {
        microops.clear();
    }

    StaticInstPtr
    fetchMicroop(MicroPC upc) const
    {
        return microops[upc];
    }

    Fault
    initiateAcc(ExecContext *xc, Trace::InstRecord *traceData) const
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    completeAcc(PacketPtr pkt, ExecContext *xc,
                Trace::InstRecord *traceData) const
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    execute(ExecContext *xc, Trace::InstRecord *traceData) const
    {
        panic("Tried to execute a macroop directly!\n");
    }
};

/**
 * Base class for all RISC-V Microops
 */
class RiscvMicroInst : public RiscvStaticInst
{
  protected:
    // Constructor
    RiscvMicroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass) :
            RiscvStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMicroop] = true;
    }

    void
    advancePC(RiscvISA::PCState &pcState) const
    {
        if (flags[IsLastMicroop]) {
            pcState.uEnd();
        } else {
            pcState.uAdvance();
        }
    }
};

}

#endif // __ARCH_RISCV_STATIC_INST_HH__
