/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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

#include "cpu/static_inst.hh"

#include <iostream>

#include "sim/core.hh"

namespace {

static TheISA::ExtMachInst nopMachInst;

class NopStaticInst : public StaticInst
{
  public:
    NopStaticInst() : StaticInst("gem5 nop", nopMachInst, No_OpClass)
    {}

    Fault
    execute(ExecContext *xc, Trace::InstRecord *traceData) const override
    {
        return NoFault;
    }

    void
    advancePC(TheISA::PCState &pcState) const override
    {
        pcState.advance();
    }

    std::string
    generateDisassembly(Addr pc,
            const Loader::SymbolTable *symtab) const override
    {
        return mnemonic;
    }

  private:
};

}

StaticInstPtr StaticInst::nullStaticInstPtr;
StaticInstPtr StaticInst::nopStaticInstPtr = new NopStaticInst;

using namespace std;

StaticInst::~StaticInst()
{
    if (cachedDisassembly)
        delete cachedDisassembly;
}

bool
StaticInst::hasBranchTarget(const TheISA::PCState &pc, ThreadContext *tc,
                            TheISA::PCState &tgt) const
{
    if (isDirectCtrl()) {
        tgt = branchTarget(pc);
        return true;
    }

    if (isIndirectCtrl()) {
        tgt = branchTarget(tc);
        return true;
    }

    return false;
}

StaticInstPtr
StaticInst::fetchMicroop(MicroPC upc) const
{
    panic("StaticInst::fetchMicroop() called on instruction "
          "that is not microcoded.");
}

TheISA::PCState
StaticInst::branchTarget(const TheISA::PCState &pc) const
{
    panic("StaticInst::branchTarget() called on instruction "
          "that is not a PC-relative branch.");
    M5_DUMMY_RETURN;
}

TheISA::PCState
StaticInst::branchTarget(ThreadContext *tc) const
{
    panic("StaticInst::branchTarget() called on instruction "
          "that is not an indirect branch.");
    M5_DUMMY_RETURN;
}

const string &
StaticInst::disassemble(Addr pc, const Loader::SymbolTable *symtab) const
{
    if (!cachedDisassembly)
        cachedDisassembly = new string(generateDisassembly(pc, symtab));

    return *cachedDisassembly;
}

void
StaticInst::printFlags(std::ostream &outs,
    const std::string &separator) const
{
    bool printed_a_flag = false;

    for (unsigned int flag = IsNop; flag < Num_Flags; flag++) {
        if (flags[flag]) {
            if (printed_a_flag)
                outs << separator;

            outs << FlagsStrings[flag];
            printed_a_flag = true;
        }
    }
}
