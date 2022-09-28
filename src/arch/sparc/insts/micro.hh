/*
 * Copyright (c) 2006-2007 The Regents of The University of Michigan
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

#ifndef __ARCH_SPARC_INSTS_MICRO_HH__
#define __ARCH_SPARC_INSTS_MICRO_HH__

#include "arch/sparc/insts/static_inst.hh"

namespace gem5
{

namespace SparcISA
{

class SparcMacroInst : public SparcStaticInst
{
  protected:
    const uint32_t numMicroops;

    // Constructor.
    SparcMacroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass, uint32_t _numMicroops) :
            SparcStaticInst(mnem, _machInst, __opClass),
            numMicroops(_numMicroops)
    {
        assert(numMicroops);
        microops = new StaticInstPtr[numMicroops];
        flags[IsMacroop] = true;
    }

    ~SparcMacroInst()
    {
        delete [] microops;
    }

    std::string generateDisassembly(
        Addr pc, const loader::SymbolTable *symtab) const override;

    StaticInstPtr *microops;

    StaticInstPtr
    fetchMicroop(MicroPC upc) const override
    {
        assert(upc < numMicroops);
        return microops[upc];
    }

    Fault
    execute(ExecContext *, trace::InstRecord *) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    initiateAcc(ExecContext *, trace::InstRecord *) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }

    Fault
    completeAcc(PacketPtr, ExecContext *, trace::InstRecord *) const override
    {
        panic("Tried to execute a macroop directly!\n");
    }
};

class SparcMicroInst : public SparcStaticInst
{
  protected:
    // Constructor.
    SparcMicroInst(const char *mnem, ExtMachInst _machInst,
                   OpClass __opClass) :
            SparcStaticInst(mnem, _machInst, __opClass)
    {
        flags[IsMicroop] = true;
    }

    void
    advancePC(PCStateBase &pc_state) const override
    {
        auto &spc = pc_state.as<PCState>();
        if (flags[IsLastMicroop])
            spc.uEnd();
        else
            spc.uAdvance();
    }

    void
    advancePC(ThreadContext *tc) const override
    {
        PCState pc = tc->pcState().as<PCState>();
        if (flags[IsLastMicroop])
            pc.uEnd();
        else
            pc.uAdvance();
        tc->pcState(pc);
    }
};

class SparcDelayedMicroInst : public SparcMicroInst
{
  protected:
    // Constructor.
    SparcDelayedMicroInst(const char *mnem, ExtMachInst _machInst,
                          OpClass __opClass) :
            SparcMicroInst(mnem, _machInst, __opClass)
    {
        flags[IsDelayedCommit] = true;
    }
};

} // namespace SparcISA
} // namespace gem5

#endif // __ARCH_SPARC_INSTS_MICRO_HH__
