/*
 * Copyright (c) 2005-2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#ifndef __CPU_OZONE_DYN_INST_IMPL_HH__
#define __CPU_OZONE_DYN_INST_IMPL_HH__

#include "config/the_isa.hh"
#include "cpu/ozone/dyn_inst.hh"
#include "kern/kernel_stats.hh"
#include "sim/faults.hh"

template <class Impl>
OzoneDynInst<Impl>::OzoneDynInst(OzoneCPU *cpu)
    : BaseDynInst<Impl>(0, 0, 0, 0, cpu)
{
    this->setResultReady();

    initInstPtrs();
}

template <class Impl>
OzoneDynInst<Impl>::OzoneDynInst(StaticInstPtr _staticInst)
    : BaseDynInst<Impl>(_staticInst)
{
    initInstPtrs();
}

template <class Impl>
OzoneDynInst<Impl>::~OzoneDynInst()
{
    DPRINTF(BE, "[sn:%lli] destructor called\n", this->seqNum);
    for (int i = 0; i < this->numSrcRegs(); ++i) {
        srcInsts[i] = NULL;
    }

    for (int i = 0; i < this->numDestRegs(); ++i) {
        prevDestInst[i] = NULL;
    }

    dependents.clear();
}

template <class Impl>
Fault
OzoneDynInst<Impl>::execute()
{
    // @todo: Pretty convoluted way to avoid squashing from happening when using
    // the XC during an instruction's execution (specifically for instructions
    // that have sideeffects that use the XC).  Fix this.
    bool no_squash_from_TC = this->thread->noSquashFromTC;
    this->thread->noSquashFromTC = true;

    this->fault = this->staticInst->execute(this, this->traceData);

    this->thread->noSquashFromTC = no_squash_from_TC;

    return this->fault;
}

template <class Impl>
Fault
OzoneDynInst<Impl>::initiateAcc()
{
    // @todo: Pretty convoluted way to avoid squashing from happening when using
    // the XC during an instruction's execution (specifically for instructions
    // that have sideeffects that use the XC).  Fix this.
    bool no_squash_from_TC = this->thread->noSquashFromTC;
    this->thread->noSquashFromTC = true;

    this->fault = this->staticInst->initiateAcc(this, this->traceData);

    this->thread->noSquashFromTC = no_squash_from_TC;

    return this->fault;
}

template <class Impl>
Fault
OzoneDynInst<Impl>::completeAcc(PacketPtr pkt)
{
    this->fault = this->staticInst->completeAcc(pkt, this, this->traceData);

    return this->fault;
}

template <class Impl>
bool
OzoneDynInst<Impl>::srcInstReady(int regIdx)
{
    return srcInsts[regIdx]->isResultReady();
}

template <class Impl>
void
OzoneDynInst<Impl>::addDependent(DynInstPtr &dependent_inst)
{
    dependents.push_back(dependent_inst);
}

template <class Impl>
void
OzoneDynInst<Impl>::wakeDependents()
{
    for (int i = 0; i < dependents.size(); ++i) {
        dependents[i]->markSrcRegReady();
    }
}

template <class Impl>
void
OzoneDynInst<Impl>::wakeMemDependents()
{
    for (int i = 0; i < memDependents.size(); ++i) {
        memDependents[i]->markMemInstReady(this);
    }
}

template <class Impl>
void
OzoneDynInst<Impl>::markMemInstReady(OzoneDynInst<Impl> *inst)
{
    ListIt mem_it = srcMemInsts.begin();
    while ((*mem_it) != inst && mem_it != srcMemInsts.end()) {
        mem_it++;
    }
    assert(mem_it != srcMemInsts.end());

    srcMemInsts.erase(mem_it);
}

template <class Impl>
void
OzoneDynInst<Impl>::initInstPtrs()
{
    for (int i = 0; i < MaxInstSrcRegs; ++i) {
        srcInsts[i] = NULL;
    }
    iqItValid = false;
}

template <class Impl>
bool
OzoneDynInst<Impl>::srcsReady()
{
    for (int i = 0; i < this->numSrcRegs(); ++i) {
        if (!srcInsts[i]->isResultReady())
            return false;
    }

    return true;
}

template <class Impl>
bool
OzoneDynInst<Impl>::eaSrcsReady()
{
    for (int i = 1; i < this->numSrcRegs(); ++i) {
        if (!srcInsts[i]->isResultReady())
            return false;
    }

    return true;
}

template <class Impl>
void
OzoneDynInst<Impl>::clearDependents()
{
    dependents.clear();
    for (int i = 0; i < this->numSrcRegs(); ++i) {
        srcInsts[i] = NULL;
    }
    for (int i = 0; i < this->numDestRegs(); ++i) {
        prevDestInst[i] = NULL;
    }
}

template <class Impl>
void
OzoneDynInst<Impl>::clearMemDependents()
{
    memDependents.clear();
}

template <class Impl>
TheISA::MiscReg
OzoneDynInst<Impl>::readMiscRegNoEffect(int misc_reg)
{
    return this->thread->readMiscRegNoEffect(misc_reg);
}

template <class Impl>
TheISA::MiscReg
OzoneDynInst<Impl>::readMiscReg(int misc_reg)
{
    return this->thread->readMiscReg(misc_reg);
}

template <class Impl>
void
OzoneDynInst<Impl>::setMiscRegNoEffect(int misc_reg, const MiscReg &val)
{
    this->setIntResult(val);
    this->thread->setMiscRegNoEffect(misc_reg, val);
}

template <class Impl>
void
OzoneDynInst<Impl>::setMiscReg(int misc_reg, const MiscReg &val)
{
    this->thread->setMiscReg(misc_reg, val);
}

template <class Impl>
Fault
OzoneDynInst<Impl>::hwrei()
{
    if (!(this->readPC() & 0x3))
        return new AlphaISA::UnimplementedOpcodeFault;

    this->setNextPC(this->thread->readMiscRegNoEffect(AlphaISA::IPR_EXC_ADDR));

    this->cpu->hwrei();

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
void
OzoneDynInst<Impl>::trap(const Fault &fault)
{
    fault->invoke(this->thread->getTC());
}

template <class Impl>
bool
OzoneDynInst<Impl>::simPalCheck(int palFunc)
{
    return this->cpu->simPalCheck(palFunc);
}

template <class Impl>
void
OzoneDynInst<Impl>::syscall(uint64_t &callnum)
{
    this->cpu->syscall(callnum);
}

#endif//__CPU_OZONE_DYN_INST_IMPL_HH__
