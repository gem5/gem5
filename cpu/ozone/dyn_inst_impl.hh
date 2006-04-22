/*
 * Copyright (c) 2005 The Regents of The University of Michigan
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

#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "config/full_system.hh"
#include "cpu/ozone/dyn_inst.hh"
#include "kern/kernel_stats.hh"

using namespace TheISA;

template <class Impl>
OzoneDynInst<Impl>::OzoneDynInst(FullCPU *cpu)
    : BaseDynInst<Impl>(0, 0, 0, 0, cpu)
{
    this->setCompleted();

    initInstPtrs();
}

template <class Impl>
OzoneDynInst<Impl>::OzoneDynInst(ExtMachInst inst, Addr PC, Addr Pred_PC,
                                 InstSeqNum seq_num, FullCPU *cpu)
    : BaseDynInst<Impl>(inst, PC, Pred_PC, seq_num, cpu)
{
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
    bool in_syscall = this->thread->inSyscall;
    this->thread->inSyscall = true;

    this->fault = this->staticInst->execute(this, this->traceData);

    this->thread->inSyscall = in_syscall;

    return this->fault;
}

template <class Impl>
Fault
OzoneDynInst<Impl>::initiateAcc()
{
    // @todo: Pretty convoluted way to avoid squashing from happening when using
    // the XC during an instruction's execution (specifically for instructions
    // that have sideeffects that use the XC).  Fix this.
    bool in_syscall = this->thread->inSyscall;
    this->thread->inSyscall = true;

    this->fault = this->staticInst->initiateAcc(this, this->traceData);

    this->thread->inSyscall = in_syscall;

    return this->fault;
}

template <class Impl>
Fault
OzoneDynInst<Impl>::completeAcc()
{
    if (this->isLoad()) {
        this->fault = this->staticInst->completeAcc(this->req->data,
                                                    this,
                                                    this->traceData);
    } else if (this->isStore()) {
        this->fault = this->staticInst->completeAcc((uint8_t*)&this->req->result,
                                                    this,
                                                    this->traceData);
    } else {
        panic("Unknown type!");
    }

    return this->fault;
}

template <class Impl>
bool
OzoneDynInst<Impl>::srcInstReady(int regIdx)
{
    return srcInsts[regIdx]->isCompleted();
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
        if (!srcInsts[i]->isCompleted())
            return false;
    }

    return true;
}

template <class Impl>
bool
OzoneDynInst<Impl>::eaSrcsReady()
{
    for (int i = 1; i < this->numSrcRegs(); ++i) {
        if (!srcInsts[i]->isCompleted())
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
MiscReg
OzoneDynInst<Impl>::readMiscReg(int misc_reg)
{
    return this->thread->readMiscReg(misc_reg);
}

template <class Impl>
MiscReg
OzoneDynInst<Impl>::readMiscRegWithEffect(int misc_reg, Fault &fault)
{
    return this->thread->readMiscRegWithEffect(misc_reg, fault);
}

template <class Impl>
Fault
OzoneDynInst<Impl>::setMiscReg(int misc_reg, const MiscReg &val)
{
    return this->thread->setMiscReg(misc_reg, val);
}

template <class Impl>
Fault
OzoneDynInst<Impl>::setMiscRegWithEffect(int misc_reg, const MiscReg &val)
{
    return this->thread->setMiscRegWithEffect(misc_reg, val);
}

#if FULL_SYSTEM

template <class Impl>
Fault
OzoneDynInst<Impl>::hwrei()
{
    if (!this->cpu->inPalMode(this->readPC()))
        return new AlphaISA::UnimplementedOpcodeFault;

    this->setNextPC(this->thread->readMiscReg(AlphaISA::IPR_EXC_ADDR));

    this->cpu->kernelStats->hwrei();

    this->cpu->checkInterrupts = true;

    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
int
OzoneDynInst<Impl>::readIntrFlag()
{
return this->cpu->readIntrFlag();
}

template <class Impl>
void
OzoneDynInst<Impl>::setIntrFlag(int val)
{
    this->cpu->setIntrFlag(val);
}

template <class Impl>
bool
OzoneDynInst<Impl>::inPalMode()
{
    return this->cpu->inPalMode();
}

template <class Impl>
void
OzoneDynInst<Impl>::trap(Fault fault)
{
    fault->invoke(this->thread->getXCProxy());
}

template <class Impl>
bool
OzoneDynInst<Impl>::simPalCheck(int palFunc)
{
    return this->cpu->simPalCheck(palFunc);
}
#else
template <class Impl>
void
OzoneDynInst<Impl>::syscall()
{
    this->cpu->syscall();
}
#endif
