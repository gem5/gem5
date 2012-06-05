/*
 * Copyright (c) 2010-2011 ARM Limited
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
 * Copyright (c) 2004-2006 The Regents of The University of Michigan
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

#include "base/cp_annotate.hh"
#include "cpu/o3/dyn_inst.hh"
#include "sim/full_system.hh"

template <class Impl>
BaseO3DynInst<Impl>::BaseO3DynInst(StaticInstPtr staticInst,
                                   StaticInstPtr macroop,
                                   TheISA::PCState pc, TheISA::PCState predPC,
                                   InstSeqNum seq_num, O3CPU *cpu)
    : BaseDynInst<Impl>(staticInst, macroop, pc, predPC, seq_num, cpu)
{
    initVars();
}

template <class Impl>
BaseO3DynInst<Impl>::BaseO3DynInst(StaticInstPtr _staticInst,
                                   StaticInstPtr _macroop)
    : BaseDynInst<Impl>(_staticInst, _macroop)
{
    initVars();
}

template <class Impl>
void
BaseO3DynInst<Impl>::initVars()
{
    // Make sure to have the renamed register entries set to the same
    // as the normal register entries.  It will allow the IQ to work
    // without any modifications.
    for (int i = 0; i < this->staticInst->numDestRegs(); i++) {
        this->_destRegIdx[i] = this->staticInst->destRegIdx(i);
    }

    for (int i = 0; i < this->staticInst->numSrcRegs(); i++) {
        this->_srcRegIdx[i] = this->staticInst->srcRegIdx(i);
    }

    this->_readySrcRegIdx.reset();

    _numDestMiscRegs = 0;

#if TRACING_ON
    fetchTick = 0;
    decodeTick = 0;
    renameTick = 0;
    dispatchTick = 0;
    issueTick = 0;
    completeTick = 0;
#endif
}

template <class Impl>
Fault
BaseO3DynInst<Impl>::execute()
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool in_syscall = this->thread->inSyscall;
    this->thread->inSyscall = true;

    this->fault = this->staticInst->execute(this, this->traceData);

    this->thread->inSyscall = in_syscall;

    return this->fault;
}

template <class Impl>
Fault
BaseO3DynInst<Impl>::initiateAcc()
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool in_syscall = this->thread->inSyscall;
    this->thread->inSyscall = true;

    this->fault = this->staticInst->initiateAcc(this, this->traceData);

    this->thread->inSyscall = in_syscall;

    return this->fault;
}

template <class Impl>
Fault
BaseO3DynInst<Impl>::completeAcc(PacketPtr pkt)
{
    // @todo: Pretty convoluted way to avoid squashing from happening
    // when using the TC during an instruction's execution
    // (specifically for instructions that have side-effects that use
    // the TC).  Fix this.
    bool in_syscall = this->thread->inSyscall;
    this->thread->inSyscall = true;

    if (this->cpu->checker) {
        if (this->isStoreConditional()) {
            this->reqToVerify->setExtraData(pkt->req->getExtraData());
        }
    }

    this->fault = this->staticInst->completeAcc(pkt, this, this->traceData);

    this->thread->inSyscall = in_syscall;

    return this->fault;
}

template <class Impl>
Fault
BaseO3DynInst<Impl>::hwrei()
{
#if THE_ISA == ALPHA_ISA
    // Can only do a hwrei when in pal mode.
    if (!(this->instAddr() & 0x3))
        return new AlphaISA::UnimplementedOpcodeFault;

    // Set the next PC based on the value of the EXC_ADDR IPR.
    AlphaISA::PCState pc = this->pcState();
    pc.npc(this->cpu->readMiscRegNoEffect(AlphaISA::IPR_EXC_ADDR,
                                          this->threadNumber));
    this->pcState(pc);
    if (CPA::available()) {
        ThreadContext *tc = this->cpu->tcBase(this->threadNumber);
        CPA::cpa()->swAutoBegin(tc, this->nextInstAddr());
    }

    // Tell CPU to clear any state it needs to if a hwrei is taken.
    this->cpu->hwrei(this->threadNumber);
#else

#endif
    // FIXME: XXX check for interrupts? XXX
    return NoFault;
}

template <class Impl>
void
BaseO3DynInst<Impl>::trap(Fault fault)
{
    this->cpu->trap(fault, this->threadNumber, this->staticInst);
}

template <class Impl>
bool
BaseO3DynInst<Impl>::simPalCheck(int palFunc)
{
#if THE_ISA != ALPHA_ISA
    panic("simPalCheck called, but PAL only exists in Alpha!\n");
#endif
    return this->cpu->simPalCheck(palFunc, this->threadNumber);
}

template <class Impl>
void
BaseO3DynInst<Impl>::syscall(int64_t callnum)
{
    if (FullSystem)
        panic("Syscall emulation isn't available in FS mode.\n");

    // HACK: check CPU's nextPC before and after syscall. If it
    // changes, update this instruction's nextPC because the syscall
    // must have changed the nextPC.
    TheISA::PCState curPC = this->cpu->pcState(this->threadNumber);
    this->cpu->syscall(callnum, this->threadNumber);
    TheISA::PCState newPC = this->cpu->pcState(this->threadNumber);
    if (!(curPC == newPC)) {
        this->pcState(newPC);
    }
}

