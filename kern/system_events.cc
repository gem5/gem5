/*
 * Copyright (c) 2004 The Regents of The University of Michigan
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

#include "cpu/exec_context.hh"
#include "cpu/base_cpu.hh"
#include "cpu/full_cpu/bpred.hh"
#include "cpu/full_cpu/full_cpu.hh"
#include "kern/system_events.hh"
#include "sim/system.hh"
#include "sim/sw_context.hh"

void
SkipFuncEvent::process(ExecContext *xc)
{
    Addr newpc = xc->regs.intRegFile[ReturnAddressReg];

    DPRINTF(PCEvent, "skipping %s: pc=%x, newpc=%x\n", description,
            xc->regs.pc, newpc);

    xc->regs.pc = newpc;
    xc->regs.npc = xc->regs.pc + sizeof(MachInst);

    BranchPred *bp = xc->cpu->getBranchPred();
    if (bp != NULL) {
        bp->popRAS(xc->thread_num);
    }
}


FnEvent::FnEvent(PCEventQueue *q, const std::string & desc, System *system)
    : PCEvent(q, desc), _name(desc)
{
    myBin = system->getBin(desc);
    assert(myBin);
}

void
FnEvent::process(ExecContext *xc)
{
    if (xc->misspeculating())
        return;
    assert(xc->system->bin && "FnEvent must be in a binned system");
    SWContext *ctx = xc->swCtx;
    DPRINTF(TCPIP, "%s: %s Event!!!\n", xc->system->name(), description);

    if (ctx && !ctx->callStack.empty()) {
        DPRINTF(TCPIP, "already a callstack!\n");
        fnCall *last = ctx->callStack.top();

        if (last->name == "idle_thread")
            ctx->calls++;

        if (!xc->system->findCaller(myname(), "" ) &&
            !xc->system->findCaller(myname(), last->name)) {

            DPRINTF(TCPIP, "but can't find parent %s\n", last->name);
            return;
        }
        ctx->calls--;

        //assert(!ctx->calls && "on a binned fn, calls should == 0 (but can happen in boot)");
    } else {
        DPRINTF(TCPIP, "no callstack yet\n");
        if (!xc->system->findCaller(myname(), "")) {
            DPRINTF(TCPIP, "not the right function, returning\n");
            return;
        }
        if (!ctx)  {
            DPRINTF(TCPIP, "creating new context for %s\n", myname());
            ctx = new SWContext;
            xc->swCtx = ctx;
        }
    }
    DPRINTF(TCPIP, "adding fn %s to context\n", myname());
    fnCall *call = new fnCall;
    call->myBin = myBin;
    call->name = myname();
    ctx->callStack.push(call);
    myBin->activate();
    xc->system->fnCalls++;
    DPRINTF(TCPIP, "fnCalls for %s is %d\n", description,
            xc->system->fnCalls.value());
    xc->system->dumpState(xc);
}
