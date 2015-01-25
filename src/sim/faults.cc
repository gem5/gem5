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
 *
 * Authors: Nathan Binkert
 *          Gabe Black
 */

#include "arch/isa_traits.hh"
#include "base/misc.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Fault.hh"
#include "mem/page_table.hh"
#include "sim/faults.hh"
#include "sim/full_system.hh"
#include "sim/process.hh"

void FaultBase::invoke(ThreadContext * tc, const StaticInstPtr &inst)
{
    if (FullSystem) {
        DPRINTF(Fault, "Fault %s at PC: %s\n", name(), tc->pcState());
    } else {
        panic("fault (%s) detected @ PC %s", name(), tc->pcState());
    }
}

void UnimpFault::invoke(ThreadContext * tc, const StaticInstPtr &inst)
{
    panic("Unimpfault: %s\n", panicStr.c_str());
}

void ReExec::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    tc->pcState(tc->pcState());
}

void GenericPageTableFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    bool handled = false;
    if (!FullSystem) {
        Process *p = tc->getProcessPtr();
        handled = p->fixupStackFault(vaddr);
    }
    if (!handled)
        panic("Page table fault when accessing virtual address %#x\n", vaddr);

}

void GenericAlignmentFault::invoke(ThreadContext *tc, const StaticInstPtr &inst)
{
    panic("Alignment fault when accessing virtual address %#x\n", vaddr);
}
