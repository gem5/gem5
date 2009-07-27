/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Gabe Black
 */

#include "arch/arm/isa_traits.hh"
#include "arch/arm/miscregs.hh"
#include "arch/arm/nativetrace.hh"
#include "cpu/thread_context.hh"
#include "params/ArmNativeTrace.hh"

namespace Trace {

static const char *regNames[] = {
    "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
    "r8", "r9", "r10", "fp", "r12", "sp", "lr", "pc",
    "cpsr"
};

void
Trace::ArmNativeTrace::check(NativeTraceRecord *record)
{
    ThreadContext *tc = record->getThread();

    uint32_t regVal, realRegVal;

    const char **regName = regNames;
    // Regular int regs
    for (int i = 0; i < 15; i++) {
        regVal = tc->readIntReg(i);
        read(&realRegVal, sizeof(realRegVal));
        realRegVal = ArmISA::gtoh(realRegVal);
        checkReg(*(regName++), regVal, realRegVal);
    }

    //R15, aliased with the PC
    regVal = tc->readNextPC();
    read(&realRegVal, sizeof(realRegVal));
    realRegVal = ArmISA::gtoh(realRegVal);
    checkReg(*(regName++), regVal, realRegVal);

    //CPSR
    regVal = tc->readMiscReg(MISCREG_CPSR);
    read(&realRegVal, sizeof(realRegVal));
    realRegVal = ArmISA::gtoh(realRegVal);
    checkReg(*(regName++), regVal, realRegVal);
}

} /* namespace Trace */

////////////////////////////////////////////////////////////////////////
//
//  ExeTracer Simulation Object
//
Trace::ArmNativeTrace *
ArmNativeTraceParams::create()
{
    return new Trace::ArmNativeTrace(this);
};
