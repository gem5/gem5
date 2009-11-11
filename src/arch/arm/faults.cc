/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 *
 * Authors: Ali Saidi
 *          Gabe Black
 */

#include "arch/arm/faults.hh"
#include "cpu/thread_context.hh"
#include "cpu/base.hh"
#include "base/trace.hh"

namespace ArmISA
{

template<> ArmFaultBase::FaultVals ArmFault<Reset>::vals = 
    {"reset", 0x00, MODE_SVC, 0, 0, true, true};

template<> ArmFaultBase::FaultVals ArmFault<UndefinedInstruction>::vals = 
    {"Undefined Instruction", 0x04, MODE_UNDEFINED, 4 ,2, false, false} ;

template<> ArmFaultBase::FaultVals ArmFault<SupervisorCall>::vals = 
    {"Supervisor Call", 0x08, MODE_SVC, 4, 2, false, false};

template<> ArmFaultBase::FaultVals ArmFault<PrefetchAbort>::vals = 
    {"Prefetch Abort", 0x0C, MODE_ABORT, 4, 4, true, false};

template<> ArmFaultBase::FaultVals ArmFault<DataAbort>::vals = 
    {"Data Abort", 0x10, MODE_ABORT, 8, 8, true, false};

template<> ArmFaultBase::FaultVals ArmFault<Interrupt>::vals = 
    {"IRQ", 0x18, MODE_IRQ, 4, 4, true, false};

template<> ArmFaultBase::FaultVals ArmFault<FastInterrupt>::vals = 
    {"FIQ", 0x1C, MODE_FIQ, 4, 4, true, true};

Addr 
ArmFaultBase::getVector(ThreadContext *tc)
{
    // ARM ARM B1-3

    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
    
    // panic if SCTLR.VE because I have no idea what to do with vectored
    // interrupts
    assert(!sctlr.ve);
    
    if (!sctlr.v)
        return offset();
    return offset() + HighVecs;

}

#if FULL_SYSTEM

void 
ArmFaultBase::invoke(ThreadContext *tc)
{
    // ARM ARM B1.6.3
    FaultBase::invoke(tc);
    countStat()++;

    SCTLR sctlr = tc->readMiscReg(MISCREG_SCTLR);
    CPSR cpsr = tc->readMiscReg(MISCREG_CPSR);
    CPSR saved_cpsr = tc->readMiscReg(MISCREG_CPSR) | 
                      tc->readIntReg(INTREG_CONDCODES);
 

    cpsr.mode = nextMode();
    cpsr.it1 = cpsr.it2 = 0;
    cpsr.j = 0;
   
    if (sctlr.te)
       cpsr.t = 1;
    cpsr.a = cpsr.a | abortDisable();
    cpsr.f = cpsr.f | fiqDisable();
    cpsr.i = 1;
    tc->setMiscReg(MISCREG_CPSR, cpsr);
    tc->setIntReg(INTREG_LR, tc->readPC() + 
            (saved_cpsr.t ? thumbPcOffset() : armPcOffset()));

    switch (nextMode()) {
      case MODE_FIQ:
        tc->setMiscReg(MISCREG_SPSR_FIQ, saved_cpsr);
        break;
      case MODE_IRQ:
        tc->setMiscReg(MISCREG_SPSR_IRQ, saved_cpsr);
        break;
      case MODE_SVC:
        tc->setMiscReg(MISCREG_SPSR_SVC, saved_cpsr);
        break;
      case MODE_UNDEFINED:
        tc->setMiscReg(MISCREG_SPSR_UND, saved_cpsr);
        break;
      case MODE_ABORT:
        tc->setMiscReg(MISCREG_SPSR_ABT, saved_cpsr);
        break;
      default:
        panic("unknown Mode\n");
    } 
   
    DPRINTF(Faults, "Invoking Fault: %s cpsr: %#x PC: %#x lr: %#x\n", name(), cpsr,
            tc->readPC(), tc->readIntReg(INTREG_LR)); 
    tc->setPC(getVector(tc));
    tc->setNextPC(getVector(tc) + cpsr.t ? 2 : 4 );
}
#endif // FULL_SYSTEM

// return via SUBS pc, lr, xxx; rfe, movs, ldm



} // namespace ArmISA

