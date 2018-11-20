/*
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2007 MIPS Technologies, Inc.
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
 * Authors: Steve Reinhardt
 *          Kevin Lim
 *          Korey Sewell
 */

#include "arch/mips/interrupts.hh"

#include "arch/mips/isa_traits.hh"
#include "arch/mips/pra_constants.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Interrupt.hh"

namespace MipsISA
{

static inline uint8_t
getCauseIP(ThreadContext *tc) {
    CauseReg cause = tc->readMiscRegNoEffect(MISCREG_CAUSE);
    return cause.ip;
}

static inline void
setCauseIP(ThreadContext *tc, uint8_t val) {
    CauseReg cause = tc->readMiscRegNoEffect(MISCREG_CAUSE);
    cause.ip = val;
    tc->setMiscRegNoEffect(MISCREG_CAUSE, cause);
}

void
Interrupts::post(int int_num, ThreadContext* tc)
{
    DPRINTF(Interrupt, "Interrupt %d posted\n", int_num);
    if (int_num < 0 || int_num >= NumInterruptLevels)
        panic("int_num out of bounds\n");

    uint8_t intstatus = getCauseIP(tc);
    intstatus |= 1 << int_num;
    setCauseIP(tc, intstatus);
}

void
Interrupts::post(int int_num, int index)
{
    fatal("Must use Thread Context when posting MIPS Interrupts in M5");
}

void
Interrupts::clear(int int_num, ThreadContext* tc)
{
    DPRINTF(Interrupt, "Interrupt %d cleared\n", int_num);
    if (int_num < 0 || int_num >= NumInterruptLevels)
        panic("int_num out of bounds\n");

    uint8_t intstatus = getCauseIP(tc);
    intstatus &= ~(1 << int_num);
    setCauseIP(tc, intstatus);
}

void
Interrupts::clear(int int_num, int index)
{
    fatal("Must use Thread Context when clearing MIPS Interrupts in M5");
}

void
Interrupts::clearAll(ThreadContext *tc)
{
    DPRINTF(Interrupt, "Interrupts all cleared\n");
    uint8_t intstatus = 0;
    setCauseIP(tc, intstatus);
}

void
Interrupts::clearAll()
{
    fatal("Must use Thread Context when clearing MIPS Interrupts in M5");
}


bool
Interrupts::checkInterrupts(ThreadContext *tc) const
{
    if (!interruptsPending(tc))
        return false;

    //Check if there are any outstanding interrupts
    StatusReg status = tc->readMiscRegNoEffect(MISCREG_STATUS);
    // Interrupts must be enabled, error level must be 0 or interrupts
    // inhibited, and exception level must be 0 or interrupts inhibited
    if ((status.ie == 1) && (status.erl == 0) && (status.exl == 0)) {
        // Software interrupts & hardware interrupts are handled in software.
        // So if any interrupt that isn't masked is detected, jump to interrupt
        // handler
        CauseReg cause = tc->readMiscRegNoEffect(MISCREG_CAUSE);
        if (status.im && cause.ip)
            return true;

    }

    return false;
}

Fault
Interrupts::getInterrupt(ThreadContext * tc)
{
    assert(checkInterrupts(tc));

    StatusReg M5_VAR_USED status = tc->readMiscRegNoEffect(MISCREG_STATUS);
    CauseReg M5_VAR_USED cause = tc->readMiscRegNoEffect(MISCREG_CAUSE);
    DPRINTF(Interrupt, "Interrupt! IM[7:0]=%d IP[7:0]=%d \n",
            (unsigned)status.im, (unsigned)cause.ip);

    return std::make_shared<InterruptFault>();
}

bool
Interrupts::onCpuTimerInterrupt(ThreadContext * tc) const
{
    RegVal compare = tc->readMiscRegNoEffect(MISCREG_COMPARE);
    RegVal count = tc->readMiscRegNoEffect(MISCREG_COUNT);
    if (compare == count && count != 0)
        return true;
    return false;
}

void
Interrupts::updateIntrInfo(ThreadContext *tc) const
{
    //Nothing needs to be done.
}

bool
Interrupts::interruptsPending(ThreadContext *tc) const
{
    //if there is a on cpu timer interrupt (i.e. Compare == Count)
    //update CauseIP before proceeding to interrupt
    if (onCpuTimerInterrupt(tc)) {
        DPRINTF(Interrupt, "Interrupts OnCpuTimerINterrupt(tc) == true\n");
        //determine timer interrupt IP #
        IntCtlReg intCtl = tc->readMiscRegNoEffect(MISCREG_INTCTL);
        uint8_t intStatus = getCauseIP(tc);
        intStatus |= 1 << intCtl.ipti;
        setCauseIP(tc, intStatus);
    }

    return (getCauseIP(tc) != 0);

}

}

MipsISA::Interrupts *
MipsInterruptsParams::create()
{
    return new MipsISA::Interrupts(this);
}
