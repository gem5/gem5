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
 */

#include "arch/mips/interrupts.hh"

#include "arch/mips/pra_constants.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Interrupt.hh"

namespace gem5
{

namespace MipsISA
{

enum InterruptLevels
{
    INTLEVEL_SOFTWARE_MIN = 4,
    INTLEVEL_SOFTWARE_MAX = 19,

    INTLEVEL_EXTERNAL_MIN = 20,
    INTLEVEL_EXTERNAL_MAX = 34,

    INTLEVEL_IRQ0 = 20,
    INTLEVEL_IRQ1 = 21,
    INTINDEX_ETHERNET = 0,
    INTINDEX_SCSI = 1,
    INTLEVEL_IRQ2 = 22,
    INTLEVEL_IRQ3 = 23,

    INTLEVEL_SERIAL = 33,

    NumInterruptLevels = INTLEVEL_EXTERNAL_MAX
};

static inline uint8_t
getCauseIP(ThreadContext *tc)
{
    CauseReg cause = tc->readMiscRegNoEffect(misc_reg::Cause);
    return cause.ip;
}

static inline void
setCauseIP(ThreadContext *tc, uint8_t val)
{
    CauseReg cause = tc->readMiscRegNoEffect(misc_reg::Cause);
    cause.ip = val;
    tc->setMiscRegNoEffect(misc_reg::Cause, cause);
}

void
Interrupts::post(int int_num)
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
Interrupts::clear(int int_num)
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
Interrupts::clearAll()
{
    DPRINTF(Interrupt, "Interrupts all cleared\n");
    uint8_t intstatus = 0;
    setCauseIP(tc, intstatus);
}

bool
Interrupts::checkInterrupts() const
{
    if (!interruptsPending())
        return false;

    // Check if there are any outstanding interrupts
    StatusReg status = tc->readMiscRegNoEffect(misc_reg::Status);
    // Interrupts must be enabled, error level must be 0 or interrupts
    // inhibited, and exception level must be 0 or interrupts inhibited
    if ((status.ie == 1) && (status.erl == 0) && (status.exl == 0)) {
        // Software interrupts & hardware interrupts are handled in software.
        // So if any interrupt that isn't masked is detected, jump to interrupt
        // handler
        CauseReg cause = tc->readMiscRegNoEffect(misc_reg::Cause);
        if (status.im && cause.ip)
            return true;
    }

    return false;
}

Fault
Interrupts::getInterrupt()
{
    assert(checkInterrupts());

    [[maybe_unused]] StatusReg status =
        tc->readMiscRegNoEffect(misc_reg::Status);
    [[maybe_unused]] CauseReg cause = tc->readMiscRegNoEffect(misc_reg::Cause);
    DPRINTF(Interrupt, "Interrupt! IM[7:0]=%d IP[7:0]=%d \n",
            (unsigned)status.im, (unsigned)cause.ip);

    return std::make_shared<InterruptFault>();
}

bool
Interrupts::onCpuTimerInterrupt() const
{
    RegVal compare = tc->readMiscRegNoEffect(misc_reg::Compare);
    RegVal count = tc->readMiscRegNoEffect(misc_reg::Count);
    if (compare == count && count != 0)
        return true;
    return false;
}

void
Interrupts::updateIntrInfo()
{} // Nothing needs to be done.

bool
Interrupts::interruptsPending() const
{
    // if there is a on cpu timer interrupt (i.e. Compare == Count)
    // update CauseIP before proceeding to interrupt
    if (onCpuTimerInterrupt()) {
        DPRINTF(Interrupt, "Interrupts OnCpuTimerInterrupt() == true\n");
        // determine timer interrupt IP #
        IntCtlReg intCtl = tc->readMiscRegNoEffect(misc_reg::Intctl);
        uint8_t intStatus = getCauseIP(tc);
        intStatus |= 1 << intCtl.ipti;
        setCauseIP(tc, intStatus);
    }

    return (getCauseIP(tc) != 0);
}

} // namespace MipsISA
} // namespace gem5
