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

#include "arch/mips/pra_constants.hh"
#include "arch/mips/isa_traits.hh"
#include "cpu/thread_context.hh"
#include "arch/mips/interrupts.hh"

namespace MipsISA
{

static inline uint8_t
getCauseIP(ThreadContext *tc) {
    MiscReg cause = tc->readMiscRegNoEffect(MipsISA::Cause);
    return bits(cause, Cause_IP7, Cause_IP0);
}

static inline void
setCauseIP_(ThreadContext *tc, uint8_t val) {
    MiscReg cause = tc->readMiscRegNoEffect(MipsISA::Cause);
    replaceBits(cause, Cause_IP7, Cause_IP0, val);
    tc->setMiscRegNoEffect(MipsISA::Cause, cause);
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



Fault
Interrupts::getInterrupt(ThreadContext * tc)
{
    DPRINTF(Interrupt, "Interrupts getInterrupt\n");

    //Check if there are any outstanding interrupts
    MiscReg status = tc->readMiscRegNoEffect(MipsISA::Status);
    // Interrupts must be enabled, error level must be 0 or interrupts
    // inhibited, and exception level must be 0 or interrupts inhibited
    if (bits(status, Status_IE_LO) == 1 &&
        bits(status, Status_ERL_HI, Status_ERL_LO) == 0 &&
        bits(status, Status_EXL_HI, Status_EXL_LO) == 0) {
        // Software interrupts & hardware interrupts are handled in software.
        // So if any interrupt that isn't masked is detected, jump to interrupt
        // handler
        uint8_t InterruptMask = bits(status, Status_IM7, Status_IM0);
        uint8_t InterruptPending = getCauseIP(tc);
        // InterruptMask and InterruptPending are already correctly aligned
        if (InterruptMask && InterruptPending){
            DPRINTF(Interrupt, "Interrupt! IM[7:0]=%d IP[7:0]=%d \n",
                    InterruptMask, InterruptPending);
            return new InterruptFault;
        }
    }

    return NoFault;
}

bool
Interrupts::onCpuTimerInterrupt(ThreadContext * tc) const
{
    MiscReg compare = tc->readMiscRegNoEffect(MipsISA::Compare);
    MiscReg count = tc->readMiscRegNoEffect(MipsISA::Count);
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
        MiscReg intctl = tc->readMiscRegNoEffect(MipsISA::IntCtl);
        uint8_t IPTI = bits(intctl, IntCtl_IPTI_HI, IntCtl_IPTI_LO);
        //set intstatus to correspond
        //post(IPTI, tc);
        uint8_t intstatus = getCauseIP(tc);
        intstatus |= 1 << IPTI;
        setCauseIP(tc, intstatus);
    }

    return (getCauseIP(tc) != 0);

}

}
