/*
 * Copyright (c) 2009-2012 ARM Limited
 * All rights reserved.
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
 */


#include "arch/arm/faults.hh"
#include "arch/arm/isa_traits.hh"
#include "arch/arm/tlb.hh"
#include "arch/arm/utility.hh"
#include "arch/arm/vtophys.hh"
#include "cpu/checker/cpu.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "sim/full_system.hh"

namespace ArmISA {

void
initCPU(ThreadContext *tc, int cpuId)
{
    // Reset CP15?? What does that mean -- ali

    // FPEXC.EN = 0

    static Fault reset = new Reset;
    reset->invoke(tc);
}

uint64_t
getArgument(ThreadContext *tc, int &number, uint16_t size, bool fp)
{
    if (!FullSystem) {
        panic("getArgument() only implemented for full system mode.\n");
        M5_DUMMY_RETURN
    }

    if (size == (uint16_t)(-1))
        size = ArmISA::MachineBytes;
    if (fp)
        panic("getArgument(): Floating point arguments not implemented\n");

    if (number < NumArgumentRegs) {
        // If the argument is 64 bits, it must be in an even regiser
        // number. Increment the number here if it isn't even.
        if (size == sizeof(uint64_t)) {
            if ((number % 2) != 0)
                number++;
            // Read the two halves of the data. Number is inc here to
            // get the second half of the 64 bit reg.
            uint64_t tmp;
            tmp = tc->readIntReg(number++);
            tmp |= tc->readIntReg(number) << 32;
            return tmp;
        } else {
           return tc->readIntReg(number);
        }
    } else {
        Addr sp = tc->readIntReg(StackPointerReg);
        FSTranslatingPortProxy &vp = tc->getVirtProxy();
        uint64_t arg;
        if (size == sizeof(uint64_t)) {
            // If the argument is even it must be aligned
            if ((number % 2) != 0)
                number++;
            arg = vp.read<uint64_t>(sp +
                    (number-NumArgumentRegs) * sizeof(uint32_t));
            // since two 32 bit args == 1 64 bit arg, increment number
            number++;
        } else {
            arg = vp.read<uint32_t>(sp +
                           (number-NumArgumentRegs) * sizeof(uint32_t));
        }
        return arg;
    }
}

void
skipFunction(ThreadContext *tc)
{
    PCState newPC = tc->pcState();
    newPC.set(tc->readIntReg(ReturnAddressReg) & ~ULL(1));

    CheckerCPU *checker = tc->getCheckerCpuPtr();
    if (checker) {
        tc->pcStateNoRecord(newPC);
    } else {
        tc->pcState(newPC);
    }
}

void
copyRegs(ThreadContext *src, ThreadContext *dest)
{
    for (int i = 0; i < NumIntRegs; i++)
        dest->setIntRegFlat(i, src->readIntRegFlat(i));

    for (int i = 0; i < NumFloatRegs; i++)
        dest->setFloatRegFlat(i, src->readFloatRegFlat(i));

    // Would need to add condition-code regs if implemented
    assert(NumCCRegs == 0);

    for (int i = 0; i < NumMiscRegs; i++)
        dest->setMiscRegNoEffect(i, src->readMiscRegNoEffect(i));

    // setMiscReg "with effect" will set the misc register mapping correctly.
    // e.g. updateRegMap(val)
    dest->setMiscReg(MISCREG_CPSR, src->readMiscRegNoEffect(MISCREG_CPSR));

    // Copy over the PC State
    dest->pcState(src->pcState());

    // Invalidate the tlb misc register cache
    dest->getITBPtr()->invalidateMiscReg();
    dest->getDTBPtr()->invalidateMiscReg();
}

Addr
truncPage(Addr addr)
{
    return addr & ~(PageBytes - 1);
}

Addr
roundPage(Addr addr)
{
    return (addr + PageBytes - 1) & ~(PageBytes - 1);
}

} // namespace ArmISA
