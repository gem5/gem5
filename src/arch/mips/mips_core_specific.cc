/*
 * Copyright (c) 2002, 2005 The Regents of The University of Michigan
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
 *          Steve Reinhardt
 */

#include "arch/mips/faults.hh"
#include "arch/mips/isa_traits.hh"
#include "arch/mips/tlb.hh"
//#include "base/kgdb.h"
#include "base/remote_gdb.hh"
#include "base/stats/events.hh"
#include "config/full_system.hh"
#include "cpu/base.hh"
#include "cpu/simple_thread.hh"
#include "cpu/thread_context.hh"
#include "kern/kernel_stats.hh"
#include "sim/debug.hh"
#include "sim/sim_exit.hh"
#include "arch/mips/mips_core_specific.hh"

#if FULL_SYSTEM

using namespace MipsCore;

////////////////////////////////////////////////////////////////////////
//
//  Machine dependent functions
//
void
MipsISA::initCPU(ThreadContext *tc, int cpuId)
{

 //    MipsFault *reset = new ResetFault;
//     tc->setPC(reset->vect());
//     tc->setNextPC(tc->readPC() + sizeof(MachInst));

//     delete reset;
}

template <class CPU>
void
MipsISA::processInterrupts(CPU *cpu)
{
    //Check if there are any outstanding interrupts
    //Handle the interrupts
  /*    int ipl = 0;
    int summary = 0;

    cpu->checkInterrupts = false;

    if (cpu->readMiscReg(IPR_ASTRR))
        panic("asynchronous traps not implemented\n");

    if (cpu->readMiscReg(IPR_SIRR)) {
        for (int i = INTLEVEL_SOFTWARE_MIN;
             i < INTLEVEL_SOFTWARE_MAX; i++) {
            if (cpu->readMiscReg(IPR_SIRR) & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = (i - INTLEVEL_SOFTWARE_MIN) + 1;
                summary |= (ULL(1) << i);
            }
        }
    }

    uint64_t interrupts = cpu->intr_status();

    if (interrupts) {
        for (int i = INTLEVEL_EXTERNAL_MIN;
             i < INTLEVEL_EXTERNAL_MAX; i++) {
            if (interrupts & (ULL(1) << i)) {
                // See table 4-19 of the 21164 hardware reference
                ipl = i;
                summary |= (ULL(1) << i);
            }
        }
    }

    if (ipl && ipl > cpu->readMiscReg(IPR_IPLR)) {
        cpu->setMiscReg(IPR_ISR, summary);
        cpu->setMiscReg(IPR_INTID, ipl);
        cpu->trap(new InterruptFault);
        DPRINTF(Flow, "Interrupt! IPLR=%d ipl=%d summary=%x\n",
                cpu->readMiscReg(IPR_IPLR), ipl, summary);
    }
  */
}


/*int
MipsISA::MiscRegFile::getInstAsid()
{
    return EV5::ITB_ASN_ASN(ipr[IPR_ITB_ASN]);
}

int
MipsISA::MiscRegFile::getDataAsid()
{
    return EV5::DTB_ASN_ASN(ipr[IPR_DTB_ASN]);
    }*/



#endif // FULL_SYSTEM || BARE_IRON
