/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * Redistribution and use of this software in source and binary forms,
 * with or without modification, are permitted provided that the
 * following conditions are met:
 *
 * The software must be used only for Non-Commercial Use which means any
 * use which is NOT directed to receiving any direct monetary
 * compensation for, or commercial advantage from such use.  Illustrative
 * examples of non-commercial use are academic research, personal study,
 * teaching, education and corporate research & development.
 * Illustrative examples of commercial use are distributing products for
 * commercial advantage and providing services using the software for
 * commercial advantage.
 *
 * If you wish to use this software or functionality therein that may be
 * covered by patents for commercial use, please contact:
 *     Director of Intellectual Property Licensing
 *     Office of Strategy and Technology
 *     Hewlett-Packard Company
 *     1501 Page Mill Road
 *     Palo Alto, California  94304
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  Redistributions
 * in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.  Neither the name of
 * the COPYRIGHT HOLDER(s), HEWLETT-PACKARD COMPANY, nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.  No right of
 * sublicense is granted herewith.  Derivatives of the software and
 * output created using the software may be prepared, but only for
 * Non-Commercial Uses.  Derivatives of the software may be shared with
 * others provided: (i) the others agree to abide by the list of
 * conditions herein which includes the Non-Commercial Use restrictions;
 * and (ii) such Derivatives of the software include the above copyright
 * notice to acknowledge the contribution from this software where
 * applicable, this list of conditions and the disclaimer below.
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

#include "arch/x86/intregs.hh"
#include "arch/x86/miscregs.hh"
#include "arch/x86/segmentregs.hh"
#include "arch/x86/utility.hh"
#include "arch/x86/x86_traits.hh"

namespace X86ISA {

uint64_t getArgument(ThreadContext *tc, int number, bool fp) {
#if FULL_SYSTEM
    panic("getArgument() not implemented for x86!\n");
#else
    panic("getArgument() only implemented for FULL_SYSTEM\n");
    M5_DUMMY_RETURN
#endif
}

# if FULL_SYSTEM
void initCPU(ThreadContext *tc, int cpuId)
{
    // TODO Figure out what the attribute registers should be set to. How this
    // information is stored isn't specified, but it's values are in table
    // 14.2.

    // The otherwise unmodified integer registers should be set to 0.
    for (int index = 0; index < NUM_INTREGS; index++) {
        tc->setIntReg(index, 0);
    }

    // These next two loops zero internal microcode and implicit registers.
    // They aren't specified by the ISA but are used internally by M5's
    // implementation.
    for (int index = 0; index < NumMicroIntRegs; index++) {
        tc->setIntReg(INTREG_MICRO(index), 0);
    }

    for (int index = 0; index < NumImplicitIntRegs; index++) {
        tc->setIntReg(INTREG_IMPLICIT(index), 0);
    }

    // Set integer register EAX to 0 to indicate that the optional BIST
    // passed. No BIST actually runs, but software may still check this
    // register for errors.
    tc->setIntReg(INTREG_RAX, 0);

    //The following values are dictated by the architecture for after a RESET#
    tc->setMiscReg(MISCREG_CR0, 0x0000000060000010);
    tc->setMiscReg(MISCREG_CR2, 0);
    tc->setMiscReg(MISCREG_CR3, 0);
    tc->setMiscReg(MISCREG_CR4, 0);
    tc->setMiscReg(MISCREG_CR8, 0);

    tc->setMiscReg(MISCREG_RFLAGS, 0x0000000000000002);

    tc->setMiscReg(MISCREG_EFER, 0);

    for (int seg = 0; seg != NUM_SEGMENTREGS; seg++) {
        tc->setMiscReg(MISCREG_SEG_SEL(seg), 0);
        tc->setMiscReg(MISCREG_SEG_BASE(seg), 0);
        tc->setMiscReg(MISCREG_SEG_LIMIT(seg), 0xffff);
        tc->setMiscReg(MISCREG_SEG_ATTR(seg), 0);
    }

    tc->setMiscReg(MISCREG_CS, 0xf000);
    tc->setMiscReg(MISCREG_CS_BASE, 0x00000000ffff0000);
    // This has the base value pre-added.
    tc->setMiscReg(MISCREG_CS_LIMIT, 0xffffffff);
    tc->setMiscReg(MISCREG_CS_ATTR, 0);

    tc->setPC(0x000000000000fff0 +
            tc->readMiscReg(MISCREG_CS_BASE));
    tc->setNextPC(tc->readPC() + sizeof(MachInst));

    tc->setMiscReg(MISCREG_GDTR_BASE, 0);
    tc->setMiscReg(MISCREG_GDTR_LIMIT, 0xffff);

    tc->setMiscReg(MISCREG_IDTR_BASE, 0);
    tc->setMiscReg(MISCREG_IDTR_LIMIT, 0xffff);

    tc->setMiscReg(MISCREG_LDTR, 0);
    tc->setMiscReg(MISCREG_LDTR_BASE, 0);
    tc->setMiscReg(MISCREG_LDTR_LIMIT, 0xffff);
    tc->setMiscReg(MISCREG_LDTR_ATTR, 0);

    tc->setMiscReg(MISCREG_TR, 0);
    tc->setMiscReg(MISCREG_TR_BASE, 0);
    tc->setMiscReg(MISCREG_TR_LIMIT, 0xffff);
    tc->setMiscReg(MISCREG_TR_ATTR, 0);

    // This value should be the family/model/stepping of the processor.
    // (page 418). It should be consistent with the value from CPUID, but the
    // actual value probably doesn't matter much.
    tc->setIntReg(INTREG_RDX, 0);

    // TODO initialize x87, 64 bit, and 128 bit media state

    // TODO Set up MTRRs (page 512)

    // TODO Set up machine check registers (page 515)

    tc->setMiscReg(MISCREG_DR0, 0);
    tc->setMiscReg(MISCREG_DR1, 0);
    tc->setMiscReg(MISCREG_DR2, 0);
    tc->setMiscReg(MISCREG_DR3, 0);

    tc->setMiscReg(MISCREG_DR6, 0x00000000ffff0ff0);
    tc->setMiscReg(MISCREG_DR7, 0x0000000000000400);

    // TODO Set time stamp counter to 0

    // TODO Set up performance monitoring registers (page 517)

    // TODO Set up the rest of the MSRs (page 507)

    // Invalidate the caches (this should already be done for us)

    // TODO Turn on the APIC. This should be handled elsewhere but it isn't
    // currently being handled at all.

    // Set the SMRAM base address (SMBASE) to 0x00030000
}

#endif

void startupCPU(ThreadContext *tc, int cpuId)
{
    if (cpuId == 0) {
        // This is the boot strap processor (BSP). Initialize it to look like
        // the boot loader has just turned control over to the 64 bit OS.

        // Enable paging, turn on long mode, etc.

        tc->activate(0);
    } else {
        // This is an application processor (AP). It should be initialized to
        // look like only the BIOS POST has run on it and put then put it into
        // a halted state.
    }
}

} //namespace X86_ISA
