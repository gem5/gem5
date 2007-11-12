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
#include "sim/system.hh"

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

    SegAttr dataAttr = 0;
    dataAttr.writable = 1;
    dataAttr.readable = 1;
    dataAttr.expandDown = 0;
    dataAttr.dpl = 0;
    dataAttr.defaultSize = 0;

    for (int seg = 0; seg != NUM_SEGMENTREGS; seg++) {
        tc->setMiscReg(MISCREG_SEG_SEL(seg), 0);
        tc->setMiscReg(MISCREG_SEG_BASE(seg), 0);
        tc->setMiscReg(MISCREG_SEG_LIMIT(seg), 0xffff);
        tc->setMiscReg(MISCREG_SEG_ATTR(seg), dataAttr);
    }

    SegAttr codeAttr = 0;
    codeAttr.writable = 0;
    codeAttr.readable = 1;
    codeAttr.expandDown = 0;
    codeAttr.dpl = 0;
    codeAttr.defaultSize = 0;

    tc->setMiscReg(MISCREG_CS, 0xf000);
    tc->setMiscReg(MISCREG_CS_BASE, 0x00000000ffff0000);
    // This has the base value pre-added.
    tc->setMiscReg(MISCREG_CS_LIMIT, 0xffffffff);
    tc->setMiscReg(MISCREG_CS_ATTR, codeAttr);

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

    tc->setMiscReg(MISCREG_MTRRCAP, 0x0508);
    for (int i = 0; i < 8; i++) {
        tc->setMiscReg(MISCREG_MTRR_PHYS_BASE(i), 0);
        tc->setMiscReg(MISCREG_MTRR_PHYS_MASK(i), 0);
    }
    tc->setMiscReg(MISCREG_MTRR_FIX_64K_00000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_16K_80000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_16K_A0000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_C0000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_C8000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_D0000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_D8000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_E0000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_E8000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_F0000, 0);
    tc->setMiscReg(MISCREG_MTRR_FIX_4K_F8000, 0);

    tc->setMiscReg(MISCREG_DEF_TYPE, 0);

    tc->setMiscReg(MISCREG_MCG_CAP, 0x104);
    tc->setMiscReg(MISCREG_MCG_STATUS, 0);
    tc->setMiscReg(MISCREG_MCG_CTL, 0);

    for (int i = 0; i < 5; i++) {
        tc->setMiscReg(MISCREG_MC_CTL(i), 0);
        tc->setMiscReg(MISCREG_MC_STATUS(i), 0);
        tc->setMiscReg(MISCREG_MC_ADDR(i), 0);
        tc->setMiscReg(MISCREG_MC_MISC(i), 0);
    }

    tc->setMiscReg(MISCREG_DR0, 0);
    tc->setMiscReg(MISCREG_DR1, 0);
    tc->setMiscReg(MISCREG_DR2, 0);
    tc->setMiscReg(MISCREG_DR3, 0);

    tc->setMiscReg(MISCREG_DR6, 0x00000000ffff0ff0);
    tc->setMiscReg(MISCREG_DR7, 0x0000000000000400);

    tc->setMiscReg(MISCREG_TSC, 0);
    tc->setMiscReg(MISCREG_TSC_AUX, 0);

    for (int i = 0; i < 4; i++) {
        tc->setMiscReg(MISCREG_PERF_EVT_SEL(i), 0);
        tc->setMiscReg(MISCREG_PERF_EVT_CTR(i), 0);
    }

    tc->setMiscReg(MISCREG_STAR, 0);
    tc->setMiscReg(MISCREG_LSTAR, 0);
    tc->setMiscReg(MISCREG_CSTAR, 0);

    tc->setMiscReg(MISCREG_SF_MASK, 0);

    tc->setMiscReg(MISCREG_KERNEL_GS_BASE, 0);

    tc->setMiscReg(MISCREG_SYSENTER_CS, 0);
    tc->setMiscReg(MISCREG_SYSENTER_ESP, 0);
    tc->setMiscReg(MISCREG_SYSENTER_EIP, 0);

    tc->setMiscReg(MISCREG_PAT, 0x0007040600070406);

    tc->setMiscReg(MISCREG_SYSCFG, 0x20601);

    tc->setMiscReg(MISCREG_IORR_BASE0, 0);
    tc->setMiscReg(MISCREG_IORR_BASE1, 0);

    tc->setMiscReg(MISCREG_IORR_MASK0, 0);
    tc->setMiscReg(MISCREG_IORR_MASK1, 0);

    tc->setMiscReg(MISCREG_TOP_MEM, 0x4000000);
    tc->setMiscReg(MISCREG_TOP_MEM2, 0x0);

    tc->setMiscReg(MISCREG_DEBUG_CTL_MSR, 0);
    tc->setMiscReg(MISCREG_LAST_BRANCH_FROM_IP, 0);
    tc->setMiscReg(MISCREG_LAST_BRANCH_TO_IP, 0);
    tc->setMiscReg(MISCREG_LAST_EXCEPTION_FROM_IP, 0);
    tc->setMiscReg(MISCREG_LAST_EXCEPTION_TO_IP, 0);

    // Invalidate the caches (this should already be done for us)

    // TODO Turn on the APIC. This should be handled elsewhere but it isn't
    // currently being handled at all.

    // TODO Set the SMRAM base address (SMBASE) to 0x00030000

    tc->setMiscReg(MISCREG_VM_CR, 0);
    tc->setMiscReg(MISCREG_IGNNE, 0);
    tc->setMiscReg(MISCREG_SMM_CTL, 0);
    tc->setMiscReg(MISCREG_VM_HSAVE_PA, 0);
}

#endif

#if FULL_SYSTEM
void startupCPU(ThreadContext *tc, int cpuId)
{
    if (cpuId == 0) {
        // This is the boot strap processor (BSP). Initialize it to look like
        // the boot loader has just turned control over to the 64 bit OS. We
        // won't actually set up real mode or legacy protected mode descriptor
        // tables because we aren't executing any code that would require
        // them. We do, however toggle the control bits in the correct order
        // while allowing consistency checks and the underlying mechansims
        // just to be safe.

        const int NumPDTs = 4;

        const Addr PageMapLevel4 = 0x70000;
        const Addr PageDirPtrTable = 0x71000;
        const Addr PageDirTable[NumPDTs] =
            {0x72000, 0x73000, 0x74000, 0x75000};
        const Addr GDTBase = 0x76000;

        const int PML4Bits = 9;
        const int PDPTBits = 9;
        const int PDTBits = 9;

        // Get a port to write the page tables and descriptor tables.
        FunctionalPort * physPort = tc->getPhysPort();

        /*
         * Set up the gdt.
         */
        // Place holder at selector 0
        uint64_t nullDescriptor = 0;
        physPort->writeBlob(GDTBase, (uint8_t *)(&nullDescriptor), 8);

        //64 bit code segment
        SegDescriptor csDesc = 0;
        csDesc.type.c = 0; // Not conforming
        csDesc.dpl = 0; // Privelege level 0
        csDesc.p = 1; // Present
        csDesc.l = 1; // 64 bit
        csDesc.d = 0; // default operand size
        //Because we're dealing with a pointer and I don't think it's
        //guaranteed that there isn't anything in a nonvirtual class between
        //it's beginning in memory and it's actual data, we'll use an
        //intermediary.
        uint64_t csDescVal = csDesc;
        physPort->writeBlob(GDTBase, (uint8_t *)(&csDescVal), 8);

        tc->setMiscReg(MISCREG_GDTR_BASE, GDTBase);
        tc->setMiscReg(MISCREG_GDTR_LIMIT, 0xF);

        /*
         * Identity map the first 4GB of memory. In order to map this region
         * of memory in long mode, there needs to be one actual page map level
         * 4 entry which points to one page directory pointer table which
         * points to 4 different page directory tables which are full of two
         * megabyte pages. All of the other entries in valid tables are set
         * to indicate that they don't pertain to anything valid and will
         * cause a fault if used.
         */

        // Put valid values in all of the various table entries which indicate
        // that those entries don't point to further tables or pages. Then
        // set the values of those entries which are needed.

        // Page Map Level 4

        // read/write, user, not present
        uint64_t pml4e = X86ISA::htog(0x6);
        for (int offset = 0; offset < (1 << PML4Bits) * 8; offset += 8) {
            physPort->writeBlob(PageMapLevel4 + offset, (uint8_t *)(&pml4e), 8);
        }
        // Point to the only PDPT
        pml4e = X86ISA::htog(0x7 | PageDirPtrTable);
        physPort->writeBlob(PageMapLevel4, (uint8_t *)(&pml4e), 8);

        // Page Directory Pointer Table

        // read/write, user, not present
        uint64_t pdpe = X86ISA::htog(0x6);
        for (int offset = 0; offset < (1 << PDPTBits) * 8; offset += 8) {
            physPort->writeBlob(PageDirPtrTable + offset,
                    (uint8_t *)(&pdpe), 8);
        }
        // Point to the PDTs
        for (int table = 0; table < NumPDTs; table++) {
            pdpe = X86ISA::htog(0x7 | PageDirTable[table]);
            physPort->writeBlob(PageDirPtrTable + table * 8,
                    (uint8_t *)(&pdpe), 8);
        }

        // Page Directory Tables

        Addr base = 0;
        const Addr pageSize = 2 << 20;
        for (int table = 0; table < NumPDTs; table++) {
            for (int offset = 0; offset < (1 << PDTBits) * 8; offset += 8) {
                // read/write, user, present, 4MB
                uint64_t pdte = X86ISA::htog(0x87 | base);
                physPort->writeBlob(PageDirTable[table] + offset,
                        (uint8_t *)(&pdte), 8);
                base += pageSize;
            }
        }

        /*
         * Transition from real mode all the way up to Long mode
         */
        CR0 cr0 = tc->readMiscRegNoEffect(MISCREG_CR0);
        //Turn off paging.
        cr0.pg = 0;
        tc->setMiscReg(MISCREG_CR0, cr0);
        //Turn on protected mode.
        cr0.pe = 1;
        tc->setMiscReg(MISCREG_CR0, cr0);

        CR4 cr4 = tc->readMiscRegNoEffect(MISCREG_CR4);
        //Turn on pae.
        cr4.pae = 1;
        tc->setMiscReg(MISCREG_CR4, cr4);

        //Point to the page tables.
        tc->setMiscReg(MISCREG_CR3, PageMapLevel4);

        Efer efer = tc->readMiscRegNoEffect(MISCREG_EFER);
        //Enable long mode.
        efer.lme = 1;
        tc->setMiscReg(MISCREG_EFER, efer);

        //Activate long mode.
        cr0.pg = 1;
        tc->setMiscReg(MISCREG_CR0, cr0);

        /*
         * Far jump into 64 bit mode.
         */
        // Set the selector
        tc->setMiscReg(MISCREG_CS, 1);
        // Manually set up the segment attributes. In the future when there's
        // other existing functionality to do this, that could be used
        // instead.
        SegAttr csAttr = 0;
        csAttr.writable = 0;
        csAttr.readable = 1;
        csAttr.expandDown = 0;
        csAttr.dpl = 0;
        csAttr.defaultSize = 0;
        csAttr.longMode = 1;
        tc->setMiscReg(MISCREG_CS_ATTR, csAttr);

        tc->setPC(tc->getSystemPtr()->kernelEntry);
        tc->setNextPC(tc->readPC());

        // We should now be in long mode. Yay!

        tc->activate(0);
    } else {
        // This is an application processor (AP). It should be initialized to
        // look like only the BIOS POST has run on it and put then put it into
        // a halted state.
        tc->suspend();
    }
}

#else

void startupCPU(ThreadContext *tc, int cpuId)
{
    tc->activate(0);
}

#endif

} //namespace X86_ISA
