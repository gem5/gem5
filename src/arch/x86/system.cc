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

#include "arch/x86/bios/smbios.hh"
#include "arch/x86/miscregs.hh"
#include "arch/x86/system.hh"
#include "arch/vtophys.hh"
#include "base/remote_gdb.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/physical.hh"
#include "params/X86System.hh"
#include "sim/byteswap.hh"


using namespace LittleEndianGuest;
using namespace X86ISA;

X86System::X86System(Params *p)
    : System(p), smbiosTable(p->smbios_table)
{}

void
X86System::startup()
{
    System::startup();
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
    FunctionalPort * physPort = threadContexts[0]->getPhysPort();

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

    threadContexts[0]->setMiscReg(MISCREG_TSG_BASE, GDTBase);
    threadContexts[0]->setMiscReg(MISCREG_TSG_LIMIT, 0xF);

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
    CR0 cr0 = threadContexts[0]->readMiscRegNoEffect(MISCREG_CR0);
    //Turn off paging.
    cr0.pg = 0;
    threadContexts[0]->setMiscReg(MISCREG_CR0, cr0);
    //Turn on protected mode.
    cr0.pe = 1;
    threadContexts[0]->setMiscReg(MISCREG_CR0, cr0);

    CR4 cr4 = threadContexts[0]->readMiscRegNoEffect(MISCREG_CR4);
    //Turn on pae.
    cr4.pae = 1;
    threadContexts[0]->setMiscReg(MISCREG_CR4, cr4);

    //Point to the page tables.
    threadContexts[0]->setMiscReg(MISCREG_CR3, PageMapLevel4);

    Efer efer = threadContexts[0]->readMiscRegNoEffect(MISCREG_EFER);
    //Enable long mode.
    efer.lme = 1;
    threadContexts[0]->setMiscReg(MISCREG_EFER, efer);

    //Activate long mode.
    cr0.pg = 1;
    threadContexts[0]->setMiscReg(MISCREG_CR0, cr0);

    /*
     * Far jump into 64 bit mode.
     */
    // Set the selector
    threadContexts[0]->setMiscReg(MISCREG_CS, 1);
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
    threadContexts[0]->setMiscReg(MISCREG_CS_ATTR, csAttr);

    threadContexts[0]->setPC(threadContexts[0]->getSystemPtr()->kernelEntry);
    threadContexts[0]->setNextPC(threadContexts[0]->readPC());

    // We should now be in long mode. Yay!

    Addr ebdaPos = 0xF0000;

    Addr headerSize, structSize;
    //Write out the SMBios/DMI table
    writeOutSMBiosTable(ebdaPos, headerSize, structSize);
    ebdaPos += (headerSize + structSize);
}

void
X86System::writeOutSMBiosTable(Addr header,
        Addr &headerSize, Addr &structSize, Addr table)
{
    // Get a port to write the table and header to memory.
    FunctionalPort * physPort = threadContexts[0]->getPhysPort();

    // If the table location isn't specified, just put it after the header.
    // The header size as of the 2.5 SMBios specification is 0x1F bytes
    if (!table)
        table = header + 0x1F;
    smbiosTable->setTableAddr(table);

    smbiosTable->writeOut(physPort, header, headerSize, structSize);

    // Do some bounds checking to make sure we at least didn't step on
    // ourselves.
    assert(header > table || header + headerSize <= table);
    assert(table > header || table + structSize <= header);
}


X86System::~X86System()
{
    delete smbiosTable;
}

void
X86System::serialize(std::ostream &os)
{
    System::serialize(os);
}


void
X86System::unserialize(Checkpoint *cp, const std::string &section)
{
    System::unserialize(cp,section);
}

X86System *
X86SystemParams::create()
{
    return new X86System(this);
}
