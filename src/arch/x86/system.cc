/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
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
 * Authors: Gabe Black
 */

#include "arch/x86/bios/intelmp.hh"
#include "arch/x86/bios/smbios.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/isa_traits.hh"
#include "arch/x86/system.hh"
#include "arch/vtophys.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/intmath.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "mem/port_proxy.hh"
#include "params/X86System.hh"
#include "sim/byteswap.hh"

using namespace LittleEndianGuest;
using namespace X86ISA;

X86System::X86System(Params *p) :
    System(p), smbiosTable(p->smbios_table),
    mpFloatingPointer(p->intel_mp_pointer),
    mpConfigTable(p->intel_mp_table),
    rsdp(p->acpi_description_table_pointer)
{
}

void
X86ISA::installSegDesc(ThreadContext *tc, SegmentRegIndex seg,
        SegDescriptor desc, bool longmode)
{
    uint64_t base = desc.baseLow + (desc.baseHigh << 24);
    bool honorBase = !longmode || seg == SEGMENT_REG_FS ||
                                  seg == SEGMENT_REG_GS ||
                                  seg == SEGMENT_REG_TSL ||
                                  seg == SYS_SEGMENT_REG_TR;
    uint64_t limit = desc.limitLow | (desc.limitHigh << 16);

    SegAttr attr = 0;

    attr.dpl = desc.dpl;
    attr.unusable = 0;
    attr.defaultSize = desc.d;
    attr.longMode = desc.l;
    attr.avl = desc.avl;
    attr.granularity = desc.g;
    attr.present = desc.p;
    attr.system = desc.s;
    attr.type = desc.type;
    if (desc.s) {
        if (desc.type.codeOrData) {
            // Code segment
            attr.expandDown = 0;
            attr.readable = desc.type.r;
            attr.writable = 0;
        } else {
            // Data segment
            attr.expandDown = desc.type.e;
            attr.readable = 1;
            attr.writable = desc.type.w;
        }
    } else {
        attr.readable = 1;
        attr.writable = 1;
        attr.expandDown = 0;
    }

    tc->setMiscReg(MISCREG_SEG_BASE(seg), base);
    tc->setMiscReg(MISCREG_SEG_EFF_BASE(seg), honorBase ? base : 0);
    tc->setMiscReg(MISCREG_SEG_LIMIT(seg), limit);
    tc->setMiscReg(MISCREG_SEG_ATTR(seg), (MiscReg)attr);
}

void
X86System::initState()
{
    System::initState();

    if (!kernel)
        fatal("No kernel to load.\n");

    if (kernel->getArch() == ObjectFile::I386)
        fatal("Loading a 32 bit x86 kernel is not supported.\n");

    ThreadContext *tc = threadContexts[0];
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

    /*
     * Set up the gdt.
     */
    uint8_t numGDTEntries = 0;
    // Place holder at selector 0
    uint64_t nullDescriptor = 0;
    physProxy.writeBlob(GDTBase + numGDTEntries * 8,
                        (uint8_t *)(&nullDescriptor), 8);
    numGDTEntries++;

    SegDescriptor initDesc = 0;
    initDesc.type.codeOrData = 0; // code or data type
    initDesc.type.c = 0;          // conforming
    initDesc.type.r = 1;          // readable
    initDesc.dpl = 0;             // privilege
    initDesc.p = 1;               // present
    initDesc.l = 1;               // longmode - 64 bit
    initDesc.d = 0;               // operand size
    initDesc.g = 1;               // granularity
    initDesc.s = 1;               // system segment
    initDesc.limitHigh = 0xFFFF;
    initDesc.limitLow = 0xF;
    initDesc.baseHigh = 0x0;
    initDesc.baseLow = 0x0;

    //64 bit code segment
    SegDescriptor csDesc = initDesc;
    csDesc.type.codeOrData = 1;
    csDesc.dpl = 0;
    //Because we're dealing with a pointer and I don't think it's
    //guaranteed that there isn't anything in a nonvirtual class between
    //it's beginning in memory and it's actual data, we'll use an
    //intermediary.
    uint64_t csDescVal = csDesc;
    physProxy.writeBlob(GDTBase + numGDTEntries * 8,
                        (uint8_t *)(&csDescVal), 8);

    numGDTEntries++;

    SegSelector cs = 0;
    cs.si = numGDTEntries - 1;

    tc->setMiscReg(MISCREG_CS, (MiscReg)cs);

    //32 bit data segment
    SegDescriptor dsDesc = initDesc;
    uint64_t dsDescVal = dsDesc;
    physProxy.writeBlob(GDTBase + numGDTEntries * 8,
                        (uint8_t *)(&dsDescVal), 8);

    numGDTEntries++;

    SegSelector ds = 0;
    ds.si = numGDTEntries - 1;

    tc->setMiscReg(MISCREG_DS, (MiscReg)ds);
    tc->setMiscReg(MISCREG_ES, (MiscReg)ds);
    tc->setMiscReg(MISCREG_FS, (MiscReg)ds);
    tc->setMiscReg(MISCREG_GS, (MiscReg)ds);
    tc->setMiscReg(MISCREG_SS, (MiscReg)ds);

    tc->setMiscReg(MISCREG_TSL, 0);
    tc->setMiscReg(MISCREG_TSG_BASE, GDTBase);
    tc->setMiscReg(MISCREG_TSG_LIMIT, 8 * numGDTEntries - 1);

    SegDescriptor tssDesc = initDesc;
    uint64_t tssDescVal = tssDesc;
    physProxy.writeBlob(GDTBase + numGDTEntries * 8,
                        (uint8_t *)(&tssDescVal), 8);

    numGDTEntries++;

    SegSelector tss = 0;
    tss.si = numGDTEntries - 1;

    tc->setMiscReg(MISCREG_TR, (MiscReg)tss);
    installSegDesc(tc, SYS_SEGMENT_REG_TR, tssDesc, true);

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
        physProxy.writeBlob(PageMapLevel4 + offset, (uint8_t *)(&pml4e), 8);
    }
    // Point to the only PDPT
    pml4e = X86ISA::htog(0x7 | PageDirPtrTable);
    physProxy.writeBlob(PageMapLevel4, (uint8_t *)(&pml4e), 8);

    // Page Directory Pointer Table

    // read/write, user, not present
    uint64_t pdpe = X86ISA::htog(0x6);
    for (int offset = 0; offset < (1 << PDPTBits) * 8; offset += 8) {
        physProxy.writeBlob(PageDirPtrTable + offset,
                            (uint8_t *)(&pdpe), 8);
    }
    // Point to the PDTs
    for (int table = 0; table < NumPDTs; table++) {
        pdpe = X86ISA::htog(0x7 | PageDirTable[table]);
        physProxy.writeBlob(PageDirPtrTable + table * 8,
                            (uint8_t *)(&pdpe), 8);
    }

    // Page Directory Tables

    Addr base = 0;
    const Addr pageSize = 2 << 20;
    for (int table = 0; table < NumPDTs; table++) {
        for (int offset = 0; offset < (1 << PDTBits) * 8; offset += 8) {
            // read/write, user, present, 4MB
            uint64_t pdte = X86ISA::htog(0x87 | base);
            physProxy.writeBlob(PageDirTable[table] + offset,
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

    //Start using longmode segments.
    installSegDesc(tc, SEGMENT_REG_CS, csDesc, true);
    installSegDesc(tc, SEGMENT_REG_DS, dsDesc, true);
    installSegDesc(tc, SEGMENT_REG_ES, dsDesc, true);
    installSegDesc(tc, SEGMENT_REG_FS, dsDesc, true);
    installSegDesc(tc, SEGMENT_REG_GS, dsDesc, true);
    installSegDesc(tc, SEGMENT_REG_SS, dsDesc, true);

    //Activate long mode.
    cr0.pg = 1;
    tc->setMiscReg(MISCREG_CR0, cr0);

    tc->pcState(tc->getSystemPtr()->kernelEntry);

    // We should now be in long mode. Yay!

    Addr ebdaPos = 0xF0000;
    Addr fixed, table;

    //Write out the SMBios/DMI table
    writeOutSMBiosTable(ebdaPos, fixed, table);
    ebdaPos += (fixed + table);
    ebdaPos = roundUp(ebdaPos, 16);

    //Write out the Intel MP Specification configuration table
    writeOutMPTable(ebdaPos, fixed, table);
    ebdaPos += (fixed + table);
}

void
X86System::writeOutSMBiosTable(Addr header,
        Addr &headerSize, Addr &structSize, Addr table)
{
    // If the table location isn't specified, just put it after the header.
    // The header size as of the 2.5 SMBios specification is 0x1F bytes
    if (!table)
        table = header + 0x1F;
    smbiosTable->setTableAddr(table);

    smbiosTable->writeOut(physProxy, header, headerSize, structSize);

    // Do some bounds checking to make sure we at least didn't step on
    // ourselves.
    assert(header > table || header + headerSize <= table);
    assert(table > header || table + structSize <= header);
}

void
X86System::writeOutMPTable(Addr fp,
        Addr &fpSize, Addr &tableSize, Addr table)
{
    // If the table location isn't specified and it exists, just put
    // it after the floating pointer. The fp size as of the 1.4 Intel MP
    // specification is 0x10 bytes.
    if (mpConfigTable) {
        if (!table)
            table = fp + 0x10;
        mpFloatingPointer->setTableAddr(table);
    }

    fpSize = mpFloatingPointer->writeOut(physProxy, fp);
    if (mpConfigTable)
        tableSize = mpConfigTable->writeOut(physProxy, table);
    else
        tableSize = 0;

    // Do some bounds checking to make sure we at least didn't step on
    // ourselves and the fp structure was the size we thought it was.
    assert(fp > table || fp + fpSize <= table);
    assert(table > fp || table + tableSize <= fp);
    assert(fpSize == 0x10);
}


X86System::~X86System()
{
    delete smbiosTable;
}

X86System *
X86SystemParams::create()
{
    return new X86System(this);
}
