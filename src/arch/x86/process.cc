/*
 * Copyright (c) 2014 Advanced Micro Devices, Inc.
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
 * Copyright (c) 2003-2006 The Regents of The University of Michigan
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

#include "arch/x86/process.hh"

#include <string>
#include <vector>

#include "arch/x86/fs_workload.hh"
#include "arch/x86/page_size.hh"
#include "arch/x86/regs/int.hh"
#include "arch/x86/regs/misc.hh"
#include "arch/x86/regs/segment.hh"
#include "arch/x86/se_workload.hh"
#include "arch/x86/types.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/logging.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "debug/Stack.hh"
#include "mem/multi_level_page_table.hh"
#include "mem/page_table.hh"
#include "params/Process.hh"
#include "sim/aux_vector.hh"
#include "sim/byteswap.hh"
#include "sim/process_impl.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_return.hh"
#include "sim/system.hh"

namespace gem5
{

using namespace X86ISA;

template class MultiLevelPageTable<LongModePTE<47, 39>,
                                   LongModePTE<38, 30>,
                                   LongModePTE<29, 21>,
                                   LongModePTE<20, 12> >;
typedef MultiLevelPageTable<LongModePTE<47, 39>,
                            LongModePTE<38, 30>,
                            LongModePTE<29, 21>,
                            LongModePTE<20, 12> > ArchPageTable;

X86Process::X86Process(const ProcessParams &params,
                       loader::ObjectFile *objFile) :
    Process(params, params.useArchPT ?
                    static_cast<EmulationPageTable *>(
                            new ArchPageTable(params.name, params.pid,
                                              params.system, PageBytes)) :
                    new EmulationPageTable(params.name, params.pid,
                                           PageBytes),
            objFile)
{
}

void X86Process::clone(ThreadContext *old_tc, ThreadContext *new_tc,
                       Process *p, RegVal flags)
{
    Process::clone(old_tc, new_tc, p, flags);
    X86Process *process = (X86Process*)p;
    *process = *this;
}

X86_64Process::X86_64Process(const ProcessParams &params,
                             loader::ObjectFile *objFile) :
    X86Process(params, objFile)
{
    vsyscallPage.base = 0xffffffffff600000ULL;
    vsyscallPage.size = PageBytes;
    vsyscallPage.vtimeOffset = 0x400;
    vsyscallPage.vgettimeofdayOffset = 0x0;

    Addr brk_point = roundUp(image.maxAddr(), PageBytes);
    Addr stack_base = 0x7FFFFFFFF000ULL;
    Addr max_stack_size = 8 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;
    Addr mmap_end = 0x7FFFF7FFF000ULL;

    memState = std::make_shared<MemState>(
            this, brk_point, stack_base, max_stack_size,
            next_thread_stack_base, mmap_end);
}


I386Process::I386Process(const ProcessParams &params,
                         loader::ObjectFile *objFile) :
    X86Process(params, objFile)
{
    if (kvmInSE)
        panic("KVM CPU model does not support 32 bit processes");

    _gdtStart = 0xffffd000ULL;
    _gdtSize = PageBytes;

    vsyscallPage.base = 0xffffe000ULL;
    vsyscallPage.size = PageBytes;
    vsyscallPage.vsyscallOffset = 0x400;
    vsyscallPage.vsysexitOffset = 0x410;

    Addr brk_point = roundUp(image.maxAddr(), PageBytes);
    Addr stack_base = _gdtStart;
    Addr max_stack_size = 8 * 1024 * 1024;
    Addr next_thread_stack_base = stack_base - max_stack_size;
    Addr mmap_end = 0xB7FFF000ULL;

    memState = std::make_shared<MemState>(
            this, brk_point, stack_base, max_stack_size,
            next_thread_stack_base, mmap_end);
}

void
X86_64Process::initState()
{
    X86Process::initState();

    if (useForClone)
        return;

    argsInit(PageBytes);

    // Set up the vsyscall page for this process.
    memState->mapRegion(vsyscallPage.base, vsyscallPage.size, "vsyscall");
    uint8_t vtimeBlob[] = {
        0x48,0xc7,0xc0,0xc9,0x00,0x00,0x00,    // mov    $0xc9,%rax
        0x0f,0x05,                             // syscall
        0xc3                                   // retq
    };
    initVirtMem->writeBlob(vsyscallPage.base + vsyscallPage.vtimeOffset,
            vtimeBlob, sizeof(vtimeBlob));

    uint8_t vgettimeofdayBlob[] = {
        0x48,0xc7,0xc0,0x60,0x00,0x00,0x00,    // mov    $0x60,%rax
        0x0f,0x05,                             // syscall
        0xc3                                   // retq
    };
    initVirtMem->writeBlob(
            vsyscallPage.base + vsyscallPage.vgettimeofdayOffset,
            vgettimeofdayBlob, sizeof(vgettimeofdayBlob));

    if (kvmInSE) {
        PortProxy physProxy = system->physProxy;

        Addr syscallCodePhysAddr = seWorkload->allocPhysPages(1);
        Addr gdtPhysAddr = seWorkload->allocPhysPages(1);
        Addr idtPhysAddr = seWorkload->allocPhysPages(1);
        Addr istPhysAddr = seWorkload->allocPhysPages(1);
        Addr tssPhysAddr = seWorkload->allocPhysPages(1);
        Addr pfHandlerPhysAddr = seWorkload->allocPhysPages(1);

        /*
         * Set up the gdt.
         */
        uint8_t numGDTEntries = 0;
        uint64_t nullDescriptor = 0;
        physProxy.writeBlob(gdtPhysAddr + numGDTEntries * 8,
                            &nullDescriptor, 8);
        numGDTEntries++;

        SegDescriptor initDesc = 0;
        initDesc.type.codeOrData = 0; // code or data type
        initDesc.type.c = 0;          // conforming
        initDesc.type.r = 1;          // readable
        initDesc.dpl = 0;             // privilege
        initDesc.p = 1;               // present
        initDesc.l = 1;               // longmode - 64 bit
        initDesc.d = 0;               // operand size
        initDesc.g = 1;
        initDesc.s = 1;               // system segment
        initDesc.limit = 0xFFFFFFFF;
        initDesc.base = 0;

        //64 bit code segment
        SegDescriptor csLowPLDesc = initDesc;
        csLowPLDesc.type.codeOrData = 1;
        csLowPLDesc.dpl = 0;
        uint64_t csLowPLDescVal = csLowPLDesc;
        physProxy.writeBlob(gdtPhysAddr + numGDTEntries * 8,
                            &csLowPLDescVal, 8);

        numGDTEntries++;

        SegSelector csLowPL = 0;
        csLowPL.si = numGDTEntries - 1;
        csLowPL.rpl = 0;

        //64 bit data segment
        SegDescriptor dsLowPLDesc = initDesc;
        dsLowPLDesc.type.codeOrData = 0;
        dsLowPLDesc.dpl = 0;
        uint64_t dsLowPLDescVal = dsLowPLDesc;
        physProxy.writeBlob(gdtPhysAddr + numGDTEntries * 8,
                            &dsLowPLDescVal, 8);

        numGDTEntries++;

        SegSelector dsLowPL = 0;
        dsLowPL.si = numGDTEntries - 1;
        dsLowPL.rpl = 0;

        //64 bit data segment
        SegDescriptor dsDesc = initDesc;
        dsDesc.type.codeOrData = 0;
        dsDesc.dpl = 3;
        uint64_t dsDescVal = dsDesc;
        physProxy.writeBlob(gdtPhysAddr + numGDTEntries * 8,
                            &dsDescVal, 8);

        numGDTEntries++;

        SegSelector ds = 0;
        ds.si = numGDTEntries - 1;
        ds.rpl = 3;

        //64 bit code segment
        SegDescriptor csDesc = initDesc;
        csDesc.type.codeOrData = 1;
        csDesc.dpl = 3;
        uint64_t csDescVal = csDesc;
        physProxy.writeBlob(gdtPhysAddr + numGDTEntries * 8,
                            &csDescVal, 8);

        numGDTEntries++;

        SegSelector cs = 0;
        cs.si = numGDTEntries - 1;
        cs.rpl = 3;

        SegSelector scall = 0;
        scall.si = csLowPL.si;
        scall.rpl = 0;

        SegSelector sret = 0;
        sret.si = dsLowPL.si;
        sret.rpl = 3;

        /* In long mode the TSS has been extended to 16 Bytes */
        TSSlow TSSDescLow = 0;
        TSSDescLow.type = 0xB;
        TSSDescLow.dpl = 0; // Privelege level 0
        TSSDescLow.p = 1; // Present
        TSSDescLow.limit = 0xFFFFFFFF;
        TSSDescLow.base = bits(TSSVirtAddr, 31, 0);

        TSShigh TSSDescHigh = 0;
        TSSDescHigh.base = bits(TSSVirtAddr, 63, 32);

        struct TSSDesc
        {
            uint64_t low;
            uint64_t high;
        } tssDescVal = {TSSDescLow, TSSDescHigh};

        physProxy.writeBlob(gdtPhysAddr + numGDTEntries * 8,
                            &tssDescVal, sizeof(tssDescVal));

        numGDTEntries++;

        SegSelector tssSel = 0;
        tssSel.si = numGDTEntries - 1;

        uint64_t tss_base_addr = (TSSDescHigh.base << 32) | TSSDescLow.base;
        uint64_t tss_limit = TSSDescLow.limit;

        SegAttr tss_attr = 0;

        tss_attr.type = TSSDescLow.type;
        tss_attr.dpl = TSSDescLow.dpl;
        tss_attr.present = TSSDescLow.p;
        tss_attr.granularity = TSSDescLow.g;
        tss_attr.unusable = 0;

        for (int i = 0; i < contextIds.size(); i++) {
            ThreadContext *tc = system->threads[contextIds[i]];

            tc->setMiscReg(misc_reg::Cs, cs);
            tc->setMiscReg(misc_reg::Ds, ds);
            tc->setMiscReg(misc_reg::Es, ds);
            tc->setMiscReg(misc_reg::Fs, ds);
            tc->setMiscReg(misc_reg::Gs, ds);
            tc->setMiscReg(misc_reg::Ss, ds);

            // LDT
            tc->setMiscReg(misc_reg::Tsl, 0);
            SegAttr tslAttr = 0;
            tslAttr.unusable = 1;
            tslAttr.present = 0;
            tslAttr.type = 2;
            tc->setMiscReg(misc_reg::TslAttr, tslAttr);

            tc->setMiscReg(misc_reg::TsgBase, GDTVirtAddr);
            tc->setMiscReg(misc_reg::TsgLimit, 8 * numGDTEntries - 1);

            tc->setMiscReg(misc_reg::Tr, tssSel);
            tc->setMiscReg(misc_reg::TrBase, tss_base_addr);
            tc->setMiscReg(misc_reg::TrEffBase, tss_base_addr);
            tc->setMiscReg(misc_reg::TrLimit, tss_limit);
            tc->setMiscReg(misc_reg::TrAttr, tss_attr);

            //Start using longmode segments.
            installSegDesc(tc, segment_idx::Cs, csDesc, true);
            installSegDesc(tc, segment_idx::Ds, dsDesc, true);
            installSegDesc(tc, segment_idx::Es, dsDesc, true);
            installSegDesc(tc, segment_idx::Fs, dsDesc, true);
            installSegDesc(tc, segment_idx::Gs, dsDesc, true);
            installSegDesc(tc, segment_idx::Ss, dsDesc, true);

            Efer efer = 0;
            efer.sce = 1; // Enable system call extensions.
            efer.lme = 1; // Enable long mode.
            efer.lma = 1; // Activate long mode.
            efer.nxe = 1; // Enable nx support.
            efer.svme = 0; // Disable svm support for now.
            efer.ffxsr = 0; // Disable fast fxsave and fxrstor.
            tc->setMiscReg(misc_reg::Efer, efer);

            //Set up the registers that describe the operating mode.
            CR0 cr0 = 0;
            cr0.pg = 1; // Turn on paging.
            cr0.cd = 0; // Don't disable caching.
            cr0.nw = 0; // This is bit is defined to be ignored.
            cr0.am = 0; // No alignment checking
            cr0.wp = 0; // Supervisor mode can write read only pages
            cr0.ne = 1;
            cr0.et = 1; // This should always be 1
            cr0.ts = 0; // We don't do task switching, so causing fp exceptions
                        // would be pointless.
            cr0.em = 0; // Allow x87 instructions to execute natively.
            cr0.mp = 1; // This doesn't really matter, but the manual suggests
                        // setting it to one.
            cr0.pe = 1; // We're definitely in protected mode.
            tc->setMiscReg(misc_reg::Cr0, cr0);

            CR0 cr2 = 0;
            tc->setMiscReg(misc_reg::Cr2, cr2);

            CR3 cr3 = dynamic_cast<ArchPageTable *>(pTable)->basePtr();
            tc->setMiscReg(misc_reg::Cr3, cr3);

            CR4 cr4 = 0;
            //Turn on pae.
            cr4.osxsave = 0; // Disable XSAVE and Proc Extended States
            cr4.osxmmexcpt = 0; // Operating System Unmasked Exception
            cr4.osfxsr = 1; // Operating System FXSave/FSRSTOR Support
            cr4.pce = 0; // Performance-Monitoring Counter Enable
            cr4.pge = 0; // Page-Global Enable
            cr4.mce = 0; // Machine Check Enable
            cr4.pae = 1; // Physical-Address Extension
            cr4.pse = 0; // Page Size Extensions
            cr4.de = 0; // Debugging Extensions
            cr4.tsd = 0; // Time Stamp Disable
            cr4.pvi = 0; // Protected-Mode Virtual Interrupts
            cr4.vme = 0; // Virtual-8086 Mode Extensions

            tc->setMiscReg(misc_reg::Cr4, cr4);

            CR8 cr8 = 0;
            tc->setMiscReg(misc_reg::Cr8, cr8);

            tc->setMiscReg(misc_reg::Mxcsr, 0x1f80);
            tc->setMiscReg(misc_reg::Ftw, 0xffff);
            tc->setMiscReg(misc_reg::Fcw, 0x037f);

            tc->setMiscReg(misc_reg::ApicBase, 0xfee00900);

            tc->setMiscReg(misc_reg::TsgBase, GDTVirtAddr);
            tc->setMiscReg(misc_reg::TsgLimit, 0xffff);

            tc->setMiscReg(misc_reg::IdtrBase, IDTVirtAddr);
            tc->setMiscReg(misc_reg::IdtrLimit, 0xffff);

            /* enabling syscall and sysret */
            RegVal star = ((RegVal)sret << 48) | ((RegVal)scall << 32);
            tc->setMiscReg(misc_reg::Star, star);
            RegVal lstar = (RegVal)syscallCodeVirtAddr;
            tc->setMiscReg(misc_reg::Lstar, lstar);
            RegVal sfmask = (1 << 8) | (1 << 10); // TF | DF
            tc->setMiscReg(misc_reg::SfMask, sfmask);
        }

        /* Set up the content of the TSS and write it to physical memory. */

        struct
        {
            uint32_t reserved0;        // +00h
            uint32_t RSP0_low;         // +04h
            uint32_t RSP0_high;        // +08h
            uint32_t RSP1_low;         // +0Ch
            uint32_t RSP1_high;        // +10h
            uint32_t RSP2_low;         // +14h
            uint32_t RSP2_high;        // +18h
            uint32_t reserved1;        // +1Ch
            uint32_t reserved2;        // +20h
            uint32_t IST1_low;         // +24h
            uint32_t IST1_high;        // +28h
            uint32_t IST2_low;         // +2Ch
            uint32_t IST2_high;        // +30h
            uint32_t IST3_low;         // +34h
            uint32_t IST3_high;        // +38h
            uint32_t IST4_low;         // +3Ch
            uint32_t IST4_high;        // +40h
            uint32_t IST5_low;         // +44h
            uint32_t IST5_high;        // +48h
            uint32_t IST6_low;         // +4Ch
            uint32_t IST6_high;        // +50h
            uint32_t IST7_low;         // +54h
            uint32_t IST7_high;        // +58h
            uint32_t reserved3;        // +5Ch
            uint32_t reserved4;        // +60h
            uint16_t reserved5;        // +64h
            uint16_t IO_MapBase;       // +66h
        } tss;

        /** setting Interrupt Stack Table */
        uint64_t IST_start = ISTVirtAddr + PageBytes;
        tss.IST1_low  = IST_start;
        tss.IST1_high = IST_start >> 32;
        tss.RSP0_low  = tss.IST1_low;
        tss.RSP0_high = tss.IST1_high;
        tss.RSP1_low  = tss.IST1_low;
        tss.RSP1_high = tss.IST1_high;
        tss.RSP2_low  = tss.IST1_low;
        tss.RSP2_high = tss.IST1_high;
        physProxy.writeBlob(tssPhysAddr, &tss, sizeof(tss));

        /* Setting IDT gates */
        GateDescriptorLow PFGateLow = 0;
        PFGateLow.offsetHigh = bits(PFHandlerVirtAddr, 31, 16);
        PFGateLow.offsetLow = bits(PFHandlerVirtAddr, 15, 0);
        PFGateLow.selector = csLowPL;
        PFGateLow.p = 1;
        PFGateLow.dpl = 0;
        PFGateLow.type = 0xe;      // gate interrupt type
        PFGateLow.IST = 0;         // setting IST to 0 and using RSP0

        GateDescriptorHigh PFGateHigh = 0;
        PFGateHigh.offset = bits(PFHandlerVirtAddr, 63, 32);

        struct
        {
            uint64_t low;
            uint64_t high;
        } PFGate = {PFGateLow, PFGateHigh};

        physProxy.writeBlob(idtPhysAddr + 0xE0, &PFGate, sizeof(PFGate));

        /* System call handler */
        // First, we write to the MMIO m5ops range (0xffffc90000007000)
        // to trap out of the VM back into gem5 to emulate the system
        // call. Upon re-entering the VM, we need to flush the TLB in
        // case the system call modified existing page mappings (e.g.,
        // munmap, mremap, brk). To do this, we can simply read/write
        // cr3; however, doing so requires saving the value to an
        // intermediate GPR (%rax, in this case). We save/restore the
        // value of %rax in the scratch region syscallDataBuf.
        const Addr syscallDataBuf = syscallCodeVirtAddr + 0x100;
        uint8_t syscallBlob[] = {
            // mov    %rax, (0xffffc90000007000)
            0x48, 0xa3, 0x00, 0x70, 0x00,
            0x00, 0x00, 0xc9, 0xff, 0xff,
            // mov    %rax, (syscallDataBuf)
            0x48, 0xa3, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            // mov    %cr3, %rax
            0x0f, 0x20, 0xd8,
            // mov    %rax, %cr3
            0x0f, 0x22, 0xd8,
            // mov    (syscallDataBuf), %rax
            0x48, 0xa1, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00,
            // sysret
            0x48, 0x0f, 0x07
        };
        assert(syscallDataBuf >= syscallCodePhysAddr + sizeof syscallBlob);
        std::memcpy(&syscallBlob[12], &syscallDataBuf, sizeof syscallDataBuf);
        std::memcpy(&syscallBlob[28], &syscallDataBuf, sizeof syscallDataBuf);

        physProxy.writeBlob(syscallCodePhysAddr,
                            syscallBlob, sizeof(syscallBlob));

        /** Page fault handler */
        uint8_t faultBlob[] = {
            // mov    %rax, (0xffffc90000007000)
            0x48, 0xa3, 0x00, 0x70, 0x00,
            0x00, 0x00, 0xc9, 0xff, 0xff,
            // add    $0x8, %rsp # skip error
            0x48, 0x83, 0xc4, 0x08,
            // iretq
            0x48, 0xcf
        };

        physProxy.writeBlob(pfHandlerPhysAddr, faultBlob, sizeof(faultBlob));

        /* Syscall handler */
        pTable->map(syscallCodeVirtAddr, syscallCodePhysAddr,
                    PageBytes, false);
        /* GDT */
        pTable->map(GDTVirtAddr, gdtPhysAddr, PageBytes, false);
        /* IDT */
        pTable->map(IDTVirtAddr, idtPhysAddr, PageBytes, false);
        /* TSS */
        pTable->map(TSSVirtAddr, tssPhysAddr, PageBytes, false);
        /* IST */
        pTable->map(ISTVirtAddr, istPhysAddr, PageBytes, false);
        /* PF handler */
        pTable->map(PFHandlerVirtAddr, pfHandlerPhysAddr, PageBytes, false);
        /* MMIO region for m5ops */
        auto m5op_range = system->m5opRange();
        if (m5op_range.size()) {
            pTable->map(MMIORegionVirtAddr, m5op_range.start(),
                        m5op_range.size(), false);
        }
    } else {
        for (int i = 0; i < contextIds.size(); i++) {
            ThreadContext * tc = system->threads[contextIds[i]];

            SegAttr dataAttr = 0;
            dataAttr.dpl = 3;
            dataAttr.unusable = 0;
            dataAttr.defaultSize = 1;
            dataAttr.longMode = 1;
            dataAttr.avl = 0;
            dataAttr.granularity = 1;
            dataAttr.present = 1;
            dataAttr.type = 3;
            dataAttr.writable = 1;
            dataAttr.readable = 1;
            dataAttr.expandDown = 0;
            dataAttr.system = 1;

            // Initialize the segment registers.
            for (int seg = 0; seg < segment_idx::NumIdxs; seg++) {
                tc->setMiscRegNoEffect(misc_reg::segBase(seg), 0);
                tc->setMiscRegNoEffect(misc_reg::segEffBase(seg), 0);
                tc->setMiscRegNoEffect(misc_reg::segAttr(seg), dataAttr);
            }

            SegAttr csAttr = 0;
            csAttr.dpl = 3;
            csAttr.unusable = 0;
            csAttr.defaultSize = 0;
            csAttr.longMode = 1;
            csAttr.avl = 0;
            csAttr.granularity = 1;
            csAttr.present = 1;
            csAttr.type = 10;
            csAttr.writable = 0;
            csAttr.readable = 1;
            csAttr.expandDown = 0;
            csAttr.system = 1;

            tc->setMiscRegNoEffect(misc_reg::CsAttr, csAttr);

            Efer efer = 0;
            efer.sce = 1; // Enable system call extensions.
            efer.lme = 1; // Enable long mode.
            efer.lma = 1; // Activate long mode.
            efer.nxe = 1; // Enable nx support.
            efer.svme = 0; // Disable svm support for now. It isn't implemented.
            efer.ffxsr = 1; // Turn on fast fxsave and fxrstor.
            tc->setMiscReg(misc_reg::Efer, efer);

            // Set up the registers that describe the operating mode.
            CR0 cr0 = 0;
            cr0.pg = 1; // Turn on paging.
            cr0.cd = 0; // Don't disable caching.
            cr0.nw = 0; // This is bit is defined to be ignored.
            cr0.am = 0; // No alignment checking
            cr0.wp = 0; // Supervisor mode can write read only pages
            cr0.ne = 1;
            cr0.et = 1; // This should always be 1
            cr0.ts = 0; // We don't do task switching, so causing fp exceptions
                        // would be pointless.
            cr0.em = 0; // Allow x87 instructions to execute natively.
            cr0.mp = 1; // This doesn't really matter, but the manual suggests
                        // setting it to one.
            cr0.pe = 1; // We're definitely in protected mode.
            tc->setMiscReg(misc_reg::Cr0, cr0);

            tc->setMiscReg(misc_reg::Mxcsr, 0x1f80);
            tc->setMiscReg(misc_reg::Ftw, 0xffff);
            tc->setMiscReg(misc_reg::Fcw, 0x037f);

            // Setting CR3 to the process pid so that concatinated
            // page addr with lower 12 bits of CR3 can be used in SE
            // mode as well to avoid conflicts between tlb entries with
            // same virtual addresses belonging to different processes
            tc->setMiscReg(misc_reg::Cr3, pTable->pid());

            // Setting pcide bit in CR4
            CR4 cr4 = tc->readMiscRegNoEffect(misc_reg::Cr4);
            cr4.pcide = 1;
            tc->setMiscReg(misc_reg::Cr4, cr4);
        }
    }
}

void
I386Process::initState()
{
    X86Process::initState();

    argsInit(PageBytes);

    /*
     * Set up a GDT for this process. The whole GDT wouldn't really be for
     * this process, but the only parts we care about are.
     */
    allocateMem(_gdtStart, _gdtSize);
    uint64_t zero = 0;
    assert(_gdtSize % sizeof(zero) == 0);
    for (Addr gdtCurrent = _gdtStart;
            gdtCurrent < _gdtStart + _gdtSize; gdtCurrent += sizeof(zero)) {
        initVirtMem->write(gdtCurrent, zero);
    }

    // Set up the vsyscall page for this process.
    memState->mapRegion(vsyscallPage.base, vsyscallPage.size, "vsyscall");
    uint8_t vsyscallBlob[] = {
        0x51,       // push %ecx
        0x52,       // push %edp
        0x55,       // push %ebp
        0x89, 0xe5, // mov %esp, %ebp
        0x0f, 0x34  // sysenter
    };
    initVirtMem->writeBlob(vsyscallPage.base + vsyscallPage.vsyscallOffset,
            vsyscallBlob, sizeof(vsyscallBlob));

    uint8_t vsysexitBlob[] = {
        0x5d,       // pop %ebp
        0x5a,       // pop %edx
        0x59,       // pop %ecx
        0xc3        // ret
    };
    initVirtMem->writeBlob(vsyscallPage.base + vsyscallPage.vsysexitOffset,
            vsysexitBlob, sizeof(vsysexitBlob));

    for (int i = 0; i < contextIds.size(); i++) {
        ThreadContext * tc = system->threads[contextIds[i]];

        SegAttr dataAttr = 0;
        dataAttr.dpl = 3;
        dataAttr.unusable = 0;
        dataAttr.defaultSize = 1;
        dataAttr.longMode = 0;
        dataAttr.avl = 0;
        dataAttr.granularity = 1;
        dataAttr.present = 1;
        dataAttr.type = 3;
        dataAttr.writable = 1;
        dataAttr.readable = 1;
        dataAttr.expandDown = 0;
        dataAttr.system = 1;

        // Initialize the segment registers.
        for (int seg = 0; seg < segment_idx::NumIdxs; seg++) {
            tc->setMiscRegNoEffect(misc_reg::segBase(seg), 0);
            tc->setMiscRegNoEffect(misc_reg::segEffBase(seg), 0);
            tc->setMiscRegNoEffect(misc_reg::segAttr(seg), dataAttr);
            tc->setMiscRegNoEffect(misc_reg::segSel(seg), 0xB);
            tc->setMiscRegNoEffect(misc_reg::segLimit(seg), (uint32_t)(-1));
        }

        SegAttr csAttr = 0;
        csAttr.dpl = 3;
        csAttr.unusable = 0;
        csAttr.defaultSize = 1;
        csAttr.longMode = 0;
        csAttr.avl = 0;
        csAttr.granularity = 1;
        csAttr.present = 1;
        csAttr.type = 0xa;
        csAttr.writable = 0;
        csAttr.readable = 1;
        csAttr.expandDown = 0;
        csAttr.system = 1;

        tc->setMiscRegNoEffect(misc_reg::CsAttr, csAttr);

        tc->setMiscRegNoEffect(misc_reg::TsgBase, _gdtStart);
        tc->setMiscRegNoEffect(misc_reg::TsgEffBase, _gdtStart);
        tc->setMiscRegNoEffect(misc_reg::TsgLimit, _gdtStart + _gdtSize - 1);

        // Set the LDT selector to 0 to deactivate it.
        tc->setMiscRegNoEffect(misc_reg::Tsl, 0);
        SegAttr attr = 0;
        attr.unusable = 1;
        tc->setMiscRegNoEffect(misc_reg::TslAttr, attr);

        Efer efer = 0;
        efer.sce = 1; // Enable system call extensions.
        efer.lme = 1; // Enable long mode.
        efer.lma = 0; // Deactivate long mode.
        efer.nxe = 1; // Enable nx support.
        efer.svme = 0; // Disable svm support for now. It isn't implemented.
        efer.ffxsr = 1; // Turn on fast fxsave and fxrstor.
        tc->setMiscReg(misc_reg::Efer, efer);

        // Set up the registers that describe the operating mode.
        CR0 cr0 = 0;
        cr0.pg = 1; // Turn on paging.
        cr0.cd = 0; // Don't disable caching.
        cr0.nw = 0; // This is bit is defined to be ignored.
        cr0.am = 0; // No alignment checking
        cr0.wp = 0; // Supervisor mode can write read only pages
        cr0.ne = 1;
        cr0.et = 1; // This should always be 1
        cr0.ts = 0; // We don't do task switching, so causing fp exceptions
                    // would be pointless.
        cr0.em = 0; // Allow x87 instructions to execute natively.
        cr0.mp = 1; // This doesn't really matter, but the manual suggests
                    // setting it to one.
        cr0.pe = 1; // We're definitely in protected mode.
        tc->setMiscReg(misc_reg::Cr0, cr0);

        tc->setMiscReg(misc_reg::Mxcsr, 0x1f80);
        tc->setMiscReg(misc_reg::Ftw, 0xffff);
        tc->setMiscReg(misc_reg::Fcw, 0x037f);
    }
}

template<class IntType>
void
X86Process::argsInit(int pageSize,
                     std::vector<gem5::auxv::AuxVector<IntType>> extraAuxvs)
{
    int intSize = sizeof(IntType);

    std::vector<gem5::auxv::AuxVector<IntType>> auxv = extraAuxvs;

    std::string filename;
    if (argv.size() < 1)
        filename = "";
    else
        filename = argv[0];

    // We want 16 byte alignment
    uint64_t align = 16;

    enum X86CpuFeature
    {
        X86_OnboardFPU = 1 << 0,
        X86_VirtualModeExtensions = 1 << 1,
        X86_DebuggingExtensions = 1 << 2,
        X86_PageSizeExtensions = 1 << 3,

        X86_TimeStampCounter = 1 << 4,
        X86_ModelSpecificRegisters = 1 << 5,
        X86_PhysicalAddressExtensions = 1 << 6,
        X86_MachineCheckExtensions = 1 << 7,

        X86_CMPXCHG8Instruction = 1 << 8,
        X86_OnboardAPIC = 1 << 9,
        X86_SYSENTER_SYSEXIT = 1 << 11,

        X86_MemoryTypeRangeRegisters = 1 << 12,
        X86_PageGlobalEnable = 1 << 13,
        X86_MachineCheckArchitecture = 1 << 14,
        X86_CMOVInstruction = 1 << 15,

        X86_PageAttributeTable = 1 << 16,
        X86_36BitPSEs = 1 << 17,
        X86_ProcessorSerialNumber = 1 << 18,
        X86_CLFLUSHInstruction = 1 << 19,

        X86_DebugTraceStore = 1 << 21,
        X86_ACPIViaMSR = 1 << 22,
        X86_MultimediaExtensions = 1 << 23,

        X86_FXSAVE_FXRSTOR = 1 << 24,
        X86_StreamingSIMDExtensions = 1 << 25,
        X86_StreamingSIMDExtensions2 = 1 << 26,
        X86_CPUSelfSnoop = 1 << 27,

        X86_HyperThreading = 1 << 28,
        X86_AutomaticClockControl = 1 << 29,
        X86_IA64Processor = 1 << 30
    };

    // Setup the auxiliary vectors. These will already have endian
    // conversion. Auxiliary vectors are loaded only for elf formatted
    // executables; the auxv is responsible for passing information from
    // the OS to the interpreter.
    auto *elfObject = dynamic_cast<loader::ElfObject *>(objFile);
    if (elfObject) {
        uint64_t features =
            X86_OnboardFPU |
            X86_VirtualModeExtensions |
            X86_DebuggingExtensions |
            X86_PageSizeExtensions |
            X86_TimeStampCounter |
            X86_ModelSpecificRegisters |
            X86_PhysicalAddressExtensions |
            X86_MachineCheckExtensions |
            X86_CMPXCHG8Instruction |
            X86_OnboardAPIC |
            X86_SYSENTER_SYSEXIT |
            X86_MemoryTypeRangeRegisters |
            X86_PageGlobalEnable |
            X86_MachineCheckArchitecture |
            X86_CMOVInstruction |
            X86_PageAttributeTable |
            X86_36BitPSEs |
//            X86_ProcessorSerialNumber |
            X86_CLFLUSHInstruction |
//            X86_DebugTraceStore |
//            X86_ACPIViaMSR |
            X86_MultimediaExtensions |
            X86_FXSAVE_FXRSTOR |
            X86_StreamingSIMDExtensions |
            X86_StreamingSIMDExtensions2 |
//            X86_CPUSelfSnoop |
//            X86_HyperThreading |
//            X86_AutomaticClockControl |
//            X86_IA64Processor |
            0;

        // Bits which describe the system hardware capabilities
        // XXX Figure out what these should be
        auxv.emplace_back(gem5::auxv::Hwcap, features);
        // The system page size
        auxv.emplace_back(gem5::auxv::Pagesz, X86ISA::PageBytes);
        // Frequency at which times() increments
        // Defined to be 100 in the kernel source.
        auxv.emplace_back(gem5::auxv::Clktck, 100);
        // This is the virtual address of the program header tables if they
        // appear in the executable image.
        auxv.emplace_back(gem5::auxv::Phdr, elfObject->programHeaderTable());
        // This is the size of a program header entry from the elf file.
        auxv.emplace_back(gem5::auxv::Phent, elfObject->programHeaderSize());
        // This is the number of program headers from the original elf file.
        auxv.emplace_back(gem5::auxv::Phnum, elfObject->programHeaderCount());
        // This is the base address of the ELF interpreter; it should be
        // zero for static executables or contain the base address for
        // dynamic executables.
        auxv.emplace_back(gem5::auxv::Base, getBias());
        // XXX Figure out what this should be.
        auxv.emplace_back(gem5::auxv::Flags, 0);
        // The entry point to the program
        auxv.emplace_back(gem5::auxv::Entry, objFile->entryPoint());
        // Different user and group IDs
        auxv.emplace_back(gem5::auxv::Uid, uid());
        auxv.emplace_back(gem5::auxv::Euid, euid());
        auxv.emplace_back(gem5::auxv::Gid, gid());
        auxv.emplace_back(gem5::auxv::Egid, egid());
        // Whether to enable "secure mode" in the executable
        auxv.emplace_back(gem5::auxv::Secure, 0);
        // The address of 16 "random" bytes.
        auxv.emplace_back(gem5::auxv::Random, 0);
        // The name of the program
        auxv.emplace_back(gem5::auxv::Execfn, 0);
        // The platform string
        auxv.emplace_back(gem5::auxv::Platform, 0);
    }

    // Figure out how big the initial stack needs to be

    // A sentry NULL void pointer at the top of the stack.
    int sentry_size = intSize;

    // This is the name of the file which is present on the initial stack
    // It's purpose is to let the user space linker examine the original file.
    int file_name_size = filename.size() + 1;

    const int numRandomBytes = 16;
    int aux_data_size = numRandomBytes;

    std::string platform = "x86_64";
    aux_data_size += platform.size() + 1;

    int env_data_size = 0;
    for (int i = 0; i < envp.size(); ++i)
        env_data_size += envp[i].size() + 1;
    int arg_data_size = 0;
    for (int i = 0; i < argv.size(); ++i)
        arg_data_size += argv[i].size() + 1;

    // The info_block needs to be padded so its size is a multiple of the
    // alignment mask. Also, it appears that there needs to be at least some
    // padding, so if the size is already a multiple, we need to increase it
    // anyway.
    int base_info_block_size =
        sentry_size + file_name_size + env_data_size + arg_data_size;

    int info_block_size = roundUp(base_info_block_size, align);

    int info_block_padding = info_block_size - base_info_block_size;

    // Each auxiliary vector is two 8 byte words
    int aux_array_size = intSize * 2 * (auxv.size() + 1);

    int envp_array_size = intSize * (envp.size() + 1);
    int argv_array_size = intSize * (argv.size() + 1);

    int argc_size = intSize;

    // Figure out the size of the contents of the actual initial frame
    int frame_size =
        aux_array_size +
        envp_array_size +
        argv_array_size +
        argc_size;

    // There needs to be padding after the auxiliary vector data so that the
    // very bottom of the stack is aligned properly.
    int partial_size = frame_size + aux_data_size;
    int aligned_partial_size = roundUp(partial_size, align);
    int aux_padding = aligned_partial_size - partial_size;

    int space_needed =
        info_block_size +
        aux_data_size +
        aux_padding +
        frame_size;

    Addr stack_base = memState->getStackBase();

    Addr stack_min = stack_base - space_needed;
    stack_min = roundDown(stack_min, align);

    unsigned stack_size = stack_base - stack_min;
    stack_size = roundUp(stack_size, pageSize);
    memState->setStackSize(stack_size);

    // map memory
    Addr stack_end = roundDown(stack_base - stack_size, pageSize);

    DPRINTF(Stack, "Mapping the stack: 0x%x %dB\n", stack_end, stack_size);
    memState->mapRegion(stack_end, stack_size, "stack");

    // map out initial stack contents
    IntType sentry_base = stack_base - sentry_size;
    IntType file_name_base = sentry_base - file_name_size;
    IntType env_data_base = file_name_base - env_data_size;
    IntType arg_data_base = env_data_base - arg_data_size;
    IntType aux_data_base = arg_data_base - info_block_padding - aux_data_size;
    IntType auxv_array_base = aux_data_base - aux_array_size - aux_padding;
    IntType envp_array_base = auxv_array_base - envp_array_size;
    IntType argv_array_base = envp_array_base - argv_array_size;
    IntType argc_base = argv_array_base - argc_size;

    DPRINTF(Stack, "The addresses of items on the initial stack:\n");
    DPRINTF(Stack, "0x%x - file name\n", file_name_base);
    DPRINTF(Stack, "0x%x - env data\n", env_data_base);
    DPRINTF(Stack, "0x%x - arg data\n", arg_data_base);
    DPRINTF(Stack, "0x%x - aux data\n", aux_data_base);
    DPRINTF(Stack, "0x%x - auxv array\n", auxv_array_base);
    DPRINTF(Stack, "0x%x - envp array\n", envp_array_base);
    DPRINTF(Stack, "0x%x - argv array\n", argv_array_base);
    DPRINTF(Stack, "0x%x - argc \n", argc_base);
    DPRINTF(Stack, "0x%x - stack min\n", stack_min);

    // write contents to stack

    // figure out argc
    IntType argc = argv.size();
    IntType guestArgc = htole(argc);

    // Write out the sentry void *
    IntType sentry_NULL = 0;
    initVirtMem->writeBlob(sentry_base, &sentry_NULL, sentry_size);

    // Write the file name
    initVirtMem->writeString(file_name_base, filename.c_str());

    // Fix up the aux vectors which point to data
    assert(auxv[auxv.size() - 3].type == gem5::auxv::Random);
    auxv[auxv.size() - 3].val = aux_data_base;
    assert(auxv[auxv.size() - 2].type == gem5::auxv::Execfn);
    auxv[auxv.size() - 2].val = argv_array_base;
    assert(auxv[auxv.size() - 1].type == gem5::auxv::Platform);
    auxv[auxv.size() - 1].val = aux_data_base + numRandomBytes;


    // Copy the aux stuff
    Addr auxv_array_end = auxv_array_base;
    for (const auto &aux: auxv) {
        initVirtMem->write(auxv_array_end, aux, ByteOrder::little);
        auxv_array_end += sizeof(aux);
    }
    // Write out the terminating zeroed auxiliary vector
    const gem5::auxv::AuxVector<uint64_t> zero(0, 0);
    initVirtMem->write(auxv_array_end, zero);
    auxv_array_end += sizeof(zero);

    initVirtMem->writeString(aux_data_base + numRandomBytes,
                             platform.c_str());

    copyStringArray(envp, envp_array_base, env_data_base,
                    ByteOrder::little, *initVirtMem);
    copyStringArray(argv, argv_array_base, arg_data_base,
                    ByteOrder::little, *initVirtMem);

    initVirtMem->writeBlob(argc_base, &guestArgc, intSize);

    ThreadContext *tc = system->threads[contextIds[0]];
    // Set the stack pointer register
    tc->setReg(int_reg::Rsp, stack_min);

    // There doesn't need to be any segment base added in since we're dealing
    // with the flat segmentation model.
    tc->pcState(getStartPC());

    // Align the "stack_min" to a page boundary.
    memState->setStackMin(roundDown(stack_min, pageSize));
}

void
X86_64Process::argsInit(int pageSize)
{
    std::vector<gem5::auxv::AuxVector<uint64_t> > extraAuxvs;
    extraAuxvs.emplace_back(auxv::SysinfoEhdr, vsyscallPage.base);
    X86Process::argsInit<uint64_t>(pageSize, extraAuxvs);
}

void
I386Process::argsInit(int pageSize)
{
    std::vector<gem5::auxv::AuxVector<uint32_t> > extraAuxvs;
    //Tell the binary where the vsyscall part of the vsyscall page is.
    extraAuxvs.emplace_back(auxv::Sysinfo,
            vsyscallPage.base + vsyscallPage.vsyscallOffset);
    extraAuxvs.emplace_back(auxv::SysinfoEhdr, vsyscallPage.base);
    X86Process::argsInit<uint32_t>(pageSize, extraAuxvs);
}

void
X86_64Process::clone(ThreadContext *old_tc, ThreadContext *new_tc,
                     Process *p, RegVal flags)
{
    X86Process::clone(old_tc, new_tc, p, flags);
    ((X86_64Process*)p)->vsyscallPage = vsyscallPage;
}

void
I386Process::clone(ThreadContext *old_tc, ThreadContext *new_tc,
                   Process *p, RegVal flags)
{
    X86Process::clone(old_tc, new_tc, p, flags);
    ((I386Process*)p)->vsyscallPage = vsyscallPage;
}

} // namespace gem5
