/*
 * Copyright 2020 Google Inc.
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

#include "arch/x86/linux/syscalls.hh"

#include "arch/x86/linux/linux.hh"
#include "arch/x86/process.hh"
#include "arch/x86/regs/misc.hh"
#include "base/trace.hh"
#include "cpu/thread_context.hh"
#include "kern/linux/linux.hh"
#include "mem/se_translating_port_proxy.hh"
#include "sim/process.hh"
#include "sim/syscall_desc.hh"
#include "sim/syscall_emul.hh"

namespace gem5
{

namespace X86ISA
{

/// Target uname() handler.
SyscallReturn
unameFunc(SyscallDesc *desc, ThreadContext *tc, VPtr<Linux::utsname> name)
{
    auto process = tc->getProcessPtr();

    strcpy(name->sysname, "Linux");
    strcpy(name->nodename, "sim.gem5.org");
    strcpy(name->release, process->release.c_str());
    strcpy(name->version, "#1 Mon Aug 18 11:32:15 EDT 2003");
    strcpy(name->machine, "x86_64");

    return 0;
}

SyscallReturn
archPrctlFunc(SyscallDesc *desc, ThreadContext *tc, int code, uint64_t addr)
{
    enum ArchPrctlCodes
    {
        SetFS = 0x1002,
        GetFS = 0x1003,
        SetGS = 0x1001,
        GetGS = 0x1004
    };

    uint64_t fsBase, gsBase;
    SETranslatingPortProxy p(tc);
    switch (code) {
    // Each of these valid options should actually check addr.
    case SetFS:
        tc->setMiscRegNoEffect(misc_reg::FsBase, addr);
        tc->setMiscRegNoEffect(misc_reg::FsEffBase, addr);
        return 0;
    case GetFS:
        fsBase = tc->readMiscRegNoEffect(misc_reg::FsBase);
        p.write(addr, fsBase);
        return 0;
    case SetGS:
        tc->setMiscRegNoEffect(misc_reg::GsBase, addr);
        tc->setMiscRegNoEffect(misc_reg::GsEffBase, addr);
        return 0;
    case GetGS:
        gsBase = tc->readMiscRegNoEffect(misc_reg::GsBase);
        p.write(addr, gsBase);
        return 0;
    default:
        return -EINVAL;
    }
}

SyscallReturn
setThreadArea32Func(SyscallDesc *desc, ThreadContext *tc,
                    VPtr<UserDesc32> userDesc)
{
    const int minTLSEntry = 6;
    const int numTLSEntries = 3;
    const int maxTLSEntry = minTLSEntry + numTLSEntries - 1;

    auto process = tc->getProcessPtr();
    SETranslatingPortProxy proxy(tc);

    X86Process *x86p = dynamic_cast<X86Process *>(process);
    assert(x86p);

    assert((maxTLSEntry + 1) * sizeof(uint64_t) <= x86p->gdtSize());

    TypedBufferArg<uint64_t> gdt(x86p->gdtStart() +
                                     minTLSEntry * sizeof(uint64_t),
                                 numTLSEntries * sizeof(uint64_t));

    if (!gdt.copyIn(proxy))
        panic("Failed to copy in GDT for %s.\n", desc->name());

    if (userDesc->entry_number == (uint32_t)(-1)) {
        // Find a free TLS entry.
        for (int i = 0; i < numTLSEntries; i++) {
            if (gdt[i] == 0) {
                userDesc->entry_number = i + minTLSEntry;
                break;
            }
        }
        // We failed to find one.
        if (userDesc->entry_number == (uint32_t)(-1))
            return -ESRCH;
    }

    int index = userDesc->entry_number;

    if (index < minTLSEntry || index > maxTLSEntry)
        return -EINVAL;

    index -= minTLSEntry;

    // Build the entry we're going to add.
    SegDescriptor segDesc = 0;
    UserDescFlags flags = userDesc->flags;

    segDesc.limitLow = bits(userDesc->limit, 15, 0);
    segDesc.baseLow = bits(userDesc->base_addr, 23, 0);
    segDesc.type.a = 1;
    if (!flags.read_exec_only)
        segDesc.type.w = 1;
    if (bits((uint8_t)flags.contents, 0))
        segDesc.type.e = 1;
    if (bits((uint8_t)flags.contents, 1))
        segDesc.type.codeOrData = 1;
    segDesc.s = 1;
    segDesc.dpl = 3;
    if (!flags.seg_not_present)
        segDesc.p = 1;
    segDesc.limitHigh = bits(userDesc->limit, 19, 16);
    if (flags.useable)
        segDesc.avl = 1;
    segDesc.l = 0;
    if (flags.seg_32bit)
        segDesc.d = 1;
    if (flags.limit_in_pages)
        segDesc.g = 1;
    segDesc.baseHigh = bits(userDesc->base_addr, 31, 24);

    gdt[index] = (uint64_t)segDesc;

    if (!gdt.copyOut(proxy))
        panic("Failed to copy out GDT for %s.\n", desc->name());

    return 0;
}

} // namespace X86ISA
} // namespace gem5
