/*
 * Copyright (c) 2010 ARM Limited
 * All rights reserved
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
 * Copyright (c) 2002-2006 The Regents of The University of Michigan
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
 * Authors: Ali Saidi
 */

#include "arch/arm/linux/atag.hh"
#include "arch/arm/linux/system.hh"
#include "arch/arm/isa_traits.hh"
#include "arch/arm/utility.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "kern/linux/events.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/physical.hh"

using namespace ArmISA;
using namespace Linux;

LinuxArmSystem::LinuxArmSystem(Params *p)
    : ArmSystem(p)
{
#ifndef NDEBUG
    kernelPanicEvent = addKernelFuncEvent<BreakPCEvent>("panic");
    if (!kernelPanicEvent)
        panic("could not find kernel symbol \'panic\'");
#endif

    // With ARM udelay() is #defined to __udelay
    Addr addr = 0;
    if (kernelSymtab->findAddress("__udelay", addr)) {
        uDelaySkipEvent = new UDelayEvent(&pcEventQueue, "__udelay",
                fixFuncEventAddr(addr), 1000, 0);
    } else {
        panic("couldn't find kernel symbol \'udelay\'");
    }

    // constant arguments to udelay() have some precomputation done ahead of
    // time. Constant comes from code.
    if (kernelSymtab->findAddress("__const_udelay", addr)) {
        constUDelaySkipEvent = new UDelayEvent(&pcEventQueue, "__const_udelay",
                fixFuncEventAddr(addr), 1000, 107374);
    } else {
        panic("couldn't find kernel symbol \'udelay\'");
    }

    secDataPtrAddr = 0;
    secDataAddr = 0;
    penReleaseAddr = 0;
    kernelSymtab->findAddress("__secondary_data", secDataPtrAddr);
    kernelSymtab->findAddress("secondary_data", secDataAddr);
    kernelSymtab->findAddress("pen_release", penReleaseAddr);

    secDataPtrAddr &= ~ULL(0x7F);
    secDataAddr &= ~ULL(0x7F);
    penReleaseAddr &= ~ULL(0x7F);
}

bool
LinuxArmSystem::adderBootUncacheable(Addr a)
{
    Addr block = a & ~ULL(0x7F);
    if (block == secDataPtrAddr || block == secDataAddr ||
            block == penReleaseAddr)
        return true;
    return false;
}

void
LinuxArmSystem::initState()
{
    // Moved from the constructor to here since it relies on the
    // address map being resolved in the interconnect

    // Call the initialisation of the super class
    ArmSystem::initState();

    // Load symbols at physical address, we might not want
    // to do this permanently, for but early bootup work
    // it is helpful.
    if (params()->early_kernel_symbols) {
        kernel->loadGlobalSymbols(kernelSymtab, loadAddrMask);
        kernel->loadGlobalSymbols(debugSymbolTable, loadAddrMask);
    }

    // Setup boot data structure
    AtagCore *ac = new AtagCore;
    ac->flags(1); // read-only
    ac->pagesize(8192);
    ac->rootdev(0);

    AddrRangeList atagRanges = physmem.getConfAddrRanges();
    if (atagRanges.size() != 1) {
        fatal("Expected a single ATAG memory entry but got %d\n",
              atagRanges.size());
    }
    AtagMem *am = new AtagMem;
    am->memSize(atagRanges.begin()->size());
    am->memStart(atagRanges.begin()->start);

    AtagCmdline *ad = new AtagCmdline;
    ad->cmdline(params()->boot_osflags);

    DPRINTF(Loader, "boot command line %d bytes: %s\n", ad->size() <<2, params()->boot_osflags.c_str());

    AtagNone *an = new AtagNone;

    uint32_t size = ac->size() + am->size() + ad->size() + an->size();
    uint32_t offset = 0;
    uint8_t *boot_data = new uint8_t[size << 2];

    offset += ac->copyOut(boot_data + offset);
    offset += am->copyOut(boot_data + offset);
    offset += ad->copyOut(boot_data + offset);
    offset += an->copyOut(boot_data + offset);

    DPRINTF(Loader, "Boot atags was %d bytes in total\n", size << 2);
    DDUMP(Loader, boot_data, size << 2);

    physProxy.writeBlob(params()->atags_addr, boot_data, size << 2);

    for (int i = 0; i < threadContexts.size(); i++) {
        threadContexts[i]->setIntReg(0, 0);
        threadContexts[i]->setIntReg(1, params()->machine_type);
        threadContexts[i]->setIntReg(2, params()->atags_addr);
    }
}

LinuxArmSystem::~LinuxArmSystem()
{
    if (uDelaySkipEvent)
        delete uDelaySkipEvent;
    if (constUDelaySkipEvent)
        delete constUDelaySkipEvent;
}

LinuxArmSystem *
LinuxArmSystemParams::create()
{
    return new LinuxArmSystem(this);
}
