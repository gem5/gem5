/*
 * Copyright (c) 2015 Ruslan Bukin <br@bsdpad.com>
 * All rights reserved.
 *
 * This software was developed by the University of Cambridge Computer
 * Laboratory as part of the CTSRD Project, with support from the UK Higher
 * Education Innovation Fund (HEIF).
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

#include "arch/arm/freebsd/system.hh"

#include "arch/arm/isa_traits.hh"
#include "arch/arm/utility.hh"
#include "arch/generic/freebsd/threadinfo.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/pc_event.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "kern/freebsd/events.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/physical.hh"
#include "sim/stat_control.hh"

using namespace ArmISA;
using namespace FreeBSD;

FreebsdArmSystem::FreebsdArmSystem(Params *p)
    : GenericArmSystem(p), dumpStatsPCEventF(nullptr),
      enableContextSwitchStatsDump(p->enable_context_switch_stats_dump),
      taskFile(nullptr), kernelPanicEvent(nullptr), kernelOopsEvent(nullptr)
{
    if (p->panic_on_panic) {
        kernelPanicEvent = addKernelFuncEventOrPanic<PanicPCEvent>(
            "panic", "Kernel panic in simulated kernel");
    } else {
#ifndef NDEBUG
        kernelPanicEvent = addKernelFuncEventOrPanic<BreakPCEvent>("panic");
#endif
    }

    if (p->panic_on_oops) {
        kernelOopsEvent = addKernelFuncEventOrPanic<PanicPCEvent>(
            "oops_exit", "Kernel oops in guest");
    }

    uDelaySkipEvent = addKernelFuncEvent<UDelayEvent>(
        "DELAY", "DELAY", 1000, 0);
}

void
FreebsdArmSystem::initState()
{
    // Moved from the constructor to here since it relies on the
    // address map being resolved in the interconnect

    // Call the initialisation of the super class
    GenericArmSystem::initState();

    // Load symbols at physical address, we might not want
    // to do this permanently, for but early bootup work
    // it is helpful.
    if (params()->early_kernel_symbols) {
        kernel->loadGlobalSymbols(kernelSymtab, 0, 0, loadAddrMask);
        kernel->loadGlobalSymbols(debugSymbolTable, 0, 0, loadAddrMask);
    }

    // Setup boot data structure
    Addr addr = 0;

    // Check if the kernel image has a symbol that tells us it supports
    // device trees.
    bool kernel_has_fdt_support =
        kernelSymtab->findAddress("fdt_get_range", addr);
    bool dtb_file_specified = params()->dtb_filename != "";

    if (!dtb_file_specified)
        fatal("dtb file is not specified\n");

    if (!kernel_has_fdt_support)
        fatal("kernel must have fdt support\n");

    // Kernel supports flattened device tree and dtb file specified.
    // Using Device Tree Blob to describe system configuration.
    inform("Loading DTB file: %s at address %#x\n", params()->dtb_filename,
            params()->atags_addr + loadAddrOffset);

    ObjectFile *dtb_file = createObjectFile(params()->dtb_filename, true);
    if (!dtb_file) {
        fatal("couldn't load DTB file: %s\n", params()->dtb_filename);
    }

    DtbObject *_dtb_file = dynamic_cast<DtbObject*>(dtb_file);

    if (_dtb_file) {
        if (!_dtb_file->addBootCmdLine(params()->boot_osflags.c_str(),
                                       params()->boot_osflags.size())) {
            warn("couldn't append bootargs to DTB file: %s\n",
                 params()->dtb_filename);
        }
    } else {
        warn("dtb_file cast failed; couldn't append bootargs "
             "to DTB file: %s\n", params()->dtb_filename);
    }

    Addr ra = _dtb_file->findReleaseAddr();
    if (ra)
        bootReleaseAddr = ra & ~ULL(0x7F);

    dtb_file->setTextBase(params()->atags_addr + loadAddrOffset);
    dtb_file->loadSections(physProxy);
    delete dtb_file;

    // Kernel boot requirements to set up r0, r1 and r2 in ARMv7
    for (int i = 0; i < threadContexts.size(); i++) {
        threadContexts[i]->setIntReg(0, 0);
        threadContexts[i]->setIntReg(1, params()->machine_type);
        threadContexts[i]->setIntReg(2, params()->atags_addr + loadAddrOffset);
    }
}

FreebsdArmSystem::~FreebsdArmSystem()
{
    if (uDelaySkipEvent)
        delete uDelaySkipEvent;
    if (constUDelaySkipEvent)
        delete constUDelaySkipEvent;

    if (dumpStatsPCEventF)
        delete dumpStatsPCEventF;
}

FreebsdArmSystem *
FreebsdArmSystemParams::create()
{
    return new FreebsdArmSystem(this);
}

void
FreebsdArmSystem::startup()
{
}
