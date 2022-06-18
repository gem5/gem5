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

#include "arch/arm/freebsd/fs_workload.hh"

#include "arch/arm/utility.hh"
#include "arch/generic/freebsd/threadinfo.hh"
#include "base/loader/dtb_file.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/pc_event.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "kern/freebsd/events.hh"
#include "mem/physical.hh"
#include "sim/stat_control.hh"

namespace gem5
{

using namespace free_bsd;

namespace ArmISA
{

FsFreebsd::FsFreebsd(const Params &p) : ArmISA::FsWorkload(p),
    enableContextSwitchStatsDump(p.enable_context_switch_stats_dump)
{
    if (p.panic_on_panic) {
        kernelPanic = addKernelFuncEventOrPanic<PanicPCEvent>(
            "panic", "Kernel panic in simulated kernel");
    } else {
#ifndef NDEBUG
        kernelPanic = addKernelFuncEventOrPanic<BreakPCEvent>("panic");
#endif
    }

    if (p.panic_on_oops) {
        kernelOops = addKernelFuncEventOrPanic<PanicPCEvent>(
            "oops_exit", "Kernel oops in guest");
    }

    skipUDelay = addSkipFunc<SkipUDelay>("DELAY", "DELAY", 1000, 0);
}

void
FsFreebsd::initState()
{
    ArmISA::FsWorkload::initState();

    // Load symbols at physical address, we might not want
    // to do this permanently, for but early bootup work
    // it is helpful.
    if (params().early_kernel_symbols) {
        auto phys_globals = kernelObj->symtab().globals()->mask(_loadAddrMask);
        kernelSymtab.insert(*phys_globals);
        loader::debugSymbolTable.insert(*phys_globals);
    }

    // Check if the kernel image has a symbol that tells us it supports
    // device trees.
    fatal_if(kernelSymtab.find("fdt_get_range") == kernelSymtab.end(),
             "Kernel must have fdt support.");
    fatal_if(params().dtb_filename == "", "dtb file is not specified.");

    // Kernel supports flattened device tree and dtb file specified.
    // Using Device Tree Blob to describe system configuration.
    inform("Loading DTB file: %s at address %#x\n", params().dtb_filename,
            params().dtb_addr);

    auto *dtb_file = new loader::DtbFile(params().dtb_filename);

    warn_if(!dtb_file->addBootCmdLine(commandLine.c_str(), commandLine.size()),
            "Couldn't append bootargs to DTB file: %s",
            params().dtb_filename);

    Addr ra = dtb_file->findReleaseAddr();
    if (ra)
        bootReleaseAddr = ra & ~0x7FULL;

    dtb_file->buildImage().
        offset(params().dtb_addr).
        write(system->physProxy);
    delete dtb_file;

    // Kernel boot requirements to set up r0, r1 and r2 in ARMv7
    for (auto *tc: system->threads) {
        tc->setReg(int_reg::R0, (RegVal)0);
        tc->setReg(int_reg::R1, params().machine_type);
        tc->setReg(int_reg::R2, params().dtb_addr);
    }
}

FsFreebsd::~FsFreebsd()
{
    delete skipUDelay;
}

} // namespace ArmISA
} // namespace gem5
