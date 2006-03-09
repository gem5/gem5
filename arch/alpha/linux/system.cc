/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
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

/**
 * @file
 * This code loads the linux kernel, console, pal and patches certain
 * functions.  The symbol tables are loaded so that traces can show
 * the executing function and we can skip functions. Various delay
 * loops are skipped and their final values manually computed to speed
 * up boot time.
 */

#include "arch/arguments.hh"
#include "arch/vtophys.hh"
#include "arch/alpha/linux/system.hh"
#include "arch/alpha/linux/threadinfo.hh"
#include "arch/alpha/system.hh"
#include "base/loader/symtab.hh"
#include "cpu/exec_context.hh"
#include "cpu/base.hh"
#include "dev/platform.hh"
#include "kern/linux/printk.hh"
#include "kern/linux/events.hh"
#include "mem/functional/memory_control.hh"
#include "mem/functional/physical.hh"
#include "sim/builder.hh"
#include "sim/byteswap.hh"

using namespace std;
using namespace AlphaISA;
using namespace Linux;

LinuxAlphaSystem::LinuxAlphaSystem(Params *p)
    : AlphaSystem(p)
{
    Addr addr = 0;
    Addr paddr = 0;

    /**
     * The symbol swapper_pg_dir marks the beginning of the kernel and
     * the location of bootloader passed arguments
     */
    if (!kernelSymtab->findAddress("swapper_pg_dir", KernelStart)) {
        panic("Could not determine start location of kernel");
    }

    /**
     * Since we aren't using a bootloader, we have to copy the
     * kernel arguments directly into the kernel's memory.
     */
    paddr = vtophys(physmem, CommandLine());
    char *commandline = (char *)physmem->dma_addr(paddr, sizeof(uint64_t));
    if (commandline)
        strncpy(commandline, params()->boot_osflags.c_str(), CommandLineSize);

    /**
     * find the address of the est_cycle_freq variable and insert it
     * so we don't through the lengthly process of trying to
     * calculated it by using the PIT, RTC, etc.
     */
    if (kernelSymtab->findAddress("est_cycle_freq", addr)) {
        paddr = vtophys(physmem, addr);
        uint8_t *est_cycle_frequency =
            physmem->dma_addr(paddr, sizeof(uint64_t));

        if (est_cycle_frequency)
            *(uint64_t *)est_cycle_frequency =
                Clock::Frequency / p->boot_cpu_frequency;
    }


    /**
     * EV5 only supports 127 ASNs so we are going to tell the kernel that the
     * paritiuclar EV6 we have only supports 127 asns.
     * @todo At some point we should change ev5.hh and the palcode to support
     * 255 ASNs.
     */
    if (kernelSymtab->findAddress("dp264_mv", addr)) {
        paddr = vtophys(physmem, addr);
        char *dp264_mv = (char *)physmem->dma_addr(paddr, sizeof(uint64_t));

        if (dp264_mv) {
            *(uint32_t*)(dp264_mv+0x18) = LittleEndianGuest::htog((uint32_t)127);
        } else
            panic("could not translate dp264_mv addr\n");

    } else
        panic("could not find dp264_mv\n");

#ifndef NDEBUG
    kernelPanicEvent = addKernelFuncEvent<BreakPCEvent>("panic");
    if (!kernelPanicEvent)
        panic("could not find kernel symbol \'panic\'");

#if 0
    kernelDieEvent = addKernelFuncEvent<BreakPCEvent>("die_if_kernel");
    if (!kernelDieEvent)
        panic("could not find kernel symbol \'die_if_kernel\'");
#endif

#endif

    /**
     * Any time ide_delay_50ms, calibarte_delay or
     * determine_cpu_caches is called just skip the
     * function. Currently determine_cpu_caches only is used put
     * information in proc, however if that changes in the future we
     * will have to fill in the cache size variables appropriately.
     */

    skipIdeDelay50msEvent =
        addKernelFuncEvent<SkipFuncEvent>("ide_delay_50ms");
    skipDelayLoopEvent =
        addKernelFuncEvent<SkipDelayLoopEvent>("calibrate_delay");
    skipCacheProbeEvent =
        addKernelFuncEvent<SkipFuncEvent>("determine_cpu_caches");
    debugPrintkEvent = addKernelFuncEvent<DebugPrintkEvent>("dprintk");
    idleStartEvent = addKernelFuncEvent<IdleStartEvent>("cpu_idle");

    if (kernelSymtab->findAddress("alpha_switch_to", addr) && DTRACE(Thread)) {
        printThreadEvent = new PrintThreadInfo(&pcEventQueue, "threadinfo",
                                               addr + sizeof(MachInst) * 6);
    } else {
        printThreadEvent = NULL;
    }

    if (params()->bin_int) {
        intStartEvent = addPalFuncEvent<InterruptStartEvent>("sys_int_21");
        if (!intStartEvent)
            panic("could not find symbol: sys_int_21\n");

        intEndEvent = addPalFuncEvent<InterruptEndEvent>("rti_to_kern");
        if (!intEndEvent)
            panic("could not find symbol: rti_to_kern\n");

        intEndEvent2 = addPalFuncEvent<InterruptEndEvent>("rti_to_user");
        if (!intEndEvent2)
            panic("could not find symbol: rti_to_user\n");

        intEndEvent3 = addKernelFuncEvent<InterruptEndEvent>("do_softirq");
        if (!intEndEvent3)
            panic("could not find symbol: do_softirq\n");
    }
}

LinuxAlphaSystem::~LinuxAlphaSystem()
{
#ifndef NDEBUG
    delete kernelPanicEvent;
#endif
    delete skipIdeDelay50msEvent;
    delete skipDelayLoopEvent;
    delete skipCacheProbeEvent;
    delete debugPrintkEvent;
    delete idleStartEvent;
    delete printThreadEvent;
    delete intStartEvent;
    delete intEndEvent;
    delete intEndEvent2;
}


void
LinuxAlphaSystem::setDelayLoop(ExecContext *xc)
{
    Addr addr = 0;
    if (kernelSymtab->findAddress("loops_per_jiffy", addr)) {
        Addr paddr = vtophys(physmem, addr);

        uint8_t *loops_per_jiffy =
            physmem->dma_addr(paddr, sizeof(uint32_t));

        Tick cpuFreq = xc->getCpuPtr()->frequency();
        Tick intrFreq = platform->intrFrequency();
        *(uint32_t *)loops_per_jiffy =
            (uint32_t)((cpuFreq / intrFreq) * 0.9988);
    }
}


void
LinuxAlphaSystem::SkipDelayLoopEvent::process(ExecContext *xc)
{
    SkipFuncEvent::process(xc);
    // calculate and set loops_per_jiffy
    ((LinuxAlphaSystem *)xc->getSystemPtr())->setDelayLoop(xc);
}

void
LinuxAlphaSystem::PrintThreadInfo::process(ExecContext *xc)
{
    Linux::ThreadInfo ti(xc);

    DPRINTF(Thread, "Currently Executing Thread %s, pid %d, started at: %d\n",
            ti.curTaskName(), ti.curTaskPID(), ti.curTaskStart());
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(LinuxAlphaSystem)

    Param<Tick> boot_cpu_frequency;
    SimObjectParam<MemoryController *> memctrl;
    SimObjectParam<PhysicalMemory *> physmem;

    Param<string> kernel;
    Param<string> console;
    Param<string> pal;

    Param<string> boot_osflags;
    Param<string> readfile;
    Param<unsigned int> init_param;

    Param<uint64_t> system_type;
    Param<uint64_t> system_rev;

    Param<bool> bin;
    VectorParam<string> binned_fns;
    Param<bool> bin_int;

END_DECLARE_SIM_OBJECT_PARAMS(LinuxAlphaSystem)

BEGIN_INIT_SIM_OBJECT_PARAMS(LinuxAlphaSystem)

    INIT_PARAM(boot_cpu_frequency, "Frequency of the boot CPU"),
    INIT_PARAM(memctrl, "memory controller"),
    INIT_PARAM(physmem, "phsyical memory"),
    INIT_PARAM(kernel, "file that contains the kernel code"),
    INIT_PARAM(console, "file that contains the console code"),
    INIT_PARAM(pal, "file that contains palcode"),
    INIT_PARAM_DFLT(boot_osflags, "flags to pass to the kernel during boot",
                    "a"),
    INIT_PARAM_DFLT(readfile, "file to read startup script from", ""),
    INIT_PARAM_DFLT(init_param, "numerical value to pass into simulator", 0),
    INIT_PARAM_DFLT(system_type, "Type of system we are emulating", 34),
    INIT_PARAM_DFLT(system_rev, "Revision of system we are emulating", 1<<10),
    INIT_PARAM_DFLT(bin, "is this system to be binned", false),
    INIT_PARAM(binned_fns, "functions to be broken down and binned"),
    INIT_PARAM_DFLT(bin_int, "is interrupt code binned seperately?", true)

END_INIT_SIM_OBJECT_PARAMS(LinuxAlphaSystem)

CREATE_SIM_OBJECT(LinuxAlphaSystem)
{
    AlphaSystem::Params *p = new AlphaSystem::Params;
    p->name = getInstanceName();
    p->boot_cpu_frequency = boot_cpu_frequency;
    p->memctrl = memctrl;
    p->physmem = physmem;
    p->kernel_path = kernel;
    p->console_path = console;
    p->palcode = pal;
    p->boot_osflags = boot_osflags;
    p->init_param = init_param;
    p->readfile = readfile;
    p->system_type = system_type;
    p->system_rev = system_rev;
    p->bin = bin;
    p->binned_fns = binned_fns;
    p->bin_int = bin_int;
    return new LinuxAlphaSystem(p);
}

REGISTER_SIM_OBJECT("LinuxAlphaSystem", LinuxAlphaSystem)

