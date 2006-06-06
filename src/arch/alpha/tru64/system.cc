/*
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Authors: Nathan Binkert
 *          Lisa Hsu
 */

#include "arch/alpha/tru64/system.hh"
#include "arch/isa_traits.hh"
#include "arch/vtophys.hh"
#include "base/loader/symtab.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "kern/tru64/tru64_events.hh"
#include "kern/system_events.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/builder.hh"

using namespace std;

Tru64AlphaSystem::Tru64AlphaSystem(Tru64AlphaSystem::Params *p)
    : AlphaSystem(p)
{
    Addr addr = 0;
    if (kernelSymtab->findAddress("enable_async_printf", addr)) {
        virtPort.write(addr, (uint32_t)0);
    }

#ifdef DEBUG
    kernelPanicEvent = addKernelFuncEvent<BreakPCEvent>("panic");
    if (!kernelPanicEvent)
        panic("could not find kernel symbol \'panic\'");
#endif

    badaddrEvent = addKernelFuncEvent<BadAddrEvent>("badaddr");
    if (!badaddrEvent)
        panic("could not find kernel symbol \'badaddr\'");

    skipPowerStateEvent =
        addKernelFuncEvent<SkipFuncEvent>("tl_v48_capture_power_state");
    skipScavengeBootEvent =
        addKernelFuncEvent<SkipFuncEvent>("pmap_scavenge_boot");

#if TRACING_ON
    printfEvent = addKernelFuncEvent<PrintfEvent>("printf");
    debugPrintfEvent = addKernelFuncEvent<DebugPrintfEvent>("m5printf");
    debugPrintfrEvent = addKernelFuncEvent<DebugPrintfrEvent>("m5printfr");
    dumpMbufEvent = addKernelFuncEvent<DumpMbufEvent>("m5_dump_mbuf");
#endif
}

Tru64AlphaSystem::~Tru64AlphaSystem()
{
#ifdef DEBUG
    delete kernelPanicEvent;
#endif
    delete badaddrEvent;
    delete skipPowerStateEvent;
    delete skipScavengeBootEvent;
#if TRACING_ON
    delete printfEvent;
    delete debugPrintfEvent;
    delete debugPrintfrEvent;
    delete dumpMbufEvent;
#endif
}

BEGIN_DECLARE_SIM_OBJECT_PARAMS(Tru64AlphaSystem)

    Param<Tick> boot_cpu_frequency;
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

END_DECLARE_SIM_OBJECT_PARAMS(Tru64AlphaSystem)

BEGIN_INIT_SIM_OBJECT_PARAMS(Tru64AlphaSystem)

    INIT_PARAM(boot_cpu_frequency, "frequency of the boot cpu"),
    INIT_PARAM(physmem, "phsyical memory"),
    INIT_PARAM(kernel, "file that contains the kernel code"),
    INIT_PARAM(console, "file that contains the console code"),
    INIT_PARAM(pal, "file that contains palcode"),
    INIT_PARAM_DFLT(boot_osflags, "flags to pass to the kernel during boot",
                    "a"),
    INIT_PARAM_DFLT(readfile, "file to read startup script from", ""),
    INIT_PARAM_DFLT(init_param, "numerical value to pass into simulator", 0),
    INIT_PARAM_DFLT(system_type, "Type of system we are emulating", 12),
    INIT_PARAM_DFLT(system_rev, "Revision of system we are emulating", 2<<1),
    INIT_PARAM_DFLT(bin, "is this system to be binned", false),
    INIT_PARAM(binned_fns, "functions to be broken down and binned")

END_INIT_SIM_OBJECT_PARAMS(Tru64AlphaSystem)

CREATE_SIM_OBJECT(Tru64AlphaSystem)
{
    AlphaSystem::Params *p = new AlphaSystem::Params;
    p->name = getInstanceName();
    p->boot_cpu_frequency = boot_cpu_frequency;
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
    p->bin_int = false;

    return new Tru64AlphaSystem(p);
}

REGISTER_SIM_OBJECT("Tru64AlphaSystem", Tru64AlphaSystem)
