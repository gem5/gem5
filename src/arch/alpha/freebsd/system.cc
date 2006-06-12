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
 *
 * Authors: Ben Nash
 */

/**
 * @file
 * Modifications for the FreeBSD kernel.
 * Based on kern/linux/linux_system.cc.
 *
 */

#include "arch/alpha/system.hh"
#include "arch/alpha/freebsd/system.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "arch/isa_traits.hh"
#include "sim/builder.hh"
#include "sim/byteswap.hh"
#include "arch/vtophys.hh"

#define TIMER_FREQUENCY 1193180

using namespace std;
using namespace AlphaISA;

FreebsdAlphaSystem::FreebsdAlphaSystem(Params *p)
    : AlphaSystem(p)
{
    /**
     * Any time DELAY is called just skip the function.
     * Shouldn't we actually emulate the delay?
     */
    skipDelayEvent = addKernelFuncEvent<SkipFuncEvent>("DELAY");
    skipCalibrateClocks =
        addKernelFuncEvent<SkipCalibrateClocksEvent>("calibrate_clocks");
}


FreebsdAlphaSystem::~FreebsdAlphaSystem()
{
    delete skipDelayEvent;
    delete skipCalibrateClocks;
}


void
FreebsdAlphaSystem::doCalibrateClocks(ThreadContext *tc)
{
    Addr ppc_vaddr = 0;
    Addr timer_vaddr = 0;

    ppc_vaddr = (Addr)tc->readIntReg(ArgumentReg1);
    timer_vaddr = (Addr)tc->readIntReg(ArgumentReg2);

    virtPort.write(ppc_vaddr, (uint32_t)Clock::Frequency);
    virtPort.write(timer_vaddr, (uint32_t)TIMER_FREQUENCY);
}


void
FreebsdAlphaSystem::SkipCalibrateClocksEvent::process(ThreadContext *tc)
{
    SkipFuncEvent::process(tc);
    ((FreebsdAlphaSystem *)tc->getSystemPtr())->doCalibrateClocks(tc);
}


BEGIN_DECLARE_SIM_OBJECT_PARAMS(FreebsdAlphaSystem)

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

END_DECLARE_SIM_OBJECT_PARAMS(FreebsdAlphaSystem)

BEGIN_INIT_SIM_OBJECT_PARAMS(FreebsdAlphaSystem)

    INIT_PARAM(boot_cpu_frequency, "Frequency of the boot CPU"),
    INIT_PARAM(physmem, "phsyical memory"),
    INIT_PARAM(kernel, "file that contains the kernel code"),
    INIT_PARAM(console, "file that contains the console code"),
    INIT_PARAM(pal, "file that contains palcode"),
    INIT_PARAM_DFLT(boot_osflags, "flags to pass to the kernel during boot",
                    "a"),
    INIT_PARAM_DFLT(readfile, "file to read startup script from", ""),
    INIT_PARAM_DFLT(init_param, "numerical value to pass into simulator", 0),
    INIT_PARAM_DFLT(system_type, "Type of system we are emulating", 34),
    INIT_PARAM_DFLT(system_rev, "Revision of system we are emulating", 1<<10)

END_INIT_SIM_OBJECT_PARAMS(FreebsdAlphaSystem)

CREATE_SIM_OBJECT(FreebsdAlphaSystem)
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
    return new FreebsdAlphaSystem(p);
}

REGISTER_SIM_OBJECT("FreebsdAlphaSystem", FreebsdAlphaSystem)

