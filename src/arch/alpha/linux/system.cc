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
 * Authors: Ali Saidi
 *          Lisa Hsu
 *          Nathan Binkert
 *          Steve Reinhardt
 */

/**
 * @file
 * This code loads the linux kernel, console, pal and patches certain
 * functions.  The symbol tables are loaded so that traces can show
 * the executing function and we can skip functions. Various delay
 * loops are skipped and their final values manually computed to speed
 * up boot time.
 */

#include "arch/alpha/linux/system.hh"

#include "arch/alpha/idle_event.hh"
#include "arch/alpha/system.hh"
#include "arch/generic/linux/threadinfo.hh"
#include "arch/vtophys.hh"
#include "base/loader/symtab.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/Thread.hh"
#include "kern/linux/events.hh"
#include "kern/linux/printk.hh"
#include "mem/physical.hh"
#include "mem/port.hh"
#include "sim/arguments.hh"
#include "sim/byteswap.hh"

using namespace std;
using namespace AlphaISA;
using namespace Linux;

LinuxAlphaSystem::LinuxAlphaSystem(Params *p)
    : AlphaSystem(p)
{
}

void
LinuxAlphaSystem::initState()
{
    // Moved from the constructor to here since it relies on the
    // address map being resolved in the interconnect

    // Call the initialisation of the super class
    AlphaSystem::initState();

    Addr addr = 0;

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
    virtProxy.writeBlob(CommandLine(),
                        params()->boot_osflags.c_str(),
                        params()->boot_osflags.length() + 1);

    /**
     * find the address of the est_cycle_freq variable and insert it
     * so we don't through the lengthly process of trying to
     * calculated it by using the PIT, RTC, etc.
     */
    if (kernelSymtab->findAddress("est_cycle_freq", addr))
        virtProxy.write(addr, (uint64_t)(SimClock::Frequency /
                                         params()->boot_cpu_frequency));


    /**
     * EV5 only supports 127 ASNs so we are going to tell the kernel that the
     * paritiuclar EV6 we have only supports 127 asns.
     * @todo At some point we should change ev5.hh and the palcode to support
     * 255 ASNs.
     */
    if (kernelSymtab->findAddress("dp264_mv", addr))
        virtProxy.write(addr + 0x18, LittleEndianGuest::htog((uint32_t)127));
    else
        panic("could not find dp264_mv\n");

}

void
LinuxAlphaSystem::setupFuncEvents()
{
    AlphaSystem::setupFuncEvents();
#ifndef NDEBUG
    kernelPanicEvent = addKernelFuncEventOrPanic<BreakPCEvent>("panic");

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

    // Disable for now as it runs into panic() calls in VPTr methods
    // (see sim/vptr.hh).  Once those bugs are fixed, we can
    // re-enable, but we should find a better way to turn it on than
    // using DTRACE(Thread), since looking at a trace flag at tick 0
    // leads to non-intuitive behavior with --trace-start.
    Addr addr = 0;
    if (false && kernelSymtab->findAddress("alpha_switch_to", addr)) {
        printThreadEvent = new PrintThreadInfo(&pcEventQueue, "threadinfo",
                                               addr + sizeof(MachInst) * 6);
    } else {
        printThreadEvent = NULL;
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
}

void
LinuxAlphaSystem::setDelayLoop(ThreadContext *tc)
{
    Addr addr = 0;
    if (kernelSymtab->findAddress("loops_per_jiffy", addr)) {
        Tick cpuFreq = tc->getCpuPtr()->frequency();
        assert(intrFreq);
        PortProxy &vp = tc->getVirtProxy();
        vp.write(addr, (uint32_t)((cpuFreq / intrFreq) * 0.9988),
                 GuestByteOrder);
    }
}

void
LinuxAlphaSystem::SkipDelayLoopEvent::process(ThreadContext *tc)
{
    SkipFuncEvent::process(tc);
    // calculate and set loops_per_jiffy
    ((LinuxAlphaSystem *)tc->getSystemPtr())->setDelayLoop(tc);
}

void
LinuxAlphaSystem::PrintThreadInfo::process(ThreadContext *tc)
{
    Linux::ThreadInfo ti(tc);

    DPRINTF(Thread, "Currently Executing Thread %s, pid %d, started at: %d\n",
            ti.curTaskName(), ti.curTaskPID(), ti.curTaskStart());
}

LinuxAlphaSystem *
LinuxAlphaSystemParams::create()
{
    return new LinuxAlphaSystem(this);
}
