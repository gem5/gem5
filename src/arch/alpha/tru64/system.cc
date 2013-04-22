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
#include "mem/fs_translating_port_proxy.hh"

using namespace std;

Tru64AlphaSystem::Tru64AlphaSystem(Tru64AlphaSystem::Params *p)
    : AlphaSystem(p)
{
    Addr addr = 0;
    if (kernelSymtab->findAddress("enable_async_printf", addr)) {
        virtProxy.write(addr, (uint32_t)0);
    }

#ifdef DEBUG
    kernelPanicEvent = addKernelFuncEventOrPanic<BreakPCEvent>("panic");
#endif

    badaddrEvent = addKernelFuncEventOrPanic<BadAddrEvent>("badaddr");

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

Tru64AlphaSystem *
Tru64AlphaSystemParams::create()
{
    return new Tru64AlphaSystem(this);
}
