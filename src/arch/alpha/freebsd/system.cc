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

#include "arch/alpha/freebsd/system.hh"

#include "arch/alpha/system.hh"
#include "arch/isa_traits.hh"
#include "arch/vtophys.hh"
#include "base/loader/symtab.hh"
#include "cpu/thread_context.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "sim/byteswap.hh"

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

    ppc_vaddr = (Addr)tc->readIntReg(17);
    timer_vaddr = (Addr)tc->readIntReg(18);

    virtProxy.write(ppc_vaddr, (uint32_t)SimClock::Frequency);
    virtProxy.write(timer_vaddr, (uint32_t)TIMER_FREQUENCY);
}

void
FreebsdAlphaSystem::SkipCalibrateClocksEvent::process(ThreadContext *tc)
{
    SkipFuncEvent::process(tc);
    ((FreebsdAlphaSystem *)tc->getSystemPtr())->doCalibrateClocks(tc);
}

FreebsdAlphaSystem *
FreebsdAlphaSystemParams::create()
{
    return new FreebsdAlphaSystem(this);
}
