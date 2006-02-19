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
 */

#include "cpu/exec_context.hh"
#include "cpu/base.hh"
#include "kern/system_events.hh"
#include "kern/tru64/tru64_events.hh"
#include "kern/tru64/dump_mbuf.hh"
#include "kern/tru64/printf.hh"
#include "mem/functional/memory_control.hh"
#include "targetarch/arguments.hh"
#include "arch/isa_traits.hh"

using namespace TheISA;

//void SkipFuncEvent::process(ExecContext *xc);

void
BadAddrEvent::process(ExecContext *xc)
{
    // The following gross hack is the equivalent function to the
    // annotation for vmunix::badaddr in:
    // simos/simulation/apps/tcl/osf/tlaser.tcl

    uint64_t a0 = xc->regs.intRegFile[ArgumentReg0];

    if (!TheISA::IsK0Seg(a0) ||
        xc->memctrl->badaddr(TheISA::K0Seg2Phys(a0) & EV5::PAddrImplMask)) {

        DPRINTF(BADADDR, "badaddr arg=%#x bad\n", a0);
        xc->regs.intRegFile[ReturnValueReg] = 0x1;
        SkipFuncEvent::process(xc);
    }
    else
        DPRINTF(BADADDR, "badaddr arg=%#x good\n", a0);
}

void
PrintfEvent::process(ExecContext *xc)
{
    if (DTRACE(Printf)) {
        DebugOut() << curTick << ": " << xc->cpu->name() << ": ";

        AlphaArguments args(xc);
        tru64::Printf(args);
    }
}

void
DebugPrintfEvent::process(ExecContext *xc)
{
    if (DTRACE(DebugPrintf)) {
        if (!raw)
            DebugOut() << curTick << ": " << xc->cpu->name() << ": ";

        AlphaArguments args(xc);
        tru64::Printf(args);
    }
}

void
DumpMbufEvent::process(ExecContext *xc)
{
    if (DTRACE(DebugPrintf)) {
        AlphaArguments args(xc);
        tru64::DumpMbuf(args);
    }
}
