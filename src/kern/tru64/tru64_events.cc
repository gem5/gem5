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

#include "arch/alpha/ev5.hh"
#include "arch/isa_traits.hh"
#include "config/the_isa.hh"
#include "cpu/base.hh"
#include "cpu/thread_context.hh"
#include "debug/BADADDR.hh"
#include "debug/DebugPrintf.hh"
#include "debug/Printf.hh"
#include "kern/tru64/dump_mbuf.hh"
#include "kern/tru64/printf.hh"
#include "kern/tru64/tru64_events.hh"
#include "kern/system_events.hh"
#include "sim/arguments.hh"
#include "sim/system.hh"

using namespace TheISA;

//void SkipFuncEvent::process(ExecContext *tc);

void
BadAddrEvent::process(ThreadContext *tc)
{
    // The following gross hack is the equivalent function to the
    // annotation for vmunix::badaddr in:
    // simos/simulation/apps/tcl/osf/tlaser.tcl

    uint64_t a0 = tc->readIntReg(16);

    bool found = false;

    MasterPort &dataPort = tc->getCpuPtr()->getDataPort();

    // get the address ranges of the connected slave port
    AddrRangeList resp = dataPort.getAddrRanges();
    for (const auto &iter : resp) {
        if (iter.contains(K0Seg2Phys(a0) & PAddrImplMask))
            found = true;
    }

    if (!IsK0Seg(a0) || found ) {

        DPRINTF(BADADDR, "badaddr arg=%#x bad\n", a0);
        tc->setIntReg(ReturnValueReg, 0x1);
        SkipFuncEvent::process(tc);
    } else {
        DPRINTF(BADADDR, "badaddr arg=%#x good\n", a0);
    }
}

void
PrintfEvent::process(ThreadContext *tc)
{
    if (DTRACE(Printf)) {
        StringWrap name(tc->getSystemPtr()->name());
        DPRINTFN("");

        Arguments args(tc);
        tru64::Printf(args);
    }
}

void
DebugPrintfEvent::process(ThreadContext *tc)
{
    if (DTRACE(DebugPrintf)) {
        if (!raw) {
            StringWrap name(tc->getSystemPtr()->name());
            DPRINTFN("");
        }

        Arguments args(tc);
        tru64::Printf(args);
    }
}

void
DumpMbufEvent::process(ThreadContext *tc)
{
    if (DTRACE(DebugPrintf)) {
        Arguments args(tc);
        tru64::DumpMbuf(args);
    }
}
