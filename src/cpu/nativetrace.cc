/*
 * Copyright (c) 2006-2009 The Regents of The University of Michigan
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

#include "cpu/nativetrace.hh"

#include "base/socket.hh"
#include "cpu/static_inst.hh"
#include "debug/GDBMisc.hh"
#include "params/NativeTrace.hh"

namespace gem5
{

namespace trace {

NativeTrace::NativeTrace(const Params &p)
    : ExeTracer(p)
{
    if (ListenSocket::allDisabled())
        fatal("All listeners are disabled!");

    int port = 8000;
    while (!native_listener.listen(port, true))
    {
        DPRINTF(GDBMisc, "Can't bind port %d\n", port);
        port++;
    }
    ccprintf(std::cerr, "Listening for native process on port %d\n", port);
    fd = native_listener.accept();
}

void
NativeTraceRecord::dump()
{
    //Don't print what happens for each micro-op, just print out
    //once at the last op, and for regular instructions.
    if (!staticInst->isMicroop() || staticInst->isLastMicroop())
        parent->check(this);
}

} // namespace trace
} // namespace gem5
