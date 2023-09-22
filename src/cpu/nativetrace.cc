/*
 * Copyright (c) 2023 Arm Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
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
    : ExeTracer(p), native_listener(listenSocketInetConfig(8000).build(p.name))
{
    if (ListenSocket::allDisabled())
        fatal("All listeners are disabled!");

    native_listener->listen();

    fd = native_listener->accept();
}

NativeTraceRecord::NativeTraceRecord(
        NativeTrace *_parent,
        Tick _when, ThreadContext *_thread,
        const StaticInstPtr _staticInst, const PCStateBase &_pc,
        const StaticInstPtr _macroStaticInst)
  : ExeTracerRecord(_when, _thread, _staticInst, _pc,
                    *_parent, _macroStaticInst),
    parent(_parent)
{
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
