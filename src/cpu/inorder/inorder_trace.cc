/*
 * Copyright (c) 2007 MIPS Technologies, Inc.
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
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
 * Authors: Korey Sewell
 */

#include <iomanip>

#include "config/the_isa.hh"
#include "cpu/inorder/inorder_trace.hh"
#include "cpu/inorder/pipeline_traits.hh"
#include "cpu/exetrace.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "debug/ExecEnable.hh"
#include "params/InOrderTrace.hh"

using namespace std;
using namespace TheISA;

namespace Trace {

inline void
Trace::InOrderTraceRecord::dumpTicks(std::ostream &outs)
{
  if (!stageTrace) {
    ccprintf(outs, "%7d: ", when);
  } else {
    ccprintf(outs, "");
    for (int i=0; i < stageCycle.size(); i++) {
        if (i < stageCycle.size() - 1)
            outs << dec << stageCycle[i] << "-";
        else
            outs << dec << stageCycle[i] << ":";
    }
  }
}

InOrderTraceRecord *
InOrderTrace::getInstRecord(unsigned num_stages, bool stage_tracing,
        ThreadContext *tc)
{
    if (!Debug::ExecEnable)
        return NULL;

    if (!Trace::enabled)
        return NULL;

    return new InOrderTraceRecord(num_stages, stage_tracing, tc, 0);
}

InOrderTraceRecord *
InOrderTrace::getInstRecord(Tick when, ThreadContext *tc,
        const StaticInstPtr staticInst, TheISA::PCState _pc,
        const StaticInstPtr macroStaticInst)
{
    return new InOrderTraceRecord(ThePipeline::NumStages, true, tc, _pc);
}

} // namespace Trace

////////////////////////////////////////////////////////////////////////
//
//  ExeTracer Simulation Object
//
Trace::InOrderTrace *
InOrderTraceParams::create()
{
    return new Trace::InOrderTrace(this);
};

