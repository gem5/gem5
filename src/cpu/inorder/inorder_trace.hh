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

#ifndef __CPU_INORDER_INORDER_TRACE_HH__
#define __CPU_INORDER_INORDER_TRACE_HH__

#include "base/trace.hh"
#include "base/types.hh"
#include "cpu/exetrace.hh"
#include "cpu/static_inst.hh"
#include "params/InOrderTrace.hh"
#include "sim/insttracer.hh"

class ThreadContext;

namespace Trace {

class InOrderTraceRecord : public ExeTracerRecord
{
  public:
    InOrderTraceRecord(unsigned num_stages, bool _stage_tracing,
           ThreadContext *_thread, TheISA::PCState _pc, bool spec = false)
        : ExeTracerRecord(0, _thread, NULL, _pc, spec)
    {
        stageTrace = _stage_tracing;
        stageCycle.resize(num_stages);
    }

    // Trace stage-by-stage execution of instructions.
    bool stageTrace;
    std::vector<Tick> stageCycle;

    void dumpTicks(std::ostream &outs);

    void
    setStageCycle(int num_stage, Tick cur_cycle)
    {
        if (stageTrace) {
            stageCycle[num_stage] = cur_cycle;
        } else {
            when = cur_cycle;
        }
    }

    void
    setStaticInst(const StaticInstPtr &_staticInst)
    {
        staticInst = _staticInst;
    }

    void setPC(TheISA::PCState _pc) { pc = _pc; }
};

class InOrderTrace : public InstTracer
{
  public:
    InOrderTrace(const InOrderTraceParams *p) : InstTracer(p)
    {}

    InOrderTraceRecord *
    getInstRecord(unsigned num_stages, bool stage_tracing, ThreadContext *tc);

    InOrderTraceRecord *getInstRecord(Tick when, ThreadContext *tc,
            const StaticInstPtr staticInst, TheISA::PCState pc,
            const StaticInstPtr macroStaticInst = NULL);
};

} // namespace Trace

#endif // __CPU_INORDER_INORDER_TRACE_HH__
