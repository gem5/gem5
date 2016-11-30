/*
 * Copyright (c) 2005 The Regents of The University of Michigan
 * Copyright (c) 2007-2008 The Florida State University
 * Copyright (c) 2009 The University of Edinburgh
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
 *          Stephen Hines
 *          Timothy M. Jones
 */

#ifndef __ARCH_RISCV_STACKTRACE_HH__
#define __ARCH_RISCV_STACKTRACE_HH__

#include "base/trace.hh"
#include "cpu/static_inst.hh"
#include "debug/Stack.hh"

class ThreadContext;
class StackTrace;

namespace RiscvISA
{

class ProcessInfo
{
  public:
    ProcessInfo(ThreadContext *_tc);

    Addr task(Addr ksp) const;
    int pid(Addr ksp) const;
    std::string name(Addr ksp) const;
};

class StackTrace
{
  protected:
    typedef TheISA::MachInst MachInst;
  private:
    ThreadContext *tc;
    std::vector<Addr> stack;

  private:
    bool isEntry(Addr addr);
    bool decodePrologue(Addr sp, Addr callpc, Addr func, int &size, Addr &ra);
    bool decodeSave(MachInst inst, int &reg, int &disp);
    bool decodeStack(MachInst inst, int &disp);

    void trace(ThreadContext *tc, bool is_call);

  public:
    StackTrace();
    StackTrace(ThreadContext *tc, const StaticInstPtr &inst);
    ~StackTrace();

    void
    clear()
    {
        tc = 0;
        stack.clear();
    }

    bool
    valid() const
    {
        return tc != nullptr;
    }

    bool trace(ThreadContext *tc, const StaticInstPtr &inst);

  public:
    const std::vector<Addr> &
    getstack() const
    {
        return stack;
    }

    static const int user = 1;
    static const int console = 2;
    static const int unknown = 3;

#if TRACING_ON
  private:
    void dump();

  public:
    void
    dprintf()
    {
        if (DTRACE(Stack))
            dump();
    }
#else
  public:
    void
    dprintf()
    {
    }
#endif
};

inline bool
StackTrace::trace(ThreadContext *tc, const StaticInstPtr &inst)
{
    if (!inst->isCall() && !inst->isReturn())
        return false;

    if (valid())
        clear();

    trace(tc, !inst->isReturn());
    return true;
}

} // namespace RiscvISA

#endif // __ARCH_RISCV_STACKTRACE_HH__
