/*
 * Copyright © 2007 MIPS Technologies, Inc.  All Rights Reserved
 *
 * This software is part of the M5 simulator.
 *
 * THIS IS A LEGAL AGREEMENT.  BY DOWNLOADING, USING, COPYING, CREATING
 * DERIVATIVE WORKS, AND/OR DISTRIBUTING THIS SOFTWARE YOU ARE AGREEING
 * TO THESE TERMS AND CONDITIONS.
 *
 * Permission is granted to use, copy, create derivative works and
 * distribute this software and such derivative works for any purpose,
 * so long as (1) the copyright notice above, this grant of permission,
 * and the disclaimer below appear in all copies and derivative works
 * made, (2) the copyright notice above is augmented as appropriate to
 * reflect the addition of any new copyrightable work in a derivative
 * work (e.g., Copyright © <Publication Year> Copyright Owner), and (3)
 * the name of MIPS Technologies, Inc. (“MIPS”) is not used in any
 * advertising or publicity pertaining to the use or distribution of
 * this software without specific, written prior authorization.
 *
 * THIS SOFTWARE IS PROVIDED “AS IS.”  MIPS MAKES NO WARRANTIES AND
 * DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, STATUTORY, IMPLIED OR
 * OTHERWISE, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 * NON-INFRINGEMENT OF THIRD PARTY RIGHTS, REGARDING THIS SOFTWARE.
 * IN NO EVENT SHALL MIPS BE LIABLE FOR ANY DAMAGES, INCLUDING DIRECT,
 * INDIRECT, INCIDENTAL, CONSEQUENTIAL, SPECIAL, OR PUNITIVE DAMAGES OF
 * ANY KIND OR NATURE, ARISING OUT OF OR IN CONNECTION WITH THIS AGREEMENT,
 * THIS SOFTWARE AND/OR THE USE OF THIS SOFTWARE, WHETHER SUCH LIABILITY
 * IS ASSERTED ON THE BASIS OF CONTRACT, TORT (INCLUDING NEGLIGENCE OR
 * STRICT LIABILITY), OR OTHERWISE, EVEN IF MIPS HAS BEEN WARNED OF THE
 * POSSIBILITY OF ANY SUCH LOSS OR DAMAGE IN ADVANCE.
 *
 * Authors: Ali G. Saidi
 *
 */

#ifndef __ARCH_MIPS_STACKTRACE_HH__
#define __ARCH_MIPS_STACKTRACE_HH__

#include "base/trace.hh"
#include "cpu/static_inst.hh"

class ThreadContext;

namespace MipsISA
{

class ProcessInfo
{
  private:
    ThreadContext *tc;

    int thread_info_size;
    int task_struct_size;
    int task_off;
    int pid_off;
    int name_off;

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
    StackTrace(ThreadContext *tc, StaticInstPtr inst);
    ~StackTrace();

    void clear()
    {
        tc = 0;
        stack.clear();
    }

    bool valid() const { return tc != NULL; }
    bool trace(ThreadContext *tc, StaticInstPtr inst);

  public:
    const std::vector<Addr> &getstack() const { return stack; }

    static const int user = 1;
    static const int console = 2;
    static const int unknown = 3;

#if TRACING_ON
  private:
    void dump();

  public:
    void dprintf() { if (DTRACE(Stack)) dump(); }
#else
  public:
    void dprintf() {}
#endif
};

inline bool
StackTrace::trace(ThreadContext *tc, StaticInstPtr inst)
{
    if (!inst->isCall() && !inst->isReturn())
        return false;

    if (valid())
        clear();

    trace(tc, !inst->isReturn());
    return true;
}

}

#endif // __ARCH_MIPS_STACKTRACE_HH__
