/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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

#ifndef __CPU_O3_THREAD_STATE_HH__
#define __CPU_O3_THREAD_STATE_HH__

#include "arch/faults.hh"
#include "arch/isa_traits.hh"
#include "cpu/exec_context.hh"
#include "cpu/thread_state.hh"

class Event;
class Process;

#if FULL_SYSTEM
class EndQuiesceEvent;
class FunctionProfile;
class ProfileNode;
#else
class FunctionalMemory;
class Process;
#endif

/**
 * Class that has various thread state, such as the status, the
 * current instruction being processed, whether or not the thread has
 * a trap pending or is being externally updated, the ExecContext
 * proxy pointer, etc.  It also handles anything related to a specific
 * thread's process, such as syscalls and checking valid addresses.
 */
template <class Impl>
struct O3ThreadState : public ThreadState {
    typedef ExecContext::Status Status;
    typedef typename Impl::FullCPU FullCPU;

    Status _status;

    // Current instruction
    TheISA::MachInst inst;
  private:
    FullCPU *cpu;
  public:

    bool inSyscall;

    bool trapPending;

#if FULL_SYSTEM
    O3ThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem)
        : ThreadState(-1, _thread_num, _mem),
          inSyscall(0), trapPending(0)
    { }
#else
    O3ThreadState(FullCPU *_cpu, int _thread_num, Process *_process, int _asid)
        : ThreadState(-1, _thread_num, _process->getMemory(), _process, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    { }

    O3ThreadState(FullCPU *_cpu, int _thread_num, FunctionalMemory *_mem,
                  int _asid)
        : ThreadState(-1, _thread_num, _mem, NULL, _asid),
          cpu(_cpu), inSyscall(0), trapPending(0)
    { }
#endif

    ExecContext *xcProxy;

    ExecContext *getXCProxy() { return xcProxy; }

    Status status() const { return _status; }

    void setStatus(Status new_status) { _status = new_status; }

#if !FULL_SYSTEM
    bool validInstAddr(Addr addr)
    { return process->validInstAddr(addr); }

    bool validDataAddr(Addr addr)
    { return process->validDataAddr(addr); }
#endif

    bool misspeculating() { return false; }

    void setInst(TheISA::MachInst _inst) { inst = _inst; }

    Counter readFuncExeInst() { return funcExeInst; }

    void setFuncExeInst(Counter new_val) { funcExeInst = new_val; }

#if !FULL_SYSTEM
    void syscall() { process->syscall(xcProxy); }
#endif
};

#endif // __CPU_O3_THREAD_STATE_HH__
