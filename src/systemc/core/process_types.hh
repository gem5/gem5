/*
 * Copyright 2018 Google, Inc.
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

#ifndef __SYSTEMC_CORE_PROCESS_TYPES_HH__
#define __SYSTEMC_CORE_PROCESS_TYPES_HH__

#include "systemc/core/process.hh"
#include "systemc/core/scheduler.hh"

namespace sc_gem5
{

class Method : public Process
{
  public:
    Method(const char *name, ProcessFuncWrapper *func, bool internal=false) :
        Process(name, func, internal)
    {}

    const char *kind() const override { return "sc_method_process"; }

    sc_core::sc_curr_proc_kind
    procKind() const override
    {
        return sc_core::SC_METHOD_PROC_;
    }
};

class Thread : public Process
{
  public:
    Thread(const char *name, ProcessFuncWrapper *func, bool internal=false) :
        Process(name, func, internal), ctx(nullptr)
    {}

    ~Thread() { delete ctx; }

    const char *kind() const override { return "sc_thread_process"; }

    sc_core::sc_curr_proc_kind
    procKind() const override
    {
        return sc_core::SC_THREAD_PROC_;
    }

    gem5::Fiber *
    fiber() override
    {
        if (!ctx)
            ctx = new Context(this, stackSize);
        return ctx;
    }

  private:
    class Context : public gem5::Fiber
    {
      public:
        Context(Thread *thread, size_t size)
          : gem5::Fiber(size), thread(thread)
        {}

      private:
        Thread *thread;

        void
        main() override
        {
            thread->_needsStart = false;
            try {
                thread->run();
            } catch (...) {
                thread->terminate();
                scheduler.throwUp();
                return;
            }
            thread->terminate();
            scheduler.yield();
        }
    };
    friend class Context;

    Context *ctx;
};

class CThread : public Thread
{
  public:
    CThread(const char *name, ProcessFuncWrapper *func, bool internal=false) :
        Thread(name, func, internal)
    {
        // We'll be in the initialization list now, but we shouldn't be.
        popListNode();
    }

    const char *kind() const override { return "sc_cthread_process"; }

    sc_core::sc_curr_proc_kind
    procKind() const override
    {
        return sc_core::SC_CTHREAD_PROC_;
    }
};

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_PROCESS_TYPES_HH__
