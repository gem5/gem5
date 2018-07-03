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
 *
 * Authors: Gabe Black
 */

#ifndef __SYSTEMC_CORE_PROCESS_HH__
#define __SYSTEMC_CORE_PROCESS_HH__

#include <functional>

#include "base/fiber.hh"
#include "systemc/core/list.hh"
#include "systemc/core/object.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_process_handle.hh"

namespace sc_gem5
{

class Process : public ::sc_core::sc_object, public ListNode
{
  public:
    virtual ::sc_core::sc_curr_proc_kind procKind() const = 0;
    bool running() const { return _running; }
    bool dynamic() const { return _dynamic; }
    bool isUnwinding() const { return _isUnwinding; }
    bool terminated() const { return _terminated; }

    void forEachKid(const std::function<void(Process *)> &work);

    bool suspended() const { return _suspended; }
    bool disabled() const { return _disabled; }

    void suspend(bool inc_kids);
    void resume(bool inc_kids);
    void disable(bool inc_kids);
    void enable(bool inc_kids);

    void kill(bool inc_kids);
    void reset(bool inc_kids);
    virtual void throw_it(ExceptionWrapperBase &exc, bool inc_kids);

    void injectException(ExceptionWrapperBase &exc);
    ExceptionWrapperBase *excWrapper;

    void syncResetOn(bool inc_kids);
    void syncResetOff(bool inc_kids);

    void incref() { refCount++; }
    void decref() { refCount--; }

    const ::sc_core::sc_event &resetEvent() { return _resetEvent; }
    const ::sc_core::sc_event &terminatedEvent() { return _terminatedEvent; }

    // This should only be called before initialization.
    void dontInitialize() { popListNode(); }

    void setStackSize(size_t size) { stackSize = size; }

    void run();

    virtual Fiber *fiber() { return Fiber::primaryFiber(); }

    static Process *newest() { return _newest; }

  protected:
    Process(const char *name, ProcessFuncWrapper *func, bool _dynamic);

    static Process *_newest;

    virtual ~Process() { delete func; }

    ::sc_core::sc_event _resetEvent;
    ::sc_core::sc_event _terminatedEvent;

    ProcessFuncWrapper *func;
    sc_core::sc_curr_proc_kind _procKind;
    bool _running;
    bool _dynamic;
    bool _isUnwinding;
    bool _terminated;

    bool _suspended;
    bool _disabled;

    bool _syncReset;

    int refCount;

    size_t stackSize;
};

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_PROCESS_HH__
