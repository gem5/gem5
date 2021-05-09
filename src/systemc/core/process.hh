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

#ifndef __SYSTEMC_CORE_PROCESS_HH__
#define __SYSTEMC_CORE_PROCESS_HH__

#include <functional>
#include <memory>
#include <vector>

#include "base/fiber.hh"
#include "systemc/core/list.hh"
#include "systemc/core/module.hh"
#include "systemc/core/object.hh"
#include "systemc/core/sched_event.hh"
#include "systemc/core/sensitivity.hh"
#include "systemc/ext/channel/sc_signal_in_if.hh"
#include "systemc/ext/core/sc_event.hh"
#include "systemc/ext/core/sc_module.hh"
#include "systemc/ext/core/sc_process_handle.hh"

namespace sc_core
{

class sc_join;

} // namespace sc_core

namespace sc_gem5
{

class ScHalt
{};

class Process;
class Reset;

class Process : public ::sc_core::sc_process_b, public ListNode
{
  public:
    virtual ::sc_core::sc_curr_proc_kind procKind() const = 0;
    bool needsStart() const { return _needsStart; }
    void needsStart(bool ns) { _needsStart = ns; }
    bool dynamic() const { return _dynamic; }
    bool isUnwinding() const { return _isUnwinding; }
    void isUnwinding(bool v) { _isUnwinding = v; }
    bool terminated() const { return _terminated; }

    bool scheduled() const { return _scheduled; }
    void scheduled(bool new_val) { _scheduled = new_val; }

    void forEachKid(const std::function<void(Process *)> &work);

    bool suspended() const { return _suspended; }
    bool disabled() const { return _disabled; }

    void suspend(bool inc_kids);
    void resume(bool inc_kids);
    void disable(bool inc_kids);
    void enable(bool inc_kids);

    void kill(bool inc_kids);
    void reset(bool inc_kids);
    void throw_it(ExceptionWrapperBase &exc, bool inc_kids);

    void injectException(ExceptionWrapperBase &exc);
    ExceptionWrapperBase *excWrapper;

    void syncResetOn(bool inc_kids);
    void syncResetOff(bool inc_kids);

    void signalReset(bool set, bool sync);

    void incref() { refCount++; }
    void decref() { refCount--; }

    ::sc_core::sc_event &resetEvent() { return _resetEvent; }
    ::sc_core::sc_event &terminatedEvent() { return _terminatedEvent; }

    void setStackSize(size_t size) { stackSize = size; }

    void run();

    void addStatic(StaticSensitivity *);
    void setDynamic(DynamicSensitivity *);
    void clearDynamic() { setDynamic(nullptr); }
    void addReset(Reset *);

    ScEvent timeoutEvent;
    void setTimeout(::sc_core::sc_time t);
    void cancelTimeout();

    void satisfySensitivity(Sensitivity *);

    void ready();

    virtual gem5::Fiber *fiber() { return gem5::Fiber::primaryFiber(); }

    static Process *newest() { return _newest; }

    void lastReport(::sc_core::sc_report *report);
    ::sc_core::sc_report *lastReport() const;

    bool hasStaticSensitivities() { return !staticSensitivities.empty(); }
    bool internal() { return _internal; }
    bool timedOut() { return _timedOut; }
    bool inReset() { return _syncReset || syncResetCount || asyncResetCount; }

    bool dontInitialize() { return _dontInitialize; }
    void dontInitialize(bool di) { _dontInitialize = di; }

    void joinWait(::sc_core::sc_join *join) { joinWaiters.push_back(join); }

    void waitCount(int count) { _waitCount = count; }

    const char *uniqueName(const char *seed) { return nameGen.gen(seed); }

  protected:
    void timeout();

    Process(const char *name, ProcessFuncWrapper *func, bool internal=false);

    static Process *_newest;

    virtual ~Process()
    {
        popListNode();
        delete func;
        for (auto s: staticSensitivities) {
            s->clear();
            delete s;
        }
        clearDynamic();
    }

    InternalScEvent _resetEvent;
    InternalScEvent _terminatedEvent;

    ProcessFuncWrapper *func;

    bool _internal;

    // Needed to support the deprecated "timed_out" function.
    bool _timedOut;

    bool _dontInitialize;

    bool _needsStart;
    bool _dynamic;
    bool _isUnwinding;
    bool _terminated;
    bool _scheduled;

    void terminate();

    bool _suspended;
    bool _suspendedReady;
    bool _disabled;

    bool _syncReset;

    int syncResetCount;
    int asyncResetCount;

    int _waitCount;

    int refCount;

    size_t stackSize;

    StaticSensitivities staticSensitivities;
    DynamicSensitivity *dynamicSensitivity;
    std::vector<Reset *> resets;

    std::unique_ptr<::sc_core::sc_report> _lastReport;

    std::vector<::sc_core::sc_join *> joinWaiters;

    UniqueNameGen nameGen;
};

class Reset
{
  public:
    Reset(Process *p, bool s, bool v) :
        _process(p), _signal(nullptr), _sync(s), _value(v)
    {}

    bool
    install(const sc_core::sc_signal_in_if<bool> *s)
    {
        _signal = s;

        if (_signal->_addReset(this)) {
            _process->addReset(this);
            if (_signal->read() == _value)
                update();
            return true;
        }
        return false;
    }
    void update() { _process->signalReset(_signal->read() == _value, _sync); }

    Process *process() { return _process; }
    const sc_core::sc_signal_in_if<bool> *signal() { return _signal; }
    bool sync() { return _sync; }
    bool value() { return _value; }

  private:
    Process *_process;
    const sc_core::sc_signal_in_if<bool> *_signal;
    bool _sync;
    bool _value;
};

void newReset(const sc_core::sc_port_base *pb, Process *p, bool s, bool v);
void newReset(const sc_core::sc_signal_in_if<bool> *sig, Process *p,
        bool s, bool v);

} // namespace sc_gem5

#endif  //__SYSTEMC_CORE_PROCESS_HH__
