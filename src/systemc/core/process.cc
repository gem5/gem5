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

#include "systemc/core/process.hh"

#include "base/logging.hh"
#include "systemc/core/event.hh"
#include "systemc/core/scheduler.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/core/sc_process_handle.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_gem5
{

SensitivityTimeout::SensitivityTimeout(Process *p, ::sc_core::sc_time t) :
    Sensitivity(p), timeoutEvent([this]() { this->timeout(); })
{
    scheduler.schedule(&timeoutEvent, t);
}

SensitivityTimeout::~SensitivityTimeout()
{
    if (timeoutEvent.scheduled())
        scheduler.deschedule(&timeoutEvent);
}

void
SensitivityTimeout::timeout()
{
    notify();
}

SensitivityEvent::SensitivityEvent(
        Process *p, const ::sc_core::sc_event *e) : Sensitivity(p), event(e)
{
    Event::getFromScEvent(event)->addSensitivity(this);
}

SensitivityEvent::~SensitivityEvent()
{
    Event::getFromScEvent(event)->delSensitivity(this);
}

SensitivityEventAndList::SensitivityEventAndList(
        Process *p, const ::sc_core::sc_event_and_list *list) :
    Sensitivity(p), list(list), count(0)
{
    for (auto e: list->events)
        Event::getFromScEvent(e)->addSensitivity(this);
}

SensitivityEventAndList::~SensitivityEventAndList()
{
    for (auto e: list->events)
        Event::getFromScEvent(e)->delSensitivity(this);
}

void
SensitivityEventAndList::notifyWork(Event *e)
{
    e->delSensitivity(this);
    count++;
    if (count == list->events.size())
        process->satisfySensitivity(this);
}

SensitivityEventOrList::SensitivityEventOrList(
        Process *p, const ::sc_core::sc_event_or_list *list) :
    Sensitivity(p), list(list)
{
    for (auto e: list->events)
        Event::getFromScEvent(e)->addSensitivity(this);
}

SensitivityEventOrList::~SensitivityEventOrList()
{
    for (auto e: list->events)
        Event::getFromScEvent(e)->delSensitivity(this);
}

void
SensitivityTimeoutAndEventAndList::notifyWork(Event *e)
{
    if (e) {
        // An event went off which must be part of the sc_event_and_list.
        SensitivityEventAndList::notifyWork(e);
    } else {
        // There's no inciting event, so this must be a timeout.
        SensitivityTimeout::notifyWork(e);
    }
}


class UnwindExceptionReset : public ::sc_core::sc_unwind_exception
{
  public:
    UnwindExceptionReset() { _isReset = true; }
};

class UnwindExceptionKill : public ::sc_core::sc_unwind_exception
{
  public:
    UnwindExceptionKill() {}
};

template <typename T>
struct BuiltinExceptionWrapper : public ExceptionWrapperBase
{
  public:
    T t;
    void throw_it() override { throw t; }
};

BuiltinExceptionWrapper<UnwindExceptionReset> resetException;
BuiltinExceptionWrapper<UnwindExceptionKill> killException;


void
Process::forEachKid(const std::function<void(Process *)> &work)
{
    for (auto &kid: get_child_objects()) {
        Process *p_kid = dynamic_cast<Process *>(kid);
        if (p_kid)
            work(p_kid);
    }
}

void
Process::suspend(bool inc_kids)
{
    if (inc_kids)
        forEachKid([](Process *p) { p->suspend(true); });

    if (!_suspended) {
        _suspended = true;
        _suspendedReady = false;
    }

    if (procKind() != ::sc_core::SC_METHOD_PROC_ &&
            scheduler.current() == this) {
        scheduler.yield();
    }
}

void
Process::resume(bool inc_kids)
{
    if (inc_kids)
        forEachKid([](Process *p) { p->resume(true); });

    if (_suspended) {
        _suspended = false;
        if (_suspendedReady)
            ready();
        _suspendedReady = false;
    }
}

void
Process::disable(bool inc_kids)
{
    if (inc_kids)
        forEachKid([](Process *p) { p->disable(true); });

    if (!::sc_core::sc_allow_process_control_corners &&
            dynamic_cast<SensitivityTimeout *>(dynamicSensitivity)) {
        std::string message("attempt to disable a thread with timeout wait: ");
        message += name();
        SC_REPORT_ERROR("Undefined process control interaction",
                message.c_str());
    }

    _disabled = true;
}

void
Process::enable(bool inc_kids)
{

    if (inc_kids)
        forEachKid([](Process *p) { p->enable(true); });

    _disabled = false;
}

void
Process::kill(bool inc_kids)
{
    if (::sc_core::sc_get_status() != ::sc_core::SC_RUNNING) {
        SC_REPORT_ERROR(
                "(E572) a process may not be killed before it is initialized",
                name());
    }

    // Propogate the kill to our children no matter what happens to us.
    if (inc_kids)
        forEachKid([](Process *p) { p->kill(true); });

    // If we're in the middle of unwinding, ignore the kill request.
    if (_isUnwinding)
        return;

    // Update our state.
    terminate();
    _isUnwinding = true;

    // Make sure this process isn't marked ready
    popListNode();

    // Inject the kill exception into this process if it's started.
    if (!_needsStart)
        injectException(killException);
}

void
Process::reset(bool inc_kids)
{
    if (::sc_core::sc_get_status() != ::sc_core::SC_RUNNING) {
        SC_REPORT_ERROR(
                "(E573) a process may not be asynchronously reset while"
                "the simulation is not running", name());
    }

    // Propogate the reset to our children no matter what happens to us.
    if (inc_kids)
        forEachKid([](Process *p) { p->reset(true); });

    // If we're in the middle of unwinding, ignore the reset request.
    if (_isUnwinding)
        return;


    if (_needsStart) {
        scheduler.runNow(this);
    } else {
        _isUnwinding = true;
        injectException(resetException);
    }

    _resetEvent.notify();
}

void
Process::throw_it(ExceptionWrapperBase &exc, bool inc_kids)
{
    if (::sc_core::sc_get_status() != ::sc_core::SC_RUNNING) {
        SC_REPORT_ERROR(
                "(E574) throw_it not allowed unless simulation is running ",
                name());
    }

    if (inc_kids)
        forEachKid([&exc](Process *p) { p->throw_it(exc, true); });

    // Only inject an exception into threads that have started.
    if (!_needsStart)
        injectException(exc);
}

void
Process::injectException(ExceptionWrapperBase &exc)
{
    excWrapper = &exc;
    scheduler.runNow(this);
};

void
Process::syncResetOn(bool inc_kids)
{
    if (inc_kids)
        forEachKid([](Process *p) { p->syncResetOn(true); });

    _syncReset = true;
}

void
Process::syncResetOff(bool inc_kids)
{
    if (inc_kids)
        forEachKid([](Process *p) { p->syncResetOff(true); });

    _syncReset = false;
}

void
Process::dontInitialize()
{
    scheduler.dontInitialize(this);
}

void
Process::finalize()
{
    for (auto &s: pendingStaticSensitivities) {
        s->finalize(staticSensitivities);
        delete s;
        s = nullptr;
    }
    pendingStaticSensitivities.clear();
};

void
Process::run()
{
    bool reset;
    do {
        reset = false;
        try {
            func->call();
        } catch(const ::sc_core::sc_unwind_exception &exc) {
            reset = exc.is_reset();
            _isUnwinding = false;
        }
    } while (reset);
    needsStart(true);
}

void
Process::addStatic(PendingSensitivity *s)
{
    pendingStaticSensitivities.push_back(s);
}

void
Process::setDynamic(Sensitivity *s)
{
    delete dynamicSensitivity;
    dynamicSensitivity = s;
}

void
Process::satisfySensitivity(Sensitivity *s)
{
    // If there's a dynamic sensitivity and this wasn't it, ignore.
    if (dynamicSensitivity && dynamicSensitivity != s)
        return;

    setDynamic(nullptr);
    ready();
}

void
Process::ready()
{
    if (disabled())
        return;
    if (suspended())
        _suspendedReady = true;
    else
        scheduler.ready(this);
}

void
Process::lastReport(::sc_core::sc_report *report)
{
    if (report) {
        _lastReport = std::unique_ptr<::sc_core::sc_report>(
                new ::sc_core::sc_report(*report));
    } else {
        _lastReport = nullptr;
    }
}

::sc_core::sc_report *Process::lastReport() const { return _lastReport.get(); }

Process::Process(const char *name, ProcessFuncWrapper *func, bool _dynamic) :
    ::sc_core::sc_process_b(name), excWrapper(nullptr), func(func),
    _needsStart(true), _dynamic(_dynamic), _isUnwinding(false),
    _terminated(false), _suspended(false), _disabled(false),
    _syncReset(false), refCount(0), stackSize(::Fiber::DefaultStackSize),
    dynamicSensitivity(nullptr)
{
    _newest = this;
}

void
Process::terminate()
{
    _terminated = true;
    _suspendedReady = false;
    _suspended = false;
    _syncReset = false;
    delete dynamicSensitivity;
    dynamicSensitivity = nullptr;
    for (auto s: staticSensitivities)
        delete s;
    staticSensitivities.clear();

    _terminatedEvent.notify();
}

Process *Process::_newest;

void
throw_it_wrapper(Process *p, ExceptionWrapperBase &exc, bool inc_kids)
{
    p->throw_it(exc, inc_kids);
}

} // namespace sc_gem5
