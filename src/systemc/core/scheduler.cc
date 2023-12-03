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

#include "systemc/core/scheduler.hh"

#include "base/fiber.hh"
#include "base/logging.hh"
#include "sim/eventq.hh"
#include "sim/sim_exit.hh"
#include "systemc/core/kernel.hh"
#include "systemc/core/sc_main_fiber.hh"
#include "systemc/ext/core/messages.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/utils/sc_report.hh"
#include "systemc/ext/utils/sc_report_handler.hh"
#include "systemc/utils/report.hh"
#include "systemc/utils/tracefile.hh"

namespace sc_gem5
{

Scheduler::Scheduler()
    : eq(nullptr),
      readyEvent(*this, false, ReadyPriority),
      pauseEvent(*this, false, PausePriority),
      stopEvent(*this, false, StopPriority),
      _throwUp(nullptr),
      starvationEvent(*this, false, StarvationPriority),
      _elaborationDone(false),
      _started(false),
      _stopNow(false),
      _status(StatusOther),
      maxTick(gem5::MaxTick),
      maxTickEvent(*this, false, MaxTickPriority),
      timeAdvancesEvent(*this, false, TimeAdvancesPriority),
      _numCycles(0),
      _changeStamp(0),
      _current(nullptr),
      initDone(false),
      runToTime(true),
      runOnce(false)
{}

Scheduler::~Scheduler()
{
    // Clear out everything that belongs to us to make sure nobody tries to
    // clear themselves out after the scheduler goes away.
    clear();
}

void
Scheduler::clear()
{
    // Delta notifications.
    while (!deltas.empty())
        deltas.front()->deschedule();

    // Timed notifications.
    for (auto &ts : timeSlots) {
        while (!ts->events.empty())
            ts->events.front()->deschedule();
        deschedule(ts);
    }
    timeSlots.clear();

    // gem5 events.
    if (readyEvent.scheduled())
        deschedule(&readyEvent);
    if (pauseEvent.scheduled())
        deschedule(&pauseEvent);
    if (stopEvent.scheduled())
        deschedule(&stopEvent);
    if (starvationEvent.scheduled())
        deschedule(&starvationEvent);
    if (maxTickEvent.scheduled())
        deschedule(&maxTickEvent);
    if (timeAdvancesEvent.scheduled())
        deschedule(&timeAdvancesEvent);

    Process *p;
    while ((p = initList.getNext()))
        p->popListNode();
    while ((p = readyListMethods.getNext()))
        p->popListNode();
    while ((p = readyListThreads.getNext()))
        p->popListNode();

    Channel *c;
    while ((c = updateList.getNext()))
        c->popListNode();
}

void
Scheduler::initPhase()
{
    runUpdate();

    for (Process *p = initList.getNext(); p; p = initList.getNext()) {
        p->popListNode();

        if (p->dontInitialize()) {
            if (!p->hasStaticSensitivities() && !p->internal()) {
                SC_REPORT_WARNING(sc_core::SC_ID_DISABLE_WILL_ORPHAN_PROCESS_,
                                  p->name());
            }
        } else {
            p->ready();
        }
    }

    runDelta();

    for (auto ets : eventsToSchedule)
        eq->schedule(ets.first, ets.second);
    eventsToSchedule.clear();

    if (_started) {
        if (!runToTime && starved())
            scheduleStarvationEvent();
        kernel->status(::sc_core::SC_RUNNING);
    }

    initDone = true;

    status(StatusOther);

    scheduleTimeAdvancesEvent();
}

void
Scheduler::reg(Process *p)
{
    if (initDone) {
        // If not marked as dontInitialize, mark as ready.
        if (!p->dontInitialize())
            p->ready();
    } else {
        // Otherwise, record that this process should be initialized once we
        // get there.
        initList.pushLast(p);
    }
}

void
Scheduler::yield()
{
    // Pull a process from the active list.
    _current = getNextReady();
    if (!_current) {
        // There are no more processes, so return control to evaluate.
        gem5::Fiber::primaryFiber()->run();
    } else {
        _current->popListNode();
        _current->scheduled(false);
        // Switch to whatever Fiber is supposed to run this process. All
        // Fibers which aren't running should be parked at this line.
        _current->fiber()->run();
        // If the current process needs to be manually started, start it.
        if (_current && _current->needsStart()) {
            _current->needsStart(false);
            // If a process hasn't started yet, "resetting" it just starts it
            // and signals its reset event.
            if (_current->inReset())
                _current->resetEvent().notify();
            try {
                _current->run();
            } catch (...) {
                throwUp();
            }
        }
    }
    if (_current && !_current->needsStart()) {
        if (_current->excWrapper) {
            auto ew = _current->excWrapper;
            _current->excWrapper = nullptr;
            ew->throw_it();
        } else if (_current->inReset()) {
            _current->reset(false);
        }
    }
}

void
Scheduler::ready(Process *p)
{
    if (_stopNow)
        return;

    p->scheduled(true);

    if (p->procKind() == ::sc_core::SC_METHOD_PROC_)
        readyListMethods.pushLast(p);
    else
        readyListThreads.pushLast(p);

    if (!inEvaluate())
        scheduleReadyEvent();
}

void
Scheduler::resume(Process *p)
{
    if (initDone)
        ready(p);
    else
        initList.pushLast(p);
}

bool
listContains(ListNode *list, ListNode *target)
{
    ListNode *n = list->nextListNode;
    while (n != list)
        if (n == target)
            return true;
    return false;
}

bool
Scheduler::suspend(Process *p)
{
    bool was_ready;
    if (initDone) {
        // After initialization, check if we're on a ready list.
        was_ready = (p->nextListNode != nullptr);
        p->popListNode();
    } else {
        // Nothing is ready before init.
        was_ready = false;
    }
    return was_ready;
}

void
Scheduler::requestUpdate(Channel *c)
{
    updateList.pushLast(c);
    if (!inEvaluate())
        scheduleReadyEvent();
}

void
Scheduler::asyncRequestUpdate(Channel *c)
{
    std::lock_guard<std::mutex> lock(asyncListMutex);
    asyncUpdateList.pushLast(c);
    hasAsyncUpdate = true;
}

void
Scheduler::scheduleReadyEvent()
{
    // Schedule the evaluate and update phases.
    if (!readyEvent.scheduled()) {
        schedule(&readyEvent);
        if (starvationEvent.scheduled())
            deschedule(&starvationEvent);
    }
}

void
Scheduler::scheduleStarvationEvent()
{
    if (!starvationEvent.scheduled()) {
        schedule(&starvationEvent);
        if (readyEvent.scheduled())
            deschedule(&readyEvent);
    }
}

void
Scheduler::runReady()
{
    scheduleTimeAdvancesEvent();

    bool empty = readyListMethods.empty() && readyListThreads.empty();
    lastReadyTick = getCurTick();

    // The evaluation phase.
    status(StatusEvaluate);
    do {
        yield();
    } while (getNextReady());
    _current = nullptr;

    if (!empty) {
        _numCycles++;
        _changeStamp++;
    }

    if (_stopNow) {
        status(StatusOther);
        return;
    }

    runUpdate();
    if (!traceFiles.empty())
        trace(true);
    runDelta();

    if (!runToTime && starved())
        scheduleStarvationEvent();

    if (runOnce)
        schedulePause();

    status(StatusOther);
}

void
Scheduler::runUpdate()
{
    status(StatusUpdate);
    if (hasAsyncUpdate) {
        std::lock_guard<std::mutex> lock(asyncListMutex);
        Channel *channel;
        while ((channel = asyncUpdateList.getNext()) != nullptr)
            updateList.pushLast(channel);
        hasAsyncUpdate = false;
    }

    try {
        Channel *channel = updateList.getNext();
        while (channel) {
            channel->popListNode();
            channel->update();
            channel = updateList.getNext();
        }
    } catch (...) {
        throwUp();
    }
}

void
Scheduler::runDelta()
{
    status(StatusDelta);

    try {
        while (!deltas.empty())
            deltas.back()->run();
    } catch (...) {
        throwUp();
    }
}

void
Scheduler::pause()
{
    using namespace gem5;

    status(StatusPaused);
    kernel->status(::sc_core::SC_PAUSED);
    runOnce = false;
    if (scMainFiber.called()) {
        if (!scMainFiber.finished())
            scMainFiber.run();
    } else {
        if (scMainFiber.finished())
            fatal("Pausing systemc after sc_main completed.");
        else
            gem5::exitSimLoopNow("systemc pause");
    }
}

void
Scheduler::stop()
{
    using namespace gem5;

    status(StatusStopped);
    kernel->stop();

    clear();

    runOnce = false;
    if (scMainFiber.called()) {
        if (!scMainFiber.finished())
            scMainFiber.run();
    } else {
        if (scMainFiber.finished())
            fatal("Stopping systemc after sc_main completed.");
        else
            gem5::exitSimLoopNow("systemc stop");
    }
}

void
Scheduler::start(gem5::Tick max_tick, bool run_to_time)
{
    _started = true;
    status(StatusOther);
    runToTime = run_to_time;

    maxTick = max_tick;
    lastReadyTick = getCurTick();

    if (initDone) {
        if (!runToTime && starved())
            scheduleStarvationEvent();
        kernel->status(::sc_core::SC_RUNNING);
    }

    schedule(&maxTickEvent, maxTick);
    scheduleTimeAdvancesEvent();

    // Return to gem5 to let it run events, etc.
    gem5::Fiber::primaryFiber()->run();

    if (pauseEvent.scheduled())
        deschedule(&pauseEvent);
    if (stopEvent.scheduled())
        deschedule(&stopEvent);
    if (maxTickEvent.scheduled())
        deschedule(&maxTickEvent);
    if (starvationEvent.scheduled())
        deschedule(&starvationEvent);

    if (_throwUp) {
        const ::sc_core::sc_report *to_throw = _throwUp;
        _throwUp = nullptr;
        throw *to_throw;
    }
}

void
Scheduler::oneCycle()
{
    runOnce = true;
    scheduleReadyEvent();
    start(gem5::MaxTick, false);
}

void
Scheduler::schedulePause()
{
    if (pauseEvent.scheduled())
        return;

    schedule(&pauseEvent);
}

void
Scheduler::throwUp()
{
    if (scMainFiber.called() && !scMainFiber.finished()) {
        ::sc_core::sc_report report = reportifyException();
        _throwUp = &report;
        status(StatusOther);
        scMainFiber.run();
    } else {
        reportHandlerProc(reportifyException(),
                          ::sc_core::sc_report_handler::get_catch_actions());
    }
}

void
Scheduler::scheduleStop(bool finish_delta)
{
    if (stopEvent.scheduled())
        return;

    if (!finish_delta) {
        _stopNow = true;
        // If we're not supposed to finish the delta cycle, flush all
        // pending activity.
        clear();
    }
    schedule(&stopEvent);
}

void
Scheduler::trace(bool delta)
{
    for (auto tf : traceFiles)
        tf->trace(delta);
}

Scheduler scheduler;

Process *
getCurrentProcess()
{
    return scheduler.current();
}

namespace
{

void
throwingReportHandler(const ::sc_core::sc_report &r,
                      const ::sc_core::sc_actions &)
{
    throw r;
}

} // anonymous namespace

const ::sc_core::sc_report
reportifyException()
{
    ::sc_core::sc_report_handler_proc old_handler = reportHandlerProc;
    ::sc_core::sc_report_handler::set_handler(&throwingReportHandler);

    try {
        try {
            // Rethrow the current exception so we can catch it and throw an
            // sc_report instead if it's not a type we recognize/can handle.
            throw;
        } catch (const ::sc_core::sc_report &) {
            // It's already a sc_report, so nothing to do.
            throw;
        } catch (const ::sc_core::sc_unwind_exception &) {
            panic("Kill/reset exception escaped a Process::run()");
        } catch (const std::exception &e) {
            SC_REPORT_ERROR(sc_core::SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_,
                            e.what());
        } catch (const char *msg) {
            SC_REPORT_ERROR(sc_core::SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_,
                            msg);
        } catch (...) {
            SC_REPORT_ERROR(sc_core::SC_ID_SIMULATION_UNCAUGHT_EXCEPTION_,
                            "UNKNOWN EXCEPTION");
        }
    } catch (const ::sc_core::sc_report &r) {
        ::sc_core::sc_report_handler::set_handler(old_handler);
        return r;
    }
    panic("No exception thrown in reportifyException.");
}

} // namespace sc_gem5
