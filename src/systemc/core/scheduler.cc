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

#include "systemc/core/scheduler.hh"

#include "base/fiber.hh"
#include "base/logging.hh"
#include "sim/eventq.hh"
#include "systemc/core/kernel.hh"
#include "systemc/ext/core/sc_main.hh"
#include "systemc/ext/utils/sc_report.hh"
#include "systemc/ext/utils/sc_report_handler.hh"

namespace sc_gem5
{

Scheduler::Scheduler() :
    eq(nullptr), readyEvent(this, false, ReadyPriority),
    pauseEvent(this, false, PausePriority),
    stopEvent(this, false, StopPriority),
    scMain(nullptr), _throwToScMain(nullptr),
    starvationEvent(this, false, StarvationPriority),
    _elaborationDone(false), _started(false), _stopNow(false),
    _status(StatusOther), maxTickEvent(this, false, MaxTickPriority),
    _numCycles(0), _changeStamp(0), _current(nullptr), initDone(false),
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
    for (auto &tsp: timeSlots) {
        TimeSlot *&ts = tsp.second;
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
    for (Process *p = initList.getNext(); p; p = initList.getNext()) {
        p->popListNode();

        if (p->dontInitialize()) {
            if (!p->hasStaticSensitivities() && !p->internal()) {
                SC_REPORT_WARNING(
                        "(W558) disable() or dont_initialize() called on "
                        "process with no static sensitivity, it will be "
                        "orphaned", p->name());
            }
        } else {
            p->ready();
        }
    }

    runUpdate();
    runDelta();

    for (auto ets: eventsToSchedule)
        eq->schedule(ets.first, ets.second);
    eventsToSchedule.clear();

    if (_started) {
        if (!runToTime && starved())
            scheduleStarvationEvent();
        kernel->status(::sc_core::SC_RUNNING);
    }

    initDone = true;

    status(StatusOther);
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
        Fiber::primaryFiber()->run();
    } else {
        _current->popListNode();
        // Switch to whatever Fiber is supposed to run this process. All
        // Fibers which aren't running should be parked at this line.
        _current->fiber()->run();
        // If the current process needs to be manually started, start it.
        if (_current && _current->needsStart()) {
            _current->needsStart(false);
            try {
                _current->run();
            } catch (...) {
                throwToScMain();
            }
        }
    }
    if (_current && _current->excWrapper) {
        // Make sure this isn't a method process.
        assert(!_current->needsStart());
        auto ew = _current->excWrapper;
        _current->excWrapper = nullptr;
        ew->throw_it();
    }
}

void
Scheduler::ready(Process *p)
{
    if (_stopNow)
        return;

    if (p->procKind() == ::sc_core::SC_METHOD_PROC_)
        readyListMethods.pushLast(p);
    else
        readyListThreads.pushLast(p);

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
    scheduleReadyEvent();
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
    bool empty = readyListMethods.empty() && readyListThreads.empty();
    lastReadyTick = getCurTick();

    // The evaluation phase.
    do {
        yield();
    } while (getNextReady());

    if (!empty) {
        _numCycles++;
        _changeStamp++;
    }

    if (_stopNow)
        return;

    runUpdate();
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

    try {
        Channel *channel = updateList.getNext();
        while (channel) {
            channel->popListNode();
            channel->update();
            channel = updateList.getNext();
        }
    } catch (...) {
        throwToScMain();
    }
}

void
Scheduler::runDelta()
{
    status(StatusDelta);

    try {
        while (!deltas.empty())
            deltas.front()->run();
    } catch (...) {
        throwToScMain();
    }
}

void
Scheduler::pause()
{
    status(StatusPaused);
    kernel->status(::sc_core::SC_PAUSED);
    runOnce = false;
    if (scMain && !scMain->finished())
        scMain->run();
}

void
Scheduler::stop()
{
    status(StatusStopped);
    kernel->stop();

    clear();

    runOnce = false;
    if (scMain && !scMain->finished())
        scMain->run();
}

void
Scheduler::start(Tick max_tick, bool run_to_time)
{
    // We should be running from sc_main. Keep track of that Fiber to return
    // to later.
    scMain = Fiber::currentFiber();

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

    // Return to gem5 to let it run events, etc.
    Fiber::primaryFiber()->run();

    if (pauseEvent.scheduled())
        deschedule(&pauseEvent);
    if (stopEvent.scheduled())
        deschedule(&stopEvent);
    if (maxTickEvent.scheduled())
        deschedule(&maxTickEvent);
    if (starvationEvent.scheduled())
        deschedule(&starvationEvent);

    if (_throwToScMain) {
        const ::sc_core::sc_report *to_throw = _throwToScMain;
        _throwToScMain = nullptr;
        throw *to_throw;
    }
}

void
Scheduler::oneCycle()
{
    runOnce = true;
    scheduleReadyEvent();
    start(::MaxTick, false);
}

void
Scheduler::schedulePause()
{
    if (pauseEvent.scheduled())
        return;

    schedule(&pauseEvent);
}

void
Scheduler::throwToScMain(const ::sc_core::sc_report *r)
{
    if (!r)
        r = reportifyException();
    _throwToScMain = r;
    status(StatusOther);
    scMain->run();
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

Scheduler scheduler;

namespace {

void
throwingReportHandler(const ::sc_core::sc_report &r,
                      const ::sc_core::sc_actions &)
{
    throw r;
}

} // anonymous namespace

const ::sc_core::sc_report *
reportifyException()
{
    ::sc_core::sc_report_handler_proc old_handler =
        ::sc_core::sc_report_handler::get_handler();
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
            SC_REPORT_ERROR("uncaught exception", e.what());
        } catch (const char *msg) {
            SC_REPORT_ERROR("uncaught exception", msg);
        } catch (...) {
            SC_REPORT_ERROR("uncaught exception", "UNKNOWN EXCEPTION");
        }
    } catch (const ::sc_core::sc_report &r) {
        ::sc_core::sc_report_handler::set_handler(old_handler);
        return &r;
    }
    panic("No exception thrown in reportifyException.");
}

} // namespace sc_gem5
