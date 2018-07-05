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

#ifndef __SYSTEMC_CORE_SCHEDULER_HH__
#define __SYSTEMC_CORE_SCHEDULER_HH__

#include "sim/eventq.hh"
#include "systemc/core/channel.hh"
#include "systemc/core/list.hh"
#include "systemc/core/process.hh"

namespace sc_gem5
{

typedef NodeList<Process> ProcessList;
typedef NodeList<Channel> ChannelList;

/*
 * The scheduler supports three different mechanisms, the initialization phase,
 * delta cycles, and timed notifications.
 *
 * INITIALIZATION PHASE
 *
 * The initialization phase has three parts:
 * 1. Run requested channel updates.
 * 2. Make processes which need to initialize runnable (methods and threads
 *    which didn't have dont_initialize called on them).
 * 3. Process delta notifications.
 *
 * First, the Kernel SimObject calls the update() method during its startup()
 * callback which handles the requested channel updates. The Kernel also
 * schedules an event to be run at time 0 with a slightly elevated priority
 * so that it happens before any "normal" event.
 *
 * When that t0 event happens, it calls the schedulers initToReady method
 * which performs step 2 above. That indirectly causes the scheduler's
 * readyEvent to be scheduled with slightly lowered priority, ensuring it
 * happens after any "normal" event.
 *
 * Because delta notifications are scheduled at the standard priority, all
 * of those events will happen next, performing step 3 above. Once they finish,
 * if the readyEvent was scheduled above, there shouldn't be any higher
 * priority events in front of it. When it runs, it will start the first
 * evaluate phase of the first delta cycle.
 *
 * DELTA CYCLE
 *
 * A delta cycle has three phases within it.
 * 1. The evaluate phase where runnable processes are allowed to run.
 * 2. The update phase where requested channel updates hapen.
 * 3. The delta notification phase where delta notifications happen.
 *
 * The readyEvent runs the first two steps of the delta cycle. It first goes
 * through the list of runnable processes and executes them until the set is
 * empty, and then immediately runs the update phase. Since these are all part
 * of the same event, there's no chance for other events to intervene and
 * break the required order above.
 *
 * During the update phase above, the spec forbids any action which would make
 * a process runnable. That means that once the update phase finishes, the set
 * of runnable processes will be empty. There may, however, have been some
 * delta notifications/timeouts which will have been scheduled during either
 * the evaluate or update phase above. Because those are scheduled at the
 * normal priority, they will now happen together until there aren't any
 * delta events left.
 *
 * If any processes became runnable during the delta notification phase, the
 * readyEvent will have been scheduled and will have been waiting patiently
 * behind the delta notification events. That will now run, effectively
 * starting the next delta cycle.
 *
 * TIMED NOTIFICATION PHASE
 *
 * If no processes became runnable, the event queue will continue to process
 * events until it comes across a timed notification, aka a notification
 * scheduled to happen in the future. Like delta notification events, those
 * will all happen together since the readyEvent priority is lower,
 * potentially marking new processes as ready. Once these events finish, the
 * readyEvent may run, starting the next delta cycle.
 */

class Scheduler
{
  public:
    Scheduler();

    const std::string name() const { return "systemc_scheduler"; }

    uint64_t numCycles() { return _numCycles; }
    Process *current() { return _current; }

    // Mark processes that need to be initialized as ready.
    void initToReady();

    // Put a process on the list of processes to be initialized.
    void init(Process *p) { initList.pushLast(p); }

    // Run the next process, if there is one.
    void yield();

    // Put a process on the ready list.
    void ready(Process *p);

    // Schedule an update for a given channel.
    void requestUpdate(Channel *c);

    // Run the given process immediately, preempting whatever may be running.
    void
    runNow(Process *p)
    {
        // If a process is running, schedule it/us to run again.
        if (_current)
            readyList.pushFirst(_current);
        // Schedule p to run first.
        readyList.pushFirst(p);
        yield();
    }

    // Set an event queue for scheduling events.
    void setEventQueue(EventQueue *_eq) { eq = _eq; }

    // Retrieve the event queue.
    EventQueue &eventQueue() const { return *eq; }

    // Run scheduled channel updates.
    void update();

  private:
    EventQueue *eq;

    void runReady();
    EventWrapper<Scheduler, &Scheduler::runReady> readyEvent;
    void scheduleReadyEvent();

    uint64_t _numCycles;

    Process *_current;

    ProcessList initList;
    ProcessList readyList;

    ChannelList updateList;
};

extern Scheduler scheduler;

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_SCHEDULER_H__
