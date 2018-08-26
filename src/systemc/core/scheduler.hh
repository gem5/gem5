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

#include <functional>
#include <map>
#include <set>
#include <vector>

#include "base/logging.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"
#include "systemc/core/channel.hh"
#include "systemc/core/list.hh"
#include "systemc/core/process.hh"
#include "systemc/core/sched_event.hh"

class Fiber;

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
 * When that t0 event happens, it calls the schedulers prepareForInit method
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
 * The readyEvent runs all three steps of the delta cycle. It first goes
 * through the list of runnable processes and executes them until the set is
 * empty, and then immediately runs the update phase. Since these are all part
 * of the same event, there's no chance for other events to intervene and
 * break the required order above.
 *
 * During the update phase above, the spec forbids any action which would make
 * a process runnable. That means that once the update phase finishes, the set
 * of runnable processes will be empty. There may, however, have been some
 * delta notifications/timeouts which will have been scheduled during either
 * the evaluate or update phase above. Those will have been accumulated in the
 * scheduler, and are now all executed.
 *
 * If any processes became runnable during the delta notification phase, the
 * readyEvent will have been scheduled and will be waiting and ready to run
 * again, effectively starting the next delta cycle.
 *
 * TIMED NOTIFICATION PHASE
 *
 * If no processes became runnable, the event queue will continue to process
 * events until it comes across an event which represents all the timed
 * notifications which are supposed to happen at a particular time. The object
 * which tracks them will execute all those notifications, and then destroy
 * itself. If the readyEvent is now ready to run, the next delta cycle will
 * start.
 *
 * PAUSE/STOP
 *
 * To inject a pause from sc_pause which should happen after the current delta
 * cycle's delta notification phase, an event is scheduled with a lower than
 * normal priority, but higher than the readyEvent. That ensures that any
 * delta notifications which are scheduled with normal priority will happen
 * first, since those are part of the current delta cycle. Then the pause
 * event will happen before the next readyEvent which would start the next
 * delta cycle. All of these events are scheduled for the current time, and so
 * would happen before any timed notifications went off.
 *
 * To inject a stop from sc_stop, the delta cycles should stop before even the
 * delta notifications have happened, but after the evaluate and update phases.
 * For that, a stop event with slightly higher than normal priority will be
 * scheduled so that it happens before any of the delta notification events
 * which are at normal priority.
 *
 * MAX RUN TIME
 *
 * When sc_start is called, it's possible to pass in a maximum time the
 * simulation should run to, at which point sc_pause is implicitly called. The
 * simulation is supposed to run up to the latest timed notification phase
 * which is less than or equal to the maximum time. In other words it should
 * run timed notifications at the maximum time, but not the subsequent evaluate
 * phase. That's implemented by scheduling an event at the max time with a
 * priority which is lower than all the others except the ready event. Timed
 * notifications will happen before it fires, but it will override any ready
 * event and prevent the evaluate phase from starting.
 */

class Scheduler
{
  public:
    typedef std::set<ScEvent *> ScEvents;

    class TimeSlot : public ::Event
    {
      public:
        TimeSlot() : ::Event(Default_Pri, AutoDelete) {}

        ScEvents events;
        void process();
    };

    typedef std::map<Tick, TimeSlot *> TimeSlots;

    Scheduler();
    ~Scheduler();

    void clear();

    const std::string name() const { return "systemc_scheduler"; }

    uint64_t numCycles() { return _numCycles; }
    Process *current() { return _current; }

    void initPhase();

    // Register a process with the scheduler.
    void reg(Process *p);

    // Tell the scheduler not to initialize a process.
    void dontInitialize(Process *p);

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

    // Get the current time according to gem5.
    Tick getCurTick() { return eq ? eq->getCurTick() : 0; }

    Tick
    delayed(const ::sc_core::sc_time &delay)
    {
        //XXX We're assuming the systemc time resolution is in ps.
        return getCurTick() + delay.value() * SimClock::Int::ps;
    }

    // For scheduling delayed/timed notifications/timeouts.
    void
    schedule(ScEvent *event, const ::sc_core::sc_time &delay)
    {
        Tick tick = delayed(delay);
        event->schedule(tick);

        // Delta notification/timeout.
        if (delay.value() == 0) {
            deltas.insert(event);
            scheduleReadyEvent();
            return;
        }

        // Timed notification/timeout.
        TimeSlot *&ts = timeSlots[tick];
        if (!ts) {
            ts = new TimeSlot;
            schedule(ts, tick);
        }
        ts->events.insert(event);
    }

    // For descheduling delayed/timed notifications/timeouts.
    void
    deschedule(ScEvent *event)
    {
        if (event->when() == getCurTick()) {
            // Attempt to remove from delta notifications.
            if (deltas.erase(event) == 1) {
                event->deschedule();
                return;
            }
        }

        // Timed notification/timeout.
        auto tsit = timeSlots.find(event->when());
        panic_if(tsit == timeSlots.end(),
                "Descheduling event at time with no events.");
        TimeSlot *ts = tsit->second;
        ScEvents &events = ts->events;
        assert(events.erase(event));
        event->deschedule();

        // If no more events are happening at this time slot, get rid of it.
        if (events.empty()) {
            deschedule(ts);
            timeSlots.erase(tsit);
        }
    }

    void
    completeTimeSlot(TimeSlot *ts)
    {
        assert(ts == timeSlots.begin()->second);
        timeSlots.erase(timeSlots.begin());
        if (!runToTime && starved())
            scheduleStarvationEvent();
    }

    // Pending activity ignores gem5 activity, much like how a systemc
    // simulation wouldn't know about asynchronous external events (socket IO
    // for instance) that might happen before time advances in a pure
    // systemc simulation. Also the spec lists what specific types of pending
    // activity needs to be counted, which obviously doesn't include gem5
    // events.

    // Return whether there's pending systemc activity at this time.
    bool
    pendingCurr()
    {
        return !readyList.empty() || !updateList.empty() || !deltas.empty();
    }

    // Return whether there are pending timed notifications or timeouts.
    bool
    pendingFuture()
    {
        return !timeSlots.empty();
    }

    // Return how many ticks there are until the first pending event, if any.
    Tick
    timeToPending()
    {
        if (pendingCurr())
            return 0;
        if (pendingFuture())
            return timeSlots.begin()->first - getCurTick();
        return MaxTick - getCurTick();
    }

    // Run scheduled channel updates.
    void update();

    void setScMainFiber(Fiber *sc_main) { scMain = sc_main; }

    void start(Tick max_tick, bool run_to_time);
    void oneCycle();

    void schedulePause();
    void scheduleStop(bool finish_delta);

    bool paused() { return _paused; }
    bool stopped() { return _stopped; }

  private:
    typedef const EventBase::Priority Priority;
    static Priority DefaultPriority = EventBase::Default_Pri;

    static Priority StopPriority = DefaultPriority - 1;
    static Priority PausePriority = DefaultPriority + 1;
    static Priority MaxTickPriority = DefaultPriority + 2;
    static Priority ReadyPriority = DefaultPriority + 3;
    static Priority StarvationPriority = ReadyPriority;

    EventQueue *eq;

    // For gem5 style events.
    void
    schedule(::Event *event, Tick tick)
    {
        if (initDone)
            eq->schedule(event, tick);
        else
            eventsToSchedule[event] = tick;
    }

    void schedule(::Event *event) { schedule(event, getCurTick()); }

    void
    deschedule(::Event *event)
    {
        if (initDone)
            eq->deschedule(event);
        else
            eventsToSchedule.erase(event);
    }

    ScEvents deltas;
    TimeSlots timeSlots;

    void runReady();
    EventWrapper<Scheduler, &Scheduler::runReady> readyEvent;
    void scheduleReadyEvent();

    void pause();
    void stop();
    EventWrapper<Scheduler, &Scheduler::pause> pauseEvent;
    EventWrapper<Scheduler, &Scheduler::stop> stopEvent;
    Fiber *scMain;

    bool
    starved()
    {
        return (readyList.empty() && updateList.empty() && deltas.empty() &&
                (timeSlots.empty() || timeSlots.begin()->first > maxTick) &&
                initList.empty());
    }
    EventWrapper<Scheduler, &Scheduler::pause> starvationEvent;
    void scheduleStarvationEvent();

    bool _started;
    bool _paused;
    bool _stopped;

    Tick maxTick;
    EventWrapper<Scheduler, &Scheduler::pause> maxTickEvent;

    uint64_t _numCycles;

    Process *_current;

    bool initDone;
    bool runToTime;
    bool runOnce;

    ProcessList initList;
    ProcessList toFinalize;
    ProcessList readyList;

    ChannelList updateList;

    std::map<::Event *, Tick> eventsToSchedule;
};

extern Scheduler scheduler;

inline void
Scheduler::TimeSlot::process()
{
    for (auto &e: events)
        e->run();
    scheduler.completeTimeSlot(this);
}

} // namespace sc_gem5

#endif // __SYSTEMC_CORE_SCHEDULER_H__
