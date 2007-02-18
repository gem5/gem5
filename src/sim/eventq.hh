/*
 * Copyright (c) 2000-2005 The Regents of The University of Michigan
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
 *
 * Authors: Steve Reinhardt
 *          Nathan Binkert
 */

/* @file
 * EventQueue interfaces
 */

#ifndef __SIM_EVENTQ_HH__
#define __SIM_EVENTQ_HH__

#include <assert.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "sim/host.hh"	// for Tick

#include "base/fast_alloc.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "sim/serialize.hh"

class EventQueue;	// forward declaration

//////////////////////
//
// Main Event Queue
//
// Events on this queue are processed at the *beginning* of each
// cycle, before the pipeline simulation is performed.
//
// defined in eventq.cc
//
//////////////////////
extern EventQueue mainEventQueue;


/*
 * An item on an event queue.  The action caused by a given
 * event is specified by deriving a subclass and overriding the
 * process() member function.
 */
class Event : public Serializable, public FastAlloc
{
    friend class EventQueue;

  private:

#ifndef NDEBUG
    /// Global counter to generate unique IDs for Event instances
    static Counter instanceCounter;

    /// This event's unique ID.  We can also use pointer values for
    /// this but they're not consistent across runs making debugging
    /// more difficult.  Thus we use a global counter value when
    /// debugging.
    Counter instanceId;
#endif // NDEBUG

    /// queue to which this event belongs (though it may or may not be
    /// scheduled on this queue yet)
    EventQueue *queue;

    Event *next;

    Tick _when;	//!< timestamp when event should be processed
    int _priority;	//!< event priority
    char _flags;

  protected:
    enum Flags {
        None = 0x0,
        Squashed = 0x1,
        Scheduled = 0x2,
        AutoDelete = 0x4,
        AutoSerialize = 0x8,
        IsExitEvent = 0x10
    };

    bool getFlags(Flags f) const { return (_flags & f) == f; }
    void setFlags(Flags f) { _flags |= f; }
    void clearFlags(Flags f) { _flags &= ~f; }

  protected:
    EventQueue *theQueue() const { return queue; }

#if TRACING_ON
    Tick when_created;	//!< Keep track of creation time For debugging
    Tick when_scheduled;	//!< Keep track of creation time For debugging

    virtual void trace(const char *action);	//!< trace event activity
#else
    void trace(const char *) {}
#endif

    unsigned annotated_value;

  public:

    /// Event priorities, to provide tie-breakers for events scheduled
    /// at the same cycle.  Most events are scheduled at the default
    /// priority; these values are used to control events that need to
    /// be ordered within a cycle.
    enum Priority {
        /// If we enable tracing on a particular cycle, do that as the
        /// very first thing so we don't miss any of the events on
        /// that cycle (even if we enter the debugger).
        Trace_Enable_Pri        = -101,

        /// Breakpoints should happen before anything else (except
        /// enabling trace output), so we don't miss any action when
        /// debugging.
        Debug_Break_Pri		= -100,

        /// CPU switches schedule the new CPU's tick event for the
        /// same cycle (after unscheduling the old CPU's tick event).
        /// The switch needs to come before any tick events to make
        /// sure we don't tick both CPUs in the same cycle.
        CPU_Switch_Pri		=   -31,

        /// For some reason "delayed" inter-cluster writebacks are
        /// scheduled before regular writebacks (which have default
        /// priority).  Steve?
        Delayed_Writeback_Pri	=   -1,

        /// Default is zero for historical reasons.
        Default_Pri		=    0,

        /// Serailization needs to occur before tick events also, so
        /// that a serialize/unserialize is identical to an on-line
        /// CPU switch.
        Serialize_Pri		=   32,

        /// CPU ticks must come after other associated CPU events
        /// (such as writebacks).
        CPU_Tick_Pri		=   50,

        /// Statistics events (dump, reset, etc.) come after
        /// everything else, but before exit.
        Stat_Event_Pri		=   90,

        /// Progress events come at the end.
        Progress_Event_Pri      =   95,

        /// If we want to exit on this cycle, it's the very last thing
        /// we do.
        Sim_Exit_Pri		=  100
    };

    /*
     * Event constructor
     * @param queue that the event gets scheduled on
     */
    Event(EventQueue *q, Priority p = Default_Pri)
        : queue(q), next(NULL), _priority(p), _flags(None),
#if TRACING_ON
          when_created(curTick), when_scheduled(0),
#endif
          annotated_value(0)
    {
#ifndef NDEBUG
        instanceId = ++instanceCounter;
#endif
    }

    ~Event() {}

    virtual const std::string name() const {
#ifndef NDEBUG
        return csprintf("Event_%d", instanceId);
#else
        return csprintf("Event_%x", (uintptr_t)this);
#endif
    }

    /// Determine if the current event is scheduled
    bool scheduled() const { return getFlags(Scheduled); }

    /// Schedule the event with the current priority or default priority
    void schedule(Tick t);

    /// Reschedule the event with the current priority
    void reschedule(Tick t);

    /// Remove the event from the current schedule
    void deschedule();

    /// Return a C string describing the event.  This string should
    /// *not* be dynamically allocated; just a const char array
    /// describing the event class.
    virtual const char *description();

    /// Dump the current event data
    void dump();

    /*
     * This member function is invoked when the event is processed
     * (occurs).  There is no default implementation; each subclass
     * must provide its own implementation.  The event is not
     * automatically deleted after it is processed (to allow for
     * statically allocated event objects).
     *
     * If the AutoDestroy flag is set, the object is deleted once it
     * is processed.
     */
    virtual void process() = 0;

    void annotate(unsigned value) { annotated_value = value; };
    unsigned annotation() { return annotated_value; }

    /// Squash the current event
    void squash() { setFlags(Squashed); }

    /// Check whether the event is squashed
    bool squashed() { return getFlags(Squashed); }

    /// See if this is a SimExitEvent (without resorting to RTTI)
    bool isExitEvent() { return getFlags(IsExitEvent); }

    /// Get the time that the event is scheduled
    Tick when() const { return _when; }

    /// Get the event priority
    int priority() const { return _priority; }

    struct priority_compare :
    public std::binary_function<Event *, Event *, bool>
    {
        bool operator()(const Event *l, const Event *r) const {
            return l->when() >= r->when() || l->priority() >= r->priority();
        }
    };

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};

template <class T, void (T::* F)()>
void
DelayFunction(Tick when, T *object)
{
    class DelayEvent : public Event
    {
      private:
        T *object;

      public:
        DelayEvent(Tick when, T *o)
            : Event(&mainEventQueue), object(o)
            { setFlags(this->AutoDestroy); schedule(when); }
        void process() { (object->*F)(); }
        const char *description() { return "delay"; }
    };

    new DelayEvent(when, object);
}

template <class T, void (T::* F)()>
class EventWrapper : public Event
{
  private:
    T *object;

  public:
    EventWrapper(T *obj, bool del = false, EventQueue *q = &mainEventQueue,
                 Priority p = Default_Pri)
        : Event(q, p), object(obj)
    {
        if (del)
            setFlags(AutoDelete);
    }
    void process() { (object->*F)(); }
};

/*
 * Queue of events sorted in time order
 */
class EventQueue : public Serializable
{
  protected:
    std::string objName;

  private:
    Event *head;

    void insert(Event *event);
    void remove(Event *event);

  public:

    // constructor
    EventQueue(const std::string &n)
        : objName(n), head(NULL)
    {}

    virtual const std::string name() const { return objName; }

    // schedule the given event on this queue
    void schedule(Event *ev);
    void deschedule(Event *ev);
    void reschedule(Event *ev);

    Tick nextTick() { return head->when(); }
    Event *serviceOne();

    // process all events up to the given timestamp.  we inline a
    // quick test to see if there are any events to process; if so,
    // call the internal out-of-line version to process them all.
    void serviceEvents(Tick when) {
        while (!empty()) {
            if (nextTick() > when)
                break;

            /**
             * @todo this assert is a good bug catcher.  I need to
             * make it true again.
             */
            //assert(head->when() >= when && "event scheduled in the past");
            serviceOne();
        }
    }

    // default: process all events up to 'now' (curTick)
    void serviceEvents() { serviceEvents(curTick); }

    // return true if no events are queued
    bool empty() { return head == NULL; }

    void dump();

    Tick nextEventTime() { return empty() ? curTick : head->when(); }

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
};


//////////////////////
//
// inline functions
//
// can't put these inside declaration due to circular dependence
// between Event and EventQueue classes.
//
//////////////////////

// schedule at specified time (place on event queue specified via
// constructor)
inline void
Event::schedule(Tick t)
{
    assert(!scheduled());
//    if (t < curTick)
//        warn("t is less than curTick, ensure you don't want cycles");

    setFlags(Scheduled);
#if TRACING_ON
    when_scheduled = curTick;
#endif
    _when = t;
    queue->schedule(this);
}

inline void
Event::deschedule()
{
    assert(scheduled());

    clearFlags(Squashed);
    clearFlags(Scheduled);
    queue->deschedule(this);
}

inline void
Event::reschedule(Tick t)
{
    assert(scheduled());
    clearFlags(Squashed);

#if TRACING_ON
    when_scheduled = curTick;
#endif
    _when = t;
    queue->reschedule(this);
}

inline void
EventQueue::schedule(Event *event)
{
    insert(event);
    if (DTRACE(Event))
        event->trace("scheduled");
}

inline void
EventQueue::deschedule(Event *event)
{
    remove(event);
    if (DTRACE(Event))
        event->trace("descheduled");
}

inline void
EventQueue::reschedule(Event *event)
{
    remove(event);
    insert(event);
    if (DTRACE(Event))
        event->trace("rescheduled");
}



#endif // __SIM_EVENTQ_HH__
