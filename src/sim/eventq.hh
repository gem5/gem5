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

#include <algorithm>
#include <cassert>
#include <climits>
#include <map>
#include <string>
#include <vector>

#include "base/fast_alloc.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "sim/serialize.hh"
#include "sim/host.hh"

class EventQueue;       // forward declaration

extern EventQueue mainEventQueue;

/*
 * An item on an event queue.  The action caused by a given
 * event is specified by deriving a subclass and overriding the
 * process() member function.
 *
 * Caution, the order of members is chosen to maximize data packing.
 */
class Event : public Serializable, public FastAlloc
{
    friend class EventQueue;

  private:
    // The event queue is now a linked list of linked lists.  The
    // 'nextBin' pointer is to find the bin, where a bin is defined as
    // when+priority.  All events in the same bin will be stored in a
    // second linked list (a stack) maintained by the 'nextInBin'
    // pointer.  The list will be accessed in LIFO order.  The end
    // result is that the insert/removal in 'nextBin' is
    // linear/constant, and the lookup/removal in 'nextInBin' is
    // constant/constant.  Hopefully this is a significant improvement
    // over the current fully linear insertion.
    Event *nextBin;
    Event *nextInBin;

    static Event *insertBefore(Event *event, Event *curr);
    static Event *removeItem(Event *event, Event *last);

    Tick _when;         //!< timestamp when event should be processed
    short _priority;    //!< event priority
    short _flags;

#ifndef NDEBUG
    /// Global counter to generate unique IDs for Event instances
    static Counter instanceCounter;

    /// This event's unique ID.  We can also use pointer values for
    /// this but they're not consistent across runs making debugging
    /// more difficult.  Thus we use a global counter value when
    /// debugging.
    Counter instance;

    /// queue to which this event belongs (though it may or may not be
    /// scheduled on this queue yet)
    EventQueue *queue;
#endif

#ifdef EVENTQ_DEBUG
    Tick whenCreated;   //!< time created
    Tick whenScheduled; //!< time scheduled
#endif

    void
    setWhen(Tick when, EventQueue *q)
    {
        _when = when;
#ifndef NDEBUG
        queue = q;
#endif
#ifdef EVENTQ_DEBUG
        whenScheduled = curTick;
#endif
    }

  protected:
    enum Flags {
        None = 0x0,
        Squashed = 0x1,
        Scheduled = 0x2,
        AutoDelete = 0x4,
        AutoSerialize = 0x8,
        IsExitEvent = 0x10,
        IsMainQueue = 0x20
    };

    bool getFlags(Flags f) const { return (_flags & f) == f; }
    void setFlags(Flags f) { _flags |= f; }
    void clearFlags(Flags f) { _flags &= ~f; }

  protected:
    // This function isn't really useful if TRACING_ON is not defined
    virtual void trace(const char *action);     //!< trace event activity

  public:
    /// Event priorities, to provide tie-breakers for events scheduled
    /// at the same cycle.  Most events are scheduled at the default
    /// priority; these values are used to control events that need to
    /// be ordered within a cycle.
    enum Priority {
        /// Minimum priority
        Minimum_Pri             = SHRT_MIN,

        /// If we enable tracing on a particular cycle, do that as the
        /// very first thing so we don't miss any of the events on
        /// that cycle (even if we enter the debugger).
        Trace_Enable_Pri        = -101,

        /// Breakpoints should happen before anything else (except
        /// enabling trace output), so we don't miss any action when
        /// debugging.
        Debug_Break_Pri         = -100,

        /// CPU switches schedule the new CPU's tick event for the
        /// same cycle (after unscheduling the old CPU's tick event).
        /// The switch needs to come before any tick events to make
        /// sure we don't tick both CPUs in the same cycle.
        CPU_Switch_Pri          =   -31,

        /// For some reason "delayed" inter-cluster writebacks are
        /// scheduled before regular writebacks (which have default
        /// priority).  Steve?
        Delayed_Writeback_Pri   =   -1,

        /// Default is zero for historical reasons.
        Default_Pri             =    0,

        /// Serailization needs to occur before tick events also, so
        /// that a serialize/unserialize is identical to an on-line
        /// CPU switch.
        Serialize_Pri           =   32,

        /// CPU ticks must come after other associated CPU events
        /// (such as writebacks).
        CPU_Tick_Pri            =   50,

        /// Statistics events (dump, reset, etc.) come after
        /// everything else, but before exit.
        Stat_Event_Pri          =   90,

        /// Progress events come at the end.
        Progress_Event_Pri      =   95,

        /// If we want to exit on this cycle, it's the very last thing
        /// we do.
        Sim_Exit_Pri            =  100,

        /// Maximum priority
        Maximum_Pri             = SHRT_MAX
    };

    /*
     * Event constructor
     * @param queue that the event gets scheduled on
     */
    Event(Priority p = Default_Pri)
        : nextBin(NULL), nextInBin(NULL), _priority(p), _flags(None)
    {
#ifndef NDEBUG
        instance = ++instanceCounter;
        queue = NULL;
#endif
#ifdef EVENTQ_DEBUG
        whenCreated = curTick;
        whenScheduled = 0;
#endif
    }

    virtual
    ~Event()
    {
    }

    virtual const std::string
    name() const
    {
#ifndef NDEBUG
        return csprintf("Event_%d", instance);
#else
        return csprintf("Event_%x", (uintptr_t)this);
#endif
    }

    /// Return a C string describing the event.  This string should
    /// *not* be dynamically allocated; just a const char array
    /// describing the event class.
    virtual const char *description() const;

    /// Dump the current event data
    void dump() const;

  public:
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

    /// Determine if the current event is scheduled
    bool scheduled() const { return getFlags(Scheduled); }

    /// Squash the current event
    void squash() { setFlags(Squashed); }

    /// Check whether the event is squashed
    bool squashed() const { return getFlags(Squashed); }

    /// See if this is a SimExitEvent (without resorting to RTTI)
    bool isExitEvent() const { return getFlags(IsExitEvent); }

    /// Get the time that the event is scheduled
    Tick when() const { return _when; }

    /// Get the event priority
    int priority() const { return _priority; }

#ifndef SWIG
    struct priority_compare
        : public std::binary_function<Event *, Event *, bool>
    {
        bool
        operator()(const Event *l, const Event *r) const
        {
            return l->when() >= r->when() || l->priority() >= r->priority();
        }
    };

    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
#endif
};

/*
 * Queue of events sorted in time order
 */
class EventQueue : public Serializable
{
  private:
    std::string objName;
    Event *head;

    void insert(Event *event);
    void remove(Event *event);

  public:
    EventQueue(const std::string &n)
        : objName(n), head(NULL)
    {}

    virtual const std::string name() const { return objName; }

    // schedule the given event on this queue
    void schedule(Event *ev, Tick when);
    void deschedule(Event *ev);
    void reschedule(Event *ev, Tick when, bool always = false);

    Tick nextTick() const { return head->when(); }
    Event *serviceOne();

    // process all events up to the given timestamp.  we inline a
    // quick test to see if there are any events to process; if so,
    // call the internal out-of-line version to process them all.
    void
    serviceEvents(Tick when)
    {
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
    bool empty() const { return head == NULL; }

    void dump() const;

    Tick nextEventTime() { return empty() ? curTick : head->when(); }

    bool debugVerify() const;

#ifndef SWIG
    virtual void serialize(std::ostream &os);
    virtual void unserialize(Checkpoint *cp, const std::string &section);
#endif
};

#ifndef SWIG
class EventManager
{
  protected:
    /** A pointer to this object's event queue */
    EventQueue *eventq;

  public:
    EventManager(EventManager &em) : eventq(em.queue()) {}
    EventManager(EventManager *em) : eventq(em ? em->queue() : NULL) {}
    EventManager(EventQueue *eq) : eventq(eq) {}

    EventQueue *
    queue() const
    {
        return eventq;
    }

    void
    schedule(Event &event, Tick when)
    {
        eventq->schedule(&event, when);
    }

    void
    deschedule(Event &event)
    {
        eventq->deschedule(&event);
    }

    void
    reschedule(Event &event, Tick when, bool always = false)
    {
        eventq->reschedule(&event, when, always);
    }

    void
    schedule(Event *event, Tick when)
    {
        eventq->schedule(event, when);
    }

    void
    deschedule(Event *event)
    {
        eventq->deschedule(event);
    }

    void
    reschedule(Event *event, Tick when, bool always = false)
    {
        eventq->reschedule(event, when, always);
    }
};

template <class T, void (T::* F)()>
void
DelayFunction(EventQueue *eventq, Tick when, T *object)
{
    class DelayEvent : public Event
    {
      private:
        T *object;

      public:
        DelayEvent(T *o)
            : object(o)
        { setFlags(this->AutoDestroy); }
        void process() { (object->*F)(); }
        const char *description() const { return "delay"; }
    };

    eventq->schedule(new DelayEvent(object), when);
}

template <class T, void (T::* F)()>
class EventWrapper : public Event
{
  private:
    T *object;

  public:
    EventWrapper(T *obj, bool del = false, Priority p = Default_Pri)
        : Event(p), object(obj)
    {
        if (del)
            setFlags(AutoDelete);
    }

    void process() { (object->*F)(); }
};

inline void
EventQueue::schedule(Event *event, Tick when)
{
    assert(when >= curTick);
    assert(!event->scheduled());

    event->setWhen(when, this);
    insert(event);
    event->setFlags(Event::Scheduled);
    if (this == &mainEventQueue)
        event->setFlags(Event::IsMainQueue);
    else
        event->clearFlags(Event::IsMainQueue);

    if (DTRACE(Event))
        event->trace("scheduled");
}

inline void
EventQueue::deschedule(Event *event)
{
    assert(event->scheduled());

    remove(event);

    event->clearFlags(Event::Squashed);
    event->clearFlags(Event::Scheduled);

    if (event->getFlags(Event::AutoDelete))
        delete event;

    if (DTRACE(Event))
        event->trace("descheduled");
}

inline void
EventQueue::reschedule(Event *event, Tick when, bool always)
{
    assert(when >= curTick);
    assert(always || event->scheduled());

    if (event->scheduled())
        remove(event);
            
    event->setWhen(when, this);
    insert(event);
    event->clearFlags(Event::Squashed);
    event->setFlags(Event::Scheduled);
    if (this == &mainEventQueue)
        event->setFlags(Event::IsMainQueue);
    else
        event->clearFlags(Event::IsMainQueue);

    if (DTRACE(Event))
        event->trace("rescheduled");
}

inline bool
operator<(const Event &l, const Event &r)
{
    return l.when() < r.when() ||
        (l.when() == r.when() && l.priority() < r.priority());
}

inline bool
operator>(const Event &l, const Event &r)
{
    return l.when() > r.when() ||
        (l.when() == r.when() && l.priority() > r.priority());
}

inline bool
operator<=(const Event &l, const Event &r)
{
    return l.when() < r.when() ||
        (l.when() == r.when() && l.priority() <= r.priority());
}
inline bool
operator>=(const Event &l, const Event &r)
{
    return l.when() > r.when() ||
        (l.when() == r.when() && l.priority() >= r.priority());
}

inline bool
operator==(const Event &l, const Event &r)
{
    return l.when() == r.when() && l.priority() == r.priority();
}

inline bool
operator!=(const Event &l, const Event &r)
{
    return l.when() != r.when() || l.priority() != r.priority();
}
#endif

#endif // __SIM_EVENTQ_HH__
