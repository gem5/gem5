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
#include <iosfwd>
#include <string>

#include "base/flags.hh"
#include "base/misc.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/Event.hh"
#include "sim/serialize.hh"

class EventQueue;       // forward declaration

extern EventQueue mainEventQueue;

/*
 * An item on an event queue.  The action caused by a given
 * event is specified by deriving a subclass and overriding the
 * process() member function.
 *
 * Caution, the order of members is chosen to maximize data packing.
 */
class Event : public Serializable
{
    friend class EventQueue;

  protected:   
    typedef unsigned short FlagsType;
    typedef ::Flags<FlagsType> Flags;

    static const FlagsType PublicRead    = 0x003f; // public readable flags
    static const FlagsType PublicWrite   = 0x001d; // public writable flags
    static const FlagsType Squashed      = 0x0001; // has been squashed
    static const FlagsType Scheduled     = 0x0002; // has been scheduled
    static const FlagsType AutoDelete    = 0x0004; // delete after dispatch
    static const FlagsType AutoSerialize = 0x0008; // must be serialized
    static const FlagsType IsExitEvent   = 0x0010; // special exit event
    static const FlagsType IsMainQueue   = 0x0020; // on main event queue
    static const FlagsType Initialized   = 0x7a40; // somewhat random bits
    static const FlagsType InitMask      = 0xffc0; // mask for init bits

    bool
    initialized() const
    {
        return this && (flags & InitMask) == Initialized;
    }

  public:
    typedef int8_t Priority;

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
    Priority _priority; //!< event priority
    Flags flags;

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
        whenScheduled = curTick();
#endif
    }

  protected:
    /// Accessor for flags.
    Flags
    getFlags() const
    {
        return flags & PublicRead;
    }

    bool
    isFlagSet(Flags _flags) const
    {
        assert(_flags.noneSet(~PublicRead));
        return flags.isSet(_flags);
    }

    /// Accessor for flags.
    void
    setFlags(Flags _flags)
    {
        assert(_flags.noneSet(~PublicWrite));
        flags.set(_flags);
    }

    void
    clearFlags(Flags _flags)
    {
        assert(_flags.noneSet(~PublicWrite));
        flags.clear(_flags);
    }

    void
    clearFlags()
    {
        flags.clear(PublicWrite);
    }

    // This function isn't really useful if TRACING_ON is not defined
    virtual void trace(const char *action);     //!< trace event activity

  public:
    /// Event priorities, to provide tie-breakers for events scheduled
    /// at the same cycle.  Most events are scheduled at the default
    /// priority; these values are used to control events that need to
    /// be ordered within a cycle.

    /// Minimum priority
    static const Priority Minimum_Pri =          SCHAR_MIN;

    /// If we enable tracing on a particular cycle, do that as the
    /// very first thing so we don't miss any of the events on
    /// that cycle (even if we enter the debugger).
    static const Priority Trace_Enable_Pri =          -101;

    /// Breakpoints should happen before anything else (except
    /// enabling trace output), so we don't miss any action when
    /// debugging.
    static const Priority Debug_Break_Pri =           -100;

    /// CPU switches schedule the new CPU's tick event for the
    /// same cycle (after unscheduling the old CPU's tick event).
    /// The switch needs to come before any tick events to make
    /// sure we don't tick both CPUs in the same cycle.
    static const Priority CPU_Switch_Pri =             -31;

    /// For some reason "delayed" inter-cluster writebacks are
    /// scheduled before regular writebacks (which have default
    /// priority).  Steve?
    static const Priority Delayed_Writeback_Pri =       -1;

    /// Default is zero for historical reasons.
    static const Priority Default_Pri =                  0;

    /// Serailization needs to occur before tick events also, so
    /// that a serialize/unserialize is identical to an on-line
    /// CPU switch.
    static const Priority Serialize_Pri =               32;

    /// CPU ticks must come after other associated CPU events
    /// (such as writebacks).
    static const Priority CPU_Tick_Pri =                50;

    /// Statistics events (dump, reset, etc.) come after
    /// everything else, but before exit.
    static const Priority Stat_Event_Pri =              90;

    /// Progress events come at the end.
    static const Priority Progress_Event_Pri =          95;

    /// If we want to exit on this cycle, it's the very last thing
    /// we do.
    static const Priority Sim_Exit_Pri =               100;

    /// Maximum priority
    static const Priority Maximum_Pri =          SCHAR_MAX;

    /*
     * Event constructor
     * @param queue that the event gets scheduled on
     */
    Event(Priority p = Default_Pri, Flags f = 0)
        : nextBin(NULL), nextInBin(NULL), _priority(p),
          flags(Initialized | f)
    {
        assert(f.noneSet(~PublicWrite));
#ifndef NDEBUG
        instance = ++instanceCounter;
        queue = NULL;
#endif
#ifdef EVENTQ_DEBUG
        whenCreated = curTick();
        whenScheduled = 0;
#endif
    }

    virtual ~Event();
    virtual const std::string name() const;

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
    bool scheduled() const { return flags.isSet(Scheduled); }

    /// Squash the current event
    void squash() { flags.set(Squashed); }

    /// Check whether the event is squashed
    bool squashed() const { return flags.isSet(Squashed); }

    /// See if this is a SimExitEvent (without resorting to RTTI)
    bool isExitEvent() const { return flags.isSet(IsExitEvent); }

    /// Get the time that the event is scheduled
    Tick when() const { return _when; }

    /// Get the event priority
    Priority priority() const { return _priority; }

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

#ifndef SWIG
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

    EventQueue(const EventQueue &);
    const EventQueue &operator=(const EventQueue &);

  public:
    EventQueue(const std::string &n);

    virtual const std::string name() const { return objName; }

    // schedule the given event on this queue
    void schedule(Event *event, Tick when);
    void deschedule(Event *event);
    void reschedule(Event *event, Tick when, bool always = false);

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

    // return true if no events are queued
    bool empty() const { return head == NULL; }

    void dump() const;

    bool debugVerify() const;

    /**
     *  function for replacing the head of the event queue, so that a
     *  different set of events can run without disturbing events that have
     *  already been scheduled. Already scheduled events can be processed
     *  by replacing the original head back.
     *  USING THIS FUNCTION CAN BE DANGEROUS TO THE HEALTH OF THE SIMULATOR.
     *  NOT RECOMMENDED FOR USE.
     */
    Event* replaceHead(Event* s);

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

    operator EventQueue *() const
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

inline void
EventQueue::schedule(Event *event, Tick when)
{
    // Typecasting Tick->Utick here since gcc
    // complains about signed overflow
    assert((UTick)when >= (UTick)curTick());
    assert(!event->scheduled());
    assert(event->initialized());

    event->setWhen(when, this);
    insert(event);
    event->flags.set(Event::Scheduled);
    if (this == &mainEventQueue)
        event->flags.set(Event::IsMainQueue);
    else
        event->flags.clear(Event::IsMainQueue);

    if (DTRACE(Event))
        event->trace("scheduled");
}

inline void
EventQueue::deschedule(Event *event)
{
    assert(event->scheduled());
    assert(event->initialized());

    remove(event);

    event->flags.clear(Event::Squashed);
    event->flags.clear(Event::Scheduled);

    if (DTRACE(Event))
        event->trace("descheduled");

    if (event->flags.isSet(Event::AutoDelete))
        delete event;
}

inline void
EventQueue::reschedule(Event *event, Tick when, bool always)
{
    // Typecasting Tick->Utick here since gcc
    // complains about signed overflow
    assert((UTick)when >= (UTick)curTick());
    assert(always || event->scheduled());
    assert(event->initialized());

    if (event->scheduled())
        remove(event);
            
    event->setWhen(when, this);
    insert(event);
    event->flags.clear(Event::Squashed);
    event->flags.set(Event::Scheduled);
    if (this == &mainEventQueue)
        event->flags.set(Event::IsMainQueue);
    else
        event->flags.clear(Event::IsMainQueue);

    if (DTRACE(Event))
        event->trace("rescheduled");
}

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
            : Event(Default_Pri, AutoDelete), object(o)
        { }
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

    EventWrapper(T &obj, bool del = false, Priority p = Default_Pri)
        : Event(p), object(&obj)
    {
        if (del)
            setFlags(AutoDelete);
    }

    void process() { (object->*F)(); }

    const std::string
    name() const
    {
        return object->name() + ".wrapped_event";
    }

    const char *description() const { return "EventWrapped"; }
};
#endif

#endif // __SIM_EVENTQ_HH__
