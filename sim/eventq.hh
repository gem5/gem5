/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
 */

/* @file
 * EventQueue interfaces
 */

#ifndef __EVENTQ_HH__
#define __EVENTQ_HH__

#include <assert.h>

#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "sim/host.hh"	// for Tick

#include "base/fast_alloc.hh"
#include "sim/serialize.hh"
#include "base/trace.hh"

class EventQueue;	// forward declaration

/*
 * An item on an event queue.  The action caused by a given
 * event is specified by deriving a subclass and overriding the
 * process() member function.
 */
class Event : public Serializeable, public FastAlloc
{
    friend class EventQueue;

  private:
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
        AutoSerialize = 0x8
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

    static const std::string defaultName;

    /*
     * Event constructor
     * @param queue that the event gets scheduled on
     */
    Event(EventQueue *q, int p = 0)
        : queue(q), next(NULL), _priority(p), _flags(None),
#if TRACING_ON
          when_created(curTick), when_scheduled(0),
#endif
          annotated_value(0)
    {
    }

    ~Event() {}

    virtual std::string name() const {
        return csprintf("%s_%x", defaultName, (uintptr_t)this);
    }

    /// Determine if the current event is scheduled
    bool scheduled() const { return getFlags(Scheduled); }

    /// Schedule the event with the current priority or default priority
    void schedule(Tick t);

    /// Schedule the event with a specific priority
    void schedule(Tick t, int priority);

    /// Reschedule the event with the current priority
    void reschedule(Tick t);

    /// Reschedule the event with a specific priority
    void reschedule(Tick t, int priority);

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
            { setFlags(AutoDestroy); schedule(when); }
        void process() { (object->*F)(); }
        const char *description() { return "delay"; }
    };

    new DelayEvent(when, object);
}

/*
 * Queue of events sorted in time order
 */
class EventQueue : public Serializeable
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

    virtual std::string name() const { return objName; }

    // schedule the given event on this queue
    void schedule(Event *ev);
    void deschedule(Event *ev);
    void reschedule(Event *ev);

    Tick nextTick() { return head->when(); }
    void serviceOne();

    // process all events up to the given timestamp.  we inline a
    // quick test to see if there are any events to process; if so,
    // call the internal out-of-line version to process them all.
    void serviceEvents(Tick when) {
        while (!empty()) {
            if (nextTick() > when)
                break;

            assert(head->when() >= when && "event scheduled in the past");
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
    setFlags(Scheduled);
#if TRACING_ON
    when_scheduled = curTick;
#endif
    _when = t;
    queue->schedule(this);
}

inline void
Event::schedule(Tick t, int p)
{
    _priority = p;
    schedule(t);
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
Event::reschedule(Tick t, int p)
{
    _priority = p;
    reschedule(t);
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

#endif // __EVENTQ_HH__
