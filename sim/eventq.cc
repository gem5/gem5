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

#include <assert.h>

#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "cpu/full_cpu/smt.hh"
#include "base/misc.hh"

#include "sim/eventq.hh"
#include "base/trace.hh"
#include "sim/universe.hh"

using namespace std;

//
// Main Event Queue
//
// Events on this queue are processed at the *beginning* of each
// cycle, before the pipeline simulation is performed.
//
EventQueue mainEventQueue("MainEventQueue");

void
EventQueue::insert(Event *event)
{
    if (head == NULL || event->when() < head->when() ||
        (event->when() == head->when() &&
         event->priority() <= head->priority())) {
        event->next = head;
        head = event;
    } else {
        Event *prev = head;
        Event *curr = head->next;

        while (curr) {
            if (event->when() <= curr->when() &&
                (event->when() < curr->when() ||
                 event->priority() <= curr->priority()))
                break;

            prev = curr;
            curr = curr->next;
        }

        event->next = curr;
        prev->next = event;
    }
}

void
EventQueue::remove(Event *event)
{
    if (head == NULL)
        return;

    if (head == event){
        head = event->next;
        return;
    }

    Event *prev = head;
    Event *curr = head->next;
    while (curr && curr != event) {
        prev = curr;
        curr = curr->next;
    }

    if (curr == event)
        prev->next = curr->next;
}

void
EventQueue::serviceOne()
{
    Event *event = head;
    event->clearFlags(Event::Scheduled);
    head = event->next;

    // handle action
    if (!event->squashed())
        event->process();
    else
        event->clearFlags(Event::Squashed);

    if (event->getFlags(Event::AutoDelete) && !event->scheduled())
        delete event;
}


void
Event::serialize(std::ostream &os)
{
    SERIALIZE_SCALAR(_when);
    SERIALIZE_SCALAR(_priority);
    SERIALIZE_ENUM(_flags);
}


void
Event::unserialize(Checkpoint *cp, const string &section)
{
    if (scheduled())
        deschedule();

    UNSERIALIZE_SCALAR(_when);
    UNSERIALIZE_SCALAR(_priority);

    // need to see if original event was in a scheduled, unsquashed
    // state, but don't want to restore those flags in the current
    // object itself (since they aren't immediately true)
    UNSERIALIZE_ENUM(_flags);
    bool wasScheduled = (_flags & Scheduled) && !(_flags & Squashed);
    _flags &= ~(Squashed | Scheduled);

    if (wasScheduled) {
        DPRINTF(Config, "rescheduling at %d\n", _when);
        schedule(_when);
    }
}

void
EventQueue::serialize(ostream &os)
{
    std::list<Event *> eventPtrs;

    int numEvents = 0;
    Event *event = head;
    while (event) {
        if (event->getFlags(Event::AutoSerialize)) {
            eventPtrs.push_back(event);
            paramOut(os, csprintf("event%d", numEvents++), event->name());
        }
        event = event->next;
    }

    SERIALIZE_SCALAR(numEvents);

    for (std::list<Event *>::iterator it=eventPtrs.begin();
         it != eventPtrs.end(); ++it) {
        (*it)->nameOut(os);
        (*it)->serialize(os);
    }
}

void
EventQueue::unserialize(Checkpoint *cp, const std::string &section)
{
    int numEvents;
    UNSERIALIZE_SCALAR(numEvents);

    std::string eventName;
    for (int i = 0; i < numEvents; i++) {
        // get the pointer value associated with the event
        paramIn(cp, section, csprintf("event%d", i), eventName);

        // create the event based on its pointer value
        Serializeable::create(cp, eventName);
    }
}

void
EventQueue::dump()
{
    cprintf("============================================================\n");
    cprintf("EventQueue Dump  (cycle %d)\n", curTick);
    cprintf("------------------------------------------------------------\n");

    if (empty())
        cprintf("<No Events>\n");
    else {
        Event *event = head;
        while (event) {
            event->dump();
            event = event->next;
        }
    }

    cprintf("============================================================\n");
}


const char *
Event::description()
{
    return "generic";
}

#if TRACING_ON
void
Event::trace(const char *action)
{
    // This DPRINTF is unconditional because calls to this function
    // are protected by an 'if (DTRACE(Event))' in the inlined Event
    // methods.
    //
    // This is just a default implementation for derived classes where
    // it's not worth doing anything special.  If you want to put a
    // more informative message in the trace, override this method on
    // the particular subclass where you have the information that
    // needs to be printed.
    DPRINTFN("%s event %s @ %d\n", description(), action, when());
}
#endif

void
Event::dump()
{
#if TRACING_ON
    cprintf("   Created: %d\n", when_created);
#endif
    if (scheduled()) {
#if TRACING_ON
        cprintf("   Scheduled at  %d\n", when_scheduled);
#endif
        cprintf("   Scheduled for %d\n", when());
    }
    else {
        cprintf("   Not Scheduled\n");
    }
}
