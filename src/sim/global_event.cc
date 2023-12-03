/*
 * Copyright (c) 2011-2013 Advanced Micro Devices, Inc.
 * Copyright (c) 2013 Mark D. Hill and David A. Wood
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

#include "sim/global_event.hh"

#include "sim/cur_tick.hh"

namespace gem5
{

std::mutex BaseGlobalEvent::globalQMutex;

BaseGlobalEvent::BaseGlobalEvent(Priority p, Flags f)
    : barrier(numMainEventQueues), barrierEvent(numMainEventQueues, NULL)
{}

BaseGlobalEvent::~BaseGlobalEvent()
{
    // see GlobalEvent::BarrierEvent::~BarrierEvent() comments
    if (barrierEvent[0] != NULL) {
        for (int i = 0; i < numMainEventQueues; ++i)
            delete barrierEvent[i];
    }
}

void
BaseGlobalEvent::schedule(Tick when)
{
    // This function is scheduling a global event, which actually is a
    // set of local events, one event on each eventq. Global events need
    // to have a total order. A thread cannot start executing events that
    // follow a global event till all other threads have executed that global
    // event as well. If global events were not in a total order, a deadlock
    // would occur for there will be two threads who would be waiting for
    // each other to execute the global events they themselves have executed.
    //
    // To ensure this total order, we do two things.
    // First, before scheduling any global event, a thread needs to acquire
    // the lock globalQMutex. This ensures that only one thread can schedule
    // global events at any given time.
    // Second, the local events corresponding to a global event are always
    // first inserted in to the asyncq, irrespective of whether or not the
    // thread scheduling the event owns the eventq on which the event is
    // being scheduled. Thus global events have the same order in the asyncq
    // of each thread. When they are inserted in the actual eventq, the
    // comparators in the Event class ensure that the total order is
    // maintained.

    globalQMutex.lock();

    for (int i = 0; i < numMainEventQueues; ++i) {
        mainEventQueue[i]->schedule(barrierEvent[i], when, true);
    }

    globalQMutex.unlock();
}

void
BaseGlobalEvent::deschedule()
{
    EventQueue *q = curEventQueue();
    for (uint32_t i = 0; i < numMainEventQueues; ++i) {
        if (barrierEvent[i]->scheduled()) {
            curEventQueue(mainEventQueue[i]);
            mainEventQueue[i]->deschedule(barrierEvent[i]);
        }
    }

    curEventQueue(q);
}

void
BaseGlobalEvent::reschedule(Tick when)
{
    // Read the comment in the schedule() function above.
    globalQMutex.lock();

    for (uint32_t i = 0; i < numMainEventQueues; ++i) {
        if (barrierEvent[i]->scheduled())
            mainEventQueue[i]->reschedule(barrierEvent[i], when);
        else
            mainEventQueue[i]->schedule(barrierEvent[i], when, true);
    }

    globalQMutex.unlock();
}

BaseGlobalEvent::BarrierEvent::~BarrierEvent()
{
    // if AutoDelete is set, local events will get deleted in event
    // loop, but we need to delete GlobalEvent object too... so let
    // the local event in slot 0 do it
    if (isFlagSet(AutoDelete) && _globalEvent->barrierEvent[0] == this) {
        // set backpointer to NULL so that global event knows not to
        // turn around and recursively delete local events
        _globalEvent->barrierEvent[0] = NULL;
        delete _globalEvent;
    }
}

void
GlobalEvent::BarrierEvent::process()
{
    // wait for all queues to arrive at barrier, then process event
    if (globalBarrier()) {
        _globalEvent->process();
    }

    // second barrier to force all queues to wait for event processing
    // to finish before continuing
    globalBarrier();
}

void
GlobalSyncEvent::BarrierEvent::process()
{
    // wait for all queues to arrive at barrier, then process event
    if (globalBarrier()) {
        _globalEvent->process();
    }

    // second barrier to force all queues to wait for event processing
    // to finish before continuing
    globalBarrier();
    curEventQueue()->handleAsyncInsertions();
}

void
GlobalSyncEvent::process()
{
    if (repeat) {
        schedule(curTick() + repeat);
    }
}

const char *
GlobalSyncEvent::description() const
{
    return "GlobalSyncEvent";
}

} // namespace gem5
