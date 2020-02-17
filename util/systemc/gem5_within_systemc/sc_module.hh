/*
 * Copyright (c) 2014 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

/**
 * @file
 *
 * Defines an sc_module type to wrap a gem5 simulation.  The 'evaluate'
 * thread on that module implements the gem5 event loop.
 *
 * This currently only supports a single event queue and strictly
 * cooperatively threaded SystemC threads and so there should be at
 * most one Gem5Module instantiated in any simulation.
 */

#ifndef __SIM_SC_MODULE_HH__
#define __SIM_SC_MODULE_HH__

#include <systemc>

#include "sim/eventq.hh"
#include "sim/sim_events.hh"

namespace Gem5SystemC
{

/** A SystemC module implementing the gem5 event queue.  This object
 *  doesn't actually own any of the simulation SimObjects (those need
 *  to be administered separately) but it does control the event
 *  queue.
 *
 *  The event loop here services gem5 events in order at the current time
 *  and then yielding to another SystemC thread.  gem5 events are not
 *  individually scheduled in SystemC.  For this reason, asynchronous events
 *  and function interaction (for example TLM) with gem5 from SystemC must
 *  notify the module so that the yielding 'wait' can be interrupted.
 *  From the point of view of another SystemC module calling into gem5,
 *  curTick can lag SystemC time, be exactly the same time but *never*
 *  lead SystemC time.
 *
 *  This functionality is wrapped in an sc_module as its intended that
 *  the a class representing top level simulation control should be derived
 *  from this class. */
class Module : public sc_core::sc_channel
{
  protected:
    /** Event to trigger (via. ::notify) for event scheduling from
     *  outside gem5 */
    sc_core::sc_event externalSchedulingEvent;

    /** Event to trigger on exit of eventLoop */
    sc_core::sc_event eventLoopExitEvent;

    /** Event to trigger to enter eventLoop */
    sc_core::sc_event eventLoopEnterEvent;

    /** Expected exit time of last eventLoop sleep */
    Tick wait_exit_time;

    /** Are we in Module::simulate?  Used to mask events when not inside
     *  the simulate loop */
    bool in_simulate;

    /** Placeholder base class for a variant event queue if this becomes
     *  useful */
    class SCEventQueue : public EventQueue
    {
      protected:
        Module &module;

      public:
        SCEventQueue(const std::string &name,
            Module &module_) : EventQueue(name), module(module_)
        { }

        /** Signal module to wakeup */
        void wakeup(Tick when);
    };

    /** Service any async event marked up in the globals event_... */
    void serviceAsyncEvent();

  public:
    /** Simulate is a process */
    SC_HAS_PROCESS(Module);

    Module(sc_core::sc_module_name name);

    /** Last exitEvent from eventLoop */
    Event *exitEvent;

    /** Setup global event queues.  Call this before any other event queues
     *  are created */
    static void setupEventQueues(Module &module);

    /** Catch gem5 time up with SystemC */
    void catchup();

    /** Notify an externalSchedulingEvent at the given time from the
     *  current SystemC time */
    void notify(sc_core::sc_time time_from_now = sc_core::SC_ZERO_TIME);

    /** Process an event triggered by externalSchedulingEvent and also
     *  call eventLoop (to try and mop up any events at this time) if there
     *  are any scheduled events */
    void serviceExternalEvent();

    /** Process gem5 events up until an exit event or there are no events
     *  left. */
    void eventLoop();

    /** Run eventLoop up to num_cycles and return the final event */
    GlobalSimLoopExitEvent *simulate(Tick num_cycles = MaxTick);
};

/** There are assumptions throughout Gem5SystemC file that a tick is 1ps.
 *  Make this the case */
void setTickFrequency();

}

#endif // __SIM_SC_MODULE_HH__
