/*
 * Copyright (c) 2021 Arm Limited
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

#include "sim/simulate.hh"

#include <atomic>
#include <thread>

#include "base/logging.hh"
#include "base/pollevent.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/EnteringEventQueue.hh"
#include "sim/async.hh"
#include "sim/eventq.hh"
#include "sim/init_signals.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/stat_control.hh"

namespace gem5
{

//! forward declaration
Event *doSimLoop(EventQueue *);

GlobalSimLoopExitEvent *simulate_limit_event = nullptr;

class SimulatorThreads
{
  public:
    SimulatorThreads() = delete;
    SimulatorThreads(const SimulatorThreads &) = delete;
    SimulatorThreads &operator=(SimulatorThreads &) = delete;

    SimulatorThreads(uint32_t num_queues)
        : terminate(false),
          numQueues(num_queues),
          barrier(num_queues)
    {
        threads.reserve(num_queues);
    }

    ~SimulatorThreads()
    {
        // This should only happen after exit has been
        // called. Subordinate event queues should normally (assuming
        // exit is called from Python) be waiting on the barrier when
        // this happens.
        //
        // N.B.: Not terminating here would make it impossible to
        // safely destroy the barrier.
        terminateThreads();
    }

    void runUntilLocalExit()
    {
        assert(!terminate);

        // Start subordinate threads if needed.
        if (threads.empty()) {
            // the main thread (the one running Python) handles queue 0,
            // so we only need to allocate new threads for queues 1..N-1.
            // We'll call these the "subordinate" threads.
            for (uint32_t i = 1; i < numQueues; i++) {
                threads.emplace_back(
                    [this](EventQueue *eq) {
                        thread_main(eq);
                    }, mainEventQueue[i]);
            }
        }

        // This method is called from the main thread. All subordinate
        // threads should be waiting on the barrier when the function
        // is called. The arrival of the main thread here will satisfy
        // the barrier and start another iteration in the thread loop.
        barrier.wait();
    }

    void
    terminateThreads()
    {
        assert(!terminate);
        if (threads.empty())
            return;

        /* This function should only be called when the simulator is
         * handling a global exit event (typically from Python). This
         * means that the helper threads will be waiting on the
         * barrier. Tell the helper threads to exit and release them from
         * their barrier. */
        terminate = true;
        barrier.wait();

        /* Wait for all of the threads to terminate */
        for (auto &t : threads) {
            t.join();
        }

        terminate = false;
        threads.clear();
    }

  protected:
    /**
     * The main function for all subordinate threads (i.e., all threads
     * other than the main thread).  These threads start by waiting on
     * threadBarrier.  Once all threads have arrived at threadBarrier,
     * they enter the simulation loop concurrently.  When they exit the
     * loop, they return to waiting on threadBarrier.  This process is
     * repeated until the simulation terminates.
     */
    void
    thread_main(EventQueue *queue)
    {
        /* Wait for all initialisation to complete */
        barrier.wait();

        while (!terminate) {
            doSimLoop(queue);
            barrier.wait();
        }
    }

    std::atomic<bool> terminate;
    uint32_t numQueues;
    std::vector<std::thread> threads;
    Barrier barrier;
};

static std::unique_ptr<SimulatorThreads> simulatorThreads;

struct DescheduleDeleter
{
    void operator()(BaseGlobalEvent *event)
    {
        if (!event)
            return;

        event->deschedule();
        delete event;
    }
};

/** Simulate for num_cycles additional cycles.  If num_cycles is -1
 * (the default), we simulate to MAX_TICKS unless the max ticks has been set
 * via the 'set_max_tick' function prior. This function is exported to Python.
 * @return The SimLoopExitEvent that caused the loop to exit.
 */
GlobalSimLoopExitEvent *global_exit_event= nullptr;
GlobalSimLoopExitEvent *
simulate(Tick num_cycles)
{
    // install the sigint handler to catch ctrl-c and exit the sim loop cleanly
    // Note: This should be done before initializing the threads
    initSigInt();
    initSigCont();

    if (global_exit_event)//cleaning last global exit event
        global_exit_event->clean();
    std::unique_ptr<GlobalSyncEvent, DescheduleDeleter> quantum_event;

    DPRINTF(EnteringEventQueue, "Entering event queue @ %d. Starting "
        "simulation...\n", curTick());

    if (!simulatorThreads)
        simulatorThreads.reset(new SimulatorThreads(numMainEventQueues));

    if (!simulate_limit_event) {
        // If the simulate_limit_event is not set, we set it to MaxTick.
        set_max_tick(MaxTick);
    }

    if (num_cycles != -1) {
        // If the user has specified an exit event after X cycles, do so here.
        // Note: This will override any prior set max_tick behaviour (such as
        // that above when it is set to MAxTick).
        const Tick max_tick = num_cycles < MaxTick - curTick() ?
                                    curTick() + num_cycles : MaxTick;

        // This is kept to `set_max_tick` instead of `schedule_tick_exit` to
        // preserve backwards functionality. It may be better to deprecate this
        // behaviour at some point in favor of `schedule_tick_exit`.
        set_max_tick(max_tick);
    }

    if (numMainEventQueues > 1) {
        fatal_if(simQuantum == 0,
                 "Quantum for multi-eventq simulation not specified");

        quantum_event.reset(
            new GlobalSyncEvent(curTick() + simQuantum, simQuantum,
                                EventBase::Progress_Event_Pri, 0));

        inParallelMode = true;
    }

    simulatorThreads->runUntilLocalExit();
    Event *local_event = doSimLoop(mainEventQueue[0]);
    assert(local_event);

    // Restore normal ctrl-c operation as soon as the event queue is done
    restoreSigInt();

    inParallelMode = false;

    // locate the global exit event and return it to Python
    BaseGlobalEvent *global_event = local_event->globalEvent();
    assert(global_event);

    global_exit_event =
        dynamic_cast<GlobalSimLoopExitEvent *>(global_event);
    assert(global_exit_event);

    return global_exit_event;
}

void set_max_tick(Tick tick)
{
    if (!simulate_limit_event) {
        simulate_limit_event = new GlobalSimLoopExitEvent(
            mainEventQueue[0]->getCurTick(),
            "simulate() limit reached", 0);
    }
    simulate_limit_event->reschedule(tick);
}


Tick get_max_tick()
{
    if (!simulate_limit_event) {
        /* If the GlobalSimLoopExitEvent has not been setup, the maximum tick
         * is `MaxTick` as declared in "src/base/types.hh".
         */
        return MaxTick;
    }

    return simulate_limit_event->when();
}

void
terminateEventQueueThreads()
{
    simulatorThreads->terminateThreads();
}


/**
 * The main per-thread simulation loop. This loop is executed by all
 * simulation threads (the main thread and the subordinate threads) in
 * parallel.
 */
Event *
doSimLoop(EventQueue *eventq)
{
    // set the per thread current eventq pointer
    curEventQueue(eventq);
    eventq->handleAsyncInsertions();

    bool mainQueue = eventq == getEventQueue(0);

    while (1) {
        // there should always be at least one event (the SimLoopExitEvent
        // we just scheduled) in the queue
        assert(!eventq->empty());
        assert(curTick() <= eventq->nextTick() &&
               "event scheduled in the past");

        if (mainQueue && async_event) {
            async_event = false;
            // Take the event queue lock in case any of the service
            // routines want to schedule new events.
            std::lock_guard<EventQueue> lock(*eventq);
            if (async_statdump || async_statreset) {
                statistics::schedStatEvent(async_statdump, async_statreset);
                async_statdump = false;
                async_statreset = false;
            }

            if (async_io) {
                async_io = false;
                pollQueue.service();
            }

            if (async_exit) {
                async_exit = false;
                exitSimLoop("user interrupt received");
            }

            if (async_exception) {
                async_exception = false;
                return NULL;
            }
            if (async_hypercall) {
                async_hypercall = false;
                processExternalSignal();
            }
        }

        Event *exit_event = eventq->serviceOne();
        if (exit_event != NULL) {
            return exit_event;
        }
    }

    // not reached... only exit is return on SimLoopExitEvent
}

} // namespace gem5
