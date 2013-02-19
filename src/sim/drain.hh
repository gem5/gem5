/*
 * Copyright (c) 2012 ARM Limited
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
 * Authors: Andreas Sandberg
 */

#ifndef __SIM_DRAIN_HH__
#define __SIM_DRAIN_HH__

#include <cassert>
#include <vector>

#include "base/flags.hh"

class Event;

/**
 * This class coordinates draining of a System.
 *
 * When draining a System, we need to make sure that all SimObjects in
 * that system have drained their state before declaring the operation
 * to be successful. This class keeps track of how many objects are
 * still in the process of draining their state. Once it determines
 * that all objects have drained their state, it exits the simulation
 * loop.
 *
 * @note A System might not be completely drained even though the
 * DrainManager has caused the simulation loop to exit. Draining needs
 * to be restarted until all Drainable objects declare that they don't
 * need further simulation to be completely drained. See Drainable for
 * more information.
 */
class DrainManager
{
  public:
    DrainManager();
    virtual ~DrainManager();

    /**
     * Get the number of objects registered with this DrainManager
     * that are currently draining their state.
     *
     * @return Number of objects currently draining.
     */
    unsigned int getCount() const { return _count; }

    void setCount(int count) { _count = count; }

    /**
     * Notify the DrainManager that a Drainable object has finished
     * draining.
     */
    void signalDrainDone() {
        assert(_count > 0);
        if (--_count == 0)
            drainCycleDone();
    }

  protected:
    /**
     * Callback when all registered Drainable objects have completed a
     * drain cycle.
     */
    virtual void drainCycleDone();

    /** Number of objects still draining. */
    unsigned int _count;
};

/**
 * Interface for objects that might require draining before
 * checkpointing.
 *
 * An object's internal state needs to be drained when creating a
 * checkpoint, switching between CPU models, or switching between
 * timing models. Once the internal state has been drained from
 * <i>all</i> objects in the system, the objects are serialized to
 * disc or the configuration change takes place. The process works as
 * follows (see simulate.py for details):
 *
 * <ol>
 * <li>An instance of a DrainManager is created to keep track of how
 *     many objects need to be drained. The object maintains an
 *     internal counter that is decreased every time its
 *     CountedDrainEvent::signalDrainDone() method is called. When the
 *     counter reaches zero, the simulation is stopped.
 *
 * <li>Call Drainable::drain() for every object in the
 *     system. Draining has completed if all of them return
 *     zero. Otherwise, the sum of the return values is loaded into
 *     the counter of the DrainManager. A pointer to the drain
 *     manager is passed as an argument to the drain() method.
 *
 * <li>Continue simulation. When an object has finished draining its
 *     internal state, it calls CountedDrainEvent::signalDrainDone()
 *     on the manager. When the counter in the manager reaches zero,
 *     the simulation stops.
 *
 * <li>Check if any object still needs draining, if so repeat the
 *     process above.
 *
 * <li>Serialize objects, switch CPU model, or change timing model.
 *
 * <li>Call Drainable::drainResume() and continue the simulation.
 * </ol>
 *
 */
class Drainable
{
  public:
    /**
     * Object drain/handover states
     *
     * An object starts out in the Running state. When the simulator
     * prepares to take a snapshot or prepares a CPU for handover, it
     * calls the drain() method to transfer the object into the
     * Draining or Drained state. If any object enters the Draining
     * state (drain() returning >0), simulation continues until it all
     * objects have entered the Drained state.
     *
     * Before resuming simulation, the simulator calls resume() to
     * transfer the object to the Running state.
     *
     * \note Even though the state of an object (visible to the rest
     * of the world through getState()) could be used to determine if
     * all objects have entered the Drained state, the protocol is
     * actually a bit more elaborate. See drain() for details.
     */
    enum State {
        Running,  /** Running normally */
        Draining, /** Draining buffers pending serialization/handover */
        Drained   /** Buffers drained, ready for serialization/handover */
    };

    Drainable();
    virtual ~Drainable();

    /**
     * Determine if an object needs draining and register a
     * DrainManager.
     *
     * When draining the state of an object, the simulator calls drain
     * with a pointer to a drain manager. If the object does not need
     * further simulation to drain internal buffers, it switched to
     * the Drained state and returns 0, otherwise it switches to the
     * Draining state and returns the number of times that it will
     * call Event::process() on the drain event. Most objects are
     * expected to return either 0 or 1.
     *
     * @note An object that has entered the Drained state can be
     * disturbed by other objects in the system and consequently be
     * forced to enter the Draining state again. The simulator
     * therefore repeats the draining process until all objects return
     * 0 on the first call to drain().
     *
     * @param drainManager DrainManager to use to inform the simulator
     * when draining has completed.
     *
     * @return 0 if the object is ready for serialization now, >0 if
     * it needs further simulation.
     */
    virtual unsigned int drain(DrainManager *drainManager) = 0;

    /**
     * Resume execution after a successful drain.
     *
     * @note This method is normally only called from the simulation
     * scripts.
     */
    virtual void drainResume();

    /**
     * Write back dirty buffers to memory using functional writes.
     *
     * After returning, an object implementing this method should have
     * written all its dirty data back to memory. This method is
     * typically used to prepare a system with caches for
     * checkpointing.
     */
    virtual void memWriteback() {};

    /**
     * Invalidate the contents of memory buffers.
     *
     * When the switching to hardware virtualized CPU models, we need
     * to make sure that we don't have any cached state in the system
     * that might become stale when we return. This method is used to
     * flush all such state back to main memory.
     *
     * @warn This does <i>not</i> cause any dirty state to be written
     * back to memory.
     */
    virtual void memInvalidate() {};

    State getDrainState() const { return _drainState; }

  protected:
    void setDrainState(State new_state) { _drainState = new_state; }


  private:
    State _drainState;

};

DrainManager *createDrainManager();
void cleanupDrainManager(DrainManager *drain_manager);

#endif
