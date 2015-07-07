/*
 * Copyright (c) 2012, 2015 ARM Limited
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

#include <atomic>
#include <mutex>
#include <unordered_set>

#include "base/flags.hh"

class Drainable;

#ifndef SWIG // SWIG doesn't support strongly typed enums
/**
 * Object drain/handover states
 *
 * An object starts out in the Running state. When the simulator
 * prepares to take a snapshot or prepares a CPU for handover, it
 * calls the drain() method to transfer the object into the Draining
 * or Drained state. If any object enters the Draining state
 * (Drainable::drain() returning >0), simulation continues until it
 * all objects have entered the Drained state.
 *
 * Before resuming simulation, the simulator calls resume() to
 * transfer the object to the Running state.
 *
 * \note Even though the state of an object (visible to the rest of
 * the world through Drainable::getState()) could be used to determine
 * if all objects have entered the Drained state, the protocol is
 * actually a bit more elaborate. See Drainable::drain() for details.
 */
enum class DrainState {
    Running,  /** Running normally */
    Draining, /** Draining buffers pending serialization/handover */
    Drained   /** Buffers drained, ready for serialization/handover */
};
#endif

/**
 * This class coordinates draining of a System.
 *
 * When draining the simulator, we need to make sure that all
 * Drainable objects within the system have ended up in the drained
 * state before declaring the operation to be successful. This class
 * keeps track of how many objects are still in the process of
 * draining. Once it determines that all objects have drained their
 * state, it exits the simulation loop.
 *
 * @note A System might not be completely drained even though the
 * DrainManager has caused the simulation loop to exit. Draining needs
 * to be restarted until all Drainable objects declare that they don't
 * need further simulation to be completely drained. See Drainable for
 * more information.
 */
class DrainManager
{
  private:
    DrainManager();
#ifndef SWIG
    DrainManager(DrainManager &) = delete;
#endif
    ~DrainManager();

  public:
    /** Get the singleton DrainManager instance */
    static DrainManager &instance() { return _instance; }

    /**
     * Try to drain the system.
     *
     * Try to drain the system and return true if all objects are in a
     * the Drained state at which point the whole simulator is in a
     * consistent state and ready for checkpointing or CPU
     * handover. The simulation script must continue simulating until
     * the simulation loop returns "Finished drain", at which point
     * this method should be called again. This cycle should continue
     * until this method returns true.
     *
     * @return true if all objects were drained successfully, false if
     * more simulation is needed.
     */
    bool tryDrain();

    /**
     * Resume normal simulation in a Drained system.
     */
    void resume();

    /**
     * Run state fixups before a checkpoint restore operation
     *
     * The drain state of an object isn't stored in a checkpoint since
     * the whole system is always going to be in the Drained state
     * when the checkpoint is created. When the checkpoint is restored
     * at a later stage, recreated objects will be in the Running
     * state since the state isn't stored in checkpoints. This method
     * performs state fixups on all Drainable objects and the
     * DrainManager itself.
     */
    void preCheckpointRestore();

    /** Check if the system is drained */
    bool isDrained() { return _state == DrainState::Drained; }

    /** Get the simulators global drain state */
    DrainState state() { return _state; }

    /**
     * Notify the DrainManager that a Drainable object has finished
     * draining.
     */
    void signalDrainDone();

  public:
    void registerDrainable(Drainable *obj);
    void unregisterDrainable(Drainable *obj);

  private:
    /**
     * Thread-safe helper function to get the number of Drainable
     * objects in a system.
     */
    size_t drainableCount() const;

    /** Lock protecting the set of drainable objects */
    mutable std::mutex globalLock;

    /** Set of all drainable objects */
    std::unordered_set<Drainable *> _allDrainable;

    /**
     * Number of objects still draining. This is flagged atomic since
     * it can be manipulated by SimObjects living in different
     * threads.
     */
    std::atomic_uint _count;

    /** Global simulator drain state */
    DrainState _state;

    /** Singleton instance of the drain manager */
    static DrainManager _instance;
};

/**
 * Interface for objects that might require draining before
 * checkpointing.
 *
 * An object's internal state needs to be drained when creating a
 * checkpoint, switching between CPU models, or switching between
 * timing models. Once the internal state has been drained from
 * <i>all</i> objects in the simulator, the objects are serialized to
 * disc or the configuration change takes place. The process works as
 * follows (see simulate.py for details):
 *
 * <ol>
 * <li>Call Drainable::drain() for every object in the
 *     system. Draining has completed if all of them return
 *     zero. Otherwise, the sum of the return values is loaded into
 *     the counter of the DrainManager. A pointer to the drain
 *     manager is passed as an argument to the drain() method.
 *
 * <li>Continue simulation. When an object has finished draining its
 *     internal state, it calls DrainManager::signalDrainDone() on the
 *     manager. When the counter in the manager reaches zero, the
 *     simulation stops.
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
    friend class DrainManager;

  public:
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

    DrainState getDrainState() const { return _drainState; }

  protected:
    void setDrainState(DrainState new_state) { _drainState = new_state; }

  private:
    DrainManager &_drainManager;
    DrainState _drainState;
};

#endif
