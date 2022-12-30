/*
 * Copyright (c) 2012, 2015, 2017 ARM Limited
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
 */

#ifndef __SIM_DRAIN_HH__
#define __SIM_DRAIN_HH__

#include <atomic>
#include <mutex>
#include <vector>

namespace gem5
{

class Drainable;

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
 * transfer the object to the Running state. This in turn results in a
 * call to drainResume() for all Drainable objects in the
 * simulator. New Drainable objects may be created while resuming. In
 * such cases, the new objects will be created in the Resuming state
 * and later resumed.
 *
 * \note Even though the state of an object (visible to the rest of
 * the world through Drainable::getState()) could be used to determine
 * if all objects have entered the Drained state, the protocol is
 * actually a bit more elaborate. See Drainable::drain() for details.
 *
 * @ingroup api_drain
 */
enum class DrainState
{
    Running,  /**< Running normally */
    Draining, /**< Draining buffers pending serialization/handover */
    Drained,  /**< Buffers drained, ready for serialization/handover */
    Resuming, /**< Transient state while the simulator is resuming */
};

class DrainManager
{
  private:
    DrainManager();
    DrainManager(DrainManager &) = delete;
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
     *
     * @ingroup api_drain
     */
    bool tryDrain();

    /**
     * Resume normal simulation in a Drained system.
     *
     * @ingroup api_drain
     */
    void resume();

    /**
     * Run state fixups before a checkpoint restore operation.
     *
     * This is called before restoring the checkpoint and to make
     * sure that everything has been set to drained.
     *
     * When restoring from a checkpoint, this function should be called
     * first before calling the resume() function. And also before
     * calling loadstate() on any object.
     *
     * The drain state of an object isn't stored in a checkpoint since
     * the whole system is always going to be in the Drained state
     * when the checkpoint is created. When the checkpoint is restored
     * at a later stage, recreated objects will be in the Running
     * state since the state isn't stored in checkpoints. This method
     * performs state fixups on all Drainable objects and the
     * DrainManager itself.
     *
     * @ingroup api_drain
     */
    void preCheckpointRestore();

    /**
     * Check if the system is drained
     *
     * @ingroup api_drain
     */
    bool isDrained() const { return _state == DrainState::Drained; }

    /**
     * Get the simulators global drain state
     *
     * @ingroup api_drain
     */
    DrainState state() const { return _state; }

    /**
     * Notify the DrainManager that a Drainable object has finished
     * draining.
     *
     * @ingroup api_drain
     */
    void signalDrainDone();

  public:
    void registerDrainable(Drainable *obj);
    void unregisterDrainable(Drainable *obj);

  private:
    /**
     * Helper function to check if all Drainable objects are in a
     * specific state.
     */
    bool allInState(DrainState state) const;

    /**
     * Thread-safe helper function to get the number of Drainable
     * objects in a system.
     */
    size_t drainableCount() const;

    /** Lock protecting the set of drainable objects */
    mutable std::mutex globalLock;

    /** Set of all drainable objects */
    std::vector<Drainable *> _allDrainable;

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
 * <li>DrainManager::tryDrain() calls Drainable::drain() for every
 *     object in the system. Draining has completed if all of them
 *     return true. Otherwise, the drain manager keeps track of the
 *     objects that requested draining and waits for them to signal
 *     that they are done draining using the signalDrainDone() method.
 *
 * <li>Continue simulation. When an object has finished draining its
 *     internal state, it calls DrainManager::signalDrainDone() on the
 *     manager. The drain manager keeps track of the objects that
 *     haven't drained yet, simulation stops when the set of
 *     non-drained objects becomes empty.
 *
 * <li>Check if any object still needs draining
 *     (DrainManager::tryDrain()), if so repeat the process above.
 *
 * <li>Serialize objects, switch CPU model, or change timing model.
 *
 * <li>Call DrainManager::resume(), which in turn calls
 *     Drainable::drainResume() for all objects, and then continue the
 *     simulation.
 * </ol>
 *
 */
class Drainable
{
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
    friend class DrainManager;

  protected:
    Drainable();
    virtual ~Drainable();

    /**
     * Draining is the process of clearing out the states of
     * SimObjects.These are the SimObjects that are partially
     * executed or are partially in flight. Draining is mostly
     * used before forking and creating a check point.
     *
     * This function notifies an object that it needs to drain its state.
     *
     * If the object does not need further simulation to drain
     * internal buffers, it returns DrainState::Drained and
     * automatically switches to the Drained state. If the object
     * needs more simulation, it returns DrainState::Draining and
     * automatically enters the Draining state. Other return values
     * are invalid.
     *
     * @note An object that has entered the Drained state can be
     * disturbed by other objects in the system and consequently stop
     * being drained. These perturbations are not visible in the drain
     * state. The simulator therefore repeats the draining process
     * until all objects return DrainState::Drained on the first call
     * to drain().
     *
     * @return DrainState::Drained if the object is drained at this
     * point in time, DrainState::Draining if it needs further
     * simulation.
     *
     * @ingroup api_drain
     */
    virtual DrainState drain() = 0;

    /**
     * Resume execution after a successful drain.
     *
     * @ingroup api_drain
     */
    virtual void drainResume() {};

    /**
     * Signal that an object is drained
     *
     * This method is designed to be called whenever an object enters
     * into a state where it is ready to be drained. The method is
     * safe to call multiple times and there is no need to check that
     * draining has been requested before calling this method.
     *
     * @ingroup api_drain
     */
    void signalDrainDone() const {
        switch (_drainState) {
          case DrainState::Running:
          case DrainState::Drained:
          case DrainState::Resuming:
            return;
          case DrainState::Draining:
            _drainState = DrainState::Drained;
            _drainManager.signalDrainDone();
            return;
        }
    }

  public:
    /**
     * Return the current drain state of an object.
     *
     * @ingroup api_drain
     */
    DrainState drainState() const { return _drainState; }

    /**
     * Notify a child process of a fork. SimObjects are told that the
     * process is going to be forked.
     *
     * Forking is a process of splitting a process in to two
     * processes, which is then used for multiprocessing.
     *
     * When calling fork in gem5, we need to ensure that resources
     * shared between the parent and the child are consistent. This
     * method is intended to be overloaded to handle that. For
     * example, an object could use this method to re-open input files
     * to get a separate file description with a private file offset.
     *
     * This method is only called in the child of the fork. The call
     * takes place in a drained system.
     *
     * @ingroup api_drain
     */
    virtual void notifyFork() {};

  private:
    /** DrainManager interface to request a drain operation */
    DrainState dmDrain();
    /** DrainManager interface to request a resume operation */
    void dmDrainResume();

    /** Convenience reference to the drain manager */
    DrainManager &_drainManager;

    /**
     * Current drain state of the object. Needs to be mutable since
     * objects need to be able to signal that they have transitioned
     * into a Drained state even if the calling method is const.
     */
    mutable DrainState _drainState;
};

} // namespace gem5

#endif
