/*
 * Copyright (c) 2001-2005 The Regents of The University of Michigan
 * Copyright (c) 2010 Advanced Micro Devices, Inc.
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
 * User Console Definitions
 */

#ifndef __SIM_OBJECT_HH__
#define __SIM_OBJECT_HH__

#include <iostream>
#include <list>
#include <map>
#include <string>
#include <vector>

#include "enums/MemoryMode.hh"
#include "params/SimObject.hh"
#include "sim/eventq.hh"
#include "sim/serialize.hh"

class BaseCPU;
class Event;

/**
 * Abstract superclass for simulation objects.  Represents things that
 * correspond to physical components and can be specified via the
 * config file (CPUs, caches, etc.).
 *
 * SimObject initialization is controlled by the instantiate method in
 * src/python/m5/simulate.py. There are slightly different
 * initialization paths when starting the simulation afresh and when
 * loading from a checkpoint.  After instantiation and connecting
 * ports, simulate.py initializes the object using the following call
 * sequence:
 *
 * <ol>
 * <li>SimObject::init()
 * <li>SimObject::regStats()
 * <li><ul>
 *     <li>SimObject::initState() if starting afresh.
 *     <li>SimObject::loadState() if restoring from a checkpoint.
 *     </ul>
 * <li>SimObject::resetStats()
 * <li>SimObject::startup()
 * <li>SimObject::resume() if resuming from a checkpoint.
 * </ol>
 *
 * An object's internal state needs to be drained when creating a
 * checkpoint, switching between CPU models, or switching between
 * timing models. Once the internal state has been drained from
 * <i>all</i> objects in the system, the objects are serialized to
 * disc or the configuration change takes place. The process works as
 * follows (see simulate.py for details):
 *
 * <ol>
 * <li>An instance of a CountedDrainEvent is created to keep track of
 *     how many objects need to be drained. The object maintains an
 *     internal counter that is decreased every time its
 *     CountedDrainEvent::process() method is called. When the counter
 *     reaches zero, the simulation is stopped.
 *
 * <li>Call SimObject::drain() for every object in the
 *     system. Draining has completed if all of them return
 *     zero. Otherwise, the sum of the return values is loaded into
 *     the counter of the CountedDrainEvent. A pointer of the drain
 *     event is passed as an argument to the drain() method.
 *
 * <li>Continue simulation. When an object has finished draining its
 *     internal state, it calls CountedDrainEvent::process() on the
 *     CountedDrainEvent. When counter in the CountedDrainEvent reaches
 *     zero, the simulation stops.
 *
 * <li>Check if any object still needs draining, if so repeat the
 *     process above.
 *
 * <li>Serialize objects, switch CPU model, or change timing model.
 *
 * <li>Call SimObject::resume() and continue the simulation.
 * </ol>
 *
 * @note Whenever a method is called on all objects in the simulator's
 * object tree (e.g., init(), startup(), or loadState()), a pre-order
 * depth-first traversal is performed (see descendants() in
 * SimObject.py). This has the effect of calling the method on the
 * parent node <i>before</i> its children.
 */
class SimObject : public EventManager, public Serializable
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
     * objects have entered the Drained.
     *
     * The before resuming simulation, the simulator calls resume() to
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

  private:
    State state;

  protected:
    void changeState(State new_state) { state = new_state; }

  public:
    State getState() { return state; }

  private:
    typedef std::vector<SimObject *> SimObjectList;

    /** List of all instantiated simulation objects. */
    static SimObjectList simObjectList;

  protected:
    /** Cached copy of the object parameters. */
    const SimObjectParams *_params;

  public:
    typedef SimObjectParams Params;
    const Params *params() const { return _params; }
    SimObject(const Params *_params);
    virtual ~SimObject() {}

  public:

    virtual const std::string name() const { return params()->name; }

    /**
     * init() is called after all C++ SimObjects have been created and
     * all ports are connected.  Initializations that are independent
     * of unserialization but rely on a fully instantiated and
     * connected SimObject graph should be done here.
     */
    virtual void init();

    /**
     * loadState() is called on each SimObject when restoring from a
     * checkpoint.  The default implementation simply calls
     * unserialize() if there is a corresponding section in the
     * checkpoint.  However, objects can override loadState() to get
     * other behaviors, e.g., doing other programmed initializations
     * after unserialize(), or complaining if no checkpoint section is
     * found.
     *
     * @param cp Checkpoint to restore the state from.
     */
    virtual void loadState(Checkpoint *cp);

    /**
     * initState() is called on each SimObject when *not* restoring
     * from a checkpoint.  This provides a hook for state
     * initializations that are only required for a "cold start".
     */
    virtual void initState();

    /**
     * Register statistics for this object.
     */
    virtual void regStats();

    /**
     * Reset statistics associated with this object.
     */
    virtual void resetStats();

    /**
     * startup() is the final initialization call before simulation.
     * All state is initialized (including unserialized state, if any,
     * such as the curTick() value), so this is the appropriate place to
     * schedule initial event(s) for objects that need them.
     */
    virtual void startup();

    /**
     * Serialize all SimObjects in the system.
     */
    static void serializeAll(std::ostream &os);

    /**
     * Determine if an object needs draining and register a drain
     * event.
     *
     * When draining the state of an object, the simulator calls drain
     * with a pointer to a drain event. If the object does not need
     * further simulation to drain internal buffers, it switched to
     * the Drained state and returns 0, otherwise it switches to the
     * Draining state and returns the number of times that it will
     * call Event::process() on the drain event. Most objects are
     * expected to return either 0 or 1.
     *
     * The default implementation simply switches to the Drained state
     * and returns 0.
     *
     * @note An object that has entered the Drained state can be
     * disturbed by other objects in the system and consequently be
     * forced to enter the Draining state again. The simulator
     * therefore repeats the draining process until all objects return
     * 0 on the first call to drain().
     *
     * @param drain_event Event to use to inform the simulator when
     * the draining has completed.
     *
     * @return 0 if the object is ready for serialization now, >0 if
     * it needs further simulation.
     */
    virtual unsigned int drain(Event *drain_event);

    /**
     * Switch an object in the Drained stated into the Running state.
     */
    virtual void resume();

    /**
     * Change the memory mode the simulator operates in.
     *
     * @note Should only be implemented in the System object.
     */
    virtual void setMemoryMode(Enums::MemoryMode new_mode);

    /**
     * Prepare a CPU model to be switched out, invoked on active CPUs
     * that are about to be replaced.
     *
     * @note This should only be implemented in CPU models.
     */
    virtual void switchOut();

    /**
     * Load the state of a CPU from the previous CPU object, invoked
     * on all new CPUs that are about to be switched in.
     *
     * A CPU model implementing this method is expected to initialize
     * its state from the old CPU and connect its memory (unless they
     * are already connected) to the memories connected to the old
     * CPU.
     *
     * @note This should only be implemented in CPU models.
     *
     * @param cpu CPU to initialize read state from.
     */
    virtual void takeOverFrom(BaseCPU *cpu);

#ifdef DEBUG
  public:
    bool doDebugBreak;
    static void debugObjectBreak(const std::string &objs);
#endif

    /**
     * Find the SimObject with the given name and return a pointer to
     * it.  Primarily used for interactive debugging.  Argument is
     * char* rather than std::string to make it callable from gdb.
     */
    static SimObject *find(const char *name);
};

#endif // __SIM_OBJECT_HH__
