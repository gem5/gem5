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
#include "sim/drain.hh"
#include "sim/eventq_impl.hh"
#include "sim/serialize.hh"

class BaseCPU;
class Event;
class ProbeManager;
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
 * <li>Drainable::drainResume() if resuming from a checkpoint.
 * </ol>
 *
 * @note Whenever a method is called on all objects in the simulator's
 * object tree (e.g., init(), startup(), or loadState()), a pre-order
 * depth-first traversal is performed (see descendants() in
 * SimObject.py). This has the effect of calling the method on the
 * parent node <i>before</i> its children.
 */
class SimObject : public EventManager, public Serializable, public Drainable
{
  private:
    typedef std::vector<SimObject *> SimObjectList;

    /** List of all instantiated simulation objects. */
    static SimObjectList simObjectList;

    /** Manager coordinates hooking up probe points with listeners. */
    ProbeManager *probeManager;

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
     * Register probe points for this object.
     */
    virtual void regProbePoints();

    /**
     * Register probe listeners for this object.
     */
    virtual void regProbeListeners();

    /**
     * Get the probe manager for this object.
     */
    ProbeManager *getProbeManager();

    /**
     * startup() is the final initialization call before simulation.
     * All state is initialized (including unserialized state, if any,
     * such as the curTick() value), so this is the appropriate place to
     * schedule initial event(s) for objects that need them.
     */
    virtual void startup();

    /**
     * Provide a default implementation of the drain interface that
     * simply returns 0 (draining completed) and sets the drain state
     * to Drained.
     */
    unsigned int drain(DrainManager *drainManger);

    /**
     * Serialize all SimObjects in the system.
     */
    static void serializeAll(std::ostream &os);

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

#ifdef DEBUG
void debugObjectBreak(const char *objs);
#endif

#endif // __SIM_OBJECT_HH__
