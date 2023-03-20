/*
 * Copyright (c) 2015, 2021 ARM Limited
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
 */

/* @file
 * User Console Definitions
 */

#ifndef __SIM_OBJECT_HH__
#define __SIM_OBJECT_HH__

#include <string>
#include <vector>

#include "base/named.hh"
#include "base/stats/group.hh"
#include "params/SimObject.hh"
#include "sim/drain.hh"
#include "sim/eventq.hh"
#include "sim/port.hh"
#include "sim/serialize.hh"

namespace gem5
{

class EventManager;
class ProbeManager;
class SimObjectResolver;

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
 *
 * The python version of a SimObject class actually represents its Params
 * structure which holds all its parameter settings and its name. When python
 * needs to create a C++ instance of one of those classes, it uses the Params
 * struct's create() method which returns one instance, set up with the
 * parameters in the struct.
 *
 * When writing a SimObject class, there are three different cases as far as
 * what you need to do to support the create() method, for hypothetical class
 * Foo.
 *
 * If you have a constructor with a signature like this:
 *
 * Foo(const FooParams &)
 *
 * you don't have to do anything, a create method will be automatically
 * defined which will call your constructor and return that instance. You
 * should use this option most of the time.
 *
 * If you have a constructor with that signature but still want to define
 * your own create method for some reason, you can do that by providing an
 * alternative implementation which will override the default. It should have
 * this signature:
 *
 * Foo *FooParams::create() const;
 *
 * If you don't have a constructor with that signature at all, then you must
 * implement the create method with that signature which will build your
 * object in some other way.
 *
 * A reference to the SimObjectParams will be returned via the params()
 * API. It is quite common for a derived class (DerivSimObject) to access its
 * derived parameters by downcasting the SimObjectParam to DerivSimObjectParams
 *
 * \code{.cpp}
 *     using Params = DerivSimObjectParams;
 *     const Params &
 *     params() const
 *     {
 *         return reinterpret_cast<const Params&>(_params);
 *     }
 * \endcode
 *
 * We provide the PARAMS(..) macro as syntactic sugar to replace the code
 * above with a much simpler:
 *
 * \code{.cpp}
 *     PARAMS(DerivSimObject);
 * \endcode
 */
class SimObject : public EventManager, public Serializable, public Drainable,
                  public statistics::Group, public Named
{
  private:
    typedef std::vector<SimObject *> SimObjectList;

    /** List of all instantiated simulation objects. */
    static SimObjectList simObjectList;

    /** Helper to resolve an object given its name. */
    static SimObjectResolver *_objNameResolver;

    /** Manager coordinates hooking up probe points with listeners. */
    ProbeManager *probeManager;

  protected:
    /**
     * Cached copy of the object parameters.
     *
     * @ingroup api_simobject
     */
    const SimObjectParams &_params;

  public:
    typedef SimObjectParams Params;
    /**
     * @return This function returns the cached copy of the object parameters.
     *
     * @ingroup api_simobject
     */
    const Params &params() const { return _params; }

    /**
     * @ingroup api_simobject
     */
    SimObject(const Params &p);

    virtual ~SimObject();

  public:
    /**
     * init() is called after all C++ SimObjects have been created and
     * all ports are connected.  Initializations that are independent
     * of unserialization but rely on a fully instantiated and
     * connected SimObject graph should be done here.
     *
     * @ingroup api_simobject
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
     *
     * @ingroup api_serialize
     */
    virtual void loadState(CheckpointIn &cp);

    /**
     * initState() is called on each SimObject when *not* restoring
     * from a checkpoint.  This provides a hook for state
     * initializations that are only required for a "cold start".
     *
     * @ingroup api_serialize
     */
    virtual void initState();

    /**
     * Register probe points for this object.
     *
     * @ingroup api_simobject
     */
    virtual void regProbePoints();

    /**
     * Register probe listeners for this object.
     *
     * @ingroup api_simobject
     */
    virtual void regProbeListeners();

    /**
     * Get the probe manager for this object.
     *
     * Probes generate traces. A trace is a file that
     * keeps a log of events. For example, we can have a probe
     * listener for an address and the trace will be a file that
     * has time stamps for all the reads and writes to that address.
     *
     * @ingroup api_simobject
     */
    ProbeManager *getProbeManager();

    /**
     * Get a port with a given name and index. This is used at binding time
     * and returns a reference to a protocol-agnostic port.
     *
     * gem5 has a request and response port interface. All memory objects
     * are connected together via ports. These ports provide a rigid
     * interface between these memory objects. These ports implement
     * three different memory system modes: timing, atomic, and
     * functional. The most important mode is the timing mode and here
     * timing mode is used for conducting cycle-level timing
     * experiments. The other modes are only used in special
     * circumstances and should *not* be used to conduct cycle-level
     * timing experiments. The other modes are only used in special
     * circumstances. These ports allow SimObjects to communicate with
     * each other.
     *
     * @param if_name Port name
     * @param idx Index in the case of a VectorPort
     *
     * @return A reference to the given port
     *
     * @ingroup api_simobject
     */
    virtual Port &getPort(const std::string &if_name,
                          PortID idx=InvalidPortID);

    /**
     * startup() is the final initialization call before simulation.
     * All state is initialized (including unserialized state, if any,
     * such as the curTick() value), so this is the appropriate place to
     * schedule initial event(s) for objects that need them.
     *
     * @ingroup api_simobject
     */
    virtual void startup();

    /**
     * Provide a default implementation of the drain interface for
     * objects that don't need draining.
     */
    DrainState drain() override { return DrainState::Drained; }

    /**
     * Write back dirty buffers to memory using functional writes.
     *
     * After returning, an object implementing this method should have
     * written all its dirty data back to memory. This method is
     * typically used to prepare a system with caches for
     * checkpointing.
     *
     * @ingroup api_simobject
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
     *
     * @ingroup api_simobject
     */
    virtual void memInvalidate() {};

    void serialize(CheckpointOut &cp) const override {};
    void unserialize(CheckpointIn &cp) override {};

    /**
     * Create a checkpoint by serializing all SimObjects in the system.
     *
     * This is the entry point in the process of checkpoint creation,
     * so it will create the checkpoint file and then unfold into
     * the serialization of all the sim objects declared.
     *
     * Each SimObject instance is explicitly and individually serialized
     * in its own section. As such, the serialization functions should not
     * be called on sim objects anywhere else; otherwise, these objects
     * would be needlessly serialized more than once.
     */
    static void serializeAll(const std::string &cpt_dir);

    /**
     * Find the SimObject with the given name and return a pointer to
     * it.  Primarily used for interactive debugging.  Argument is
     * char* rather than std::string to make it callable from gdb.
     *
     * @ingroup api_simobject
     */
    static SimObject *find(const char *name);

    /**
     * There is a single object name resolver, and it is only set when
     * simulation is restoring from checkpoints.
     *
     * @param Pointer to the single sim object name resolver.
     */
    static void setSimObjectResolver(SimObjectResolver *resolver);

    /**
     * There is a single object name resolver, and it is only set when
     * simulation is restoring from checkpoints.
     *
     * @return Pointer to the single sim object name resolver.
     */
    static SimObjectResolver *getSimObjectResolver();
};

/* Add PARAMS(ClassName) to every descendant of SimObject that needs
 * params.
 *
 * Strictly speaking, we need static_cast here, because the types are
 * related by inheritance, but since the target type may be
 * incomplete, the compiler does not know the relation.
 */
#define PARAMS(type)                                     \
    using Params = type ## Params;                       \
    const Params &                                       \
    params() const                                       \
    {                                                    \
        return reinterpret_cast<const Params&>(_params); \
    }

/**
 * Base class to wrap object resolving functionality.
 *
 * This can be provided to the serialization framework to allow it to
 * map object names onto C++ objects.
 */
class SimObjectResolver
{
  public:
    virtual ~SimObjectResolver() { }

    /**
     * Find a SimObject given a full path name
     *
     * @ingroup api_serialize
     */
    virtual SimObject *resolveSimObject(const std::string &name) = 0;
};


/**
 * To avoid circular dependencies the unserialization of SimObjects must be
 * implemented here.
 *
 * @ingroup api_serialize
 */
void objParamIn(CheckpointIn &cp, const std::string &name, SimObject * &param);

void debug_serialize(const std::string &cpt_dir);

} // namespace gem5

#endif // __SIM_OBJECT_HH__
