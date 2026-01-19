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
 *  C++-only configuration and instantiation support.  This allows a
 *  config to be read back from a config file and instantiated without
 *  Python.  Useful if you want to embed gem5 within a larger system
 *  without carrying the integration cost of the fully-featured
 *  configuration system.
 *
 *  This file contains the config loading/storing manager class
 */

#ifndef __SIM_CXX_MANAGER_HH__
#define __SIM_CXX_MANAGER_HH__

#include <list>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "base/cprintf.hh"
#include "sim/cxx_config.hh"

namespace gem5
{

class CheckpointIn;

/** This class allows a config file to be read into gem5 (generating the
 *  appropriate SimObjects) from C++ */
class CxxConfigManager
{
  protected:
    /** Configuration file being read */
    CxxConfigFileBase &configFile;

    /** Flags to pass to affect param setting */
    CxxConfigParams::Flags flags;

  public:
    /** Exception for instantiate/post-instantiate errors */
    class Exception : public std::exception
    {
      public:
        std::string name;
        std::string message;

      public:
        Exception(const std::string &name_, const std::string &message_) :
            name(name_), message(message_)
        { }

        const char *what() const throw() { return message.c_str(); }

        ~Exception() throw() { }
    };

    /** Name substitution when instantiating any object whose name starts
     *  with fromPrefix.  Where both renamed and unrenamed names are used
     *  in the code, `object' as part of a name usually refers to the
     *  unrenamed name (the name as it appears in the config file) and
     *  `instance' is part of the renamed name */
    struct Renaming
    {
        std::string fromPrefix;
        std::string toPrefix;

        Renaming(const std::string &from_prefix,
            const std::string &to_prefix) :
            fromPrefix(from_prefix),
            toPrefix(to_prefix)
        { }
    };

  public:
    /** SimObject indexed by name */
    std::map<std::string, SimObject *> objectsByName;

    /** ...Params objects created by this manager */
    std::map<std::string, CxxConfigParams *> objectParamsByName;

    /** SimObjects in order.  This is populated by findAllObjects */
    std::list<SimObject *> objectsInOrder;

  protected:
    /** While configuring, inVisit contains names of SimObjects visited in
     *  this recursive configuration walk */
    std::set<std::string> inVisit;

    /** All the renamings applicable when instantiating objects */
    std::list<Renaming> renamings;

    /** Bind a single connection between two objects' ports */
    void bindPort(SimObject *requestorObject, const std::string &requestPort,
        PortID requestPortIndex, SimObject *responderObject,
        const std::string &responsePort, PortID responsePortIndex);

    /** Bind a single (possibly vectored) request port to peers from the
     *  unparsed list peers with elements in the .ini connection format:
     *  path(.path)*.port[index] */
    void bindRequestPort(SimObject *object,
        const CxxConfigDirectoryEntry::PortDesc &port,
        const std::vector<std::string> &peers);

    /** Apply the first matching renaming in renamings to the given name */
    std::string rename(const std::string &from_name);

    /** Apply the first matching renaming in reverse (toPrefix -> fromPrefix
     *  for the given name */
    std::string unRename(const std::string &to_name);

  protected:
    /** Bind the ports of all the objects in objectInOrder order.
     *  Also */
    void bindAllPorts();

    /** Class for resolving SimObject names to SimObjects usable by the
     *  checkpoint restore mechanism */
    class SimObjectResolver : public gem5::SimObjectResolver
    {
      protected:
        CxxConfigManager &configManager;

      public:
        SimObjectResolver(CxxConfigManager &configManager_) :
            configManager(configManager_)
        { }

        SimObject *resolveSimObject(const std::string &name)
        { return &(configManager.getObject<SimObject>(name)); }
    };

    /** Singleton instance of SimObjectResolver */
    SimObjectResolver simObjectResolver;

  public:
    CxxConfigManager(CxxConfigFileBase &configFile_);

    /** Find the type field for a named object and return both the
     *  name of the type to object_type and the object's directory
     *  entry as the return value */
    const CxxConfigDirectoryEntry &findObjectType(
        const std::string &object_name, std::string &object_type);

    /** Add a name prefix renaming to those currently applied.  Call this
     *  before trying to instantiate any object as the name mappings are
     *  not applied to the config tree read from the config file but are
     *  applied while processing instantiations */
    void addRenaming(const Renaming &renaming);

  public:
    /** Bind the ports of a single SimObject */
    void bindObjectPorts(SimObject *object);

    /** Walk the configuration starting with object object_name and fill
     *  in all the elements of this object on the way.  This involves:
     *  <ul>
     *    <li>Calling findObjectParams to make the ...Params object
     *      If findObjectParams has already been called for this object,
     *      the ...Params object generated by that called (stored in
     *      (objectParamsByName[object_name] will be used)</li>
     *    <li>Populating the ...Params object references to other
     *      SimObjects by recursively descending into the trees formed
     *      by SimObject references</li>
     *    <li>Building the final SimObject and adding it to
     *      objectsByName</li>
     *    <li>If visit_children is true, recursively visit all this
     *      object's children and build/find them too</li>
     *  </ul>
     *  After the first call, this function will return
     *  objectsByName[object_name] */
    SimObject *findObject(const std::string &object_name,
        bool visit_children = false);

    /** Assign a parameter to the named object.
     *  This is called from within the findObject method */
    void populateParams(const std::string &object_name,
        const std::string &instance_name,
        CxxConfigParams *object_params,
        const CxxConfigDirectoryEntry::ParamDesc *param);

    /** Find the parameters for the named object.  Returns NULL if the
     *  object isn't in the configuration.  For the first call with a
     *  particular object name, a new CxxConfigParams descended object
     *  is made with the configuration file contents for this object.
     *  This involves populating that ...Params object with:
     *  <ul>
     *    <li>parameter values from the configuration file</li>
     *    <li>port connection connection counts from the connection counts
     *      indicated by the number of peer ports in the configuration
     *      file</li>
     *    <li>nulled (or vector<>::clear'ed) SimObject references for
     *      SimObject-values parameters</li>
     *  </ul>
     *  The ...Params object is then added to objectParamsByName
     *  After the first call, this function will return
     *  objectParamsByName[object_name] */
    CxxConfigParams *findObjectParams(const std::string &object_name);

    /** Populate objectsInOrder with a preorder, depth first traversal from
     *  the given object name down through all its children */
    void findTraversalOrder(const std::string &object_name);

    /** Find an object from objectsByName with a type-checking cast.
     *  This function is provided for manipulating objects after
     *  instantiate as it assumes the named object exists. */
    template<typename SimObjectType>
    SimObjectType &
    getObject(const std::string &object_name)
    {
        if (objectsByName.find(object_name) == objectsByName.end()) {
            throw Exception("", csprintf("No sim object named: %s",
                object_name));
        }

        SimObjectType *object = dynamic_cast<SimObjectType *>(
            objectsByName[object_name]);

        if (!object) {
            throw Exception("", csprintf("Sim object: %s  has the wrong"
                " type", object_name));
        }

        return *object;
    }

    /** Perform mem_func on each SimObject */
    void forEachObject(void (SimObject::*mem_func)());

    /** Find all objects by iterating over the object names in the config
     *  file with findObject.  Also populate the traversal order */
    void findAllObjects();

    /** Parse a port string of the form 'path(.path)*.port[index]' into
     *  path, port and index */
    static void parsePort(const std::string &inp,
        std::string &path, std::string &port, unsigned int &index);

    /** Build all objects (if build_all is true, otherwise objects must
     *  have been individually findObject-ed and added to the traversal
     *  order) and perform all the configuration specific actions up to,
     *  but not including initState.
     *
     *  If you want to set some parameters before completing instantiation,
     *  call findObjectParams on the objects you want to modify, then call
     *  instantiate */
    void instantiate(bool build_all = true);

    /** Call initState on all objects */
    void initState();

    /** Call startup on all objects */
    void startup();

    /** Drain all objects */
    unsigned int drain();

    /** Resume from drain */
    void drainResume();

    /** Serialize (checkpoint) all objects to the given stream */
    void serialize(std::ostream &os);

    /** Load all objects' state from the given Checkpoint */
    void loadState(CheckpointIn &checkpoint);

    /** Delete all objects and clear objectsByName and objectsByOrder */
    void deleteObjects();

    /** Get the resolver used to map SimObject names to SimObjects for
     *  checkpoint restore */
    SimObjectResolver &getSimObjectResolver() { return simObjectResolver; }

    /** Convenience functions for calling set... member functions on a
     *  CxxConfigParams for an object.  These functions throw Exception
     *  rather than return a bool on failure */
    void setParam(const std::string &object_name,
        const std::string &param_name, const std::string &param_value);
    void setParamVector(const std::string &object_name,
        const std::string &param_name,
        const std::vector<std::string> &param_values);
};

} // namespace gem5

#endif // __SIM_CXX_MANAGER_HH__
