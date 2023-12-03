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
 *  config to be read back from a .ini and instantiated without
 *  Python.  Useful if you want to embed gem5 within a larger system
 *  without carrying the integration cost of the fully-featured
 *  configuration system.
 *
 *  This file contains definitions needed to store summaries of a
 *  SimObject's parameter structure
 */

#ifndef __SIM_CXX_CONFIG_HH__
#define __SIM_CXX_CONFIG_HH__

#include <map>
#include <string>
#include <vector>

#include "sim/sim_object.hh"

namespace gem5
{

class CxxConfigParams;

/** Config details entry for a SimObject.  Instances of this class contain
 *  enough configuration layout information to popular a ...Param structure
 *  and build a SimObject from it with the help of the 'set' functions in
 *  each ...Param class */
class CxxConfigDirectoryEntry
{
  public:
    /* Class to represent parameters and SimObject references within
     *  SimObjects */
    class ParamDesc
    {
      public:
        const std::string name;

        /* Is this a vector or singleton parameters/SimObject */
        const bool isVector;

        /** Is this a SimObject, and so is to be set with setSimObject...
         *  or another from-string parameter set with setParam... */
        const bool isSimObject;

        ParamDesc(const std::string &name_, bool isVector_, bool isSimObject_)
            : name(name_), isVector(isVector_), isSimObject(isSimObject_)
        {}
    };

    /** Similar to ParamDesc to describe ports */
    class PortDesc
    {
      public:
        const std::string name;

        /* Is this a vector or singleton parameters/SimObject */
        const bool isVector;

        /** Is this a request or response port */
        const bool isRequestor;

        PortDesc(const std::string &name_, bool isVector_, bool isRequestor_)
            : name(name_), isVector(isVector_), isRequestor(isRequestor_)
        {}
    };

    /** All parameters (including SimObjects) in order */
    std::map<std::string, ParamDesc *> parameters;

    /** Ports */
    std::map<std::string, PortDesc *> ports;

    /** Make a ...Param structure for the SimObject class of this entry */
    virtual CxxConfigParams *
    makeParamsObject() const
    {
        return NULL;
    }

    virtual ~CxxConfigDirectoryEntry() {}
};

/** Directory of all SimObject classes config details */
std::map<std::string, CxxConfigDirectoryEntry *> &cxxConfigDirectory();

/** Base for peer classes of SimObjectParams derived classes with parameter
 *  modifying member functions. C++ configuration will offer objects of
 *  these classes to SimObjects as params rather than SimObjectParams
 *  objects */
class CxxConfigParams
{
  private:
    static const std::string invalidName;

  protected:
    struct AddToConfigDir
    {
        AddToConfigDir(const std::string &name, CxxConfigDirectoryEntry *entry)
        {
            auto it_success = cxxConfigDirectory().insert({ name, entry });
            panic_if(
                !it_success.second,
                "Failed to insert config directory entry %s (duplicate?).",
                name);
        }
    };

  public:
    /** Flags passable to setParam... to smooth over any parsing difference
     *  between different config files */
    typedef uint32_t FlagsType;
    typedef gem5::Flags<FlagsType> Flags;

    /** Example flag */
    /* static const FlagsType MY_NEW_FLAG = 0x00000001; */

  public:
    /** Set future object's full path name */
    virtual void
    setName(const std::string &name_)
    {}

    /** Get full path name string */
    virtual const std::string &
    getName()
    {
        return invalidName;
    }

    /** Set a SimObject valued parameter with a reference to the given
     *  SimObject.  This will return false if the parameter name is not
     *  valid or the object is of the wrong type */
    virtual bool
    setSimObject(const std::string &name, SimObject *simObject)
    {
        return false;
    }

    /** As setSimObjectVector but set a whole vector of references */
    virtual bool
    setSimObjectVector(const std::string &name,
                       const std::vector<SimObject *> &simObjects)
    {
        return false;
    }

    /** Set a parameter with a value parsed from the given string.  The
     *  parsing regime matches the format of .ini config files.  Returns
     *  false if the parameter name is not valid or the string cannot be
     *  parsed as the type of the parameter */
    virtual bool
    setParam(const std::string &name, const std::string &value,
             const Flags flags)
    {
        return false;
    }

    /** As setParamVector but for parameters given as vectors pre-separated
     *  into elements */
    virtual bool
    setParamVector(const std::string &name,
                   const std::vector<std::string> &values, const Flags flags)
    {
        return false;
    }

    /** Set the number of connections expected for the named port.  Returns
     *  false if the port name is not valid */
    virtual bool
    setPortConnectionCount(const std::string &name, unsigned int count)
    {
        return false;
    }

    /** Create the associated SimObject */
    virtual SimObject *
    simObjectCreate()
    {
        return NULL;
    }

    CxxConfigParams() {}

    virtual ~CxxConfigParams() {}
};

/** Config file wrapper providing a common interface to CxxConfigManager */
class CxxConfigFileBase
{
  public:
    CxxConfigFileBase() {}

    virtual ~CxxConfigFileBase() {}

    /** Get a single parameter value as a string returned in value.
     *  For booleans, the function expects "true" or "false" in value.
     *  For NULL SimObjects, it expects "Null" */
    virtual bool getParam(const std::string &object_name,
                          const std::string &param_name,
                          std::string &value) const = 0;

    /** Get a list/vector parameter */
    virtual bool getParamVector(const std::string &object_name,
                                const std::string &param_name,
                                std::vector<std::string> &values) const = 0;

    /** Get the peer (connected) ports of the named ports */
    virtual bool getPortPeers(const std::string &object_name,
                              const std::string &port_name,
                              std::vector<std::string> &peers) const = 0;

    /** Does an object with this path exist? */
    virtual bool objectExists(const std::string &object_name) const = 0;

    /** Get all SimObjects in the config */
    virtual void getAllObjectNames(std::vector<std::string> &list) const = 0;

    /** Get the names or paths of all the children SimObjects of this
     *  SimObject.  If return_paths is true then full paths are returned.
     *  If false, only the last name component for each object is returned */
    virtual void getObjectChildren(const std::string &object_name,
                                   std::vector<std::string> &children,
                                   bool return_paths = false) const = 0;

    /** Load config file */
    virtual bool load(const std::string &filename) = 0;

    /** Get the flags which should be used to modify parameter parsing
     *  behaviour */
    virtual CxxConfigParams::Flags
    getFlags() const
    {
        return 0;
    }
};

} // namespace gem5

#endif // __SIM_CXX_CONFIG_HH__
