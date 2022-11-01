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
 *  Top level definitions for exporting gem5 across a dlopen interface.
 *
 *  Gem5Control should be instantiated once to build the gem5 root.
 *  Systems from that root's config file can then be instantiated as
 *  Gem5System's using Gem5Control::makeSystem.
 *
 *  Gem5Control contains a Gem5TopLevelModule which is a SystemC
 *  module providing the gem5 `simulate' function as its sole
 *  thread action.
 */

#ifndef __SIM_SC_GEM5_CONTROL_HH__
#define __SIM_SC_GEM5_CONTROL_HH__

#include <string>
#include <vector>

namespace gem5
{
class CxxConfigManager;
} // namespace gem5

namespace Gem5SystemC
{

class Gem5TopLevelModule;
class Gem5Control;

/** Gem5System's wrap gem5::CxxConfigManager's instantiating a gem5 System
 *  object (and its children).  New Gem5Systems are created by
 *  Gem5Control::makeSystem.  A new system can have its parameters
 *  tweaked using setParam{,Vector} before being instantiated using
 *  Gem5System::instantiate.  After instantiation, any external ports
 *  declared by the system should be visible in SystemC.
 *
 *  It is recommended that a SystemC wrapper sc_module is declared to
 *  own each Gem5System with the call to Gem5System::instantiate being
 *  made in that wrapper's constructor
 *
 *  Note that *every* `normal' member function in this class *must*
 *  be virtual to ensure that references to the functions go through
 *  the pointer acquired using makeSystem and not looked up as
 *  name-mangled symbols
 *
 *  */
class Gem5System
{
  private:
    /** Config management for *just* this system's objects (notably
     *  excluding root */
    gem5::CxxConfigManager *manager;

    /** The config file prototype for the system */
    std::string systemName;

    /** The instantiated (in gem5) name of the system */
    std::string instanceName;

  public:
    /** A constructor only used by Gem5Control */
    Gem5System(gem5::CxxConfigManager *manager_,
        const std::string &system_name, const std::string &instance_name);

    virtual ~Gem5System();

    /** Parameter setting functions callable before instantiate */
    virtual void setParam(const std::string &object,
        const std::string &param_name, const std::string &param_value);

    virtual void setParamVector(const std::string &system_name,
        const std::string &param_name,
        const std::vector<std::string> &param_values);

    /** Build the system's gem5 infrastructure, bind its ports (note
     *  that all ports *must* be internal to the system), init and
     *  gem5::SimObject::startup the system */
    virtual void instantiate();
};

/** Singleton class containing gem5 simulation control.
 *
 *  Note that *every* `normal' member function in this class *must*
 *  be virtual to ensure that references to the functions go through
 *  the pointer acquired using makeGem5Control and not looked up as
 *  name-mangled symbols
 */
class Gem5Control
{
  private:
    /** Private SystemC module containing top level simulation control */
    Gem5TopLevelModule *module;

    /** One-time-settable version string */
    std::string version;

  public:
    Gem5Control(const std::string &config_filename);

    virtual ~Gem5Control();

    /** Set/clear a gem5 debug flag */
    virtual void setDebugFlag(const char *flag);
    virtual void clearDebugFlag(const char *flag);

    /* Register an action to happen at the end of elaboration */
    virtual void registerEndOfElaboration(void (*func)());

    /** Make a System from the config file description for system
     *  system_name and call it instance_name in gem5 */
    virtual Gem5System *makeSystem(const std::string &system_name,
        const std::string &top_instance);

    /** set/get version string */
    virtual const std::string &getVersion() const;
    virtual void setVersion(const std::string &new_version);
};

}

/** Instantiate a Gem5Control.  This can be called using dlopen/dlsym
 *  to kick-start gem5 */
extern "C" Gem5SystemC::Gem5Control *makeGem5Control(
    const std::string &config_filename);

#endif // __SIM_SC_GEM5_CONTROL_HH__
