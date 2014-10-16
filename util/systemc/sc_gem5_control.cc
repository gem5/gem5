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
 *
 * Authors: Andrew Bardsley
 */

#include <cstdlib>
#include <iostream>

#include "base/statistics.hh"
#include "sim/cxx_config_ini.hh"
#include "sim/cxx_manager.hh"
#include "sim/init_signals.hh"
#include "sim/stat_control.hh"
#include "sc_gem5_control.hh"
#include "sc_logger.hh"
#include "sc_module.hh"
#include "stats.hh"

namespace Gem5SystemC
{

/** This is the private side of Gem5Control */
class Gem5TopLevelModule : public Gem5SystemC::Module
{
    friend class Gem5Control;

  protected:
    CxxConfigFileBase *config_file;
    CxxConfigManager *root_manager;
    Gem5SystemC::Logger logger;

  public:
    SC_HAS_PROCESS(Gem5TopLevelModule);

    Gem5TopLevelModule(sc_core::sc_module_name name,
        const std::string &config_filename);
    ~Gem5TopLevelModule();

    /** gem5 simulate.  @todo for more interesting simulation control,
     *  this needs to be more complicated */
    void run();
};

Gem5System::Gem5System(CxxConfigManager *manager_,
    const std::string &system_name, const std::string &instance_name) :
    manager(manager_),
    systemName(system_name),
    instanceName(instance_name)
{
    manager->addRenaming(CxxConfigManager::Renaming(
        system_name, instance_name));
}

Gem5System::~Gem5System()
{
    delete manager;
}

void Gem5System::setParam(const std::string &object,
    const std::string &param_name, const std::string &param_value)
{
    manager->setParam(systemName + (object != "" ? "." + object : ""),
        param_name, param_value);
}

void Gem5System::setParamVector(const std::string &object,
    const std::string &param_name,
    const std::vector<std::string> &param_values)
{
    manager->setParamVector(systemName +
        (object != "" ? "." + object : ""), param_name, param_values);
}

void Gem5System::instantiate()
{
    try {
        /* Make a new System */
        SimObject *obj = manager->findObject(systemName, true);

        /* Add the System's objects to the list of managed
         *  objects for initialisation */
        manager->findTraversalOrder(systemName);

        /* Bound ports *must* be internal to System */
        for (auto i = manager->objectsInOrder.begin();
             i != manager->objectsInOrder.end();
             ++ i)
        {
            manager->bindObjectPorts(*i);
        }

        /* gem5 startup sequence */
        manager->instantiate(false);
        manager->initState();
        manager->startup();
    } catch (CxxConfigManager::Exception &e) {
        fatal("Config problem in Gem5System: %s: %s",
            e.name, e.message);
    }
}

Gem5Control::Gem5Control(const std::string &config_filename)
{
    module = new Gem5TopLevelModule("gem5", config_filename);
}

Gem5Control::~Gem5Control()
{ }

void
Gem5Control::setDebugFlag(const char *flag)
{
    ::setDebugFlag(flag);
}

void
Gem5Control::clearDebugFlag(const char *flag)
{
    ::clearDebugFlag(flag);
}

Gem5System *
Gem5Control::makeSystem(const std::string &system_name,
    const std::string &instance_name)
{
    Gem5System *ret = new Gem5System(
        new CxxConfigManager(*(module->config_file)),
        system_name, instance_name);

    return ret;
}

Gem5TopLevelModule::Gem5TopLevelModule(sc_core::sc_module_name name,
    const std::string &config_filename) :
    Gem5SystemC::Module(name),
    config_file(NULL),
    root_manager(NULL)
{
    SC_THREAD(run);

    cxxConfigInit();

    /* Pass DPRINTF messages to SystemC */
    Trace::setDebugLogger(&logger);

    /* @todo need this as an option */
    Gem5SystemC::setTickFrequency();
    sc_core::sc_set_time_resolution(1, sc_core::SC_PS);

    /* Make a SystemC-synchronising event queue and install it as the
     *  sole top level gem5 EventQueue */
    Gem5SystemC::Module::setupEventQueues(*this);

    /* Enable keyboard interrupt, async I/O etc. */
    initSignals();

    /* Enable stats */
    Stats::initSimStats();
    Stats::registerHandlers(CxxConfig::statsReset, CxxConfig::statsDump);

    Trace::enabled = true;

    config_file = new CxxIniFile();

    if (!config_file->load(config_filename)) {
        fatal("Gem5TopLevelModule: Can't open config file: %s",
            config_filename);
    }

    root_manager = new CxxConfigManager(*config_file);

    CxxConfig::statsEnable();

    /* Make the root object */
    try {
        SimObject *root = root_manager->findObject("root", false);

        /* Make sure we don't traverse into root's children */
        root_manager->objectsInOrder.push_back(root);

        root_manager->instantiate(false);
        root_manager->initState();
        root_manager->startup();
    } catch (CxxConfigManager::Exception &e) {
        fatal("Config problem in Gem5TopLevelModule: %s: %s",
            e.name, e.message);
    }
}

Gem5TopLevelModule::~Gem5TopLevelModule()
{
    delete config_file;
    delete root_manager;
}

void Gem5TopLevelModule::run()
{
    GlobalSimLoopExitEvent *exit_event = NULL;

    exit_event = simulate();

    std::cerr << "Exit at tick " << curTick()
        << ", cause: " << exit_event->getCause() << '\n';

    getEventQueue(0)->dump();
}

}

Gem5SystemC::Gem5Control *makeGem5Control(const std::string &config_filename)
{
    return new Gem5SystemC::Gem5Control(config_filename);
}

