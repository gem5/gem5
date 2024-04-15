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

#include <cstdlib>
#include <iostream>
#include <list>

#include "base/statistics.hh"
#include "sc_gem5_control.hh"
#include "sc_logger.hh"
#include "sc_module.hh"
#include "sim/cxx_config_ini.hh"
#include "sim/cxx_manager.hh"
#include "sim/debug.hh"
#include "sim/init_signals.hh"
#include "sim/stat_control.hh"
#include "stats.hh"

namespace Gem5SystemC
{

/** This is the private side of Gem5Control */
class Gem5TopLevelModule : public Gem5SystemC::Module
{
    friend class Gem5Control;

  protected:
    gem5::CxxConfigFileBase *config_file;
    gem5::CxxConfigManager *root_manager;
    Gem5SystemC::Logger logger;

    /** Things to do at end_of_elaborate */
    std::list<void (*)()> endOfElaborationFuncs;

  public:
    SC_HAS_PROCESS(Gem5TopLevelModule);

    Gem5TopLevelModule(sc_core::sc_module_name name,
                       const std::string &config_filename);
    ~Gem5TopLevelModule();

    /** gem5 simulate.  @todo for more interesting simulation control,
     *  this needs to be more complicated */
    void run();

    /* Register an action to happen at the end of elaboration */
    void
    registerEndOfElaboration(void (*func)())
    {
        endOfElaborationFuncs.push_back(func);
    }

    /** SystemC startup */
    void end_of_elaboration();
};

Gem5System::Gem5System(gem5::CxxConfigManager *manager_,
                       const std::string &system_name,
                       const std::string &instance_name)
    : manager(manager_), systemName(system_name), instanceName(instance_name)
{
    manager->addRenaming(
        gem5::CxxConfigManager::Renaming(system_name, instance_name));
}

Gem5System::~Gem5System() { delete manager; }

void
Gem5System::setParam(const std::string &object, const std::string &param_name,
                     const std::string &param_value)
{
    manager->setParam(systemName + (object != "" ? "." + object : ""),
                      param_name, param_value);
}

void
Gem5System::setParamVector(const std::string &object,
                           const std::string &param_name,
                           const std::vector<std::string> &param_values)
{
    manager->setParamVector(systemName + (object != "" ? "." + object : ""),
                            param_name, param_values);
}

void
Gem5System::instantiate()
{
    try {
        /* Make a new System */
        gem5::SimObject *obj = manager->findObject(systemName, true);

        /* Add the System's objects to the list of managed
         *  objects for initialisation */
        manager->findTraversalOrder(systemName);

        /* Bound ports *must* be internal to System */
        for (auto i = manager->objectsInOrder.begin();
             i != manager->objectsInOrder.end(); ++i) {
            manager->bindObjectPorts(*i);
        }

        /* gem5 startup sequence */
        manager->instantiate(false);
        manager->initState();
        manager->startup();
    } catch (gem5::CxxConfigManager::Exception &e) {
        fatal("Config problem in Gem5System: %s: %s", e.name, e.message);
    }
}

Gem5Control::Gem5Control(const std::string &config_filename)
{
    module = new Gem5TopLevelModule("gem5", config_filename);
}

Gem5Control::~Gem5Control() {}

void
Gem5Control::registerEndOfElaboration(void (*func)())
{
    module->registerEndOfElaboration(func);
}

void
Gem5Control::setDebugFlag(const char *flag)
{
    ::gem5::setDebugFlag(flag);
}

void
Gem5Control::clearDebugFlag(const char *flag)
{
    ::gem5::clearDebugFlag(flag);
}

Gem5System *
Gem5Control::makeSystem(const std::string &system_name,
                        const std::string &instance_name)
{
    Gem5System *ret =
        new Gem5System(new gem5::CxxConfigManager(*(module->config_file)),
                       system_name, instance_name);

    return ret;
}

const std::string &
Gem5Control::getVersion() const
{
    return version;
}

void
Gem5Control::setVersion(const std::string &new_version)
{
    if (version != "")
        fatal("Gem5Control::setVersion called for a second time");

    version = new_version;
}

Gem5TopLevelModule::Gem5TopLevelModule(sc_core::sc_module_name name,
                                       const std::string &config_filename)
    : Gem5SystemC::Module(name), config_file(NULL), root_manager(NULL)
{
    SC_THREAD(run);

    /* Pass DPRINTF messages to SystemC */
    gem5::trace::setDebugLogger(&logger);

    /* @todo need this as an option */
    Gem5SystemC::setTickFrequency();

    /* Make a SystemC-synchronising event queue and install it as the
     *  sole top level gem5 EventQueue */
    Gem5SystemC::Module::setupEventQueues(*this);

    if (sc_core::sc_get_time_resolution() !=
        sc_core::sc_time(1, sc_core::SC_PS)) {
        fatal("Time resolution must be set to 1 ps for gem5 to work");
    }

    /* Enable keyboard interrupt, async I/O etc. */
    gem5::initSignals();

    /* Enable stats */
    gem5::statistics::initSimStats();
    gem5::statistics::registerHandlers(CxxConfig::statsReset,
                                       CxxConfig::statsDump);

    gem5::trace::enable();

    config_file = new gem5::CxxIniFile();

    if (!config_file->load(config_filename)) {
        fatal("Gem5TopLevelModule: Can't open config file: %s",
              config_filename);
    }

    root_manager = new gem5::CxxConfigManager(*config_file);

    CxxConfig::statsEnable();

    /* Make the root object */
    try {
        gem5::SimObject *root = root_manager->findObject("root", false);

        /* Make sure we don't traverse into root's children */
        root_manager->objectsInOrder.push_back(root);

        root_manager->instantiate(false);
        root_manager->initState();
        root_manager->startup();
    } catch (gem5::CxxConfigManager::Exception &e) {
        fatal("Config problem in Gem5TopLevelModule: %s: %s", e.name,
              e.message);
    }
}

Gem5TopLevelModule::~Gem5TopLevelModule()
{
    delete config_file;
    delete root_manager;
}

void
Gem5TopLevelModule::run()
{
    gem5::GlobalSimLoopExitEvent *exit_event = NULL;

    exit_event = simulate();

    std::cerr << "Exit at tick " << gem5::curTick()
              << ", cause: " << exit_event->getCause() << '\n';

    gem5::getEventQueue(0)->dump();
}

void
Gem5TopLevelModule::end_of_elaboration()
{
    for (auto i = endOfElaborationFuncs.begin();
         i != endOfElaborationFuncs.end(); ++i) {
        (*i)();
    }
}

} // namespace Gem5SystemC

Gem5SystemC::Gem5Control *
makeGem5Control(const std::string &config_filename)
{
    return new Gem5SystemC::Gem5Control(config_filename);
}
