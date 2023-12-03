/*
 * Copyright (c) 2015, University of Kaiserslautern
 * Copyright (c) 2016, Dresden University of Technology (TU Dresden)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 *
 *  Example top level file for SystemC-TLM integration with C++-only
 *  instantiation.
 *
 */

#include <systemc>
#include <tlm>

#include "sc_master_port.hh"
#include "sc_slave_port.hh"
#include "sim/cxx_config_ini.hh"
#include "sim/init_signals.hh"
#include "sim/stat_control.hh"
#include "sim_control.hh"
#include "stats.hh"

// Define global string variable decalred in stats.hh
std::string filename = "m5out/stats-systemc.txt";

namespace Gem5SystemC
{

Gem5SimControl *Gem5SimControl::instance = nullptr;

Gem5SimControl::Gem5SimControl(sc_core::sc_module_name name,
                               const std::string &configFile,
                               uint64_t simulationEnd,
                               const std::string &gem5DebugFlags)
    : Gem5SystemC::Module(name), simulationEnd(simulationEnd)
{
    SC_THREAD(run);

    if (instance != nullptr) {
        panic("Tried to instantiate Gem5SimControl more than once!\n");
    }
    instance = this;

    // register the systemc slave and master port handler
    gem5::ExternalSlave::registerHandler("tlm_slave",
                                         new SCSlavePortHandler(*this));
    gem5::ExternalMaster::registerHandler("tlm_master",
                                          new SCMasterPortHandler(*this));

    gem5::trace::setDebugLogger(&logger);

    Gem5SystemC::setTickFrequency();
    assert(sc_core::sc_get_time_resolution() ==
           sc_core::sc_time(1, sc_core::SC_PS));

    Gem5SystemC::Module::setupEventQueues(*this);
    gem5::initSignals();

    gem5::statistics::initSimStats();
    gem5::statistics::registerHandlers(CxxConfig::statsReset,
                                       CxxConfig::statsDump);

    gem5::trace::enable();

    gem5::CxxConfigFileBase *conf = new gem5::CxxIniFile();

    if (configFile.empty()) {
        std::cerr << "No gem5 config file specified!\n";
        std::exit(EXIT_FAILURE);
    }

    if (!conf->load(configFile.c_str())) {
        std::cerr << "Can't open config file: " << configFile << '\n';
        std::exit(EXIT_FAILURE);
    }

    config_manager = new gem5::CxxConfigManager(*conf);

    // parse debug flags string and clear/set flags accordingly
    std::stringstream ss;
    ss.str(gem5DebugFlags);
    std::string flag;
    while (std::getline(ss, flag, ' ')) {
        if (flag.at(0) == '-') {
            flag.erase(0, 1); // remove the '-'
            gem5::clearDebugFlag(flag.c_str());
        } else {
            gem5::setDebugFlag(flag.c_str());
        }
    }

    CxxConfig::statsEnable();
    gem5::getEventQueue(0)->dump();

    try {
        config_manager->instantiate();
    } catch (gem5::CxxConfigManager::Exception &e) {
        std::cerr << "Config problem in sim object " << e.name << ": "
                  << e.message << "\n";
        std::exit(EXIT_FAILURE);
    }
}

void
Gem5SimControl::end_of_elaboration()
{
    try {
        config_manager->initState();
        config_manager->startup();
    } catch (gem5::CxxConfigManager::Exception &e) {
        std::cerr << "Config problem in sim object " << e.name << ": "
                  << e.message << "\n";
        std::exit(EXIT_FAILURE);
    }
}

void
Gem5SimControl::run()
{
    // notify callback
    beforeSimulate();

    gem5::GlobalSimLoopExitEvent *exit_event = NULL;

    if (simulationEnd == 0) {
        exit_event = simulate();
    } else {
        exit_event = simulate(simulationEnd);
    }

    std::cerr << "Exit at tick " << gem5::curTick()
              << ", cause: " << exit_event->getCause() << '\n';

    gem5::getEventQueue(0)->dump();

    // notify callback
    afterSimulate();

#if TRY_CLEAN_DELETE
    config_manager->deleteObjects();
#endif
}

void
Gem5SimControl::registerSlavePort(const std::string &name, SCSlavePort *port)
{
    if (slavePorts.find(name) == slavePorts.end()) {
        slavePorts[name] = port;
    } else {
        std::cerr << "Slave Port " << name << " is already registered!\n";
        std::exit(EXIT_FAILURE);
    }
}

void
Gem5SimControl::registerMasterPort(const std::string &name, SCMasterPort *port)
{
    if (masterPorts.find(name) == masterPorts.end()) {
        masterPorts[name] = port;
    } else {
        std::cerr << "Master Port " << name << " is already registered!\n";
        std::exit(EXIT_FAILURE);
    }
}

SCSlavePort *
Gem5SimControl::getSlavePort(const std::string &name)
{
    if (slavePorts.find(name) == slavePorts.end()) {
        std::cerr << "Slave Port " << name << " was not found!\n";
        std::exit(EXIT_FAILURE);
    }

    return slavePorts.at(name);
}

SCMasterPort *
Gem5SimControl::getMasterPort(const std::string &name)
{
    if (masterPorts.find(name) == masterPorts.end()) {
        std::cerr << "Master Port " << name << " was not found!\n";
        std::exit(EXIT_FAILURE);
    }

    return masterPorts.at(name);
}

} // namespace Gem5SystemC
