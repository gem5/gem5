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
 *
 * Authors: Matthias Jung
 *          Christian Menard
 */

#ifndef __SC_SIM_CONTROL_HH__
#define __SC_SIM_CONTROL_HH__

#include <tlm_utils/simple_target_socket.h>

#include <systemc>
#include <tlm>

#include "sc_logger.hh"
#include "sc_module.hh"
#include "sim/cxx_manager.hh"
#include "sim/system.hh"

namespace Gem5SystemC
{

/**
 * This is the central SystemC module that orchestrates the gem5 simulation.
 *
 * The module is responsible for loading the configuration file, setting up and
 * maintaining the event queues, as well as starting and ending the simulation.
 * While it is mandatory to have one instance of this class for running a gem5
 * simulation in SystemC, it is not allowed to have multiple instances!
 */
class Gem5SimControl : public Gem5SystemC::Module
{
  protected:
    CxxConfigManager* config_manager;
    Gem5SystemC::Logger logger;

    Tick simulationEnd;

    /// Pointer to a previously created instance.
    static Gem5SimControl* instance;

  public:
    SC_HAS_PROCESS(Gem5SimControl);

    /**
     * Constructor.
     *
     * This class has a public constructor although the class is actually a
     * singleton. The public constructor is required to ensure compatibility
     * to external SystemC based tools. For the same reason, the constructor
     * parameters are basic types (int, string).
     *
     * @param configFile     location of the gem5 configuration file
     * @param simulationEnd  number of ticks to simulate
     * @param gem5DebugFlags a space separated list of gem5 debug flags to be
     *                       set, a prepended '-' clears the flag
     */
    Gem5SimControl(sc_core::sc_module_name name,
                   const std::string& configFile,
                   uint64_t simulationEnd,
                   const std::string& gem5DebugFlags);

    void before_end_of_elaboration();

    void run();
};

}

#endif
