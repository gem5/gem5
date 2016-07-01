/*
 * Copyright (c) 2015, University of Kaiserslautern
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
 *          Abdul Mutaal Ahmad
 */

/**
 * @file
 *
 *  Example top level file for SystemC-TLM integration with C++-only
 *  instantiation.
 *
 */

#include <tlm_utils/simple_target_socket.h>

#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <systemc>
#include <tlm>
#include <typeinfo>

#include "base/statistics.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "sc_logger.hh"
#include "sc_module.hh"
#include "sc_port.hh"
#include "sc_target.hh"
#include "sim/cxx_config_ini.hh"
#include "sim/cxx_manager.hh"
#include "sim/init_signals.hh"
#include "sim/serialize.hh"
#include "sim/simulate.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"
#include "stats.hh"

// Defining global string variable decalred in stats.hh
std::string filename;

void usage(const std::string &prog_name)
{
    std::cerr << "Usage: " << prog_name << (
        " <config_file.ini> [ <option> ]\n\n"
        "OPTIONS:\n"

        "    -o <offset>                  -- set memory offset\n"
        "    -p <object> <param> <value>  -- set a parameter\n"
        "    -v <object> <param> <values> -- set a vector parameter from a\n"
        "                                    comma separated values string\n"
        "    -d <flag>                    -- set a debug flag\n"
        "                                    (-<flag> clear a flag)\n"
        "    -D                           -- debug on\n"
        "    -e <ticks>                   -- end of simulation after a \n"
        "                                    given number of ticks\n"
        "\n"
        );
    std::exit(EXIT_FAILURE);
}

class SimControl : public Gem5SystemC::Module
{
    protected:
    int argc;
    char **argv;
    CxxConfigManager *config_manager;
    Gem5SystemC::Logger logger;

    Tick sim_end;
    bool debug;
    unsigned int offset;

    public:
    SC_HAS_PROCESS(SimControl);

    SimControl(sc_core::sc_module_name name, int argc_, char **argv_);

    void before_end_of_elaboration();

    bool getDebugFlag() { return debug; }

    unsigned int getOffset() { return offset; }

    void run();
};

SimControl::SimControl(sc_core::sc_module_name name,
                       int argc_,
                       char **argv_) : Gem5SystemC::Module(name),
                                       argc(argc_),
                                       argv(argv_)
{
    SC_THREAD(run);

    std::string prog_name(argv[0]);
    unsigned int arg_ptr = 1;

    if (argc == 1) {
        usage(prog_name);
    }

    cxxConfigInit();
    Gem5SystemC::registerSCPorts();

    Trace::setDebugLogger(&logger);

    Gem5SystemC::setTickFrequency();
    sc_core::sc_set_time_resolution(1, sc_core::SC_PS);

    Gem5SystemC::Module::setupEventQueues(*this);
    initSignals();

    Stats::initSimStats();
    Stats::registerHandlers(CxxConfig::statsReset, CxxConfig::statsDump);

    Trace::enable();

    sim_end = 0;
    debug = false;
    offset = 0;

    const std::string config_file(argv[arg_ptr]);

    CxxConfigFileBase *conf = new CxxIniFile();

    if (!conf->load(config_file.c_str())) {
        std::cerr << "Can't open config file: " << config_file << '\n';
        std::exit(EXIT_FAILURE);
    }
    arg_ptr++;

    config_manager = new CxxConfigManager(*conf);

    try {
        while (arg_ptr < argc) {
            std::string option(argv[arg_ptr]);
            arg_ptr++;
            unsigned num_args = argc - arg_ptr;

            if (option == "-p") {
                if (num_args < 3) {
                    usage(prog_name);
                }

                config_manager->setParam(argv[arg_ptr], argv[arg_ptr + 1],
                argv[arg_ptr + 2]);
                arg_ptr += 3;
            } else if (option == "-v") {
                std::vector<std::string> values;

                if (num_args < 3) {
                    usage(prog_name);
                }
                tokenize(values, argv[2], ',');
                config_manager->setParamVector(argv[arg_ptr],
                                               argv[arg_ptr],
                                               values);
                arg_ptr += 3;
            } else if (option == "-d") {
                if (num_args < 1) {
                    usage(prog_name);
                }
                if (argv[arg_ptr][0] == '-') {
                    clearDebugFlag(argv[arg_ptr] + 1);
                } else {
                    setDebugFlag(argv[arg_ptr]);
                }
                arg_ptr++;
            } else if (option == "-e") {
                if (num_args < 1) {
                    usage(prog_name);
                }
                std::istringstream(argv[arg_ptr]) >> sim_end;
                arg_ptr++;
            } else if (option == "-D") {
                debug = true;
            } else if (option == "-o") {
                if (num_args < 1) {
                    usage(prog_name);
                }
                std::istringstream(argv[arg_ptr]) >> offset;
                arg_ptr++;
                /* code */
            } else {
                usage(prog_name);
            }
        }
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << e.name << ": " << e.message << "\n";
        std::exit(EXIT_FAILURE);
    }

    CxxConfig::statsEnable();
    getEventQueue(0)->dump();

    try {
        config_manager->instantiate();
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << "Config problem in sim object "
                  << e.name << ": " << e.message << "\n";
        std::exit(EXIT_FAILURE);
    }
}

void
SimControl::before_end_of_elaboration()
{
    try {
        config_manager->initState();
        config_manager->startup();
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << "Config problem in sim object "
            << e.name << ": " << e.message << "\n";
        std::exit(EXIT_FAILURE);
    }
}

void
SimControl::run()
{
    GlobalSimLoopExitEvent *exit_event = NULL;

    if (sim_end == 0) {
        exit_event = simulate();
    } else {
        exit_event = simulate(sim_end);
    }

    std::cerr << "Exit at tick " << curTick()
              << ", cause: " << exit_event->getCause() << '\n';

    getEventQueue(0)->dump();

#if TRY_CLEAN_DELETE
    config_manager->deleteObjects();
#endif
}


void
reportHandler(const sc_core::sc_report &report,
              const sc_core::sc_actions &actions)
{
    uint64_t systemc_time = report.get_time().value();
    uint64_t gem5_time = curTick();

    std::cerr << report.get_time();

    if (gem5_time < systemc_time) {
        std::cerr << " (<) ";
    } else if (gem5_time > systemc_time) {
        std::cerr << " (!) ";
    } else {
        std::cerr << " (=) ";
    }

    std::cerr << ": " << report.get_msg_type()
              << ' ' << report.get_msg() << '\n';
}


int
sc_main(int argc, char **argv)
{
    sc_core::sc_report_handler::set_handler(reportHandler);

    SimControl sim_control("gem5", argc, argv);
    Target *memory;

    filename = "m5out/stats-tlm.txt";

    tlm::tlm_initiator_socket <> *mem_port =
        dynamic_cast<tlm::tlm_initiator_socket<> *>(
                    sc_core::sc_find_object("gem5.memory")
                );

    if (mem_port) {
        SC_REPORT_INFO("sc_main", "Port Found");
        unsigned long long int size = 512*1024*1024ULL;
        memory = new Target("memory",
                            sim_control.getDebugFlag(),
                            size,
                            sim_control.getOffset());

        memory->socket.bind(*mem_port);
    } else {
        SC_REPORT_FATAL("sc_main", "Port Not Found");
        std::exit(EXIT_FAILURE);
    }

    sc_core::sc_start();

    SC_REPORT_INFO("sc_main", "End of Simulation");

    CxxConfig::statsDump();

    return EXIT_SUCCESS;
}
