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
 *  This file contains a demonstration main using CxxConfigManager.
 *  Build with something like:
 *
 *      scons --without-python build/ARM/libgem5_opt.so
 *
 *      g++ -DTRACING_ON -std=c++0x -Ibuild/ARM src/sim/cxx_main.cc \
 *          -o gem5cxx.opt -Lbuild/ARM -lgem5_opt
 */

#include <cstdlib>
#include <iostream>
#include <sstream>

#include "base/inifile.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/trace.hh"
#include "cpu/base.hh"
#include "sim/cxx_config_ini.hh"
#include "sim/cxx_manager.hh"
#include "sim/init_signals.hh"
#include "sim/serialize.hh"
#include "sim/sim_events.hh"
#include "sim/simulate.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"
#include "stats.hh"

using namespace gem5;

void
usage(const std::string &prog_name)
{
    std::cerr << "Usage: " << prog_name << (
        " <config-file.ini> [ <option> ]\n\n"
        "OPTIONS:\n"
        "    -p <object> <param> <value>  -- set a parameter\n"
        "    -v <object> <param> <values> -- set a vector parameter from"
        " a comma\n"
        "                                    separated values string\n"
        "    -d <flag>                    -- set a debug flag (-<flag>\n"
        "                                    clear a flag)\n"
        "    -s <dir> <ticks>             -- save checkpoint to dir after"
        " the given\n"
        "                                    number of ticks\n"
        "    -r <dir>                     -- restore checkpoint from dir\n"
        "    -c <from> <to> <ticks>       -- switch from cpu 'from' to cpu"
        " 'to' after\n"
        "                                    the given number of ticks\n"
        "\n"
        );

    std::exit(EXIT_FAILURE);
}

int
main(int argc, char **argv)
{
    std::string prog_name(argv[0]);
    unsigned int arg_ptr = 1;

    if (argc == 1)
        usage(prog_name);

    initSignals();

    setClockFrequency(1000000000000);
    fixClockFrequency();
    curEventQueue(getEventQueue(0));

    statistics::initSimStats();
    statistics::registerHandlers(CxxConfig::statsReset, CxxConfig::statsDump);

    Trace::enable();
    setDebugFlag("Terminal");
    // setDebugFlag("CxxConfig");

    const std::string config_file(argv[arg_ptr]);

    CxxConfigFileBase *conf = new CxxIniFile();

    if (!conf->load(config_file.c_str())) {
        std::cerr << "Can't open config file: " << config_file << '\n';
        return EXIT_FAILURE;
    }
    arg_ptr++;

    CxxConfigManager *config_manager = new CxxConfigManager(*conf);

    bool checkpoint_restore = false;
    bool checkpoint_save = false;
    bool switch_cpus = false;
    std::string checkpoint_dir = "";
    std::string from_cpu = "";
    std::string to_cpu = "";
    Tick pre_run_time = 1000000;
    Tick pre_switch_time = 1000000;

    try {
        while (arg_ptr < argc) {
            std::string option(argv[arg_ptr]);
            arg_ptr++;
            unsigned num_args = argc - arg_ptr;

            if (option == "-p") {
                if (num_args < 3)
                    usage(prog_name);
                config_manager->setParam(argv[arg_ptr], argv[arg_ptr + 1],
                    argv[arg_ptr + 2]);
                arg_ptr += 3;
            } else if (option == "-v") {
                std::vector<std::string> values;

                if (num_args < 3)
                    usage(prog_name);
                tokenize(values, argv[arg_ptr + 2], ',');
                config_manager->setParamVector(argv[arg_ptr],
                    argv[arg_ptr + 1], values);
                arg_ptr += 3;
            } else if (option == "-d") {
                if (num_args < 1)
                    usage(prog_name);
                if (argv[arg_ptr][0] == '-')
                    clearDebugFlag(argv[arg_ptr] + 1);
                else
                    setDebugFlag(argv[arg_ptr]);
                arg_ptr++;
            } else if (option == "-r") {
                if (num_args < 1)
                    usage(prog_name);
                checkpoint_dir = argv[arg_ptr];
                checkpoint_restore = true;
                arg_ptr++;
            } else if (option == "-s") {
                if (num_args < 2)
                    usage(prog_name);
                checkpoint_dir = argv[arg_ptr];
                std::istringstream(argv[arg_ptr + 1]) >> pre_run_time;
                checkpoint_save = true;
                arg_ptr += 2;
            } else if (option == "-c") {
                if (num_args < 3)
                    usage(prog_name);
                switch_cpus = true;
                from_cpu = argv[arg_ptr];
                to_cpu = argv[arg_ptr + 1];
                std::istringstream(argv[arg_ptr + 2]) >> pre_switch_time;
                arg_ptr += 3;
            } else {
                usage(prog_name);
            }
        }
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << e.name << ": " << e.message << "\n";
        return EXIT_FAILURE;
    }

    if (checkpoint_save && checkpoint_restore) {
        std::cerr << "Don't try and save and restore a checkpoint in the"
            " same run\n";
        return EXIT_FAILURE;
    }

    CxxConfig::statsEnable();
    getEventQueue(0)->dump();

    try {
        config_manager->instantiate();
        if (!checkpoint_restore) {
            config_manager->initState();
            config_manager->startup();
        }
    } catch (CxxConfigManager::Exception &e) {
        std::cerr << "Config problem in sim object " << e.name
            << ": " << e.message << "\n";

        return EXIT_FAILURE;
    }

    GlobalSimLoopExitEvent *exit_event = NULL;

    if (checkpoint_save) {
        exit_event = simulate(pre_run_time);

        unsigned int drain_count = 1;
        do {
            drain_count = config_manager->drain();

            std::cerr << "Draining " << drain_count << '\n';

            if (drain_count > 0) {
                exit_event = simulate();
            }
        } while (drain_count > 0);

        std::cerr << "Simulation stop at tick " << curTick()
            << ", cause: " << exit_event->getCause() << '\n';

        std::cerr << "Checkpointing\n";

        /* FIXME, this should really be serialising just for
         *  config_manager rather than using serializeAll's ugly
         *  SimObject static object list */
        SimObject::serializeAll(checkpoint_dir);

        std::cerr << "Completed checkpoint\n";

        config_manager->drainResume();
    }

    if (checkpoint_restore) {
        std::cerr << "Restoring checkpoint\n";

        SimObject::setSimObjectResolver(
            &config_manager->getSimObjectResolver());
        CheckpointIn *checkpoint = new CheckpointIn(checkpoint_dir);

        DrainManager::instance().preCheckpointRestore();
        config_manager->loadState(*checkpoint);
        config_manager->startup();

        config_manager->drainResume();

        std::cerr << "Restored from checkpoint\n";
    }

    if (switch_cpus) {
        exit_event = simulate(pre_switch_time);

        std::cerr << "Switching CPU\n";

        /* Assume the system is called system */
        System &system = config_manager->getObject<System>("system");
        BaseCPU &old_cpu = config_manager->getObject<BaseCPU>(from_cpu);
        BaseCPU &new_cpu = config_manager->getObject<BaseCPU>(to_cpu);

        unsigned int drain_count = 1;
        do {
            drain_count = config_manager->drain();

            std::cerr << "Draining " << drain_count << '\n';

            if (drain_count > 0) {
                exit_event = simulate();
            }
        } while (drain_count > 0);

        old_cpu.switchOut();
        system.setMemoryMode(enums::timing);
        new_cpu.takeOverFrom(&old_cpu);
        config_manager->drainResume();

        std::cerr << "Switched CPU\n";
    }

    exit_event = simulate();

    std::cerr << "Exit at tick " << curTick()
        << ", cause: " << exit_event->getCause() << '\n';

    getEventQueue(0)->dump();

#if TRY_CLEAN_DELETE
    config_manager->deleteObjects();
#endif

    delete config_manager;

    return EXIT_SUCCESS;
}
