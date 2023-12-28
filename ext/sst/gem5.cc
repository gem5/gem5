// Copyright (c) 2021-2023 The Regents of the University of California
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Copyright (c) 2015-2016 ARM Limited
// All rights reserved.
//
// The license below extends only to copyright in the software and shall
// not be construed as granting a license to any other intellectual
// property including but not limited to intellectual property relating
// to a hardware implementation of the functionality of the software
// licensed hereunder.  You may use the software subject to the license
// terms below provided that you ensure that this notice is replicated
// unmodified and in its entirety in all distributions of the software,
// modified or unmodified, in source code or in binary form.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met: redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer;
// redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution;
// neither the name of the copyright holders nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Copyright 2009-2014 Sandia Coporation.  Under the terms
// of Contract DE-AC04-94AL85000 with Sandia Corporation, the U.S.
// Government retains certain rights in this software.
//
// Copyright (c) 2009-2014, Sandia Corporation
// All rights reserved.
//
// For license information, see the LICENSE file in the current directory.

#include <sst/core/sst_config.h>
#include <sst/core/componentInfo.h>
#include <sst/elements/memHierarchy/memEvent.h>
#include <sst/elements/memHierarchy/memTypes.h>
#include <sst/elements/memHierarchy/util.h>

#include <Python.h>  // Before serialization to prevent spurious warnings

#include "gem5.hh"

#include "util.hh"

// System headers
#include <algorithm>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <pybind11/embed.h>
#include <pybind11/pybind11.h>

// gem5 Headers
#include <sim/core.hh>
#include <sim/init.hh>
#include <sim/init_signals.hh>
#include <sim/root.hh>
#include <sim/system.hh>
#include <sim/sim_events.hh>
#include <sim/sim_object.hh>
#include <base/logging.hh>
#include <base/debug.hh>

#include <base/pollevent.hh>
#include <base/types.hh>
#include <sim/async.hh>
#include <sim/eventq.hh>
#include <sim/sim_exit.hh>
#include <sim/stat_control.hh>

#include <sst/outgoing_request_bridge.hh>

#include <cassert>

#ifdef fatal  // gem5 sets this
#undef fatal
#endif

// More SST Headers
#include <core/timeConverter.h>

namespace py = pybind11;

gem5Component::gem5Component(SST::ComponentId_t id, SST::Params& params):
    SST::Component(id), threadInitialized(false)
{
    output.init("gem5Component-" + getName() + "->", 1, 0,
                SST::Output::STDOUT);

    std::string cpu_frequency = params.find<std::string>("frequency", "");
    if (cpu_frequency.empty()) {
        output.fatal(
            CALL_INFO, -1, "The frequency of the CPU must be specified.\n"
        );
    }

    // Register a handler to be called on a set frequency.
    timeConverter = registerClock(
        cpu_frequency,
        new SST::Clock::Handler<gem5Component>(this, &gem5Component::clockTick)
    );

    // "cmd" -> gem5's Python
    std::string cmd = params.find<std::string>("cmd", "");
    if (cmd.empty()) {
        output.fatal(
            CALL_INFO, -1, "Component %s must have a 'cmd' parameter.\n",
            getName().c_str()
        );
    }

    // Telling SST the command line call to gem5
    args.push_back(const_cast<char*>("sst.x"));
    splitCommandArgs(cmd, args);
    output.output(CALL_INFO, "Command string:  [sst.x %s]\n", cmd.c_str());
    for (size_t i = 0; i < args.size(); ++i) {
        output.output(CALL_INFO, "  Arg [%02zu] = %s\n", i, args[i]);
    }

    // Parsing and setting gem5 debug flags
    std::string gem5_debug_flags = params.find<std::string>("debug_flags", "");
    for (auto const debug_flag: tokenizeString(gem5_debug_flags, {' ', ','})) {
        output.output(CALL_INFO, "Debug flag += %s\n", debug_flag.c_str());
        gem5::setDebugFlag(debug_flag.c_str());
    }

    registerAsPrimaryComponent();
    primaryComponentDoNotEndSim();

    // We need to add another parameter when invoking gem5 scripts from SST to
    // keep a track of all the OutgoingBridges. This will allow to add or
    // remove OutgoingBridges from gem5 configs without the need to recompile
    // the ext/sst source everytime.
    std::string ports = params.find<std::string>("ports", "");
    if (ports.empty()) {
        output.fatal(
            CALL_INFO, -1, "Component %s must have a 'ports' parameter.\n",
            getName().c_str()
        );
    }
    // Split the port names using the util method defined.
    splitPortNames(ports);
    for (int i = 0 ; i < sstPortCount ; i++) {
        std::cout << sstPortNames[i] << std::endl;
        sstPorts.push_back(
            loadUserSubComponent<SSTResponderSubComponent>(sstPortNames[i], 0)
        );
        // If the name defined in the `ports` is incorrect, then the program
        // will crash when calling `setTimeConverter`.
        sstPorts[i]->setTimeConverter(timeConverter);
        sstPorts[i]->setOutputStream(&(output));
    }
}

gem5Component::~gem5Component()
{
}

void
gem5Component::init(unsigned phase)
{
    output.output(CALL_INFO," init phase: %u\n", phase);

    if (phase == 0) {
        initPython(args.size(), &args[0]);
        // m5.instantiate() was moved to the gem5 script.
        // calling SimObject.startup()
        const std::vector<std::string> simobject_setup_commands = {
            "import atexit",
            "import _m5.core",
            "import m5",
            "import m5.stats",
            "import m5.objects.Root",
            "root = m5.objects.Root.getInstance()",
            "for obj in root.descendants(): obj.startup()",
            "atexit.register(m5.stats.dump)",
            "atexit.register(_m5.core.doExitCleanup)",
            "m5.stats.reset()"
        };
        execPythonCommands(simobject_setup_commands);

        // find the corresponding SimObject for each SSTResponderSubComponent
        gem5::Root* gem5_root = gem5::Root::root();
        for (auto &port : sstPorts) {
            port->findCorrespondingSimObject(gem5_root);
        }

        // initialize the gem5 event queue
        if (!(threadInitialized)) {
            threadInitialized = true;
            gem5::simulate_limit_event = new gem5::GlobalSimLoopExitEvent(
                gem5::mainEventQueue[0]->getCurTick(),
                "simulate() limit reached",
                0
            );
        }

    }
    for (auto &port : sstPorts) {
        port->init(phase);
    }
}

void
gem5Component::setup()
{
    output.verbose(CALL_INFO, 1, 0, "Component is being setup.\n");
    for (auto &port : sstPorts) {
        port->setup();
    }
}

void
gem5Component::finish()
{
    output.verbose(CALL_INFO, 1, 0, "Component is being finished.\n");
}

bool
gem5Component::clockTick(SST::Cycle_t currentCycle)
{
    // what to do in a SST's cycle
    gem5::GlobalSimLoopExitEvent *event = simulateGem5(currentCycle);
    clocksProcessed++;
    // gem5 exits due to reasons other than reaching simulation limit
    if (event != gem5::simulate_limit_event) {
        output.output("exiting: curTick()=%lu cause=`%s` code=%d\n",
            gem5::curTick(), event->getCause().c_str(), event->getCode()
        );
        // output gem5 stats
        const std::vector<std::string> output_stats_commands = {
            "import m5.stats",
            "m5.stats.dump()"
        };
        execPythonCommands(output_stats_commands);

        primaryComponentOKToEndSim();
        return true;
    }

    // returning False means the simulation should go on
    return false;

}

#define PyCC(x) (const_cast<char *>(x))

gem5::GlobalSimLoopExitEvent*
gem5Component::simulateGem5(uint64_t current_cycle)
{
    // This function should be similar to simulate() of src/sim/simulate.cc
    // with synchronization barriers removed.

    inform_once("Entering event queue @ %d.  Starting simulation...\n",
                gem5::curTick());

    // Tick conversion
    // The main logic for synchronize SST Tick and gem5 Tick is here.
    // next_end_tick = current_cycle * timeConverter->getFactor()
    uint64_t next_end_tick = \
        timeConverter->convertToCoreTime(current_cycle);

    // Here, if the next event in gem5's queue is not executed within the next
    // cycle, there's no need to enter the gem5's sim loop.
    if (gem5::mainEventQueue[0]->empty() ||
        next_end_tick < gem5::mainEventQueue[0]->getHead()->when()) {
        return gem5::simulate_limit_event;
    }
    gem5::simulate_limit_event->reschedule(next_end_tick);
    gem5::Event *local_event = doSimLoop(gem5::mainEventQueue[0]);
    gem5::BaseGlobalEvent *global_event = local_event->globalEvent();
    gem5::GlobalSimLoopExitEvent *global_exit_event =
        dynamic_cast<gem5::GlobalSimLoopExitEvent *>(global_event);
    return global_exit_event;
}

gem5::Event*
gem5Component::doSimLoop(gem5::EventQueue* eventq)
{
    // This function should be similar to doSimLoop() in src/sim/simulate.cc
    // with synchronization barriers removed.
    gem5::curEventQueue(eventq);
    eventq->handleAsyncInsertions();

    while (true)
    {
        // there should always be at least one event (the SimLoopExitEvent
        // we just scheduled) in the queue

        assert(!eventq->empty());
        assert(gem5::curTick() <= eventq->nextTick() &&
               "event scheduled in the past");

        if (gem5::async_event) {
            // Take the event queue lock in case any of the service
            // routines want to schedule new events.
            if (gem5::async_statdump || gem5::async_statreset) {
                gem5::statistics::schedStatEvent(gem5::async_statdump,
                                                 gem5::async_statreset);
                gem5::async_statdump = false;
                gem5::async_statreset = false;
            }

            if (gem5::async_io) {
                gem5::async_io = false;
                gem5::pollQueue.service();
            }

            if (gem5::async_exit) {
                gem5::async_exit = false;
                gem5::exitSimLoop("user interrupt received");
            }

            if (gem5::async_exception) {
                gem5::async_exception = false;
                return NULL;
            }
        }

        gem5::Event *exit_event = eventq->serviceOne();
        if (exit_event != NULL) {
            return exit_event;
        }
    }
}

int
gem5Component::execPythonCommands(const std::vector<std::string>& commands)
{
    static PyObject *dict =
        py::module_::import("__main__").attr("__dict__").ptr();

    PyObject *result;

    for (auto const command: commands) {
        result = PyRun_String(command.c_str(), Py_file_input, dict, dict);
        if (!result) {
            PyErr_Print();
            return 1;
        }
        Py_DECREF(result);
    }
    return 0;
}

void
gem5Component::initPython(int argc, char *_argv[])
{
    // Initialize gem5 special signal handling.
    gem5::initSignals();

    if (!Py_IsInitialized()) {
        py::initialize_interpreter(true, argc, _argv);
    } else {
        // pybind doesn't provide a way to set sys.argv if not initializing the
        // interpreter, so we have to do that manually if it's already running.
        py::list py_argv;
        auto sys = py::module::import("sys");
        if (py::hasattr(sys, "argv")) {
            // sys.argv already exists, so grab that.
            py_argv = sys.attr("argv");
        } else {
            // sys.argv doesn't exist, so create it.
            sys.add_object("argv", py_argv);
        }
        // Clear out argv just in case it has something in it.
        py_argv.attr("clear")();

        // Fill it with our argvs.
        for (int i = 0; i < argc; i++)
            py_argv.append(_argv[i]);
    }

    auto importer = py::module_::import("importer");
    importer.attr("install")();

    try {
        py::module_::import("m5").attr("main")();
    } catch (py::error_already_set &e) {
        if (!e.matches(PyExc_SystemExit)) {
            std::cerr << e.what();
            output.output(CALL_INFO, "Calling m5.main(...) failed.\n");
        }
    }
}

void
gem5Component::splitCommandArgs(std::string &cmd, std::vector<char*> &args)
{
    std::vector<std::string> parsed_args = tokenizeString(
        cmd, {'\\', ' ', '\'', '\"'}
    );

    for (auto part: parsed_args)
        args.push_back(strdup(part.c_str()));
}

void
gem5Component::splitPortNames(std::string port_names)
{
    std::vector<std::string> parsed_args = tokenizeString(
        port_names, {'\\', ' ', '\'', '\"'}
    );
    sstPortCount = 0;
    for (auto part: parsed_args) {
        sstPortNames.push_back(strdup(part.c_str()));
        sstPortCount++;
    }
}
