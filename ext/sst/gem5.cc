// Copyright (c) 2015 ARM Limited
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

#include <sst_config.h>
#include <Python.h>  // Before serialization to prevent spurious warnings
#include <sst/core/serialization.h>

#include "gem5.hh"

// System headers
#include <boost/tokenizer.hpp>
#include <string>

// gem5 Headers
#include <sim/core.hh>
#include <sim/init.hh>
#include <sim/init_signals.hh>
#include <sim/system.hh>
#include <sim/sim_object.hh>
#include <base/misc.hh>
#include <base/debug.hh>

#ifdef fatal  // gem5 sets this
#undef fatal
#endif

// More SST Headers
#include <sst/core/params.h>
#include <sst/core/link.h>
#include <sst/core/timeConverter.h>
#include <sst/core/debug.h>

using namespace SST;
using namespace SST::gem5;

gem5Component::gem5Component(ComponentId_t id, Params &params) :
    SST::Component(id)
{
    dbg.init("@t:gem5:@p():@l " + getName() + ": ", 0, 0,
            (Output::output_location_t)params.find_integer("comp_debug", 0));
    info.init("gem5:" + getName() + ": ", 0, 0, Output::STDOUT);

    TimeConverter *clock = registerClock(
            params.find_string("frequency", "1GHz"),
            new Clock::Handler<gem5Component>(this, &gem5Component::clockTick));

    // This sets how many gem5 cycles we'll need to simulate per clock tick
    sim_cycles = clock->getFactor();

    // Disable gem5's inform() messages.
    want_info = false;

    std::string cmd = params.find_string("cmd", "");
    if (cmd.empty()) {
        _abort(gem5Component, "Component %s must have a 'cmd' parameter.\n",
               getName().c_str());
    }

    std::vector<char*> args;
    args.push_back(const_cast<char*>("sst.x")); // TODO: Compute this somehow?
    splitCommandArgs(cmd, args);
    args.push_back(const_cast<char*>("--initialize-only"));
    dbg.output(CALL_INFO, "Command string:  [sst.x %s --initialize-only]\n",
               cmd.c_str());
    for (size_t i = 0; i < args.size(); ++i) {
        dbg.output(CALL_INFO, "  Arg [%02zu] = %s\n", i, args[i]);
    }

    std::vector<char*> flags;
    std::string gem5DbgFlags = params.find_string("gem5DebugFlags", "");
    splitCommandArgs(gem5DbgFlags, flags);
    for (auto flag : flags) {
        dbg.output(CALL_INFO, "  Setting Debug Flag [%s]\n", flag);
        setDebugFlag(flag);
    }

    ExternalMaster::registerHandler("sst", this); // these are idempotent
    ExternalSlave ::registerHandler("sst", this);

    // Initialize m5 special signal handling.
    initSignals();

    initPython(args.size(), &args[0]);

    // tell the simulator not to end without us
    registerAsPrimaryComponent();
    primaryComponentDoNotEndSim();

    clocks_processed = 0;
}

gem5Component::~gem5Component(void)
{
    Py_Finalize();
}

void
gem5Component::init(unsigned phase)
{
    for (auto m : masters) {
        m->init(phase);
    }
    for (auto s : slaves) {
        s->init(phase);
    }
}

void
gem5Component::setup(void)
{
    // Switch connectors from initData to regular Sends
    for (auto m : masters) {
        m->setup();
    }
    for (auto s : slaves) {
        s->setup();
    }
}

void
gem5Component::finish(void)
{
    for (auto m : masters) {
        m->finish();
    }
    info.output("Complete. Clocks Processed: %"PRIu64"\n", clocks_processed);
}

bool
gem5Component::clockTick(Cycle_t cycle)
{
    dbg.output(CALL_INFO, "Cycle %lu\n", cycle);

    for (auto m : masters) {
        m->clock();
    }

    GlobalSimLoopExitEvent *event = simulate(sim_cycles);
    ++clocks_processed;
    if (event != simulate_limit_event) {
        info.output("exiting: curTick()=%lu cause=`%s` code=%d\n",
                curTick(), event->getCause().c_str(), event->getCode());
        primaryComponentOKToEndSim();
        return true;
    }

    return false;
}


void
gem5Component::splitCommandArgs(std::string &cmd,
                                std::vector<char *> &args)
{
    std::string sep1("\\");
    std::string sep2(" ");
    std::string sep3("\"\'");

    boost::escaped_list_separator<char> els(sep1, sep2, sep3);
    boost::tokenizer<boost::escaped_list_separator<char>> tok(cmd, els);

    for (auto beg : tok) {
        args.push_back(strdup(beg.c_str()));
    }
}


void
gem5Component::initPython(int argc, char *argv[])
{
    const char * m5MainCommands[] = {
        "import m5",
        "m5.main()",
        0 // sentinel is required
    };

    PyObject *mainModule,*mainDict;

    Py_SetProgramName(argv[0]); // optional but recommended

    Py_Initialize();

    int ret = initM5Python();
    if (ret != 0) {
        _abort(gem5Component, "Python failed to initialize. Code: %d\n", ret);
    }

    PySys_SetArgv(argc, argv);

    mainModule = PyImport_AddModule("__main__");
    assert(mainModule);

    mainDict = PyModule_GetDict(mainModule);
    assert(mainDict);

    PyObject *result;
    const char **command = m5MainCommands;

    // evaluate each command in the m5MainCommands array (basically a
    // bunch of python statements.
    while (*command) {
        result = PyRun_String(*command, Py_file_input, mainDict, mainDict);
        if (!result) {
            PyErr_Print();
            break;
        }
        Py_DECREF(result);

        command++;
    }
}

ExternalMaster::Port*
gem5Component::getExternalPort(const std::string &name,
    ExternalMaster &owner, const std::string &port_data)
{
    std::string s(name); // bridges non-& result and &-arg
    auto master = new ExtMaster(this, info, owner, s);
    masters.push_back(master);
    return master;
}

ExternalSlave::Port*
gem5Component::getExternalPort(const std::string &name,
    ExternalSlave &owner, const std::string &port_data)
{
    std::string s(name); // bridges non-& result and &-arg
    auto slave = new ExtSlave(this, info, owner, s);
    slaves.push_back(slave);
    return slave;
}
