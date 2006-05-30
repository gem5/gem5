/*
 * Copyright (c) 2000-2005 The Regents of The University of Michigan
 * All rights reserved.
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

///
/// @file sim/main.cc
///
#include <Python.h>	// must be before system headers... see Python docs

#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <libgen.h>
#include <stdlib.h>
#include <signal.h>

#include <list>
#include <string>
#include <vector>

#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/pollevent.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/time.hh"
#include "cpu/base.hh"
#include "cpu/smt.hh"
#include "sim/async.hh"
#include "sim/builder.hh"
#include "sim/configfile.hh"
#include "sim/host.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"
#include "sim/root.hh"

using namespace std;

// See async.h.
volatile bool async_event = false;
volatile bool async_dump = false;
volatile bool async_dumpreset = false;
volatile bool async_exit = false;
volatile bool async_io = false;
volatile bool async_alarm = false;

/// Stats signal handler.
void
dumpStatsHandler(int sigtype)
{
    async_event = true;
    async_dump = true;
}

void
dumprstStatsHandler(int sigtype)
{
    async_event = true;
    async_dumpreset = true;
}

/// Exit signal handler.
void
exitNowHandler(int sigtype)
{
    async_event = true;
    async_exit = true;
}

/// Abort signal handler.
void
abortHandler(int sigtype)
{
    cerr << "Program aborted at cycle " << curTick << endl;

#if TRACING_ON
    // dump trace buffer, if there is one
    Trace::theLog.dump(cerr);
#endif
}

/// Simulator executable name
char *myProgName = "";

///
/// Echo the command line for posterity in such a way that it can be
/// used to rerun the same simulation (given the same .ini files).
///
void
echoCommandLine(int argc, char **argv, ostream &out)
{
    out << "command line: " << argv[0];
    for (int i = 1; i < argc; i++) {
        string arg(argv[i]);

        out << ' ';

        // If the arg contains spaces, we need to quote it.
        // The rest of this is overkill to make it look purty.

        // print dashes first outside quotes
        int non_dash_pos = arg.find_first_not_of("-");
        out << arg.substr(0, non_dash_pos);	// print dashes
        string body = arg.substr(non_dash_pos);	// the rest

        // if it's an assignment, handle the lhs & rhs separately
        int eq_pos = body.find("=");
        if (eq_pos == string::npos) {
            out << quote(body);
        }
        else {
            string lhs(body.substr(0, eq_pos));
            string rhs(body.substr(eq_pos + 1));

            out << quote(lhs) << "=" << quote(rhs);
        }
    }
    out << endl << endl;
}

int
main(int argc, char **argv)
{
    // Save off program name
    myProgName = argv[0];

    signal(SIGFPE, SIG_IGN);		// may occur on misspeculated paths
    signal(SIGTRAP, SIG_IGN);
    signal(SIGUSR1, dumpStatsHandler);		// dump intermediate stats
    signal(SIGUSR2, dumprstStatsHandler);	// dump and reset stats
    signal(SIGINT, exitNowHandler);		// dump final stats and exit
    signal(SIGABRT, abortHandler);

    // Python embedded interpreter invocation
    Py_SetProgramName(argv[0]);
    const char *fileName = Py_GetProgramFullPath();
    Py_Initialize();
    PySys_SetArgv(argc, argv);

    // loadSwigModules();

    // Set Python module path to include current file to find embedded
    // zip archive
    if (PyRun_SimpleString("import sys") != 0)
        panic("Python error importing 'sys' module\n");
    string pathCmd = csprintf("sys.path[1:1] = ['%s']", fileName);
    if (PyRun_SimpleString(pathCmd.c_str()) != 0)
        panic("Python error setting sys.path\n");

    // Pass compile timestamp string to Python
    extern const char *compileDate;	// from date.cc
    string setCompileDate = csprintf("compileDate = '%s'", compileDate);
    if (PyRun_SimpleString(setCompileDate.c_str()) != 0)
        panic("Python error setting compileDate\n");

    // PyRun_InteractiveLoop(stdin, "stdin");
    // m5/__init__.py currently contains main argv parsing loop etc.,
    // and will write out config.ini file before returning.
    if (PyImport_ImportModule("defines") == NULL)
        panic("Python error importing 'defines.py'\n");
    if (PyImport_ImportModule("m5") == NULL)
        panic("Python error importing 'm5' module\n");
    Py_Finalize();

    configStream = simout.find("config.out");

    // The configuration database is now complete; start processing it.
    IniFile inifile;
    inifile.load("config.ini");

    // Initialize statistics database
    Stats::InitSimStats();

    // Now process the configuration hierarchy and create the SimObjects.
    ConfigHierarchy configHierarchy(inifile);
    configHierarchy.build();
    configHierarchy.createSimObjects();

    // Parse and check all non-config-hierarchy parameters.
    ParamContext::parseAllContexts(inifile);
    ParamContext::checkAllContexts();

    // Echo command line and all parameter settings to stats file as well.
    echoCommandLine(argc, argv, *outputStream);
    ParamContext::showAllContexts(*configStream);

    // Any objects that can't connect themselves until after construction should
    // do so now
    SimObject::connectAll();

    // Do a second pass to finish initializing the sim objects
    SimObject::initAll();

    // Restore checkpointed state, if any.
    configHierarchy.unserializeSimObjects();

    // Done processing the configuration database.
    // Check for unreferenced entries.
    if (inifile.printUnreferenced())
        panic("unreferenced sections/entries in the intermediate ini file");

    SimObject::regAllStats();

    // uncomment the following to get PC-based execution-time profile
#ifdef DO_PROFILE
    init_profile((char *)&_init, (char *)&_fini);
#endif

    // Check to make sure that the stats package is properly initialized
    Stats::check();

    // Reset to put the stats in a consistent state.
    Stats::reset();

    warn("Entering event queue.  Starting simulation...\n");
    SimStartup();
    while (!mainEventQueue.empty()) {
        assert(curTick <= mainEventQueue.nextTick() &&
               "event scheduled in the past");

        // forward current cycle to the time of the first event on the
        // queue
        curTick = mainEventQueue.nextTick();
        mainEventQueue.serviceOne();

        if (async_event) {
            async_event = false;
            if (async_dump) {
                async_dump = false;

                using namespace Stats;
                SetupEvent(Dump, curTick);
            }

            if (async_dumpreset) {
                async_dumpreset = false;

                using namespace Stats;
                SetupEvent(Dump | Reset, curTick);
            }

            if (async_exit) {
                async_exit = false;
                new SimExitEvent("User requested STOP");
            }

            if (async_io || async_alarm) {
                async_io = false;
                async_alarm = false;
                pollQueue.service();
            }
        }
    }

    // This should never happen... every conceivable way for the
    // simulation to terminate (hit max cycles/insts, signal,
    // simulated system halts/exits) generates an exit event, so we
    // should never run out of events on the queue.
    exitNow("no events on event loop!  All CPUs must be idle.", 1);

    return 0;
}
