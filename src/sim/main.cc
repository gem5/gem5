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
 *
 * Authors: Steve Raasch
 *          Nathan Binkert
 *          Steve Reinhardt
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
#include <getopt.h>

#include <list>
#include <string>
#include <vector>

#include "base/callback.hh"
#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/pollevent.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/time.hh"
#include "cpu/base.hh"
#include "cpu/smt.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"
#include "sim/async.hh"
#include "sim/builder.hh"
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

/// Show brief help message.
void
showBriefHelp(ostream &out)
{
    char *prog = basename(myProgName);

    ccprintf(out, "Usage:\n");
    ccprintf(out,
"%s [-p <path>] [-i ] [-h] <config file>\n"
"\n"
" -p, --path <path>  prepends <path> to PYTHONPATH instead of using\n"
"                    built-in zip archive.  Useful when developing/debugging\n"
"                    changes to built-in Python libraries, as the new Python\n"
"                    can be tested without building a new m5 binary.\n\n"
" -i, --interactive  forces entry into interactive mode after the supplied\n"
"                    script is executed (just like the -i option to  the\n"
"                    Python interpreter).\n\n"
" -h                 Prints this help\n\n"
" <configfile>       config file name (ends in .py)\n\n",
             prog);

}

const char *briefCopyright =
"Copyright (c) 2001-2006\n"
"The Regents of The University of Michigan\n"
"All Rights Reserved\n";

/// Print welcome message.
void
sayHello(ostream &out)
{
    extern const char *compileDate;     // from date.cc

    ccprintf(out, "M5 Simulator System\n");
    // display copyright
    ccprintf(out, "%s\n", briefCopyright);
    ccprintf(out, "M5 compiled %d\n", compileDate);
    ccprintf(out, "M5 started %s\n", Time::start);

    char *host = getenv("HOSTNAME");
    if (!host)
        host = getenv("HOST");

    if (host)
        ccprintf(out, "M5 executing on %s\n", host);
}


extern "C" { void init_main(); }

int
main(int argc, char **argv)
{
    // Saze off program name
    myProgName = argv[0];

    sayHello(cerr);

    signal(SIGFPE, SIG_IGN);		// may occur on misspeculated paths
    signal(SIGTRAP, SIG_IGN);
    signal(SIGUSR1, dumpStatsHandler);		// dump intermediate stats
    signal(SIGUSR2, dumprstStatsHandler);	// dump and reset stats
    signal(SIGINT, exitNowHandler);		// dump final stats and exit
    signal(SIGABRT, abortHandler);

    Py_SetProgramName(argv[0]);

    // default path to m5 python code is the currently executing
    // file... Python ZipImporter will find embedded zip archive
    char *pythonpath = argv[0];

    bool interactive = false;
    bool show_help = false;
    bool getopt_done = false;
    int opt_index = 0;

    static struct option long_options[] = {
        {"python", 1, 0, 'p'},
        {"interactive", 0, 0, 'i'},
        {"help", 0, 0, 'h'},
        {0,0,0,0}
    };

    do {
        switch (getopt_long(argc, argv, "+p:ih", long_options, &opt_index)) {
            // -p <path> prepends <path> to PYTHONPATH instead of
            // using built-in zip archive.  Useful when
            // developing/debugging changes to built-in Python
            // libraries, as the new Python can be tested without
            // building a new m5 binary.
          case 'p':
            pythonpath = optarg;
            break;

            // -i forces entry into interactive mode after the
            // supplied script is executed (just like the -i option to
            // the Python interpreter).
          case 'i':
            interactive = true;
            break;

          case 'h':
            show_help = true;
            break;
          case -1:
            getopt_done = true;
            break;

          default:
            fatal("Unrecognized option %c\n", optopt);
        }
    } while (!getopt_done);

    if (show_help) {
        showBriefHelp(cerr);
        exit(1);
    }

    // Fix up argc & argv to hide arguments we just processed.
    // getopt() sets optind to the index of the first non-processed
    // argv element.
    argc -= optind;
    argv += optind;

    // Set up PYTHONPATH to make sure the m5 module is found
    string newpath(pythonpath);

    char *oldpath = getenv("PYTHONPATH");
    if (oldpath != NULL) {
        newpath += ":";
        newpath += oldpath;
    }

    if (setenv("PYTHONPATH", newpath.c_str(), true) == -1)
        fatal("setenv: %s\n", strerror(errno));

    // initialize embedded Python interpreter
    Py_Initialize();
    PySys_SetArgv(argc, argv);

    // initialize SWIG 'main' module
    init_main();

    if (argc > 0) {
        // extra arg(s): first is script file, remaining ones are args
        // to script file
        char *filename = argv[0];
        FILE *fp = fopen(filename, "r");
        if (!fp) {
            fatal("cannot open file '%s'\n", filename);
        }

        PyRun_AnyFile(fp, filename);
    } else {
        // no script file argument... force interactive prompt
        interactive = true;
    }

    if (interactive) {
        // The following code to import readline was copied from Python
        // 2.4.3's Modules/main.c.
        // Copyright (c) 2001, 2002, 2003, 2004, 2005, 2006
        // Python Software Foundation; All Rights Reserved
        // We should only enable this if we're actually using an
        // interactive prompt.
        PyObject *v;
        v = PyImport_ImportModule("readline");
        if (v == NULL)
            PyErr_Clear();
        else
            Py_DECREF(v);

        PyRun_InteractiveLoop(stdin, "stdin");
    }

    // clean up Python intepreter.
    Py_Finalize();
}

IniFile inifile;

SimObject *
createSimObject(const string &name)
{
    return SimObjectClass::createObject(inifile, name);
}


/**
 * Pointer to the Python function that maps names to SimObjects.
 */
PyObject *resolveFunc = NULL;

/**
 * Convert a pointer to the Python object that SWIG wraps around a C++
 * SimObject pointer back to the actual C++ pointer.  See main.i.
 */
extern "C" SimObject *convertSwigSimObjectPtr(PyObject *);


SimObject *
resolveSimObject(const string &name)
{
    PyObject *pyPtr = PyEval_CallFunction(resolveFunc, "(s)", name.c_str());
    if (pyPtr == NULL) {
        PyErr_Print();
        panic("resolveSimObject: failure on call to Python for %s", name);
    }

    SimObject *simObj = convertSwigSimObjectPtr(pyPtr);
    if (simObj == NULL)
        panic("resolveSimObject: failure on pointer conversion for %s", name);

    return simObj;
}


/**
 * Load config.ini into C++ database.  Exported to Python via SWIG;
 * invoked from m5.instantiate().
 */
void
loadIniFile(PyObject *_resolveFunc)
{
    resolveFunc = _resolveFunc;
    configStream = simout.find("config.out");

    // The configuration database is now complete; start processing it.
    inifile.load("config.ini");

    // Initialize statistics database
    Stats::InitSimStats();
}


/**
 * Look up a MemObject port.  Helper function for connectPorts().
 */
Port *
lookupPort(SimObject *so, const std::string &name, int i)
{
    MemObject *mo = dynamic_cast<MemObject *>(so);
    if (mo == NULL) {
        warn("error casting SimObject %s to MemObject", so->name());
        return NULL;
    }

    Port *p = mo->getPort(name, i);
    if (p == NULL)
        warn("error looking up port %s on object %s", name, so->name());
    return p;
}


/**
 * Connect the described MemObject ports.  Called from Python via SWIG.
 */
int
connectPorts(SimObject *o1, const std::string &name1, int i1,
             SimObject *o2, const std::string &name2, int i2)
{
    Port *p1 = lookupPort(o1, name1, i1);
    Port *p2 = lookupPort(o2, name2, i2);

    if (p1 == NULL || p2 == NULL) {
        warn("connectPorts: port lookup error");
        return 0;
    }

    p1->setPeer(p2);
    p2->setPeer(p1);

    return 1;
}

/**
 * Do final initialization steps after object construction but before
 * start of simulation.
 */
void
finalInit()
{
    // Parse and check all non-config-hierarchy parameters.
    ParamContext::parseAllContexts(inifile);
    ParamContext::checkAllContexts();

    // Echo all parameter settings to stats file as well.
    ParamContext::showAllContexts(*configStream);

    // Do a second pass to finish initializing the sim objects
    SimObject::initAll();

    // Restore checkpointed state, if any.
#if 0
    configHierarchy.unserializeSimObjects();
#endif

    SimObject::regAllStats();

    // uncomment the following to get PC-based execution-time profile
#ifdef DO_PROFILE
    init_profile((char *)&_init, (char *)&_fini);
#endif

    // Check to make sure that the stats package is properly initialized
    Stats::check();

    // Reset to put the stats in a consistent state.
    Stats::reset();

    SimStartup();
}


/** Simulate for num_cycles additional cycles.  If num_cycles is -1
 * (the default), do not limit simulation; some other event must
 * terminate the loop.  Exported to Python via SWIG.
 * @return The SimLoopExitEvent that caused the loop to exit.
 */
SimLoopExitEvent *
simulate(Tick num_cycles = -1)
{
    warn("Entering event queue @ %d.  Starting simulation...\n", curTick);

    // Fix up num_cycles.  Special default value -1 means simulate
    // "forever"... schedule event at MaxTick just to be safe.
    // Otherwise it's a delta for additional cycles to simulate past
    // curTick, and thus must be non-negative.
    if (num_cycles == -1)
        num_cycles = MaxTick;
    else if (num_cycles < 0)
        fatal("simulate: num_cycles must be >= 0 (was %d)\n", num_cycles);
    else
        num_cycles = curTick + num_cycles;

    Event *limit_event = new SimLoopExitEvent(num_cycles,
                                              "simulate() limit reached");

    while (1) {
        // there should always be at least one event (the SimLoopExitEvent
        // we just scheduled) in the queue
        assert(!mainEventQueue.empty());
        assert(curTick <= mainEventQueue.nextTick() &&
               "event scheduled in the past");

        // forward current cycle to the time of the first event on the
        // queue
        curTick = mainEventQueue.nextTick();
        Event *exit_event = mainEventQueue.serviceOne();
        if (exit_event != NULL) {
            // hit some kind of exit event; return to Python
            // event must be subclass of SimLoopExitEvent...
            SimLoopExitEvent *se_event = dynamic_cast<SimLoopExitEvent *>(exit_event);
            if (se_event == NULL)
                panic("Bogus exit event class!");

            // if we didn't hit limit_event, delete it
            if (se_event != limit_event) {
                assert(limit_event->scheduled());
                limit_event->deschedule();
                delete limit_event;
            }

            return se_event;
        }

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
                exitSimLoop("user interrupt received");
            }

            if (async_io || async_alarm) {
                async_io = false;
                async_alarm = false;
                pollQueue.service();
            }
        }
    }

    // not reached... only exit is return on SimLoopExitEvent
}

/**
 * Queue of C++ callbacks to invoke on simulator exit.
 */
CallbackQueue exitCallbacks;

/**
 * Register an exit callback.
 */
void
registerExitCallback(Callback *callback)
{
    exitCallbacks.add(callback);
}

/**
 * Do C++ simulator exit processing.  Exported to SWIG to be invoked
 * when simulator terminates via Python's atexit mechanism.
 */
void
doExitCleanup()
{
    exitCallbacks.process();
    exitCallbacks.clear();

    cout.flush();

    ParamContext::cleanupAllContexts();

    // print simulation stats
    Stats::DumpNow();
}
