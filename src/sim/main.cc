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
#include "config/pythonhome.hh"
#include "cpu/base.hh"
#include "cpu/smt.hh"
#include "mem/mem_object.hh"
#include "mem/port.hh"
#include "sim/async.hh"
#include "sim/builder.hh"
#include "sim/host.hh"
#include "sim/serialize.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"
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

extern "C" {
void init_main();
void init_debug();
}

int
main(int argc, char **argv)
{
    signal(SIGFPE, SIG_IGN);		// may occur on misspeculated paths
    signal(SIGTRAP, SIG_IGN);
    signal(SIGUSR1, dumpStatsHandler);		// dump intermediate stats
    signal(SIGUSR2, dumprstStatsHandler);	// dump and reset stats
    signal(SIGINT, exitNowHandler);		// dump final stats and exit
    signal(SIGABRT, abortHandler);

    Py_SetProgramName(argv[0]);

    // default path to m5 python code is the currently executing
    // file... Python ZipImporter will find embedded zip archive.
    // The M5_ARCHIVE environment variable can be used to override this.
    char *m5_archive = getenv("M5_ARCHIVE");
    string pythonpath = m5_archive ? m5_archive : argv[0];

    char *oldpath = getenv("PYTHONPATH");
    if (oldpath != NULL) {
        pythonpath += ":";
        pythonpath += oldpath;
    }

    if (setenv("PYTHONPATH", pythonpath.c_str(), true) == -1)
        fatal("setenv: %s\n", strerror(errno));

    char *python_home = getenv("PYTHONHOME");
    if (!python_home)
        python_home = PYTHONHOME;
    Py_SetPythonHome(python_home);

    // initialize embedded Python interpreter
    Py_Initialize();
    PySys_SetArgv(argc, argv);

    // initialize SWIG modules
    init_main();
    init_debug();

    PyRun_SimpleString("import m5.main");
    PyRun_SimpleString("m5.main.main()");

    // clean up Python intepreter.
    Py_Finalize();
}


void
setOutputDir(const string &dir)
{
    simout.setDirectory(dir);
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
    inifile.load(simout.resolve("config.ini"));

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
simulate(Tick num_cycles = MaxTick)
{
    warn("Entering event queue @ %d.  Starting simulation...\n", curTick);

    if (num_cycles < 0)
        fatal("simulate: num_cycles must be >= 0 (was %d)\n", num_cycles);
    else if (curTick + num_cycles < 0)  //Overflow
        num_cycles = MaxTick;
    else
        num_cycles = curTick + num_cycles;

    Event *limit_event = schedExitSimLoop("simulate() limit reached",
                                          num_cycles);

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

Event *
createCountedDrain()
{
    return new CountedDrainEvent();
}

void
cleanupCountedDrain(Event *counted_drain)
{
    CountedDrainEvent *event =
        dynamic_cast<CountedDrainEvent *>(counted_drain);
    if (event == NULL) {
        fatal("Called cleanupCountedDrain() on an event that was not "
              "a CountedDrainEvent.");
    }
    assert(event->getCount() == 0);
    delete event;
}

void
serializeAll(const std::string &cpt_dir)
{
    Serializable::serializeAll(cpt_dir);
}

void
unserializeAll(const std::string &cpt_dir)
{
    Serializable::unserializeAll(cpt_dir);
}

/**
 * Queue of C++ callbacks to invoke on simulator exit.
 */
CallbackQueue&
exitCallbacks()
{
    static CallbackQueue theQueue;
    return theQueue;
}

/**
 * Register an exit callback.
 */
void
registerExitCallback(Callback *callback)
{
    exitCallbacks().add(callback);
}

BaseCPU *
convertToBaseCPUPtr(SimObject *obj)
{
    BaseCPU *ptr = dynamic_cast<BaseCPU *>(obj);

    if (ptr == NULL)
        warn("Casting to BaseCPU pointer failed");
    return ptr;
}

System *
convertToSystemPtr(SimObject *obj)
{
    System *ptr = dynamic_cast<System *>(obj);

    if (ptr == NULL)
        warn("Casting to System pointer failed");
    return ptr;
}


/**
 * Do C++ simulator exit processing.  Exported to SWIG to be invoked
 * when simulator terminates via Python's atexit mechanism.
 */
void
doExitCleanup()
{
    exitCallbacks().process();
    exitCallbacks().clear();

    cout.flush();

    ParamContext::cleanupAllContexts();

    // print simulation stats
    Stats::DumpNow();
}
