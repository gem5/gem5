/*
 * Copyright (c) 2000-2004 The Regents of The University of Michigan
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
#include <sys/types.h>
#include <sys/stat.h>
#include <libgen.h>
#include <stdlib.h>
#include <signal.h>

#include <list>
#include <string>
#include <vector>

#include "base/copyright.hh"
#include "base/embedfile.hh"
#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/output.hh"
#include "base/pollevent.hh"
#include "base/statistics.hh"
#include "base/str.hh"
#include "base/time.hh"
#include "cpu/base_cpu.hh"
#include "cpu/full_cpu/smt.hh"
#include "sim/async.hh"
#include "sim/builder.hh"
#include "sim/configfile.hh"
#include "sim/host.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_object.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"
#include "sim/universe.hh"
#include "sim/pyconfig/pyconfig.hh"

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
"%s [-d <dir>] [-E <var>[=<val>]] [-I <dir>] [-P <python>]\n"
"        [--<var>=<val>] <config file>\n"
"\n"
"   -d            set the output directory to <dir>\n"
"   -E            set the environment variable <var> to <val> (or 'True')\n"
"   -I            add the directory <dir> to python's path\n"
"   -P            execute <python> directly in the configuration\n"
"   --var=val     set the python variable <var> to '<val>'\n"
"   <configfile>  config file name (.py or .mpy)\n",
             prog);

    ccprintf(out, "%s -X\n    -X            extract embedded files\n", prog);
    ccprintf(out, "%s -h\n    -h            print long help\n", prog);
}

/// Show verbose help message.  Includes parameter listing from
/// showBriefHelp(), plus an exhaustive list of ini-file parameters
/// and SimObjects (with their parameters).
void
showLongHelp(ostream &out)
{
    showBriefHelp(out);

    out << endl
        << endl
        << "-----------------" << endl
        << "Global Parameters" << endl
        << "-----------------" << endl
        << endl;

    ParamContext::describeAllContexts(out);

    out << endl
        << endl
        << "-----------------" << endl
        << "Simulator Objects" << endl
        << "-----------------" << endl
        << endl;

    SimObjectClass::describeAllClasses(out);
}

/// Print welcome message.
void
sayHello(ostream &out)
{
    extern const char *compileDate;	// from date.cc

    ccprintf(out, "M5 Simulator System\n");
    // display copyright
    ccprintf(out, "%s\n", briefCopyright);
    ccprintf(out, "M5 compiled on %d\n", compileDate);

    char *host = getenv("HOSTNAME");
    if (!host)
        host = getenv("HOST");

    if (host)
        ccprintf(out, "M5 executing on %s\n", host);

    ccprintf(out, "M5 simulation started %s\n", Time::start);
}

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

char *
getOptionString(int &index, int argc, char **argv)
{
    char *option = argv[index] + 2;
    if (*option != '\0')
        return option;

    // We didn't find an argument, it must be in the next variable.
    if (++index >= argc)
        panic("option string for option '%s' not found", argv[index - 1]);

    return argv[index];
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

    sayHello(cerr);

    bool configfile_found = false;
    PythonConfig pyconfig;
    string outdir;

    // Parse command-line options.
    // Since most of the complex options are handled through the
    // config database, we don't mess with getopts, and just parse
    // manually.
    for (int i = 1; i < argc; ++i) {
        char *arg_str = argv[i];

        // if arg starts with '--', parse as a special python option
        // of the format --<python var>=<string value>, if the arg
        // starts with '-', it should be a simulator option with a
        // format similar to getopt.  In any other case, treat the
        // option as a configuration file name and load it.
        if (arg_str[0] == '-' && arg_str[1] == '-') {
            string str = &arg_str[2];
            string var, val;

            if (!split_first(str, var, val, '='))
                panic("Could not parse configuration argument '%s'\n"
                      "Expecting --<variable>=<value>\n", arg_str);

            pyconfig.setVariable(var, val);
        } else if (arg_str[0] == '-') {
            char *option;
            string var, val;

            // switch on second char
            switch (arg_str[1]) {
              case 'd':
                outdir = getOptionString(i, argc, argv);
                break;

              case 'h':
                showLongHelp(cerr);
                exit(1);

              case 'E':
                option = getOptionString(i, argc, argv);
                if (!split_first(option, var, val, '='))
                    val = "True";

                if (setenv(var.c_str(), val.c_str(), true) == -1)
                    panic("setenv: %s\n", strerror(errno));
                break;

              case 'I':
                option = getOptionString(i, argc, argv);
                pyconfig.addPath(option);
                break;

              case 'P':
                option = getOptionString(i, argc, argv);
                pyconfig.writeLine(option);
                break;

              case 'X': {
                  list<EmbedFile> lst;
                  EmbedMap::all(lst);
                  list<EmbedFile>::iterator i = lst.begin();
                  list<EmbedFile>::iterator end = lst.end();

                  while (i != end) {
                      cprintf("Embedded File: %s\n", i->name);
                      cout.write(i->data, i->length);
                      ++i;
                  }

                  return 0;
              }

              default:
                showBriefHelp(cerr);
                panic("invalid argument '%s'\n", arg_str);
            }
        } else {
            string file(arg_str);
            string base, ext;

            if (!split_last(file, base, ext, '.') ||
                ext != "py" && ext != "mpy")
                panic("Config file '%s' must end in '.py' or '.mpy'\n", file);

            pyconfig.load(file);
            configfile_found = true;
        }
    }

    if (outdir.empty()) {
        char *env = getenv("OUTPUT_DIR");
        outdir = env ? env : ".";
    }

    simout.setDirectory(outdir);

    char *env = getenv("CONFIG_OUTPUT");
    if (!env)
        env = "config.out";
    configStream = simout.find(env);

    if (!configfile_found)
        panic("no configuration file specified!");

    // The configuration database is now complete; start processing it.
    IniFile inifile;
    if (!pyconfig.output(inifile))
        panic("Error processing python code");

    // Initialize statistics database
    Stats::InitSimStats();

    // Now process the configuration hierarchy and create the SimObjects.
    ConfigHierarchy configHierarchy(inifile);
    configHierarchy.build();
    configHierarchy.createSimObjects();

    // Parse and check all non-config-hierarchy parameters.
    ParamContext::parseAllContexts(inifile);
    ParamContext::checkAllContexts();

    // Print hello message to stats file if it's actually a file.  If
    // it's not (i.e. it's cout or cerr) then we already did it above.
    if (simout.isFile(*outputStream))
        sayHello(*outputStream);

    // Echo command line and all parameter settings to stats file as well.
    echoCommandLine(argc, argv, *outputStream);
    ParamContext::showAllContexts(*configStream);

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

    // Nothing to simulate if we don't have at least one CPU somewhere.
    if (BaseCPU::numSimulatedCPUs() == 0) {
        cerr << "Fatal: no CPUs to simulate." << endl;
        exit(1);
    }

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
