/*
 * Copyright (c) 2003 The Regents of The University of Michigan
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
#include <stdlib.h>
#include <signal.h>

#include <string>
#include <vector>

#include "base/copyright.hh"
#include "base/inifile.hh"
#include "base/misc.hh"
#include "base/pollevent.hh"
#include "base/statistics.hh"
#include "base/time.hh"
#include "cpu/base_cpu.hh"
#include "cpu/full_cpu/smt.hh"
#include "sim/async.hh"
#include "sim/builder.hh"
#include "sim/configfile.hh"
#include "sim/host.hh"
#include "sim/sim_events.hh"
#include "sim/sim_exit.hh"
#include "sim/sim_init.hh"
#include "sim/sim_object.hh"
#include "sim/stat_control.hh"
#include "sim/stats.hh"
#include "sim/universe.hh"

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

/// Simulator executable name
const char *myProgName = "";

/// Show brief help message.
static void
showBriefHelp(ostream &out)
{
    out << "Usage: " << myProgName
         << " [-hn] [-Dname[=def]] [-Uname] [-I[dir]] "
         << "[--<section>:<param>=<value>] [<config file> ...]" << endl
         << "   -h: print long help (including parameter listing)" << endl
         << "   -n: don't load default.ini" << endl
         << "   -u: don't quit on unreferenced parameters" << endl
         << "   -D,-U,-I: passed to cpp for preprocessing .ini files" << endl;
}

/// Show verbose help message.  Includes parameter listing from
/// showBriefHelp(), plus an exhaustive list of ini-file parameters
/// and SimObjects (with their parameters).
static void
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
static void
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
static void
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


///
/// The simulator configuration database.  This is the union of all
/// specified .ini files.  This shouldn't need to be visible outside
/// this file, as it is passed as a parameter to all the param-parsing
/// routines.
///
static IniFile simConfigDB;

/// Check for a default.ini file and load it if necessary.
static void
handleDefaultIni(bool &loadIt, vector<char *> &cppArgs)
{
    struct stat sb;

    if (loadIt) {
        if (stat("default.ini", &sb) == 0) {
            if (!simConfigDB.loadCPP("default.ini", cppArgs)) {
                cout << "Error processing file default.ini" << endl;
                exit(1);
            }
        }

        // set this whether it actually was found or not, so we don't
        // bother to check again next time
        loadIt = false;
    }
}


/// M5 entry point.
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

    sayHello(cerr);

    // Initialize statistics database
    Statistics::InitSimStats();

    vector<char *> cppArgs;

    // Should we use default.ini if it exists?  By default, yes.  (Use
    // -n to override.)
    bool loadDefaultIni = true;

    // Should we quit if there are unreferenced parameters?  By
    // default, yes... it's a good way of catching typos in
    // section/parameter names (which otherwise go by silently).  Use
    // -u to override.
    bool quitOnUnreferenced = true;

    // Parse command-line options.  The tricky part here is figuring
    // out whether to look for & load default.ini, and if needed,
    // doing so at the right time w.r.t. processing the other
    // parameters.
    //
    // Since most of the complex options are handled through the
    // config database, we don't mess with getopts, and just parse
    // manually.
    for (int i = 1; i < argc; ++i) {
        char *arg_str = argv[i];

        // if arg starts with '-', parse as option,
        // else treat it as a configuration file name and load it
        if (arg_str[0] == '-') {

            // switch on second char
            switch (arg_str[1]) {
              case 'h':
                // -h: show help
                showLongHelp(cerr);
                exit(1);

              case 'n':
                // -n: don't load default.ini
                if (!loadDefaultIni) {
                    cerr << "Warning: -n option needs to precede any "
                         << "explicit configuration file name " << endl
                         << "         or command-line configuration parameter."
                         << endl;
                }
                loadDefaultIni = false;
                break;

              case 'u':
                // -u: don't quit on unreferenced parameters
                quitOnUnreferenced = false;
                break;

              case 'D':
              case 'U':
              case 'I':
                // cpp options: record & pass to cpp.  Note that these
                // cannot have spaces, i.e., '-Dname=val' is OK, but
                // '-D name=val' is not.  I don't consider this a
                // problem, since even though gnu cpp accepts the
                // latter, other cpp implementations do not (Tru64,
                // for one).
                cppArgs.push_back(arg_str);
                break;

              case '-':
                // command-line configuration parameter:
                // '--<section>:<parameter>=<value>'

                // Load default.ini if necessary -- see comment in
                // else clause below.
                handleDefaultIni(loadDefaultIni, cppArgs);

                if (!simConfigDB.add(arg_str + 2)) {
                    // parse error
                    ccprintf(cerr,
                             "Could not parse configuration argument '%s'\n"
                             "Expecting --<section>:<parameter>=<value>\n",
                             arg_str);
                    exit(0);
                }
                break;

              default:
                showBriefHelp(cerr);
                ccprintf(cerr, "Fatal: invalid argument '%s'\n", arg_str);
                exit(0);
            }
        }
        else {
            // no '-', treat as config file name

            // If we haven't loaded default.ini yet, and we want to,
            // now is the time.  Can't do it sooner because we need to
            // look for '-n', can't do it later since we want
            // default.ini loaded first (so that any other settings
            // override it).
            handleDefaultIni(loadDefaultIni, cppArgs);

            if (!simConfigDB.loadCPP(arg_str, cppArgs)) {
                cprintf("Error processing file %s\n", arg_str);
                exit(1);
            }
        }
    }

    // Final check for default.ini, in case no config files or
    // command-line config parameters were given.
    handleDefaultIni(loadDefaultIni, cppArgs);

    // The configuration database is now complete; start processing it.

    // Parse and check all non-config-hierarchy parameters.
    ParamContext::parseAllContexts(simConfigDB);
    ParamContext::checkAllContexts();

    // Print header info into stats file.  Can't do this sooner since
    // the stat file name is set via a .ini param... thus it just got
    // opened above during ParamContext::checkAllContexts().

    // Print hello message to stats file if it's actually a file.  If
    // it's not (i.e. it's cout or cerr) then we already did it above.
    if (outputStream != &cout && outputStream != &cerr)
        sayHello(*outputStream);

    // Echo command line and all parameter settings to stats file as well.
    echoCommandLine(argc, argv, *outputStream);
    ParamContext::showAllContexts(builderStream());

    // Now process the configuration hierarchy and create the SimObjects.
    ConfigHierarchy configHierarchy(simConfigDB);
    configHierarchy.build();
    configHierarchy.createSimObjects();

    // Restore checkpointed state, if any.
    configHierarchy.unserializeSimObjects();

    // Done processing the configuration database.
    // Check for unreferenced entries.
    if (simConfigDB.printUnreferenced() && quitOnUnreferenced) {
        cerr << "Fatal: unreferenced .ini sections/entries." << endl
             << "If this is not an error, add 'unref_section_ok=y' or "
             << "'unref_entries_ok=y' to the appropriate sections "
             << "to suppress this message." << endl;
        exit(1);
    }

    SimObject::regAllStats();

    // uncomment the following to get PC-based execution-time profile
#ifdef DO_PROFILE
    init_profile((char *)&_init, (char *)&_fini);
#endif

    // Check to make sure that the stats package is properly initialized
    Statistics::check();

    // Reset to put the stats in a consistent state.
    Statistics::reset();

    // Nothing to simulate if we don't have at least one CPU somewhere.
    if (BaseCPU::numSimulatedCPUs() == 0) {
        cerr << "Fatal: no CPUs to simulate." << endl;
        exit(1);
    }

    SimInit();
    warn("Entering event queue.  Starting simulation...\n");

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

                using namespace Statistics;
                SetupEvent(Dump, curTick);
            }

            if (async_dumpreset) {
                async_dumpreset = false;

                using namespace Statistics;
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
