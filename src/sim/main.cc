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
 * Authors: Nathan Binkert
 */

#include <Python.h>

#include <iostream>
#include <string>

#include "base/cprintf.hh"
#include "base/misc.hh"
#include "config/pythonhome.hh"
#include "python/swig/init.hh"
#include "sim/async.hh"
#include "sim/host.hh"
#include "sim/root.hh"

using namespace std;

/// Stats signal handler.
void
dumpStatsHandler(int sigtype)
{
    async_event = true;
    async_statdump = true;
}

void
dumprstStatsHandler(int sigtype)
{
    async_event = true;
    async_statdump = true;
    async_statreset = true;
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
    ccprintf(cerr, "Program aborted at cycle %d\n", curTick);
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
    init_swig();

    PyRun_SimpleString("import m5.main");
    PyRun_SimpleString("m5.main.main()");

    // clean up Python intepreter.
    Py_Finalize();
}
