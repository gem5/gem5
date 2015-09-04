/*
 * Copyright (c) 2012 ARM Limited
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
 * Copyright (c) 2000-2005 The Regents of The University of Michigan
 * Copyright (c) 2008 The Hewlett-Packard Development Company
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

#include <csignal>
#include <iostream>
#include <string>

#include "base/cprintf.hh"
#include "sim/async.hh"
#include "sim/core.hh"
#include "sim/eventq.hh"
#include "sim/init_signals.hh"

using namespace std;

/// Stats signal handler.
void
dumpStatsHandler(int sigtype)
{
    async_event = true;
    async_statdump = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

void
dumprstStatsHandler(int sigtype)
{
    async_event = true;
    async_statdump = true;
    async_statreset = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

/// Exit signal handler.
void
exitNowHandler(int sigtype)
{
    async_event = true;
    async_exit = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

/// Abort signal handler.
void
abortHandler(int sigtype)
{
    ccprintf(cerr, "Program aborted at tick %llu\n", curTick());
}

// Handle SIGIO
static void
ioHandler(int sigtype)
{
    async_event = true;
    async_io = true;
    /* Wake up some event queue to handle event */
    getEventQueue(0)->wakeup();
}

static void
installSignalHandler(int signal, void (*handler)(int sigtype))
{
    struct sigaction sa;

    memset(&sa, 0, sizeof(sa));
    sigemptyset(&sa.sa_mask);
    sa.sa_handler = handler;
    sa.sa_flags = SA_RESTART;

    if (sigaction(signal, &sa, NULL) == -1)
        panic("Failed to setup handler for signal %i\n", signal);
}

/*
 * M5 can do several special things when various signals are sent.
 * None are mandatory.
 */
void
initSignals()
{
    // Floating point exceptions may happen on misspeculated paths, so
    // ignore them
    signal(SIGFPE, SIG_IGN);

    // We use SIGTRAP sometimes for debugging
    signal(SIGTRAP, SIG_IGN);

    // Dump intermediate stats
    installSignalHandler(SIGUSR1, dumpStatsHandler);

    // Dump intermediate stats and reset them
    installSignalHandler(SIGUSR2, dumprstStatsHandler);

    // Exit cleanly on Interrupt (Ctrl-C)
    installSignalHandler(SIGINT, exitNowHandler);

    // Print out cycle number on abort
    installSignalHandler(SIGABRT, abortHandler);

    // Install a SIGIO handler to handle asynchronous file IO. See the
    // PollQueue class.
    installSignalHandler(SIGIO, ioHandler);
}

