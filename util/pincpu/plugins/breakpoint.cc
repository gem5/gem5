/*
 * Copyright (c) 2026 The Board of Trustees of the Leland Stanford
 * Junior University
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

#include <iostream>
#include <list>

#include <pin.H>

#include "breakpoint.hh"
#include "ops.h"
#include "pintool.hh"
#include "plugin.hh"

static std::map<std::string, const ADDRINT *> counters;
static std::map<const ADDRINT *, ADDRINT> breakpoints;

static void
SetBreakpoint(const ADDRINT *counter, ADDRINT target)
{
    // Remove instrumentation if we're adding a new type of breakpoint.
    if (!breakpoints.count(counter)) {
        PIN_RemoveInstrumentation();
    }
    breakpoints[counter] = target;
}

static void
ClearBreakpoint(const ADDRINT *counter)
{ breakpoints.at(counter) = std::numeric_limits<ADDRINT>::max(); }

static ADDRINT
AnalyzeIf(ADDRINT *target, const ADDRINT *counter)
{ return *counter >= *target; }

static void
AnalyzeThen(CONTEXT *ctx, const ADDRINT *counter)
{
    ClearBreakpoint(counter);
    RunResult result;
    result.result = result.RUNRESULT_BREAK;
    std::cerr << "instbreak: switching to guest\n";
    ContextSwitchToGuest(ctx, result);
    PIN_ExecuteAt(ctx);
}

static void
Instrument(TRACE trace, void *)
{
    if (IsGuestCode(trace)) {
        return;
    }
    for (const auto &[counter, target] : breakpoints) {
        TRACE_InsertIfCall(trace, IPOINT_BEFORE, (AFUNPTR)AnalyzeIf,
                           IARG_ADDRINT, &target, IARG_PTR, counter, IARG_END);
        TRACE_InsertThenCall(trace, IPOINT_BEFORE, (AFUNPTR)AnalyzeThen,
                             IARG_CONTEXT, IARG_PTR, counter, IARG_END);
    }
}

void
RegisterCounter(const std::string &name, const ADDRINT *counter)
{ counters[name] = counter; }

namespace
{
struct BreakpointPlugin final : Plugin
{
    const char *
    name() const override
    { return "breakpoint"; }

    int
    priority() const override
    { return -1; }

    bool
    enabled() const override
    { return true; }

    bool
    reg() override
    {
        TRACE_AddInstrumentFunction(Instrument, nullptr);
        return true;
    }

    bool
    command(const std::string &cmd, const std::vector<std::string> &args,
            std::string &result) override
    {
        if (cmd != "breakpoint") {
            return false;
        }

        if (args.size() != 2) {
            std::cerr << "breakpoint: error: bad usage\n";
            std::cerr << "usage: breakpoint <counter> <target>\n";
            std::abort();
        }

        const ADDRINT target = std::stoull(args.at(1));
        const std::string &counter_name = args.at(0);
        const auto counter_it = counters.find(counter_name);
        if (counter_it == counters.end()) {
            std::cerr << "breakpoint: error: no such counter: " << counter_name
                      << "\n";
            std::abort();
        }

        SetBreakpoint(counter_it->second, target);
        return true;
    }
} plugin;
} // namespace
