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
#include <set>

#include "pintool.hh"
#include "plugin.hh"

static std::set<ADDRINT> sysbreaks;

static void
Analyze(ADDRINT sysno, CONTEXT *ctx)
{
    if (!sysbreaks.erase(sysno)) {
        return;
    }
    // TODO: Factor this out into shared code for breaking.
    RunResult result;
    result.result = result.RUNRESULT_BREAK;
    std::cerr << "instbreak: switching to guest\n";
    ContextSwitchToGuest(ctx, result);
    PIN_ExecuteAt(ctx);
}

static void
Instrument(INS ins, void *)
{
    if (IsGuestCode(ins) || !INS_IsSyscall(ins)) {
        return;
    }

    INS_InsertCall(ins, IPOINT_BEFORE, (AFUNPTR)Analyze, IARG_SYSCALL_NUMBER,
                   IARG_CONTEXT, IARG_END);
}

namespace
{
struct SyscallBreakpointPlugin final : Plugin
{
    const char *
    name() const override
    { return "sysbreak"; }

    int
    priority() const override
    { return -1; }

    bool
    enabled() const override
    { return true; }

    bool
    reg() override
    {
        INS_AddInstrumentFunction(Instrument, nullptr);
        return true;
    }

    bool
    command(const std::string &cmd, const std::vector<std::string> &args,
            std::string &result) override
    {
        if (cmd != "sysbreak") {
            return false;
        }

        if (args.size() != 1) {
            std::cerr << "sysbreak: error: bad usage\n";
            std::cerr << "usage: sysbreak <sysno>\n";
            std::abort(); // TODO: Add return value for bad usage.
        }

        const ADDRINT sysno = std::stoull(args.at(0));
        sysbreaks.insert(sysno);
        return true;
    }
} plugin;
} // namespace
