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
#include <string>

#include <pin.H>

#include "breakpoint.hh"
#include "instcount.hh"
#include "pintool.hh"
#include "plugin.hh"

static KNOB<bool> enable(KNOB_MODE_WRITEONCE, "pintool", "instcount", "0",
                         "Enable instruction counting");

// TODO: Make this static.
ADDRINT instcount;

static void
Analyze(ADDRINT n)
{ instcount += n; }

static void
Instrument(TRACE trace, void *)
{
    if (IsGuestCode(trace)) {
        return;
    }
    for (BBL bbl = TRACE_BblHead(trace); BBL_Valid(bbl); bbl = BBL_Next(bbl)) {
        BBL_InsertCall(bbl, IPOINT_BEFORE, (AFUNPTR)Analyze, IARG_ADDRINT,
                       BBL_NumIns(bbl), IARG_END);
    }
}

static void
Finish(int32_t code, void *)
{ std::cerr << "instcount=" << std::dec << instcount << std::endl; }

namespace
{
struct InstCountPlugin final : Plugin
{
    const char *
    name() const override
    { return "instcount"; }

    int
    priority() const override
    { return 1; }

    bool
    enabled() const override
    { return enable.Value(); }

    bool
    reg() override
    {
        assert(enabled());
        TRACE_AddInstrumentFunction(Instrument, nullptr);
        RegisterCounter("inst", &instcount);
        PIN_AddFiniFunction(Finish, nullptr);
        return true;
    }

    bool
    command(const std::string &cmd, const std::vector<std::string> &args,
            std::string &result) override
    {
        if (cmd == "instcount") {
            result = std::to_string(instcount);
            return true;
        }

        return false;
    }
} plugin;
} // namespace
