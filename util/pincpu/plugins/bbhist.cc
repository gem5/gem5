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

#include <cassert>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <pin.H>

#include "pintool.hh"
#include "plugin.hh"

namespace
{

KNOB<bool> enable(KNOB_MODE_WRITEONCE, "pintool", "bbhist", "0",
                  "Enable basic block histogram collection");

std::vector<ADDRINT>
getInstVec(BBL bbl)
{
    std::vector<ADDRINT> insts;
    for (INS ins = BBL_InsHead(bbl); INS_Valid(ins); ins = INS_Next(ins)) {
        insts.push_back(INS_Address(ins));
    }
    return insts;
}

std::string
getBlockName(BBL bbl)
{
    std::string name;
    bool first = true;
    for (INS ins = BBL_InsHead(bbl); INS_Valid(ins); ins = INS_Next(ins)) {
        char buf[256];
        std::sprintf(buf, "%s%lx", first ? "" : ",", INS_Address(ins));
        name += buf;
        first = false;
    }
    return name;
}

struct BlockData
{
    ADDRINT hits;
    std::string name;

    BlockData(BBL bbl) : hits(0) { name = getBlockName(bbl); }
};

std::map<std::vector<ADDRINT>, BlockData> blocks;

BlockData &
getBlockData(BBL bbl)
{ return blocks.emplace(getInstVec(bbl), bbl).first->second; }

void
Analyze(ADDRINT *counter)
{ ++*counter; }

void
InstrumentBBL(BBL bbl)
{
    ADDRINT &counter = getBlockData(bbl).hits;
    BBL_InsertCall(bbl, IPOINT_BEFORE, (AFUNPTR)Analyze, IARG_PTR, &counter,
                   IARG_END);
}

void
InstrumentTRACE(TRACE trace, void *)
{
    if (IsGuestCode(trace)) {
        return;
    }
    for (BBL bbl = TRACE_BblHead(trace); BBL_Valid(bbl); bbl = BBL_Next(bbl)) {
        InstrumentBBL(bbl);
    }
}

void
Dump(std::string &s)
{
    for (const auto &[insts, block] : blocks) {
        if (block.hits) {
            s += std::to_string(block.hits) + ' ' + block.name + '\n';
        }
    }
}

void
Reset()
{
    for (auto &[insts, block] : blocks) {
        block.hits = 0;
    }
}

struct BasicBlockHistogramPlugin final : Plugin
{
    const char *
    name() const override
    { return "bbhist"; }

    int
    priority() const override
    { return 1; }

    bool
    enabled() const override
    { return enable.Value(); }

    bool
    reg() override
    {
        TRACE_AddInstrumentFunction(InstrumentTRACE, nullptr);
        return true;
    }

    bool
    command(const std::string &cmd, const std::vector<std::string> &args,
            std::string &result) override
    {
        if (cmd != "bbhist") {
            return false;
        }

        if (args.at(0) == "dump") {
            Dump(result);
            return true;
        } else if (args.at(0) == "reset") {
            Reset();
            return true;
        }

        std::cerr << "bbhist: error: bad usage\n";
        std::cerr << "usage: bbhist (dump|reset)\n";
        std::abort();
    }
} plugin;

} // namespace
