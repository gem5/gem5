/*
 * Copyright (c) 2026 CkingX256
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

#include "mem/probes/dram_command_trace.hh"

#include "base/logging.hh"
#include "base/output.hh"
#include "mem/dram_interface.hh"
#include "params/DramCommandTrace.hh"
#include "sim/core.hh"

namespace gem5
{

namespace memory
{

DramCommandTrace::DramCommandTrace(const DramCommandTraceParams &p)
    : SimObject(p),
      output(simout.create(p.trace_file)->stream()),
      maxRecords(p.max_records)
{
    fatal_if(!maxRecords, "max_records must be positive");
    *output << "sequence,scheduled_at,command_at,command,rank,bank,row,"
               "implicit\n";
    registerExitCallback([this]() { output->flush(); });
}

void
DramCommandTrace::regProbeListeners()
{
    const auto &p = static_cast<const DramCommandTraceParams &>(params());
    auto *manager = p.dram->getProbeManager();
    listener =
        manager->connect<ProbeListenerArg<DramCommandTrace, DramCommandInfo>>(
            this, "CommandScheduled", &DramCommandTrace::record);
}

void
DramCommandTrace::record(const DramCommandInfo &event)
{
    fatal_if(records == maxRecords, "DRAM command trace record limit reached");
    *output << records++ << ',' << event.scheduledAt << ',' << event.commandAt
            << ',' << event.name() << ',' << unsigned(event.rank) << ',';
    if (event.bank) {
        *output << unsigned(*event.bank);
    }
    *output << ',';
    if (event.row) {
        *output << *event.row;
    }
    *output << ',' << event.implicit << '\n';
    fatal_if(!*output, "Failed to write DRAM command trace");
}

} // namespace memory
} // namespace gem5
