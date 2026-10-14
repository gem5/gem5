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

#ifndef __MEM_PROBES_DRAM_COMMAND_TRACE_HH__
#define __MEM_PROBES_DRAM_COMMAND_TRACE_HH__

#include <ostream>

#include "mem/dram_command.hh"
#include "sim/probe/probe.hh"
#include "sim/sim_object.hh"

namespace gem5
{

struct DramCommandTraceParams;

namespace memory
{

/** A bounded CSV example of the CommandScheduled observation interface. */
class DramCommandTrace : public SimObject
{
  public:
    DramCommandTrace(const DramCommandTraceParams &params);
    void regProbeListeners() override;

  private:
    void record(const DramCommandInfo &event);
    std::ostream *output;
    uint64_t records = 0;
    const uint64_t maxRecords;
    ProbeListenerPtr<> listener;
};

} // namespace memory
} // namespace gem5

#endif // __MEM_PROBES_DRAM_COMMAND_TRACE_HH__
