/*
 * Copyright (c) 2025 Technical University of Munich
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

#include "cpu/o3/tdm.hh"

#include "cpu/o3/commit.hh"
#include "cpu/o3/cpu.hh"
#include "cpu/o3/decode.hh"
#include "cpu/o3/fetch.hh"
#include "cpu/o3/iew.hh"
#include "cpu/o3/rename.hh"

namespace gem5
{

namespace o3
{
TopDownStats::TopDownStats(CPU *cpu, Fetch *fetch, Rename *rename,
                           Decode *decode, IEW *iew, Commit *commit)
    : statistics::Group(cpu, "TopDownStats"),
      topDownL1(cpu, fetch, rename, decode, iew, commit),
      topDownFbL2(cpu, fetch),
      topDownBsL2(cpu, decode, iew),
      topDownBbL2(cpu, rename, iew),
      topDownBbMem(cpu, rename, iew)
{}

TopDownStats::TopDownL1::TopDownL1(CPU *cpu, Fetch *fetch, Rename *rename,
                                   Decode *decode, IEW *iew, Commit *commit)
    : statistics::Group(cpu, "TopDownL1"),
      ADD_STAT(frontendBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Frontend Bound, fraction of slots lost due to frontend "
               "undersupplying the backend"),
      ADD_STAT(badSpeculation,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Bad Speculation, fraction of slots lost due to mispeculation"),
      ADD_STAT(backendBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Backend Bound, fraction of slots lost due to backend resource"
               " constraints."),
      ADD_STAT(
          retiring,
          statistics::units::Rate<statistics::units::Count,
                                  statistics::units::Count>::get(),
          "Retiring, fraction of slots successfully retired by the backend")
{
    // Total Slots
    statistics::Temp totalSlots =
        rename->getWidth() * cpu->baseStats.numCycles;

    // L1 Frontend Bound
    frontendBound = fetch->getStats().fetchBubbles / (totalSlots);

    // L1 Bad Speculation
    // Recovery cycles for mispredictions detected at Decode
    int recoveryCycleToDecode = decode->getFetchToDecodeDelay();

    auto decodeBranchMispred =
        (int)recoveryCycleToDecode * decode->getStats().branchMispred;

    // Recovery cycles for mispredictions detected at IEW
    int recoveryCycleToIEW = decode->getFetchToDecodeDelay() +
                             rename->getDecodeToRenameDelay() +
                             iew->getRenameToIEWDelay();

    auto iewBadSpec =
        (int)recoveryCycleToIEW * (iew->getStats().branchMispredicts +
                                   iew->getStats().memOrderViolationEvents);

    // Number of wasted slots due to bad speculation
    auto wastedSlots =
        rename->getStats().renamedInsts - commit->getStats().committedInst;

    badSpeculation = (wastedSlots + (decodeBranchMispred + iewBadSpec) *
                                        rename->getWidth()) /
                     (totalSlots);

    // L1 Retiring
    retiring = commit->getStats().committedInst / (totalSlots);

    // L1 Backend Bound
    backendBound = 1 - (frontendBound + badSpeculation + retiring);
}

TopDownStats::TopDownFrontendBoundL2::TopDownFrontendBoundL2(CPU *cpu,
                                                             Fetch *fetch)
    : statistics::Group(cpu, "TopDownL2_FrontendBound"),
      ADD_STAT(fetchLatency,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Fetch Latency Bound, frontend stalls due to instruction cache "
               "inefficiency"),
      ADD_STAT(fetchBandwidth,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Fetch Bandwidth Bound, frontend stalls due to decoder "
               "inefficiency")
{
    // Frontend L2
    fetchLatency =
        fetch->getStats().fetchBubblesMax / (cpu->baseStats.numCycles);
    fetchBandwidth =
        cpu->cpuStats.topDownStats.topDownL1.frontendBound - fetchLatency;
}

TopDownStats::TopDownBadSpeculationL2 ::TopDownBadSpeculationL2(CPU *cpu,
                                                                Decode *decode,
                                                                IEW *iew)
    : statistics::Group(cpu, "TopDownL2_BadSpeculation"),
      ADD_STAT(branchMissPredicts,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Branch Miss Predicts"),
      ADD_STAT(machineClears,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Memory Order Violations")
{
    auto &iewMissPred = iew->getStats().branchMispredicts;
    auto &decodeMissPred = decode->getStats().branchMispred;
    auto &memOrderViolations = iew->getStats().memOrderViolationEvents;

    auto brMispredictFraction =
        (iewMissPred + decodeMissPred) /
        (iewMissPred + decodeMissPred + memOrderViolations);

    branchMissPredicts = brMispredictFraction *
                         cpu->cpuStats.topDownStats.topDownL1.badSpeculation;

    machineClears = cpu->cpuStats.topDownStats.topDownL1.badSpeculation -
                    branchMissPredicts;
}

TopDownStats::TopDownBackendBoundL2::TopDownBackendBoundL2(CPU *cpu,
                                                           Rename *rename,
                                                           IEW *iew)
    : statistics::Group(cpu, "TopDownL2_BackendBound"),
      ADD_STAT(memoryBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Memory Bound, backend stalls due to memory subsystem"),
      ADD_STAT(coreBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Core Bound, backend stalls due to functional unit constraints")
{
    // Backend L2
    executionStalls = (iew->instQueue.getStats().numInstsExec0 -
                       rename->getStats().status[Rename::ThreadStatus::Idle] +
                       iew->instQueue.getStats().numInstsExec1 +
                       iew->instQueue.getStats().numInstsExec2) /
                      (cpu->baseStats.numCycles);
    auto memoryBoundRaw = (iew->instQueue.getStats().loadStallCycles +
                           rename->getStats().storeStalls) /
                          (cpu->baseStats.numCycles);
    auto coreBoundRaw = executionStalls - memoryBoundRaw;

    auto &totalBackendBound =
        cpu->cpuStats.topDownStats.topDownL1.backendBound;

    memoryBound =
        memoryBoundRaw / (memoryBoundRaw + coreBoundRaw) * (totalBackendBound);
    coreBound =
        coreBoundRaw / (memoryBoundRaw + coreBoundRaw) * (totalBackendBound);
}

TopDownStats::TopDownBackendBoundL3::TopDownBackendBoundL3(CPU *cpu,
                                                           Rename *rename,
                                                           IEW *iew)
    : statistics::Group(cpu, "TopDownL3_BackendBound_MemoryBound"),
      ADD_STAT(l1Bound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "L1 Cache Bound"),
      ADD_STAT(l2Bound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "L2 Cache Bound"),
      ADD_STAT(l3Bound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "L3 Cache Bound"),
      ADD_STAT(extMemBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "External Memory Bound"),
      ADD_STAT(storeBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Store Bound")
{

    auto &totalBackendBound =
        cpu->cpuStats.topDownStats.topDownBbL2.memoryBound;

    // Backend Bound / Memory Bound L3
    auto l1BoundRaw = (iew->instQueue.getStats().loadStallCycles -
                       iew->instQueue.getStats().L1miss) /
                      (cpu->baseStats.numCycles);
    auto l2BoundRaw =
        (iew->instQueue.getStats().L1miss - iew->instQueue.getStats().L2miss) /
        (cpu->baseStats.numCycles);
    auto l3BoundRaw =
        (iew->instQueue.getStats().L2miss - iew->instQueue.getStats().L3miss) /
        (cpu->baseStats.numCycles);
    auto extMemBoundRaw =
        (iew->instQueue.getStats().L3miss) / (cpu->baseStats.numCycles);
    auto storeBoundRaw =
        (rename->getStats().storeStalls) / (cpu->baseStats.numCycles);

    auto totalMemoryBound =
        l1BoundRaw + l2BoundRaw + l3BoundRaw + extMemBoundRaw + storeBoundRaw;

    l1Bound = l1BoundRaw / totalMemoryBound * totalBackendBound;
    l2Bound = l2BoundRaw / totalMemoryBound * totalBackendBound;
    l3Bound = l3BoundRaw / totalMemoryBound * totalBackendBound;
    extMemBound = extMemBoundRaw / totalMemoryBound * totalBackendBound;
    storeBound = storeBoundRaw / totalMemoryBound * totalBackendBound;
}

} // namespace o3
} // namespace gem5
