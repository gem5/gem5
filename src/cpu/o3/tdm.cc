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
          "Retiring, fraction of slots successfully retired by the backend"),
      ADD_STAT(
          squashedSlots,
          statistics::units::Rate<statistics::units::Count,
                                  statistics::units::Count>::get(),
          "Retiring, fraction of slots successfully retired by the backend"),
    ADD_STAT(
          badSpecCycles,
          statistics::units::Rate<statistics::units::Count,
                                  statistics::units::Count>::get(),
          "Retiring, fraction of slots successfully retired by the backend"),
    ADD_STAT(
          refillSlots,
          statistics::units::Rate<statistics::units::Count,
                                  statistics::units::Count>::get(),
          "Retiring, fraction of slots successfully retired by the backend"),
    ADD_STAT(
          FESlots,
          statistics::units::Rate<statistics::units::Count,
                                  statistics::units::Count>::get(),
          "Retiring, fraction of slots successfully retired by the backend"),
    ADD_STAT(
          BESlots,
          statistics::units::Rate<statistics::units::Count,
                                  statistics::units::Count>::get(),
          "Retiring, fraction of slots successfully retired by the backend"),
    ADD_STAT(
          serializeSlots,
          statistics::units::Rate<statistics::units::Count,
                                  statistics::units::Count>::get(),
          "Retiring, fraction of slots successfully retired by the backend")
{

  // Total Slots
  statistics::Temp totalSlots =
      rename->getWidth() * cpu->cpuStats.tdaCycles;


  // L1 Bad Speculation

//   // Recovery cycles for mispredictions detected at Decode
//   int recoveryCycleToDecode = decode->getFetchToDecodeDelay();

//   auto decodeBranchMispred =
//       (int)recoveryCycleToDecode * decode->getStats().branchMispred;

    /** L1 Bad Speculation ----
     * Bad speculation are:
     * 1) all renamed instructions (slots) that get squashed do to a earlier
     *    misprediction and will not be committed.
     * 2) The penalty it takes to refill the pipeline.
     */

    // Number of squashed slots due to bad speculation
    squashedSlots = rename->getStats().renamedInsts -
                         commit->getStats().committedInst;

    // Recovery cycles for mispredictions detected at IEW
    int recoveryCycles = decode->getFetchToDecodeDelay() +
                             rename->getDecodeToRenameDelay()
                             + iew->getRenameToIEWDelay()
                            //  @todo Do we also want to add IEW->commit delay?
                             ;

    badSpecCycles =
        (int)recoveryCycles * (iew->getStats().branchMispredicts +
                               iew->getStats().memOrderViolationEvents);

    auto renameSquashCycles = rename->getStats().status[Rename::ThreadStatus::Squashing];

    // Slots wasted do to pipeline refills. These slots do not
    // count to the front-end bound category.
    // refillSlots = badSpecCycles * rename->getWidth();
    refillSlots = rename->getStats().refillBubbles;


//   badSpeculation = (wastedSlots + (decodeBranchMispred + iewBadSpec) *
//                                       rename->getWidth()) /
//                    (totalSlots);

    badSpeculation = (squashedSlots + refillSlots + (renameSquashCycles*rename->getWidth())) / totalSlots;

    // L1 Frontend Bound
    //   frontendBound = cpu->fetch.getStats().fetchBubbles / (totalSlots);
    // Frontend bound are all slots where the front-end cannot deliver
    // instructions to the rename stage.
    // Refill slots are not part of the frontend bound category as those
    // bubbles are caused by a misprediction.
    // We also need to make sure we substract the cycles where rename is still
    // squashing from the refill penalty.
    frontendBound = (rename->getStats().fetchBubbles
                    //  - refillSlots + (renameSquashCycles*rename->getWidth())
                    ) / (totalSlots);
    
    FESlots = rename->getStats().fetchBubbles;
    // L1 Retiring
    retiring = commit->getStats().committedInst / (totalSlots);

    // L1 Backend Bound
    backendBound = 1 - (frontendBound + badSpeculation + retiring);
    BESlots = backendBound * totalSlots;

    serializeSlots = rename->getStats().status[Rename::ThreadStatus::SerializeStall] * rename->getWidth();
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
        fetch->getStats().fetchBubblesMax / (cpu->cpuStats.tdaCycles);
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
      ADD_STAT(executionStalls,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "The execution unit stalls."),
      ADD_STAT(memoryBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Memory Bound, backend stalls due to memory subsystem"),
      ADD_STAT(coreBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Core Bound, backend stalls due to functional unit constraints"),
      ADD_STAT(serializeBound,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Core Bound, backend stalls due to functional unit constraints"),
    ADD_STAT(serializeStalls,
               statistics::units::Rate<statistics::units::Count,
                                       statistics::units::Count>::get(),
               "Core Bound, backend stalls due to instructions requiring serialization")
{
    serializeStalls = rename->getStats().status[Rename::ThreadStatus::SerializeStall];
    //  /
                        //   (cpu->cpuStats.tdaCycles);
    // Backend L2
    executionStalls = (iew->instQueue.getStats().numInstsExec0 -
                       rename->getStats().status[Rename::ThreadStatus::Idle] +
                       iew->instQueue.getStats().numInstsExec1 +
                       iew->instQueue.getStats().numInstsExec2);
                    //     /
                    //   (cpu->cpuStats.tdaCycles);
    auto memoryBoundRaw = (iew->instQueue.getStats().loadStallCycles +
                           rename->getStats().storeStalls);
                        //     /
                        //   (cpu->cpuStats.tdaCycles);
    // @todo doesn't make sense
    auto coreBoundRaw = (executionStalls) - memoryBoundRaw;

    auto &totalBackendBound =
        cpu->cpuStats.topDownStats.topDownL1.backendBound;

    auto &BESlots = cpu->cpuStats.topDownStats.topDownL1.BESlots;
    auto &serializeSlots = cpu->cpuStats.topDownStats.topDownL1.serializeSlots;
    
    serializeBound = serializeSlots / (BESlots) * (totalBackendBound);
    auto beremaining = totalBackendBound - serializeBound;

    memoryBound =
        memoryBoundRaw / (executionStalls) * (beremaining);
    coreBound =
        coreBoundRaw / (executionStalls) * (beremaining);
    
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
                      (cpu->cpuStats.tdaCycles);
    auto l2BoundRaw =
        (iew->instQueue.getStats().L1miss - iew->instQueue.getStats().L2miss) /
        (cpu->cpuStats.tdaCycles);
    auto l3BoundRaw =
        (iew->instQueue.getStats().L2miss - iew->instQueue.getStats().L3miss) /
        (cpu->cpuStats.tdaCycles);
    auto extMemBoundRaw =
        (iew->instQueue.getStats().L3miss) / (cpu->cpuStats.tdaCycles);
    auto storeBoundRaw =
        (rename->getStats().storeStalls) / (cpu->cpuStats.tdaCycles);

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
