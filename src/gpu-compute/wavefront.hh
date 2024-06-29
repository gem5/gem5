/*
 * Copyright (c) 2011-2017 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __GPU_COMPUTE_WAVEFRONT_HH__
#define __GPU_COMPUTE_WAVEFRONT_HH__

#include <cassert>
#include <deque>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include "arch/gpu_isa.hh"
#include "base/logging.hh"
#include "base/statistics.hh"
#include "base/stats/group.hh"
#include "base/types.hh"
#include "config/the_gpu_isa.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/hsa_queue_entry.hh"
#include "gpu-compute/lds_state.hh"
#include "gpu-compute/misc.hh"
#include "params/Wavefront.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class Wavefront : public SimObject
{
  public:
    enum status_e
    {
        // wavefront is stalled
        S_STOPPED,
        // wavefront is returning from a kernel
        S_RETURNING,
        // wavefront is running normally
        S_RUNNING,
        // wavefront is stalled
        S_STALLED,

        S_STALLED_SLEEP,

        /**
         * wavefront has unsatisfied wait counts
         *
         * while in this state the WF will only execute if
         * the oldest instruction is the waitcnt. while in
         * S_WAITCNT, the wavefront will not be ready until
         * all of its waitcnts have been satisfied. the
         * scoreboard ready() function will check the status
         * of the waitcnts whenever the WF is in S_WAITCNT,
         * and once they are satisfied, it will resume normal
         * operation.
         */
        S_WAITCNT,
        /**
         * WF is stalled at a barrier.
         */
        S_BARRIER
    };

    // gfx version wavefront is executing
    GfxVersion gfxVersion;
    // HW slot id where the WF is mapped to inside a SIMD unit
    const int wfSlotId;
    int kernId;
    // SIMD unit where the WV has been scheduled
    const int simdId;
    // id of the execution unit (or pipeline) where the oldest instruction
    // of the WF is scheduled
    int execUnitId;
    int flatLmUnitId;
    int flatGmUnitId;
    // pointer to parent CU
    ComputeUnit *computeUnit;
    int maxIbSize;

    std::deque<GPUDynInstPtr> instructionBuffer;

    bool pendingFetch;
    bool dropFetch;
    // last tick during which all WFs in the CU are not idle
    Tick lastNonIdleTick;

    // Execution unit resource ID's associated with this WF
    // These are static mappings set at WF slot construction and
    // based off of the simdId and wfSlotId.

    // Index to scalarALUs resource vector in CU
    int scalarAlu;

    // Indices into readyList/dispatchList of resources used by this
    // wavefront
    int scalarAluGlobalIdx;
    int globalMem;
    int localMem;
    int scalarMem;

    // number of VGPRs required by WF
    uint32_t maxVgprs;
    // number of SGPRs required by WF
    uint32_t maxSgprs;
    // first accumulation vgpr number
    uint32_t accumOffset;
    void freeResources();
    GPUDynInstPtr nextInstr();
    void setStatus(status_e newStatus);
    status_e getStatus() { return status; }
    void resizeRegFiles(int num_vregs, int num_sregs);
    bool isGmInstruction(GPUDynInstPtr ii);
    bool isLmInstruction(GPUDynInstPtr ii);
    bool isOldestInstWaitcnt();
    bool isOldestInstSleep();
    bool isOldestInstGMem();
    bool isOldestInstLMem();
    bool isOldestInstPrivMem();
    bool isOldestInstFlatMem();
    bool isOldestInstVectorALU();
    bool isOldestInstScalarALU();
    bool isOldestInstScalarMem();
    bool isOldestInstBarrier();

    // used for passing spill address to DDInstGPU
    std::vector<Addr> lastAddr;
    std::vector<uint32_t> workItemId[3];
    std::vector<uint32_t> workItemFlatId;
    /* kernel launch parameters */
    uint32_t workGroupId[3];
    uint32_t workGroupSz[3];
    uint32_t gridSz[3];
    uint32_t wgId;
    uint32_t wgSz;
    /* the actual WG size can differ than the maximum size */
    uint32_t actualWgSz[3];
    uint32_t actualWgSzTotal;
    void computeActualWgSz(HSAQueueEntry *task);
    // wavefront id within a workgroup
    uint32_t wfId;
    uint32_t maxDynWaveId;
    uint32_t dispatchId;
    // vector and scalar memory requests pending in memory system
    int outstandingReqs;
    // outstanding global memory write requests
    int outstandingReqsWrGm;
    // outstanding local memory write requests
    int outstandingReqsWrLm;
    // outstanding global memory read requests
    int outstandingReqsRdGm;
    // outstanding local memory read requests
    int outstandingReqsRdLm;
    // outstanding scalar memory read requests
    int scalarOutstandingReqsRdGm;
    // outstanding scalar memory write requests
    int scalarOutstandingReqsWrGm;
    int rdLmReqsInPipe;
    int rdGmReqsInPipe;
    int wrLmReqsInPipe;
    int wrGmReqsInPipe;
    int scalarRdGmReqsInPipe;
    int scalarWrGmReqsInPipe;

    int memTraceBusy;
    uint64_t lastTrace;
    // number of virtual vector registers reserved by WF
    int reservedVectorRegs;
    // number of virtual scalar registers reserved by WF
    int reservedScalarRegs;
    // Index into the Vector Register File's namespace where the WF's registers
    // will live while the WF is executed
    uint32_t startVgprIndex;
    // Index into the Scalar Register File's namespace where the WF's registers
    // will live while the WF is executed
    uint32_t startSgprIndex;

    // Architected flat scratch address for MI300+
    Addr archFlatScratchAddr = 0;

    // Old value of destination gpr (for trace)
    std::vector<uint32_t> oldVgpr;
    // Id of destination gpr (for trace)
    uint32_t oldVgprId;
    // Tick count of last old_vgpr copy
    uint64_t oldVgprTcnt;

    // Old value of destination gpr (for trace)
    std::vector<uint64_t> oldDgpr;
    // Id of destination gpr (for trace)
    uint32_t oldDgprId;
    // Tick count of last old_vgpr copy
    uint64_t oldDgprTcnt;

    // Execution mask at wavefront start
    VectorMask initMask;

    // a pointer to the fraction of the LDS allocated
    // to this workgroup (thus this wavefront)
    LdsChunk *ldsChunk;

    // unique WF id over all WFs executed across all CUs
    uint64_t wfDynId;

    // dyn inst id (per SIMD) of last instruction exec from this wave
    uint64_t lastInstExec;

    // Map to track the dyn instruction id of each vector register value
    // produced, indexed by physical vector register ID
    std::unordered_map<int,uint64_t> rawDist;

    // Counts the number of reads performed to each physical register
    // - counts are reset to 0 for each dynamic wavefront launched
    std::vector<int> vecReads;

    void initRegState(HSAQueueEntry *task, int wgSizeInWorkItems);

    // context for save/restore
    uint8_t *context;

    typedef WavefrontParams Params;
    Wavefront(const Params &p);
    ~Wavefront();
    virtual void init();

    void
    setParent(ComputeUnit *cu)
    {
        computeUnit = cu;
    }

    void validateRequestCounters();
    void start(uint64_t _wfDynId, uint64_t _base_ptr);
    void exec();
    // called by SCH stage to reserve
    std::vector<int> reserveResources();
    bool stopFetch();

    Addr pc() const;
    void pc(Addr new_pc);

    VectorMask& execMask();
    bool execMask(int lane) const;


    void discardFetch();

    bool waitCntsSatisfied();
    void setWaitCnts(int vm_wait_cnt, int exp_wait_cnt, int lgkm_wait_cnt);
    void clearWaitCnts();

    void incVMemInstsIssued();
    void incExpInstsIssued();
    void incLGKMInstsIssued();
    void decVMemInstsIssued();
    void decExpInstsIssued();
    void decLGKMInstsIssued();

    /** Freeing VRF space */
    void freeRegisterFile();

    bool sleepDone();
    void setSleepTime(int sleep_time);

    TheGpuISA::GPUISA&
    gpuISA()
    {
        return _gpuISA;
    }

    void barrierId(int bar_id);
    int barrierId() const;
    bool hasBarrier() const;
    void releaseBarrier();

  private:
    TheGpuISA::GPUISA _gpuISA;

    void reserveGmResource(GPUDynInstPtr ii);
    void reserveLmResource(GPUDynInstPtr ii);

    /**
     * the following are used for waitcnt instructions
     * vmWaitCnt: once set, we wait for the oustanding
     *            number of vector mem instructions to be
     *            at, or below vmWaitCnt.
     *
     * expWaitCnt: once set, we wait for the outstanding
     *             number outstanding VM writes or EXP
     *             insts to be at, or below expWaitCnt.
     *
     * lgkmWaitCnt: once set, we wait for the oustanding
     *              number of LDS, GDS, scalar memory,
     *              and message instructions to be at, or
     *              below lgkmCount. we currently do not
     *              support GDS/message ops.
     */
    int vmWaitCnt;
    int expWaitCnt;
    int lgkmWaitCnt;
    int vmemInstsIssued;
    int expInstsIssued;
    int lgkmInstsIssued;
    int sleepCnt;
    status_e status;
    Addr _pc;
    VectorMask _execMask;
    int barId;

  public:
    struct WavefrontStats : public statistics::Group
    {
        WavefrontStats(statistics::Group *parent);

        // Number of instructions executed by this wavefront slot across all
        // dynamic wavefronts
        statistics::Scalar numInstrExecuted;

        // Number of cycles this WF spends in SCH stage
        statistics::Scalar schCycles;

        // Number of stall cycles encounterd by this WF in SCH stage
        statistics::Scalar schStalls;

        // The following stats sum to the value of schStalls, and record, per
        // WF slot, what the cause of each stall was at a coarse granularity.

        // Cycles WF is selected by scheduler, but RFs cannot support
        // instruction
        statistics::Scalar schRfAccessStalls;
        // Cycles spent waiting for execution resources
        statistics::Scalar schResourceStalls;
        // cycles spent waiting for RF reads to complete in SCH stage
        statistics::Scalar schOpdNrdyStalls;
        // LDS arbitration stall cycles. WF attempts to execute LM instruction,
        // but another wave is executing FLAT, which requires LM and GM and
        // forces this WF to stall.
        statistics::Scalar schLdsArbStalls;

        // number of times an instruction of a WF is blocked from being issued
        // due to WAR and WAW dependencies
        statistics::Scalar numTimesBlockedDueWAXDependencies;
        // number of times an instruction of a WF is blocked from being issued
        // due to WAR and WAW dependencies
        statistics::Scalar numTimesBlockedDueRAWDependencies;

        // Distribution to track the distance between producer and consumer
        // for vector register values
        statistics::Distribution vecRawDistance;

        // Distribution to track the number of times every vector register
        // value produced is consumed.
        statistics::Distribution readsPerWrite;
    } stats;
};

} // namespace gem5

#endif // __GPU_COMPUTE_WAVEFRONT_HH__
