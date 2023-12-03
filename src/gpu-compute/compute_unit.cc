/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
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

#include "gpu-compute/compute_unit.hh"

#include <limits>

#include "arch/amdgpu/common/gpu_translation_state.hh"
#include "arch/amdgpu/common/tlb.hh"
#include "base/output.hh"
#include "debug/GPUDisp.hh"
#include "debug/GPUExec.hh"
#include "debug/GPUFetch.hh"
#include "debug/GPUMem.hh"
#include "debug/GPUPort.hh"
#include "debug/GPUPrefetch.hh"
#include "debug/GPUReg.hh"
#include "debug/GPURename.hh"
#include "debug/GPUSync.hh"
#include "debug/GPUTLB.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/gpu_command_processor.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/register_file_cache.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/simple_pool_manager.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#include "sim/sim_exit.hh"

namespace gem5
{

ComputeUnit::ComputeUnit(const Params &p)
    : ClockedObject(p),
      numVectorGlobalMemUnits(p.num_global_mem_pipes),
      numVectorSharedMemUnits(p.num_shared_mem_pipes),
      numScalarMemUnits(p.num_scalar_mem_pipes),
      numVectorALUs(p.num_SIMDs),
      numScalarALUs(p.num_scalar_cores),
      vrfToCoalescerBusWidth(p.vrf_to_coalescer_bus_width),
      coalescerToVrfBusWidth(p.coalescer_to_vrf_bus_width),
      registerManager(p.register_manager),
      fetchStage(p, *this),
      scoreboardCheckStage(p, *this, scoreboardCheckToSchedule),
      scheduleStage(p, *this, scoreboardCheckToSchedule, scheduleToExecute),
      execStage(p, *this, scheduleToExecute),
      globalMemoryPipe(p, *this),
      localMemoryPipe(p, *this),
      scalarMemoryPipe(p, *this),
      tickEvent([this] { exec(); }, "Compute unit tick event", false,
                Event::CPU_Tick_Pri),
      cu_id(p.cu_id),
      vrf(p.vector_register_file),
      srf(p.scalar_register_file),
      rfc(p.register_file_cache),
      simdWidth(p.simd_width),
      spBypassPipeLength(p.spbypass_pipe_length),
      dpBypassPipeLength(p.dpbypass_pipe_length),
      rfcPipeLength(p.rfc_pipe_length),
      scalarPipeStages(p.scalar_pipe_length),
      operandNetworkLength(p.operand_network_length),
      issuePeriod(p.issue_period),
      vrf_gm_bus_latency(p.vrf_gm_bus_latency),
      srf_scm_bus_latency(p.srf_scm_bus_latency),
      vrf_lm_bus_latency(p.vrf_lm_bus_latency),
      perLaneTLB(p.perLaneTLB),
      prefetchDepth(p.prefetch_depth),
      prefetchStride(p.prefetch_stride),
      prefetchType(p.prefetch_prev_type),
      debugSegFault(p.debugSegFault),
      functionalTLB(p.functionalTLB),
      localMemBarrier(p.localMemBarrier),
      countPages(p.countPages),
      req_tick_latency(p.mem_req_latency * p.clk_domain->clockPeriod()),
      resp_tick_latency(p.mem_resp_latency * p.clk_domain->clockPeriod()),
      scalar_req_tick_latency(p.scalar_mem_req_latency *
                              p.clk_domain->clockPeriod()),
      scalar_resp_tick_latency(p.scalar_mem_resp_latency *
                               p.clk_domain->clockPeriod()),
      _requestorId(p.system->getRequestorId(this, "ComputeUnit")),
      lds(*p.localDataStore),
      gmTokenPort(name() + ".gmTokenPort", this),
      ldsPort(csprintf("%s-port", name()), this),
      scalarDataPort(csprintf("%s-port", name()), this),
      scalarDTLBPort(csprintf("%s-port", name()), this),
      sqcPort(csprintf("%s-port", name()), this),
      sqcTLBPort(csprintf("%s-port", name()), this),
      _cacheLineSize(p.system->cacheLineSize()),
      _numBarrierSlots(p.num_barrier_slots),
      globalSeqNum(0),
      wavefrontSize(p.wf_size),
      scoreboardCheckToSchedule(p),
      scheduleToExecute(p),
      stats(this, p.n_wf)
{
    // This is not currently supported and would require adding more handling
    // for system vs. device memory requests on the functional paths, so we
    // fatal immediately in the constructor if this configuration is seen.
    fatal_if(functionalTLB && FullSystem,
             "Functional TLB not supported in full-system GPU simulation");

    /**
     * This check is necessary because std::bitset only provides conversion
     * to unsigned long or unsigned long long via to_ulong() or to_ullong().
     * there are a few places in the code where to_ullong() is used, however
     * if wavefrontSize is larger than a value the host can support then
     * bitset will throw a runtime exception. We should remove all use of
     * to_long() or to_ullong() so we can have wavefrontSize greater than 64b,
     * however until that is done this assert is required.
     */
    fatal_if(p.wf_size > std::numeric_limits<unsigned long long>::digits ||
                 p.wf_size <= 0,
             "WF size is larger than the host can support");
    fatal_if(!isPowerOf2(wavefrontSize),
             "Wavefront size should be a power of 2");
    // calculate how many cycles a vector load or store will need to transfer
    // its data over the corresponding buses
    numCyclesPerStoreTransfer =
        (uint32_t)ceil((double)(wfSize() * sizeof(uint32_t)) /
                       (double)vrfToCoalescerBusWidth);

    numCyclesPerLoadTransfer =
        (wfSize() * sizeof(uint32_t)) / coalescerToVrfBusWidth;

    // Initialization: all WF slots are assumed STOPPED
    idleWfs = p.n_wf * numVectorALUs;
    lastVaddrWF.resize(numVectorALUs);
    wfList.resize(numVectorALUs);

    wfBarrierSlots.resize(p.num_barrier_slots, WFBarrier());

    for (int i = 0; i < p.num_barrier_slots; ++i) {
        freeBarrierIds.insert(i);
    }

    for (int j = 0; j < numVectorALUs; ++j) {
        lastVaddrWF[j].resize(p.n_wf);

        for (int i = 0; i < p.n_wf; ++i) {
            lastVaddrWF[j][i].resize(wfSize());

            wfList[j].push_back(p.wavefronts[j * p.n_wf + i]);
            wfList[j][i]->setParent(this);

            for (int k = 0; k < wfSize(); ++k) {
                lastVaddrWF[j][i][k] = 0;
            }
        }
    }

    lastVaddrSimd.resize(numVectorALUs);

    for (int i = 0; i < numVectorALUs; ++i) {
        lastVaddrSimd[i].resize(wfSize(), 0);
    }

    lastVaddrCU.resize(wfSize());

    lds.setParent(this);

    if (p.execPolicy == "OLDEST-FIRST") {
        exec_policy = EXEC_POLICY::OLDEST;
    } else if (p.execPolicy == "ROUND-ROBIN") {
        exec_policy = EXEC_POLICY::RR;
    } else {
        fatal("Invalid WF execution policy (CU)\n");
    }

    for (int i = 0; i < p.port_memory_port_connection_count; ++i) {
        memPort.emplace_back(csprintf("%s-port%d", name(), i), this, i);
    }

    for (int i = 0; i < p.port_translation_port_connection_count; ++i) {
        tlbPort.emplace_back(csprintf("%s-port%d", name(), i), this, i);
    }

    // Setup tokens for response ports. The number of tokens in memPortTokens
    // is the total token count for the entire vector port (i.e., this CU).
    memPortTokens = new TokenManager(p.max_cu_tokens);

    registerExitCallback([this]() { exitCallback(); });

    lastExecCycle.resize(numVectorALUs, 0);

    for (int i = 0; i < vrf.size(); ++i) {
        vrf[i]->setParent(this);
        rfc[i]->setParent(this);
    }
    for (int i = 0; i < srf.size(); ++i) {
        srf[i]->setParent(this);
    }
    numVecRegsPerSimd = vrf[0]->numRegs();
    numScalarRegsPerSimd = srf[0]->numRegs();

    registerManager->setParent(this);

    activeWaves = 0;

    instExecPerSimd.resize(numVectorALUs, 0);

    // Calculate the number of bits to address a cache line
    panic_if(!isPowerOf2(_cacheLineSize),
             "Cache line size should be a power of two.");
    cacheLineBits = floorLog2(_cacheLineSize);
}

ComputeUnit::~ComputeUnit()
{
    // Delete wavefront slots
    for (int j = 0; j < numVectorALUs; ++j) {
        for (int i = 0; i < shader->n_wf; ++i) {
            delete wfList[j][i];
        }
        lastVaddrSimd[j].clear();
    }
    lastVaddrCU.clear();
}

int
ComputeUnit::numExeUnits() const
{
    return numVectorALUs + numScalarALUs + numVectorGlobalMemUnits +
           numVectorSharedMemUnits + numScalarMemUnits;
}

// index into readyList of the first memory unit
int
ComputeUnit::firstMemUnit() const
{
    return numVectorALUs + numScalarALUs;
}

// index into readyList of the last memory unit
int
ComputeUnit::lastMemUnit() const
{
    return numExeUnits() - 1;
}

// index into scalarALUs vector of SALU used by the wavefront
int
ComputeUnit::mapWaveToScalarAlu(Wavefront *w) const
{
    if (numScalarALUs == 1) {
        return 0;
    } else {
        return w->simdId % numScalarALUs;
    }
}

// index into readyList of Scalar ALU unit used by wavefront
int
ComputeUnit::mapWaveToScalarAluGlobalIdx(Wavefront *w) const
{
    return numVectorALUs + mapWaveToScalarAlu(w);
}

// index into readyList of Global Memory unit used by wavefront
int
ComputeUnit::mapWaveToGlobalMem(Wavefront *w) const
{
    // TODO: FIXME if more than 1 GM pipe supported
    return numVectorALUs + numScalarALUs;
}

// index into readyList of Local Memory unit used by wavefront
int
ComputeUnit::mapWaveToLocalMem(Wavefront *w) const
{
    // TODO: FIXME if more than 1 LM pipe supported
    return numVectorALUs + numScalarALUs + numVectorGlobalMemUnits;
}

// index into readyList of Scalar Memory unit used by wavefront
int
ComputeUnit::mapWaveToScalarMem(Wavefront *w) const
{
    // TODO: FIXME if more than 1 ScM pipe supported
    return numVectorALUs + numScalarALUs + numVectorGlobalMemUnits +
           numVectorSharedMemUnits;
}

void
ComputeUnit::fillKernelState(Wavefront *w, HSAQueueEntry *task)
{
    w->resizeRegFiles(task->numVectorRegs(), task->numScalarRegs());
    w->workGroupSz[0] = task->wgSize(0);
    w->workGroupSz[1] = task->wgSize(1);
    w->workGroupSz[2] = task->wgSize(2);
    w->wgSz = w->workGroupSz[0] * w->workGroupSz[1] * w->workGroupSz[2];
    w->gridSz[0] = task->gridSize(0);
    w->gridSz[1] = task->gridSize(1);
    w->gridSz[2] = task->gridSize(2);
    w->computeActualWgSz(task);
}

void
ComputeUnit::startWavefront(Wavefront *w, int waveId, LdsChunk *ldsChunk,
                            HSAQueueEntry *task, int bar_id, bool fetchContext)
{
    static int _n_wave = 0;

    VectorMask init_mask;
    init_mask.reset();

    for (int k = 0; k < wfSize(); ++k) {
        if (k + waveId * wfSize() < w->actualWgSzTotal)
            init_mask[k] = 1;
    }

    w->execMask() = init_mask;

    w->kernId = task->dispatchId();
    w->wfId = waveId;
    w->initMask = init_mask.to_ullong();

    if (bar_id > WFBarrier::InvalidID) {
        w->barrierId(bar_id);
    } else {
        assert(!w->hasBarrier());
    }

    for (int k = 0; k < wfSize(); ++k) {
        w->workItemId[0][k] = (k + waveId * wfSize()) % w->actualWgSz[0];
        w->workItemId[1][k] =
            ((k + waveId * wfSize()) / w->actualWgSz[0]) % w->actualWgSz[1];
        w->workItemId[2][k] =
            (k + waveId * wfSize()) / (w->actualWgSz[0] * w->actualWgSz[1]);

        w->workItemFlatId[k] =
            w->workItemId[2][k] * w->actualWgSz[0] * w->actualWgSz[1] +
            w->workItemId[1][k] * w->actualWgSz[0] + w->workItemId[0][k];
    }

    // WG state
    w->wgId = task->globalWgId();
    w->dispatchId = task->dispatchId();
    w->workGroupId[0] = w->wgId % task->numWg(0);
    w->workGroupId[1] = (w->wgId / task->numWg(0)) % task->numWg(1);
    w->workGroupId[2] = w->wgId / (task->numWg(0) * task->numWg(1));

    // set the wavefront context to have a pointer to this section of the LDS
    w->ldsChunk = ldsChunk;

    [[maybe_unused]] int32_t refCount =
        lds.increaseRefCounter(w->dispatchId, w->wgId);
    DPRINTF(GPUDisp, "CU%d: increase ref ctr wg[%d] to [%d]\n", cu_id, w->wgId,
            refCount);

    w->instructionBuffer.clear();

    if (w->pendingFetch)
        w->dropFetch = true;

    DPRINTF(GPUDisp,
            "Scheduling wfDynId/barrier_id %d/%d on CU%d: "
            "WF[%d][%d]. Ref cnt:%d\n",
            _n_wave, w->barrierId(), cu_id, w->simdId, w->wfSlotId, refCount);

    w->initRegState(task, w->actualWgSzTotal);
    w->start(_n_wave++, task->codeAddr());

    stats.waveLevelParallelism.sample(activeWaves);
    activeWaves++;

    panic_if(w->wrGmReqsInPipe, "GM write counter for wavefront non-zero\n");
    panic_if(w->rdGmReqsInPipe, "GM read counter for wavefront non-zero\n");
    panic_if(w->wrLmReqsInPipe, "LM write counter for wavefront non-zero\n");
    panic_if(w->rdLmReqsInPipe, "GM read counter for wavefront non-zero\n");
    panic_if(w->outstandingReqs,
             "Outstanding reqs counter for wavefront non-zero\n");
}

/**
 * trigger invalidate operation in the CU
 *
 * req: request initialized in shader, carrying the invalidate flags
 */
void
ComputeUnit::doInvalidate(RequestPtr req, int kernId)
{
    GPUDynInstPtr gpuDynInst = std::make_shared<GPUDynInst>(
        this, nullptr, new KernelLaunchStaticInst(), getAndIncSeqNum());

    // kern_id will be used in inv responses
    gpuDynInst->kern_id = kernId;
    // update contextId field
    req->setContext(gpuDynInst->wfDynId);

    injectGlobalMemFence(gpuDynInst, true, req);
}

/**
 * trigger flush operation in the cu
 *
 * gpuDynInst: inst passed to the request
 */
void
ComputeUnit::doFlush(GPUDynInstPtr gpuDynInst)
{
    injectGlobalMemFence(gpuDynInst, true);
}

/**
 * trigger SQCinvalidate operation in the CU
 *
 * req: request initialized in shader, carrying the invalidate flags
 */
void
ComputeUnit::doSQCInvalidate(RequestPtr req, int kernId)
{
    GPUDynInstPtr gpuDynInst = std::make_shared<GPUDynInst>(
        this, nullptr, new KernelLaunchStaticInst(), getAndIncSeqNum());

    // kern_id will be used in inv responses
    gpuDynInst->kern_id = kernId;
    // update contextId field
    req->setContext(gpuDynInst->wfDynId);

    gpuDynInst->staticInstruction()->setFlag(GPUStaticInst::Scalar);
    scalarMemoryPipe.injectScalarMemFence(gpuDynInst, true, req);
}

// reseting SIMD register pools
// I couldn't think of any other place and
// I think it is needed in my implementation
void
ComputeUnit::resetRegisterPool()
{
    for (int i = 0; i < numVectorALUs; i++) {
        registerManager->vrfPoolMgrs[i]->resetRegion(numVecRegsPerSimd);
        registerManager->srfPoolMgrs[i]->resetRegion(numScalarRegsPerSimd);
    }
}

void
ComputeUnit::dispWorkgroup(HSAQueueEntry *task, int num_wfs_in_wg)
{
    // If we aren't ticking, start it up!
    if (!tickEvent.scheduled()) {
        DPRINTF(GPUDisp, "CU%d: Scheduling wakeup next cycle\n", cu_id);
        schedule(tickEvent, nextCycle());
    }

    // the kernel's invalidate must have finished before any wg dispatch
    assert(task->isInvDone());

    // reserve the LDS capacity allocated to the work group
    // disambiguated by the dispatch ID and workgroup ID, which should be
    // globally unique
    LdsChunk *ldsChunk = lds.reserveSpace(task->dispatchId(),
                                          task->globalWgId(), task->ldsSize());

    panic_if(!ldsChunk, "was not able to reserve space for this WG");

    // calculate the number of 32-bit vector registers required
    // by each work item
    int vregDemand = task->numVectorRegs();
    int sregDemand = task->numScalarRegs();
    int wave_id = 0;

    int barrier_id = WFBarrier::InvalidID;

    /**
     * If this WG only has one WF it will not consume any barrier
     * resources because it has no need of them.
     */
    if (num_wfs_in_wg > 1) {
        /**
         * Find a free barrier slot for this WG. Each WF in the WG will
         * receive the same barrier ID.
         */
        barrier_id = getFreeBarrierId();
        auto &wf_barrier = barrierSlot(barrier_id);
        assert(!wf_barrier.maxBarrierCnt());
        assert(!wf_barrier.numAtBarrier());
        wf_barrier.setMaxBarrierCnt(num_wfs_in_wg);

        DPRINTF(GPUSync,
                "CU[%d] - Dispatching WG with barrier Id%d. "
                "%d waves using this barrier.\n",
                cu_id, barrier_id, num_wfs_in_wg);
    }

    // Assign WFs according to numWfsToSched vector, which is computed by
    // hasDispResources()
    for (int j = 0; j < shader->n_wf; ++j) {
        for (int i = 0; i < numVectorALUs; ++i) {
            Wavefront *w = wfList[i][j];
            // Check if this wavefront slot is available and there are WFs
            // remaining to be dispatched to current SIMD:
            // WF slot must be stopped and not waiting
            // for a release to complete S_RETURNING
            if (w->getStatus() == Wavefront::S_STOPPED &&
                numWfsToSched[i] > 0) {
                // decrement number of WFs awaiting dispatch to current SIMD
                numWfsToSched[i] -= 1;

                fillKernelState(w, task);

                DPRINTF(GPURename,
                        "SIMD[%d] wfSlotId[%d] WF[%d] "
                        "vregDemand[%d] sregDemand[%d]\n",
                        i, j, w->wfDynId, vregDemand, sregDemand);

                registerManager->allocateRegisters(w, vregDemand, sregDemand);

                startWavefront(w, wave_id, ldsChunk, task, barrier_id);
                ++wave_id;
            }
        }
    }
}

void
ComputeUnit::insertInPipeMap(Wavefront *w)
{
    panic_if(w->instructionBuffer.empty(),
             "Instruction Buffer of WF%d can't be empty", w->wgId);
    GPUDynInstPtr ii = w->instructionBuffer.front();
    pipeMap.emplace(ii->seqNum());
}

void
ComputeUnit::deleteFromPipeMap(Wavefront *w)
{
    panic_if(w->instructionBuffer.empty(),
             "Instruction Buffer of WF%d can't be empty", w->wgId);
    GPUDynInstPtr ii = w->instructionBuffer.front();
    // delete the dynamic instruction from the pipeline map
    auto it = pipeMap.find(ii->seqNum());
    panic_if(it == pipeMap.end(), "Pipeline Map is empty\n");
    pipeMap.erase(it);
}

bool
ComputeUnit::hasDispResources(HSAQueueEntry *task, int &num_wfs_in_wg)
{
    // compute true size of workgroup (after clamping to grid size)
    int trueWgSize[HSAQueueEntry::MAX_DIM];
    int trueWgSizeTotal = 1;

    for (int d = 0; d < HSAQueueEntry::MAX_DIM; ++d) {
        trueWgSize[d] =
            std::min(task->wgSize(d),
                     task->gridSize(d) - task->wgId(d) * task->wgSize(d));

        trueWgSizeTotal *= trueWgSize[d];
        DPRINTF(GPUDisp, "trueWgSize[%d] =  %d\n", d, trueWgSize[d]);
    }

    DPRINTF(GPUDisp, "trueWgSizeTotal =  %d\n", trueWgSizeTotal);

    // calculate the number of WFs in this WG
    int numWfs = (trueWgSizeTotal + wfSize() - 1) / wfSize();
    num_wfs_in_wg = numWfs;

    bool barrier_avail = true;

    if (numWfs > 1 && !freeBarrierIds.size()) {
        barrier_avail = false;
    }

    // calculate the number of 32-bit vector registers required by each
    // work item of the work group
    int vregDemandPerWI = task->numVectorRegs();
    // calculate the number of 32-bit scalar registers required by each
    // work item of the work group
    int sregDemandPerWI = task->numScalarRegs();

    // check if the total number of VGPRs snd SGPRs required by all WFs
    // of the WG fit in the VRFs of all SIMD units and the CU's SRF
    panic_if((numWfs * vregDemandPerWI) > (numVectorALUs * numVecRegsPerSimd),
             "WG with %d WFs and %d VGPRs per WI can not be allocated to CU "
             "that has %d VGPRs\n",
             numWfs, vregDemandPerWI, numVectorALUs * numVecRegsPerSimd);
    panic_if((numWfs * sregDemandPerWI) > numScalarRegsPerSimd,
             "WG with %d WFs and %d SGPRs per WI can not be scheduled to CU "
             "with %d SGPRs\n",
             numWfs, sregDemandPerWI, numScalarRegsPerSimd);

    // number of WF slots that are not occupied
    int freeWfSlots = 0;
    // number of Wfs from WG that were successfully mapped to a SIMD
    int numMappedWfs = 0;
    numWfsToSched.clear();
    numWfsToSched.resize(numVectorALUs, 0);

    // attempt to map WFs to the SIMDs, based on WF slot availability
    // and register file availability
    for (int j = 0; j < shader->n_wf; ++j) {
        for (int i = 0; i < numVectorALUs; ++i) {
            if (wfList[i][j]->getStatus() == Wavefront::S_STOPPED) {
                ++freeWfSlots;
                // check if current WF will fit onto current SIMD/VRF
                // if all WFs have not yet been mapped to the SIMDs
                if (numMappedWfs < numWfs &&
                    registerManager->canAllocateSgprs(i, numWfsToSched[i] + 1,
                                                      sregDemandPerWI) &&
                    registerManager->canAllocateVgprs(i, numWfsToSched[i] + 1,
                                                      vregDemandPerWI)) {
                    numWfsToSched[i]++;
                    numMappedWfs++;
                }
            }
        }
    }

    // check that the number of mapped WFs is not greater
    // than the actual number of WFs
    assert(numMappedWfs <= numWfs);

    bool vregAvail = true;
    bool sregAvail = true;
    // if a WF to SIMD mapping was not found, find the limiting resource
    if (numMappedWfs < numWfs) {
        for (int j = 0; j < numVectorALUs; ++j) {
            // find if there are enough free VGPRs in the SIMD's VRF
            // to accomodate the WFs of the new WG that would be mapped
            // to this SIMD unit
            vregAvail &= registerManager->canAllocateVgprs(j, numWfsToSched[j],
                                                           vregDemandPerWI);
            // find if there are enough free SGPRs in the SIMD's SRF
            // to accomodate the WFs of the new WG that would be mapped
            // to this SIMD unit
            sregAvail &= registerManager->canAllocateSgprs(j, numWfsToSched[j],
                                                           sregDemandPerWI);
        }
    }

    DPRINTF(GPUDisp, "Free WF slots =  %d, Mapped WFs = %d, \
            VGPR Availability = %d, SGPR Availability = %d\n",
            freeWfSlots, numMappedWfs, vregAvail, sregAvail);

    if (!vregAvail) {
        ++stats.numTimesWgBlockedDueVgprAlloc;
    }

    if (!sregAvail) {
        ++stats.numTimesWgBlockedDueSgprAlloc;
    }

    // Return true if enough WF slots to submit workgroup and if there are
    // enough VGPRs to schedule all WFs to their SIMD units
    bool ldsAvail = lds.canReserve(task->ldsSize());
    if (!ldsAvail) {
        stats.wgBlockedDueLdsAllocation++;
    }

    if (!barrier_avail) {
        stats.wgBlockedDueBarrierAllocation++;
    }

    // Return true if the following are all true:
    // (a) all WFs of the WG were mapped to free WF slots
    // (b) there are enough VGPRs to schedule all WFs to their SIMD units
    // (c) there are enough SGPRs on the CU to schedule all WFs
    // (d) there is enough space in LDS to allocate for all WFs
    bool can_dispatch = numMappedWfs == numWfs && vregAvail && sregAvail &&
                        ldsAvail && barrier_avail;
    return can_dispatch;
}

int
ComputeUnit::numYetToReachBarrier(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    return wf_barrier.numYetToReachBarrier();
}

bool
ComputeUnit::allAtBarrier(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    return wf_barrier.allAtBarrier();
}

void
ComputeUnit::incNumAtBarrier(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    wf_barrier.incNumAtBarrier();
}

int
ComputeUnit::numAtBarrier(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    return wf_barrier.numAtBarrier();
}

int
ComputeUnit::maxBarrierCnt(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    return wf_barrier.maxBarrierCnt();
}

void
ComputeUnit::resetBarrier(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    wf_barrier.reset();
}

void
ComputeUnit::decMaxBarrierCnt(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    wf_barrier.decMaxBarrierCnt();
}

void
ComputeUnit::releaseBarrier(int bar_id)
{
    auto &wf_barrier = barrierSlot(bar_id);
    wf_barrier.release();
    freeBarrierIds.insert(bar_id);
}

void
ComputeUnit::releaseWFsFromBarrier(int bar_id)
{
    for (int i = 0; i < numVectorALUs; ++i) {
        for (int j = 0; j < shader->n_wf; ++j) {
            Wavefront *wf = wfList[i][j];
            if (wf->barrierId() == bar_id) {
                assert(wf->getStatus() == Wavefront::S_BARRIER);
                wf->setStatus(Wavefront::S_RUNNING);
            }
        }
    }
}

// Execute one clock worth of work on the ComputeUnit.
void
ComputeUnit::exec()
{
    // process reads and writes in the RFs
    for (auto &vecRegFile : vrf) {
        vecRegFile->exec();
    }

    for (auto &scRegFile : srf) {
        scRegFile->exec();
    }

    // Execute pipeline stages in reverse order to simulate
    // the pipeline latency
    scalarMemoryPipe.exec();
    globalMemoryPipe.exec();
    localMemoryPipe.exec();
    execStage.exec();
    scheduleStage.exec();
    scoreboardCheckStage.exec();
    fetchStage.exec();

    stats.totalCycles++;

    // Put this CU to sleep if there is no more work to be done.
    if (!isDone()) {
        schedule(tickEvent, nextCycle());
    } else {
        shader->notifyCuSleep();
        DPRINTF(GPUDisp, "CU%d: Going to sleep\n", cu_id);
    }
}

void
ComputeUnit::init()
{
    // Initialize CU Bus models and execution resources

    // Vector ALUs
    vectorALUs.clear();
    for (int i = 0; i < numVectorALUs; i++) {
        vectorALUs.emplace_back(this, clockPeriod());
    }

    // Scalar ALUs
    scalarALUs.clear();
    for (int i = 0; i < numScalarALUs; i++) {
        scalarALUs.emplace_back(this, clockPeriod());
    }

    // Vector Global Memory
    fatal_if(numVectorGlobalMemUnits > 1,
             "No support for multiple Global Memory Pipelines exists!!!");
    vectorGlobalMemUnit.init(this, clockPeriod());
    vrfToGlobalMemPipeBus.init(this, clockPeriod());
    glbMemToVrfBus.init(this, clockPeriod());

    // Vector Local/Shared Memory
    fatal_if(numVectorSharedMemUnits > 1,
             "No support for multiple Local Memory Pipelines exists!!!");
    vectorSharedMemUnit.init(this, clockPeriod());
    vrfToLocalMemPipeBus.init(this, clockPeriod());
    locMemToVrfBus.init(this, clockPeriod());

    // Scalar Memory
    fatal_if(numScalarMemUnits > 1,
             "No support for multiple Scalar Memory Pipelines exists!!!");
    scalarMemUnit.init(this, clockPeriod());
    srfToScalarMemPipeBus.init(this, clockPeriod());
    scalarMemToSrfBus.init(this, clockPeriod());

    vectorRegsReserved.resize(numVectorALUs, 0);
    scalarRegsReserved.resize(numVectorALUs, 0);

    fetchStage.init();
    scheduleStage.init();
    execStage.init();
    globalMemoryPipe.init();

    gmTokenPort.setTokenManager(memPortTokens);
}

bool
ComputeUnit::DataPort::recvTimingResp(PacketPtr pkt)
{
    return handleResponse(pkt);
}

bool
ComputeUnit::DataPort::handleResponse(PacketPtr pkt)
{
    // Ruby has completed the memory op. Schedule the mem_resp_event at the
    // appropriate cycle to process the timing memory response
    // This delay represents the pipeline delay
    SenderState *sender_state = safe_cast<SenderState *>(pkt->senderState);
    PortID index = sender_state->port_index;
    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    GPUDispatcher &dispatcher = computeUnit->shader->dispatcher();

    // MemSyncResp + WriteAckResp are handled completely here and we don't
    // schedule a MemRespEvent to process the responses further
    if (pkt->cmd == MemCmd::MemSyncResp) {
        // This response is for 1 of the following request types:
        //  - kernel launch
        //  - kernel end
        //  - non-kernel mem sync

        // Kernel Launch
        // wavefront was nullptr when launching kernel, so it is meaningless
        // here (simdId=-1, wfSlotId=-1)
        if (gpuDynInst->isKernelLaunch()) {
            // for kernel launch, the original request must be both kernel-type
            // and INV_L1
            assert(pkt->req->isKernel());
            assert(pkt->req->isInvL1());

            // one D-Cache inv is done, decrement counter
            dispatcher.updateInvCounter(gpuDynInst->kern_id);

            delete pkt->senderState;
            delete pkt;
            return true;
        }

        // retrieve wavefront from inst
        Wavefront *w = gpuDynInst->wavefront();

        // Check if we are waiting on Kernel End Flush
        if (w->getStatus() == Wavefront::S_RETURNING &&
            gpuDynInst->isEndOfKernel()) {
            // for kernel end, the original request must be both kernel-type
            // and last-level GPU cache should be flushed if it contains
            // dirty data.  This request may have been quiesced and
            // immediately responded to if the GL2 is a write-through /
            // read-only cache.
            assert(pkt->req->isKernel());
            assert(pkt->req->isGL2CacheFlush());

            // once flush done, decrement counter, and return whether all
            // dirty writeback operations are done for the kernel
            bool isWbDone = dispatcher.updateWbCounter(gpuDynInst->kern_id);

            // not all wbs are done for the kernel, just release pkt
            // resources
            if (!isWbDone) {
                delete pkt->senderState;
                delete pkt;
                return true;
            }

            // all wbs are completed for the kernel, do retirement work
            // for the workgroup
            DPRINTF(GPUDisp, "CU%d: WF[%d][%d][wv=%d]: WG %d completed\n",
                    computeUnit->cu_id, w->simdId, w->wfSlotId, w->wfDynId,
                    w->wgId);

            dispatcher.notifyWgCompl(w);
            w->setStatus(Wavefront::S_STOPPED);
        }

        if (!pkt->req->isKernel()) {
            w = computeUnit->wfList[gpuDynInst->simdId][gpuDynInst->wfSlotId];
            DPRINTF(GPUExec,
                    "MemSyncResp: WF[%d][%d] WV%d %s decrementing "
                    "outstanding reqs %d => %d\n",
                    gpuDynInst->simdId, gpuDynInst->wfSlotId,
                    gpuDynInst->wfDynId, gpuDynInst->disassemble(),
                    w->outstandingReqs, w->outstandingReqs - 1);
            computeUnit->globalMemoryPipe.handleResponse(gpuDynInst);
        }

        delete pkt->senderState;
        delete pkt;
        return true;
    }

    EventFunctionWrapper *mem_resp_event =
        computeUnit->memPort[index].createMemRespEvent(pkt);

    DPRINTF(GPUPort,
            "CU%d: WF[%d][%d]: gpuDynInst: %d, index %d, addr %#x received!\n",
            computeUnit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
            gpuDynInst->seqNum(), index, pkt->req->getPaddr());

    computeUnit->schedule(mem_resp_event,
                          curTick() + computeUnit->resp_tick_latency);

    return true;
}

bool
ComputeUnit::ScalarDataPort::recvTimingResp(PacketPtr pkt)
{
    return handleResponse(pkt);
}

bool
ComputeUnit::ScalarDataPort::handleResponse(PacketPtr pkt)
{
    assert(!pkt->req->isKernel());

    // retrieve sender state
    SenderState *sender_state = safe_cast<SenderState *>(pkt->senderState);
    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;

    assert(pkt->isRead() || pkt->isWrite());
    assert(gpuDynInst->numScalarReqs > 0);

    gpuDynInst->numScalarReqs--;

    /**
     * for each returned scalar request we decrement the
     * numScalarReqs counter that is associated with this
     * gpuDynInst, which should have been set to correspond
     * to the number of packets sent for the memory op.
     * once all packets return, the memory op is finished
     * and we can push it into the response queue.
     */
    if (!gpuDynInst->numScalarReqs) {
        if (gpuDynInst->isLoad() || gpuDynInst->isAtomic()) {
            computeUnit->scalarMemoryPipe.getGMLdRespFIFO().push(gpuDynInst);
        } else {
            computeUnit->scalarMemoryPipe.getGMStRespFIFO().push(gpuDynInst);
        }
    }

    delete pkt->senderState;
    delete pkt;

    return true;
}

void
ComputeUnit::ScalarDataPort::recvReqRetry()
{
    for (const auto &pkt : retries) {
        if (!sendTimingReq(pkt)) {
            break;
        } else {
            retries.pop_front();
        }
    }
}

void
ComputeUnit::DataPort::recvReqRetry()
{
    int len = retries.size();

    assert(len > 0);

    for (int i = 0; i < len; ++i) {
        PacketPtr pkt = retries.front().first;
        [[maybe_unused]] GPUDynInstPtr gpuDynInst = retries.front().second;
        DPRINTF(GPUMem, "CU%d: WF[%d][%d]: retry mem inst addr %#x\n",
                computeUnit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                pkt->req->getPaddr());

        /** Currently Ruby can return false due to conflicts for the particular
         *  cache block or address.  Thus other requests should be allowed to
         *  pass and the data port should expect multiple retries. */
        if (!sendTimingReq(pkt)) {
            DPRINTF(GPUMem, "failed again!\n");
            break;
        } else {
            DPRINTF(GPUMem, "successful!\n");
            retries.pop_front();
        }
    }
}

bool
ComputeUnit::SQCPort::recvTimingResp(PacketPtr pkt)
{
    SenderState *sender_state = safe_cast<SenderState *>(pkt->senderState);
    /** Process the response only if there is a wavefront associated with it.
     * Otherwise, it is from SQC invalidate that was issued at kernel start
     * and doesn't have a wavefront or instruction associated with it.
     */
    if (sender_state->wavefront != nullptr) {
        computeUnit->handleSQCReturn(pkt);
    }

    return true;
}

void
ComputeUnit::handleSQCReturn(PacketPtr pkt)
{
    fetchStage.processFetchReturn(pkt);
}

void
ComputeUnit::SQCPort::recvReqRetry()
{
    int len = retries.size();

    assert(len > 0);

    for (int i = 0; i < len; ++i) {
        PacketPtr pkt = retries.front().first;
        [[maybe_unused]] Wavefront *wavefront = retries.front().second;
        DPRINTF(GPUFetch, "CU%d: WF[%d][%d]: retrying FETCH addr %#x\n",
                computeUnit->cu_id, wavefront->simdId, wavefront->wfSlotId,
                pkt->req->getPaddr());
        if (!sendTimingReq(pkt)) {
            DPRINTF(GPUFetch, "failed again!\n");
            break;
        } else {
            DPRINTF(GPUFetch, "successful!\n");
            retries.pop_front();
        }
    }
}

const char *
ComputeUnit::SQCPort::MemReqEvent::description() const
{
    return "ComputeUnit SQC memory request event";
}

void
ComputeUnit::SQCPort::MemReqEvent::process()
{
    SenderState *sender_state = safe_cast<SenderState *>(pkt->senderState);
    [[maybe_unused]] ComputeUnit *compute_unit = sqcPort.computeUnit;

    assert(!pkt->req->systemReq());

    if (!(sqcPort.sendTimingReq(pkt))) {
        sqcPort.retries.push_back(
            std::pair<PacketPtr, Wavefront *>(pkt, sender_state->wavefront));
    }
}

void
ComputeUnit::sendRequest(GPUDynInstPtr gpuDynInst, PortID index, PacketPtr pkt)
{
    // There must be a way around this check to do the globalMemStart...
    Addr tmp_vaddr = pkt->req->getVaddr();

    updatePageDivergenceDist(tmp_vaddr);

    // set PC in request
    pkt->req->setPC(gpuDynInst->wavefront()->pc());

    pkt->req->setReqInstSeqNum(gpuDynInst->seqNum());

    // figure out the type of the request to set read/write
    BaseMMU::Mode TLB_mode;
    assert(pkt->isRead() || pkt->isWrite());

    // only do some things if actually accessing data
    bool isDataAccess = pkt->isWrite() || pkt->isRead();

    // For dGPUs, real hardware will extract MTYPE from the PTE. SE mode
    // uses x86 pagetables which don't have fields to track GPU MTYPEs.
    // Rather than hacking up the pagetable to add these bits in, we just
    // keep a structure local to our GPUs that are populated in our
    // emulated driver whenever memory is allocated.  Consult that structure
    // here in case we need a memtype override.
    //
    // In full system mode these can be extracted from the PTE and assigned
    // after address translation takes place.
    if (!FullSystem) {
        shader->gpuCmdProc.driver()->setMtype(pkt->req);
    }

    // Check write before read for atomic operations
    // since atomic operations should use BaseMMU::Write
    if (pkt->isWrite()) {
        TLB_mode = BaseMMU::Write;
    } else if (pkt->isRead()) {
        TLB_mode = BaseMMU::Read;
    } else {
        fatal("pkt is not a read nor a write\n");
    }

    if (!functionalTLB) {
        stats.tlbCycles -= curTick();
    }
    ++stats.tlbRequests;

    PortID tlbPort_index = perLaneTLB ? index : 0;

    if (shader->timingSim) {
        if (!FullSystem && debugSegFault) {
            Process *p = shader->gpuTc->getProcessPtr();
            Addr vaddr = pkt->req->getVaddr();
            unsigned size = pkt->getSize();

            if ((vaddr + size - 1) % 64 < vaddr % 64) {
                panic("CU%d: WF[%d][%d]: Access to addr %#x is unaligned!\n",
                      cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, vaddr);
            }

            Addr paddr;

            if (!p->pTable->translate(vaddr, paddr)) {
                if (!p->fixupFault(vaddr)) {
                    panic("CU%d: WF[%d][%d]: Fault on addr %#x!\n", cu_id,
                          gpuDynInst->simdId, gpuDynInst->wfSlotId, vaddr);
                }
            }
        }

        // This is the SenderState needed upon return
        pkt->senderState = new DTLBPort::SenderState(gpuDynInst, index);

        // This is the senderState needed by the TLB hierarchy to function
        GpuTranslationState *translation_state = new GpuTranslationState(
            TLB_mode, shader->gpuTc, false, pkt->senderState);

        pkt->senderState = translation_state;

        if (functionalTLB) {
            tlbPort[tlbPort_index].sendFunctional(pkt);

            // update the hitLevel distribution
            int hit_level = translation_state->hitLevel;
            assert(hit_level != -1);
            stats.hitsPerTLBLevel[hit_level]++;

            // New SenderState for the memory access
            GpuTranslationState *sender_state =
                safe_cast<GpuTranslationState *>(pkt->senderState);

            delete sender_state->tlbEntry;
            delete sender_state->saved;
            delete sender_state;

            assert(pkt->req->hasPaddr());
            assert(pkt->req->hasSize());

            // this is necessary because the GPU TLB receives packets instead
            // of requests. when the translation is complete, all relevent
            // fields in the request will be populated, but not in the packet.
            // here we create the new packet so we can set the size, addr,
            // and proper flags.
            PacketPtr oldPkt = pkt;
            pkt = new Packet(oldPkt->req, oldPkt->cmd);
            if (isDataAccess) {
                uint8_t *tmpData = oldPkt->getPtr<uint8_t>();
                pkt->dataStatic(tmpData);
            }
            delete oldPkt;

            // New SenderState for the memory access
            pkt->senderState = new ComputeUnit::DataPort::SenderState(
                gpuDynInst, index, nullptr);

            gpuDynInst->memStatusVector[pkt->getAddr()].push_back(index);
            gpuDynInst->tlbHitLevel[index] = hit_level;

            // translation is done. Schedule the mem_req_event at the
            // appropriate cycle to send the timing memory request to ruby
            EventFunctionWrapper *mem_req_event =
                memPort[index].createMemReqEvent(pkt);

            DPRINTF(GPUPort,
                    "CU%d: WF[%d][%d]: index %d, addr %#x data "
                    "scheduled\n",
                    cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, index,
                    pkt->req->getPaddr());

            schedule(mem_req_event, curTick() + req_tick_latency);
        } else if (tlbPort[tlbPort_index].isStalled()) {
            assert(tlbPort[tlbPort_index].retries.size() > 0);

            DPRINTF(GPUTLB,
                    "CU%d: WF[%d][%d]: Translation for addr %#x "
                    "failed!\n",
                    cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                    tmp_vaddr);

            tlbPort[tlbPort_index].retries.push_back(pkt);
        } else if (!tlbPort[tlbPort_index].sendTimingReq(pkt)) {
            // Stall the data port;
            // No more packet will be issued till
            // ruby indicates resources are freed by
            // a recvReqRetry() call back on this port.
            tlbPort[tlbPort_index].stallPort();

            DPRINTF(GPUTLB,
                    "CU%d: WF[%d][%d]: Translation for addr %#x "
                    "failed!\n",
                    cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                    tmp_vaddr);

            tlbPort[tlbPort_index].retries.push_back(pkt);
        } else {
            DPRINTF(GPUTLB,
                    "CU%d: WF[%d][%d]: Translation for addr %#x from "
                    "instruction %s sent!\n",
                    cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, tmp_vaddr,
                    gpuDynInst->disassemble().c_str());
        }
    } else {
        if (pkt->cmd == MemCmd::MemSyncReq) {
            gpuDynInst->resetEntireStatusVector();
        } else {
            gpuDynInst->decrementStatusVector(index);
        }

        // New SenderState for the memory access
        delete pkt->senderState;

        // Because it's atomic operation, only need TLB translation state
        pkt->senderState = new GpuTranslationState(TLB_mode, shader->gpuTc);

        tlbPort[tlbPort_index].sendFunctional(pkt);

        // the addr of the packet is not modified, so we need to create a new
        // packet, or otherwise the memory access will have the old virtual
        // address sent in the translation packet, instead of the physical
        // address returned by the translation.
        PacketPtr new_pkt = new Packet(pkt->req, pkt->cmd);
        new_pkt->dataStatic(pkt->getPtr<uint8_t>());

        // Translation is done. It is safe to send the packet to memory.
        memPort[0].sendFunctional(new_pkt);

        DPRINTF(GPUMem, "Functional sendRequest\n");
        DPRINTF(GPUMem, "CU%d: WF[%d][%d]: index %d: addr %#x\n", cu_id,
                gpuDynInst->simdId, gpuDynInst->wfSlotId, index,
                new_pkt->req->getPaddr());

        // safe_cast the senderState
        GpuTranslationState *sender_state =
            safe_cast<GpuTranslationState *>(pkt->senderState);

        delete sender_state->tlbEntry;
        delete new_pkt;
        delete pkt->senderState;
        delete pkt;
    }
}

void
ComputeUnit::sendScalarRequest(GPUDynInstPtr gpuDynInst, PacketPtr pkt)
{
    assert(pkt->isWrite() || pkt->isRead());

    BaseMMU::Mode tlb_mode = pkt->isRead() ? BaseMMU::Read : BaseMMU::Write;

    pkt->senderState =
        new ComputeUnit::ScalarDTLBPort::SenderState(gpuDynInst);

    pkt->senderState = new GpuTranslationState(tlb_mode, shader->gpuTc, false,
                                               pkt->senderState);

    if (scalarDTLBPort.isStalled()) {
        assert(scalarDTLBPort.retries.size());
        scalarDTLBPort.retries.push_back(pkt);
    } else if (!scalarDTLBPort.sendTimingReq(pkt)) {
        scalarDTLBPort.stallPort();
        scalarDTLBPort.retries.push_back(pkt);
    } else {
        DPRINTF(GPUTLB, "sent scalar %s translation request for addr %#x\n",
                tlb_mode == BaseMMU::Read ? "read" : "write",
                pkt->req->getVaddr());
    }
}

void
ComputeUnit::injectGlobalMemFence(GPUDynInstPtr gpuDynInst, bool kernelMemSync,
                                  RequestPtr req)
{
    assert(gpuDynInst->isGlobalSeg() ||
           gpuDynInst->executedAs() == enums::SC_GLOBAL);

    // Fences will never be issued to system memory, so we can mark the
    // requestor as a device memory ID here.
    if (!req) {
        req = std::make_shared<Request>(0, 0, 0, vramRequestorId(), 0,
                                        gpuDynInst->wfDynId);
    } else {
        req->requestorId(vramRequestorId());
    }

    // all mem sync requests have Paddr == 0
    req->setPaddr(0);

    PacketPtr pkt = nullptr;

    if (kernelMemSync) {
        if (gpuDynInst->isKernelLaunch()) {
            req->setCacheCoherenceFlags(Request::INV_L1);
            req->setReqInstSeqNum(gpuDynInst->seqNum());
            req->setFlags(Request::KERNEL);
            pkt = new Packet(req, MemCmd::MemSyncReq);
            pkt->pushSenderState(new ComputeUnit::DataPort::SenderState(
                gpuDynInst, 0, nullptr));

            EventFunctionWrapper *mem_req_event =
                memPort[0].createMemReqEvent(pkt);

            DPRINTF(GPUPort,
                    "CU%d: WF[%d][%d]: index %d, addr %#x scheduling "
                    "an acquire\n",
                    cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, 0,
                    pkt->req->getPaddr());

            schedule(mem_req_event, curTick() + req_tick_latency);
        } else {
            // kernel end flush of GL2 cache may be quiesced by Ruby if the
            // GL2 is a read-only cache
            assert(shader->impl_kern_end_rel);
            assert(gpuDynInst->isEndOfKernel());

            req->setCacheCoherenceFlags(Request::FLUSH_L2);
            req->setReqInstSeqNum(gpuDynInst->seqNum());
            req->setFlags(Request::KERNEL);
            pkt = new Packet(req, MemCmd::MemSyncReq);
            pkt->pushSenderState(new ComputeUnit::DataPort::SenderState(
                gpuDynInst, 0, nullptr));

            EventFunctionWrapper *mem_req_event =
                memPort[0].createMemReqEvent(pkt);

            DPRINTF(GPUPort,
                    "CU%d: WF[%d][%d]: index %d, addr %#x scheduling "
                    "a release\n",
                    cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, 0,
                    pkt->req->getPaddr());

            schedule(mem_req_event, curTick() + req_tick_latency);
        }
    } else {
        gpuDynInst->setRequestFlags(req);

        req->setReqInstSeqNum(gpuDynInst->seqNum());

        pkt = new Packet(req, MemCmd::MemSyncReq);
        pkt->pushSenderState(
            new ComputeUnit::DataPort::SenderState(gpuDynInst, 0, nullptr));

        EventFunctionWrapper *mem_req_event =
            memPort[0].createMemReqEvent(pkt);

        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: index %d, addr %#x sync scheduled\n", cu_id,
                gpuDynInst->simdId, gpuDynInst->wfSlotId, 0,
                pkt->req->getPaddr());

        schedule(mem_req_event, curTick() + req_tick_latency);
    }
}

void
ComputeUnit::DataPort::processMemRespEvent(PacketPtr pkt)
{
    DataPort::SenderState *sender_state =
        safe_cast<DataPort::SenderState *>(pkt->senderState);

    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    ComputeUnit *compute_unit = computeUnit;

    assert(gpuDynInst);

    DPRINTF(GPUPort, "CU%d: WF[%d][%d]: Response for addr %#x, index %d\n",
            compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
            pkt->req->getPaddr(), id);

    Addr paddr = pkt->req->getPaddr();

    // mem sync resp callback must be handled already in
    // DataPort::recvTimingResp
    assert(pkt->cmd != MemCmd::MemSyncResp);

    // The status vector and global memory response for WriteResp packets get
    // handled by the WriteCompleteResp packets.
    if (pkt->cmd == MemCmd::WriteResp) {
        if (!FullSystem || !pkt->req->systemReq()) {
            delete pkt;
            return;
        }
    }

    // this is for read, write and atomic
    int index = gpuDynInst->memStatusVector[paddr].back();

    DPRINTF(GPUMem, "Response for addr %#x, index %d\n", pkt->req->getPaddr(),
            id);

    gpuDynInst->memStatusVector[paddr].pop_back();
    gpuDynInst->pAddr = pkt->req->getPaddr();

    gpuDynInst->decrementStatusVector(index);
    DPRINTF(GPUMem, "bitvector is now %s\n", gpuDynInst->printStatusVector());

    if (gpuDynInst->allLanesZero()) {
        auto iter = gpuDynInst->memStatusVector.begin();
        auto end = gpuDynInst->memStatusVector.end();

        while (iter != end) {
            assert(iter->second.empty());
            ++iter;
        }

        // Calculate the difference between the arrival of the first cache
        // block and the last cache block to arrive if we have the time
        // for the first cache block.
        if (compute_unit->headTailMap.count(gpuDynInst)) {
            Tick headTick = compute_unit->headTailMap.at(gpuDynInst);
            compute_unit->stats.headTailLatency.sample(curTick() - headTick);
            compute_unit->headTailMap.erase(gpuDynInst);
        }

        gpuDynInst->memStatusVector.clear();

        gpuDynInst->profileRoundTripTime(curTick(), InstMemoryHop::GMEnqueue);
        compute_unit->globalMemoryPipe.handleResponse(gpuDynInst);

        DPRINTF(GPUMem, "CU%d: WF[%d][%d]: packet totally complete\n",
                compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId);
    } else {
        if (pkt->isRead()) {
            if (!compute_unit->headTailMap.count(gpuDynInst)) {
                compute_unit->headTailMap.insert(
                    std::make_pair(gpuDynInst, curTick()));
            }
        }
    }

    delete pkt->senderState;
    delete pkt;
}

bool
ComputeUnit::DTLBPort::recvTimingResp(PacketPtr pkt)
{
    Addr line = pkt->req->getPaddr();

    DPRINTF(GPUTLB, "CU%d: DTLBPort received %#x->%#x\n", computeUnit->cu_id,
            pkt->req->getVaddr(), line);

    assert(pkt->senderState);
    computeUnit->stats.tlbCycles += curTick();

    // pop off the TLB translation state
    GpuTranslationState *translation_state =
        safe_cast<GpuTranslationState *>(pkt->senderState);

    // no PageFaults are permitted for data accesses
    if (!translation_state->tlbEntry) {
        DTLBPort::SenderState *sender_state =
            safe_cast<DTLBPort::SenderState *>(translation_state->saved);

        [[maybe_unused]] Wavefront *w =
            computeUnit->wfList[sender_state->_gpuDynInst->simdId]
                               [sender_state->_gpuDynInst->wfSlotId];

        DPRINTFN("Wave %d couldn't tranlate vaddr %#x\n", w->wfDynId,
                 pkt->req->getVaddr());
    }

    // update the hitLevel distribution
    int hit_level = translation_state->hitLevel;
    computeUnit->stats.hitsPerTLBLevel[hit_level]++;

    delete translation_state->tlbEntry;
    assert(!translation_state->ports.size());
    pkt->senderState = translation_state->saved;

    // for prefetch pkt
    BaseMMU::Mode TLB_mode = translation_state->tlbMode;

    delete translation_state;

    // use the original sender state to know how to close this transaction
    DTLBPort::SenderState *sender_state =
        safe_cast<DTLBPort::SenderState *>(pkt->senderState);

    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    PortID mp_index = sender_state->portIndex;
    Addr vaddr = pkt->req->getVaddr();
    gpuDynInst->memStatusVector[line].push_back(mp_index);
    gpuDynInst->tlbHitLevel[mp_index] = hit_level;

    MemCmd requestCmd;

    if (pkt->cmd == MemCmd::ReadResp) {
        requestCmd = MemCmd::ReadReq;
    } else if (pkt->cmd == MemCmd::WriteResp) {
        requestCmd = MemCmd::WriteReq;
    } else if (pkt->cmd == MemCmd::SwapResp) {
        requestCmd = MemCmd::SwapReq;
    } else {
        panic("unsupported response to request conversion %s\n",
              pkt->cmd.toString());
    }

    if (computeUnit->prefetchDepth) {
        int simdId = gpuDynInst->simdId;
        int wfSlotId = gpuDynInst->wfSlotId;
        Addr last = 0;

        switch (computeUnit->prefetchType) {
        case enums::PF_CU:
            last = computeUnit->lastVaddrCU[mp_index];
            break;
        case enums::PF_PHASE:
            last = computeUnit->lastVaddrSimd[simdId][mp_index];
            break;
        case enums::PF_WF:
            last = computeUnit->lastVaddrWF[simdId][wfSlotId][mp_index];
        default:
            break;
        }

        DPRINTF(GPUPrefetch, "CU[%d][%d][%d][%d]: %#x was last\n",
                computeUnit->cu_id, simdId, wfSlotId, mp_index, last);

        int stride = last ? (roundDown(vaddr, X86ISA::PageBytes) -
                             roundDown(last, X86ISA::PageBytes)) >>
                                X86ISA::PageShift :
                            0;

        DPRINTF(GPUPrefetch, "Stride is %d\n", stride);

        computeUnit->lastVaddrCU[mp_index] = vaddr;
        computeUnit->lastVaddrSimd[simdId][mp_index] = vaddr;
        computeUnit->lastVaddrWF[simdId][wfSlotId][mp_index] = vaddr;

        stride = (computeUnit->prefetchType == enums::PF_STRIDE) ?
                     computeUnit->prefetchStride :
                     stride;

        DPRINTF(GPUPrefetch, "%#x to: CU[%d][%d][%d][%d]\n", vaddr,
                computeUnit->cu_id, simdId, wfSlotId, mp_index);

        DPRINTF(GPUPrefetch, "Prefetching from %#x:", vaddr);

        // Prefetch Next few pages atomically
        for (int pf = 1; pf <= computeUnit->prefetchDepth; ++pf) {
            DPRINTF(GPUPrefetch, "%d * %d: %#x\n", pf, stride,
                    vaddr + stride * pf * X86ISA::PageBytes);

            if (!stride)
                break;

            RequestPtr prefetch_req = std::make_shared<Request>(
                vaddr + stride * pf * X86ISA::PageBytes, sizeof(uint8_t), 0,
                computeUnit->requestorId(), 0, 0, nullptr);

            PacketPtr prefetch_pkt = new Packet(prefetch_req, requestCmd);
            uint8_t foo = 0;
            prefetch_pkt->dataStatic(&foo);

            // Because it's atomic operation, only need TLB translation state
            prefetch_pkt->senderState = new GpuTranslationState(
                TLB_mode, computeUnit->shader->gpuTc, true);

            // Currently prefetches are zero-latency, hence the sendFunctional
            sendFunctional(prefetch_pkt);

            /* safe_cast the senderState */
            GpuTranslationState *tlb_state =
                safe_cast<GpuTranslationState *>(prefetch_pkt->senderState);

            delete tlb_state->tlbEntry;
            delete tlb_state;
            delete prefetch_pkt;
        }
    }

    // First we must convert the response cmd back to a request cmd so that
    // the request can be sent through the cu's request port
    PacketPtr new_pkt = new Packet(pkt->req, requestCmd);
    new_pkt->dataStatic(pkt->getPtr<uint8_t>());
    delete pkt->senderState;
    delete pkt;

    // New SenderState for the memory access
    new_pkt->senderState =
        new ComputeUnit::DataPort::SenderState(gpuDynInst, mp_index, nullptr);

    // Set VRAM ID for device requests
    // For now, system vmem requests use functional reads. This is not that
    // critical to model as the region of interest should always be accessing
    // device memory. System vmem requests are used by blit kernels to do
    // memcpys and load code objects into device memory.
    if (new_pkt->req->systemReq()) {
        // There will be multiple packets returned for the same gpuDynInst,
        // so first check if systemReq is not already set and if so, return
        // the token acquired when the dispatch list is filled as system
        // requests do not require a GPU coalescer token.
        if (!gpuDynInst->isSystemReq()) {
            computeUnit->getTokenManager()->recvTokens(1);
            gpuDynInst->setSystemReq();
        }
    } else {
        new_pkt->req->requestorId(computeUnit->vramRequestorId());
    }

    // translation is done. Schedule the mem_req_event at the appropriate
    // cycle to send the timing memory request to ruby
    EventFunctionWrapper *mem_req_event =
        computeUnit->memPort[mp_index].createMemReqEvent(new_pkt);

    DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x data scheduled\n",
            computeUnit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
            mp_index, new_pkt->req->getPaddr());

    computeUnit->schedule(mem_req_event,
                          curTick() + computeUnit->req_tick_latency);

    return true;
}

EventFunctionWrapper *
ComputeUnit::DataPort::createMemReqEvent(PacketPtr pkt)
{
    return new EventFunctionWrapper([this, pkt] { processMemReqEvent(pkt); },
                                    "ComputeUnit memory request event", true);
}

EventFunctionWrapper *
ComputeUnit::DataPort::createMemRespEvent(PacketPtr pkt)
{
    return new EventFunctionWrapper([this, pkt] { processMemRespEvent(pkt); },
                                    "ComputeUnit memory response event", true);
}

void
ComputeUnit::DataPort::processMemReqEvent(PacketPtr pkt)
{
    SenderState *sender_state = safe_cast<SenderState *>(pkt->senderState);
    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    [[maybe_unused]] ComputeUnit *compute_unit = computeUnit;

    if (pkt->req->systemReq()) {
        assert(compute_unit->shader->systemHub);
        SystemHubEvent *resp_event = new SystemHubEvent(pkt, this);
        compute_unit->shader->systemHub->sendRequest(pkt, resp_event);
    } else if (!(sendTimingReq(pkt))) {
        retries.push_back(std::make_pair(pkt, gpuDynInst));

        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: index %d, addr %#x data req failed!\n",
                compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                id, pkt->req->getPaddr());
    } else {
        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: gpuDynInst: %d, index %d, addr %#x data "
                "req sent!\n",
                compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                gpuDynInst->seqNum(), id, pkt->req->getPaddr());
    }
}

const char *
ComputeUnit::ScalarDataPort::MemReqEvent::description() const
{
    return "ComputeUnit scalar memory request event";
}

void
ComputeUnit::ScalarDataPort::MemReqEvent::process()
{
    SenderState *sender_state = safe_cast<SenderState *>(pkt->senderState);
    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    [[maybe_unused]] ComputeUnit *compute_unit = scalarDataPort.computeUnit;

    if (pkt->req->systemReq()) {
        assert(compute_unit->shader->systemHub);
        SystemHubEvent *resp_event = new SystemHubEvent(pkt, &scalarDataPort);
        compute_unit->shader->systemHub->sendRequest(pkt, resp_event);
    } else if (!(scalarDataPort.sendTimingReq(pkt))) {
        scalarDataPort.retries.push_back(pkt);

        DPRINTF(GPUPort, "CU%d: WF[%d][%d]: addr %#x data req failed!\n",
                compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                pkt->req->getPaddr());
    } else {
        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: gpuDynInst: %d, addr %#x data "
                "req sent!\n",
                compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                gpuDynInst->seqNum(), pkt->req->getPaddr());
    }
}

/*
 * The initial translation request could have been rejected,
 * if <retries> queue is not Retry sending the translation
 * request. sendRetry() is called from the peer port whenever
 * a translation completes.
 */
void
ComputeUnit::DTLBPort::recvReqRetry()
{
    int len = retries.size();

    DPRINTF(GPUTLB, "CU%d: DTLB recvReqRetry - %d pending requests\n",
            computeUnit->cu_id, len);

    assert(len > 0);
    assert(isStalled());
    // recvReqRetry is an indication that the resource on which this
    // port was stalling on is freed. So, remove the stall first
    unstallPort();

    for (int i = 0; i < len; ++i) {
        PacketPtr pkt = retries.front();
        [[maybe_unused]] Addr vaddr = pkt->req->getVaddr();
        DPRINTF(GPUTLB, "CU%d: retrying D-translaton for address%#x", vaddr);

        if (!sendTimingReq(pkt)) {
            // Stall port
            stallPort();
            DPRINTF(GPUTLB, ": failed again\n");
            break;
        } else {
            DPRINTF(GPUTLB, ": successful\n");
            retries.pop_front();
        }
    }
}

bool
ComputeUnit::ScalarDTLBPort::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->senderState);

    GpuTranslationState *translation_state =
        safe_cast<GpuTranslationState *>(pkt->senderState);

    // Page faults are not allowed
    fatal_if(!translation_state->tlbEntry, "Translation of vaddr %#x failed\n",
             pkt->req->getVaddr());

    delete translation_state->tlbEntry;
    assert(!translation_state->ports.size());

    pkt->senderState = translation_state->saved;
    delete translation_state;

    ScalarDTLBPort::SenderState *sender_state =
        safe_cast<ScalarDTLBPort::SenderState *>(pkt->senderState);

    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    delete pkt->senderState;

    [[maybe_unused]] Wavefront *w = gpuDynInst->wavefront();

    DPRINTF(GPUTLB,
            "CU%d: WF[%d][%d][wv=%d]: scalar DTLB port received "
            "translation: PA %#x -> %#x\n",
            computeUnit->cu_id, w->simdId, w->wfSlotId, w->kernId,
            pkt->req->getVaddr(), pkt->req->getPaddr());

    MemCmd mem_cmd;

    if (pkt->cmd == MemCmd::ReadResp) {
        mem_cmd = MemCmd::ReadReq;
    } else if (pkt->cmd == MemCmd::WriteResp) {
        mem_cmd = MemCmd::WriteReq;
    } else {
        fatal("Scalar DTLB receieved unexpected MemCmd response %s\n",
              pkt->cmd.toString());
    }

    PacketPtr req_pkt = new Packet(pkt->req, mem_cmd);
    req_pkt->dataStatic(pkt->getPtr<uint8_t>());
    delete pkt;

    req_pkt->senderState =
        new ComputeUnit::ScalarDataPort::SenderState(gpuDynInst);

    // For a system request we want to mark the GPU instruction as a system
    // load/store so that after the request is issued to system memory we can
    // return any token acquired for the request. Since tokens are returned
    // by the coalescer and system requests do not take that path, this needs
    // to be tracked.
    //
    // Device requests change the requestor ID to something in the device
    // memory Ruby network.
    if (req_pkt->req->systemReq()) {
        gpuDynInst->setSystemReq();
    } else {
        req_pkt->req->requestorId(computeUnit->vramRequestorId());
    }

    ComputeUnit::ScalarDataPort::MemReqEvent *scalar_mem_req_event =
        new ComputeUnit::ScalarDataPort::MemReqEvent(
            computeUnit->scalarDataPort, req_pkt);
    computeUnit->schedule(scalar_mem_req_event,
                          curTick() + computeUnit->scalar_req_tick_latency);

    return true;
}

bool
ComputeUnit::ITLBPort::recvTimingResp(PacketPtr pkt)
{
    [[maybe_unused]] Addr line = pkt->req->getPaddr();
    DPRINTF(GPUTLB, "CU%d: ITLBPort received %#x->%#x\n", computeUnit->cu_id,
            pkt->req->getVaddr(), line);

    assert(pkt->senderState);

    // pop off the TLB translation state
    GpuTranslationState *translation_state =
        safe_cast<GpuTranslationState *>(pkt->senderState);

    bool success = translation_state->tlbEntry != nullptr;
    delete translation_state->tlbEntry;
    assert(!translation_state->ports.size());
    pkt->senderState = translation_state->saved;
    delete translation_state;

    // use the original sender state to know how to close this transaction
    ITLBPort::SenderState *sender_state =
        safe_cast<ITLBPort::SenderState *>(pkt->senderState);

    // get the wavefront associated with this translation request
    Wavefront *wavefront = sender_state->wavefront;
    delete pkt->senderState;

    if (success) {
        // pkt is reused in fetch(), don't delete it here.  However, we must
        // reset the command to be a request so that it can be sent through
        // the cu's request port
        assert(pkt->cmd == MemCmd::ReadResp);
        pkt->cmd = MemCmd::ReadReq;

        computeUnit->fetchStage.fetch(pkt, wavefront);
    } else {
        if (wavefront->dropFetch) {
            assert(wavefront->instructionBuffer.empty());
            wavefront->dropFetch = false;
        }

        wavefront->pendingFetch = 0;
    }

    return true;
}

/*
 * The initial translation request could have been rejected, if
 * <retries> queue is not empty. Retry sending the translation
 * request. sendRetry() is called from the peer port whenever
 * a translation completes.
 */
void
ComputeUnit::ITLBPort::recvReqRetry()
{
    int len = retries.size();
    DPRINTF(GPUTLB, "CU%d: ITLB recvReqRetry - %d pending requests\n", len);

    assert(len > 0);
    assert(isStalled());

    // recvReqRetry is an indication that the resource on which this
    // port was stalling on is freed. So, remove the stall first
    unstallPort();

    for (int i = 0; i < len; ++i) {
        PacketPtr pkt = retries.front();
        [[maybe_unused]] Addr vaddr = pkt->req->getVaddr();
        DPRINTF(GPUTLB, "CU%d: retrying I-translaton for address%#x", vaddr);

        if (!sendTimingReq(pkt)) {
            stallPort(); // Stall port
            DPRINTF(GPUTLB, ": failed again\n");
            break;
        } else {
            DPRINTF(GPUTLB, ": successful\n");
            retries.pop_front();
        }
    }
}

void
ComputeUnit::updateInstStats(GPUDynInstPtr gpuDynInst)
{
    if (gpuDynInst->isScalar()) {
        if (gpuDynInst->isALU() && !gpuDynInst->isWaitcnt()) {
            stats.sALUInsts++;
            stats.instCyclesSALU++;
        } else if (gpuDynInst->isLoad()) {
            stats.scalarMemReads++;
        } else if (gpuDynInst->isStore()) {
            stats.scalarMemWrites++;
        }
    } else {
        if (gpuDynInst->isALU()) {
            shader->total_valu_insts++;
            if (shader->total_valu_insts == shader->max_valu_insts) {
                exitSimLoop("max vALU insts");
            }
            stats.vALUInsts++;
            stats.instCyclesVALU++;
            stats.threadCyclesVALU +=
                gpuDynInst->wavefront()->execMask().count();
        } else if (gpuDynInst->isFlat()) {
            if (gpuDynInst->isLocalMem()) {
                stats.flatLDSInsts++;
            } else {
                stats.flatVMemInsts++;
            }
        } else if (gpuDynInst->isFlatGlobal()) {
            stats.flatVMemInsts++;
        } else if (gpuDynInst->isFlatScratch()) {
            stats.flatVMemInsts++;
        } else if (gpuDynInst->isLocalMem()) {
            stats.ldsNoFlatInsts++;
        } else if (gpuDynInst->isLoad()) {
            stats.vectorMemReads++;
        } else if (gpuDynInst->isStore()) {
            stats.vectorMemWrites++;
        }

        if (gpuDynInst->isLoad()) {
            switch (gpuDynInst->executedAs()) {
            case enums::SC_SPILL:
                stats.spillReads++;
                break;
            case enums::SC_GLOBAL:
                stats.globalReads++;
                break;
            case enums::SC_GROUP:
                stats.groupReads++;
                break;
            case enums::SC_PRIVATE:
                stats.privReads++;
                break;
            case enums::SC_READONLY:
                stats.readonlyReads++;
                break;
            case enums::SC_KERNARG:
                stats.kernargReads++;
                break;
            case enums::SC_ARG:
                stats.argReads++;
                break;
            case enums::SC_NONE:
                /**
                 * this case can occur for flat mem insts
                 * who execute with EXEC = 0
                 */
                break;
            default:
                fatal("%s has no valid segment\n", gpuDynInst->disassemble());
                break;
            }
        } else if (gpuDynInst->isStore()) {
            switch (gpuDynInst->executedAs()) {
            case enums::SC_SPILL:
                stats.spillWrites++;
                break;
            case enums::SC_GLOBAL:
                stats.globalWrites++;
                break;
            case enums::SC_GROUP:
                stats.groupWrites++;
                break;
            case enums::SC_PRIVATE:
                stats.privWrites++;
                break;
            case enums::SC_READONLY:
                stats.readonlyWrites++;
                break;
            case enums::SC_KERNARG:
                stats.kernargWrites++;
                break;
            case enums::SC_ARG:
                stats.argWrites++;
                break;
            case enums::SC_NONE:
                /**
                 * this case can occur for flat mem insts
                 * who execute with EXEC = 0
                 */
                break;
            default:
                fatal("%s has no valid segment\n", gpuDynInst->disassemble());
                break;
            }
        }
    }
}

void
ComputeUnit::updatePageDivergenceDist(Addr addr)
{
    Addr virt_page_addr = roundDown(addr, X86ISA::PageBytes);

    if (!pagesTouched.count(virt_page_addr))
        pagesTouched[virt_page_addr] = 1;
    else
        pagesTouched[virt_page_addr]++;
}

void
ComputeUnit::exitCallback()
{
    if (countPages) {
        std::ostream *page_stat_file = simout.create(name().c_str())->stream();

        *page_stat_file << "page, wavefront accesses, workitem accesses"
                        << std::endl;

        for (auto iter : pageAccesses) {
            *page_stat_file << std::hex << iter.first << ",";
            *page_stat_file << std::dec << iter.second.first << ",";
            *page_stat_file << std::dec << iter.second.second << std::endl;
        }
    }
}

bool
ComputeUnit::isDone() const
{
    for (int i = 0; i < numVectorALUs; ++i) {
        if (!isVectorAluIdle(i)) {
            return false;
        }
    }

    // TODO: FIXME if more than 1 of any memory pipe supported
    if (!srfToScalarMemPipeBus.rdy()) {
        return false;
    }
    if (!vrfToGlobalMemPipeBus.rdy()) {
        return false;
    }
    if (!vrfToLocalMemPipeBus.rdy()) {
        return false;
    }

    if (!globalMemoryPipe.isGMReqFIFOWrRdy() ||
        !localMemoryPipe.isLMReqFIFOWrRdy() ||
        !localMemoryPipe.isLMRespFIFOWrRdy() || !locMemToVrfBus.rdy() ||
        !glbMemToVrfBus.rdy() || !scalarMemToSrfBus.rdy()) {
        return false;
    }

    return true;
}

int32_t
ComputeUnit::getRefCounter(const uint32_t dispatchId,
                           const uint32_t wgId) const
{
    return lds.getRefCounter(dispatchId, wgId);
}

bool
ComputeUnit::isVectorAluIdle(uint32_t simdId) const
{
    assert(simdId < numVectorALUs);

    for (int i_wf = 0; i_wf < shader->n_wf; ++i_wf) {
        if (wfList[simdId][i_wf]->getStatus() != Wavefront::S_STOPPED) {
            return false;
        }
    }

    return true;
}

/**
 * send a general request to the LDS
 * make sure to look at the return value here as your request might be
 * NACK'd and returning false means that you have to have some backup plan
 */
bool
ComputeUnit::sendToLds(GPUDynInstPtr gpuDynInst)
{
    // this is just a request to carry the GPUDynInstPtr
    // back and forth
    RequestPtr newRequest = std::make_shared<Request>();
    newRequest->setPaddr(0x0);

    // ReadReq is not evaluted by the LDS but the Packet ctor requires this
    PacketPtr newPacket = new Packet(newRequest, MemCmd::ReadReq);

    // This is the SenderState needed upon return
    newPacket->senderState = new LDSPort::SenderState(gpuDynInst);

    return ldsPort.sendTimingReq(newPacket);
}

/**
 * Forward the VRAM requestor ID needed for device memory from shader.
 */
RequestorID
ComputeUnit::vramRequestorId()
{
    return FullSystem ? shader->vramRequestorId() : requestorId();
}

/**
 * get the result of packets sent to the LDS when they return
 */
bool
ComputeUnit::LDSPort::recvTimingResp(PacketPtr packet)
{
    const ComputeUnit::LDSPort::SenderState *senderState =
        dynamic_cast<ComputeUnit::LDSPort::SenderState *>(packet->senderState);

    fatal_if(!senderState, "did not get the right sort of sender state");

    GPUDynInstPtr gpuDynInst = senderState->getMemInst();

    delete packet->senderState;
    delete packet;

    computeUnit->localMemoryPipe.getLMRespFIFO().push(gpuDynInst);
    return true;
}

/**
 * attempt to send this packet, either the port is already stalled, the request
 * is nack'd and must stall or the request goes through
 * when a request cannot be sent, add it to the retries queue
 */
bool
ComputeUnit::LDSPort::sendTimingReq(PacketPtr pkt)
{
    ComputeUnit::LDSPort::SenderState *sender_state =
        dynamic_cast<ComputeUnit::LDSPort::SenderState *>(pkt->senderState);
    fatal_if(!sender_state, "packet without a valid sender state");

    [[maybe_unused]] GPUDynInstPtr gpuDynInst = sender_state->getMemInst();

    if (isStalled()) {
        fatal_if(retries.empty(), "must have retries waiting to be stalled");

        retries.push(pkt);

        DPRINTF(GPUPort, "CU%d: WF[%d][%d]: LDS send failed!\n",
                computeUnit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId);
        return false;
    } else if (!RequestPort::sendTimingReq(pkt)) {
        // need to stall the LDS port until a recvReqRetry() is received
        // this indicates that there is more space
        stallPort();
        retries.push(pkt);

        DPRINTF(GPUPort, "CU%d: WF[%d][%d]: addr %#x lds req failed!\n",
                computeUnit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                pkt->req->getPaddr());
        return false;
    } else {
        DPRINTF(GPUPort, "CU%d: WF[%d][%d]: addr %#x lds req sent!\n",
                computeUnit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                pkt->req->getPaddr());
        return true;
    }
}

/**
 * the bus is telling the port that there is now space so retrying stalled
 * requests should work now
 * this allows the port to have a request be nack'd and then have the receiver
 * say when there is space, rather than simply retrying the send every cycle
 */
void
ComputeUnit::LDSPort::recvReqRetry()
{
    auto queueSize = retries.size();

    DPRINTF(GPUPort, "CU%d: LDSPort recvReqRetry - %d pending requests\n",
            computeUnit->cu_id, queueSize);

    fatal_if(queueSize < 1,
             "why was there a recvReqRetry() with no pending reqs?");
    fatal_if(!isStalled(),
             "recvReqRetry() happened when the port was not stalled");

    unstallPort();

    while (!retries.empty()) {
        PacketPtr packet = retries.front();

        DPRINTF(GPUPort, "CU%d: retrying LDS send\n", computeUnit->cu_id);

        if (!RequestPort::sendTimingReq(packet)) {
            // Stall port
            stallPort();
            DPRINTF(GPUPort, ": LDS send failed again\n");
            break;
        } else {
            DPRINTF(GPUTLB, ": LDS send successful\n");
            retries.pop();
        }
    }
}

ComputeUnit::ComputeUnitStats::ComputeUnitStats(statistics::Group *parent,
                                                int n_wf)
    : statistics::Group(parent),
      ADD_STAT(vALUInsts, "Number of vector ALU insts issued."),
      ADD_STAT(vALUInstsPerWF, "The avg. number of vector ALU insts issued "
                               "per-wavefront."),
      ADD_STAT(sALUInsts, "Number of scalar ALU insts issued."),
      ADD_STAT(sALUInstsPerWF, "The avg. number of scalar ALU insts issued "
                               "per-wavefront."),
      ADD_STAT(instCyclesVALU,
               "Number of cycles needed to execute VALU insts."),
      ADD_STAT(instCyclesSALU,
               "Number of cycles needed to execute SALU insts."),
      ADD_STAT(threadCyclesVALU,
               "Number of thread cycles used to execute "
               "vector ALU ops. Similar to instCyclesVALU but multiplied by "
               "the number of active threads."),
      ADD_STAT(vALUUtilization,
               "Percentage of active vector ALU threads in a wave."),
      ADD_STAT(ldsNoFlatInsts, "Number of LDS insts issued, not including FLAT"
                               " accesses that resolve to LDS."),
      ADD_STAT(ldsNoFlatInstsPerWF,
               "The avg. number of LDS insts (not "
               "including FLAT accesses that resolve to LDS) per-wavefront."),
      ADD_STAT(flatVMemInsts,
               "The number of FLAT insts that resolve to vmem issued."),
      ADD_STAT(flatVMemInstsPerWF, "The average number of FLAT insts that "
                                   "resolve to vmem issued per-wavefront."),
      ADD_STAT(flatLDSInsts,
               "The number of FLAT insts that resolve to LDS issued."),
      ADD_STAT(flatLDSInstsPerWF, "The average number of FLAT insts that "
                                  "resolve to LDS issued per-wavefront."),
      ADD_STAT(vectorMemWrites,
               "Number of vector mem write insts (excluding FLAT insts)."),
      ADD_STAT(vectorMemWritesPerWF,
               "The average number of vector mem write "
               "insts (excluding FLAT insts) per-wavefront."),
      ADD_STAT(vectorMemReads,
               "Number of vector mem read insts (excluding FLAT insts)."),
      ADD_STAT(vectorMemReadsPerWF, "The avg. number of vector mem read insts "
                                    "(excluding FLAT insts) per-wavefront."),
      ADD_STAT(scalarMemWrites, "Number of scalar mem write insts."),
      ADD_STAT(scalarMemWritesPerWF,
               "The average number of scalar mem write insts per-wavefront."),
      ADD_STAT(scalarMemReads, "Number of scalar mem read insts."),
      ADD_STAT(scalarMemReadsPerWF,
               "The average number of scalar mem read insts per-wavefront."),
      ADD_STAT(vectorMemReadsPerKiloInst,
               "Number of vector mem reads per kilo-instruction"),
      ADD_STAT(vectorMemWritesPerKiloInst,
               "Number of vector mem writes per kilo-instruction"),
      ADD_STAT(vectorMemInstsPerKiloInst,
               "Number of vector mem insts per kilo-instruction"),
      ADD_STAT(scalarMemReadsPerKiloInst,
               "Number of scalar mem reads per kilo-instruction"),
      ADD_STAT(scalarMemWritesPerKiloInst,
               "Number of scalar mem writes per kilo-instruction"),
      ADD_STAT(scalarMemInstsPerKiloInst,
               "Number of scalar mem insts per kilo-instruction"),
      ADD_STAT(instCyclesVMemPerSimd,
               "Number of cycles to send address, "
               "command, data from VRF to vector memory unit, per SIMD"),
      ADD_STAT(instCyclesScMemPerSimd,
               "Number of cycles to send address, "
               "command, data from SRF to scalar memory unit, per SIMD"),
      ADD_STAT(instCyclesLdsPerSimd,
               "Number of cycles to send address, "
               "command, data from VRF to LDS unit, per SIMD"),
      ADD_STAT(globalReads, "Number of reads to the global segment"),
      ADD_STAT(globalWrites, "Number of writes to the global segment"),
      ADD_STAT(globalMemInsts,
               "Number of memory instructions sent to the global segment"),
      ADD_STAT(argReads, "Number of reads to the arg segment"),
      ADD_STAT(argWrites, "NUmber of writes to the arg segment"),
      ADD_STAT(argMemInsts,
               "Number of memory instructions sent to the arg segment"),
      ADD_STAT(spillReads, "Number of reads to the spill segment"),
      ADD_STAT(spillWrites, "Number of writes to the spill segment"),
      ADD_STAT(spillMemInsts,
               "Number of memory instructions sent to the spill segment"),
      ADD_STAT(groupReads, "Number of reads to the group segment"),
      ADD_STAT(groupWrites, "Number of writes to the group segment"),
      ADD_STAT(groupMemInsts,
               "Number of memory instructions sent to the group segment"),
      ADD_STAT(privReads, "Number of reads to the private segment"),
      ADD_STAT(privWrites, "Number of writes to the private segment"),
      ADD_STAT(privMemInsts,
               "Number of memory instructions sent to the private segment"),
      ADD_STAT(readonlyReads, "Number of reads to the readonly segment"),
      ADD_STAT(readonlyWrites,
               "Number of memory instructions sent to the readonly segment"),
      ADD_STAT(readonlyMemInsts,
               "Number of memory instructions sent to the readonly segment"),
      ADD_STAT(kernargReads, "Number of reads sent to the kernarg segment"),
      ADD_STAT(kernargWrites,
               "Number of memory instructions sent to the kernarg segment"),
      ADD_STAT(kernargMemInsts,
               "Number of memory instructions sent to the kernarg segment"),
      ADD_STAT(waveLevelParallelism,
               "wave level parallelism: count of active waves at wave launch"),
      ADD_STAT(tlbRequests, "number of uncoalesced requests"),
      ADD_STAT(tlbCycles,
               "total number of cycles for all uncoalesced requests"),
      ADD_STAT(tlbLatency, "Avg. translation latency for data translations"),
      ADD_STAT(hitsPerTLBLevel,
               "TLB hits distribution (0 for page table, x for Lx-TLB)"),
      ADD_STAT(ldsBankAccesses, "Total number of LDS bank accesses"),
      ADD_STAT(ldsBankConflictDist,
               "Number of bank conflicts per LDS memory packet"),
      ADD_STAT(pageDivergenceDist,
               "pages touched per wf (over all mem. instr.)"),
      ADD_STAT(dynamicGMemInstrCnt,
               "dynamic non-flat global memory instruction count"),
      ADD_STAT(dynamicFlatMemInstrCnt,
               "dynamic flat global memory instruction count"),
      ADD_STAT(dynamicLMemInstrCnt, "dynamic local memory intruction count"),
      ADD_STAT(wgBlockedDueBarrierAllocation,
               "WG dispatch was blocked due to lack of barrier resources"),
      ADD_STAT(wgBlockedDueLdsAllocation,
               "Workgroup blocked due to LDS capacity"),
      ADD_STAT(numInstrExecuted, "number of instructions executed"),
      ADD_STAT(execRateDist, "Instruction Execution Rate: Number of executed "
                             "vector instructions per cycle"),
      ADD_STAT(numVecOpsExecuted,
               "number of vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedF16,
               "number of f16 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedF32,
               "number of f32 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedF64,
               "number of f64 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedFMA16,
               "number of fma16 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedFMA32,
               "number of fma32 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedFMA64,
               "number of fma64 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedMAC16,
               "number of mac16 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedMAC32,
               "number of mac32 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedMAC64,
               "number of mac64 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedMAD16,
               "number of mad16 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedMAD32,
               "number of mad32 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedMAD64,
               "number of mad64 vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(numVecOpsExecutedTwoOpFP,
               "number of two op FP vec ops executed (e.g. WF size/inst)"),
      ADD_STAT(totalCycles, "number of cycles the CU ran for"),
      ADD_STAT(vpc, "Vector Operations per cycle (this CU only)"),
      ADD_STAT(vpc_f16, "F16 Vector Operations per cycle (this CU only)"),
      ADD_STAT(vpc_f32, "F32 Vector Operations per cycle (this CU only)"),
      ADD_STAT(vpc_f64, "F64 Vector Operations per cycle (this CU only)"),
      ADD_STAT(ipc, "Instructions per cycle (this CU only)"),
      ADD_STAT(controlFlowDivergenceDist,
               "number of lanes active per "
               "instruction (over all instructions)"),
      ADD_STAT(activeLanesPerGMemInstrDist,
               "number of active lanes per global memory instruction"),
      ADD_STAT(activeLanesPerLMemInstrDist,
               "number of active lanes per local memory instruction"),
      ADD_STAT(numALUInstsExecuted,
               "Number of dynamic non-GM memory insts executed"),
      ADD_STAT(numTimesWgBlockedDueVgprAlloc,
               "Number of times WGs are "
               "blocked due to VGPR allocation per SIMD"),
      ADD_STAT(numTimesWgBlockedDueSgprAlloc,
               "Number of times WGs are "
               "blocked due to SGPR allocation per SIMD"),
      ADD_STAT(numCASOps, "number of compare and swap operations"),
      ADD_STAT(numFailedCASOps,
               "number of compare and swap operations that failed"),
      ADD_STAT(completedWfs, "number of completed wavefronts"),
      ADD_STAT(completedWGs, "number of completed workgroups"),
      ADD_STAT(headTailLatency, "ticks between first and last cache block "
                                "arrival at coalescer"),
      ADD_STAT(instInterleave, "Measure of instruction interleaving per SIMD")
{
    ComputeUnit *cu = static_cast<ComputeUnit *>(parent);

    instCyclesVMemPerSimd.init(cu->numVectorALUs);
    instCyclesScMemPerSimd.init(cu->numVectorALUs);
    instCyclesLdsPerSimd.init(cu->numVectorALUs);

    hitsPerTLBLevel.init(4);
    execRateDist.init(0, 10, 2);
    ldsBankConflictDist.init(0, cu->wfSize(), 2);

    pageDivergenceDist.init(1, cu->wfSize(), 4);
    controlFlowDivergenceDist.init(1, cu->wfSize(), 4);
    activeLanesPerGMemInstrDist.init(1, cu->wfSize(), 4);
    activeLanesPerLMemInstrDist.init(1, cu->wfSize(), 4);

    headTailLatency.init(0, 1000000, 10000)
        .flags(statistics::pdf | statistics::oneline);
    waveLevelParallelism.init(0, n_wf * cu->numVectorALUs, 1);
    instInterleave.init(cu->numVectorALUs, 0, 20, 1);

    vALUInstsPerWF = vALUInsts / completedWfs;
    sALUInstsPerWF = sALUInsts / completedWfs;
    vALUUtilization = (threadCyclesVALU / (64 * instCyclesVALU)) * 100;
    ldsNoFlatInstsPerWF = ldsNoFlatInsts / completedWfs;
    flatVMemInstsPerWF = flatVMemInsts / completedWfs;
    flatLDSInstsPerWF = flatLDSInsts / completedWfs;
    vectorMemWritesPerWF = vectorMemWrites / completedWfs;
    vectorMemReadsPerWF = vectorMemReads / completedWfs;
    scalarMemWritesPerWF = scalarMemWrites / completedWfs;
    scalarMemReadsPerWF = scalarMemReads / completedWfs;

    vectorMemReadsPerKiloInst = (vectorMemReads / numInstrExecuted) * 1000;
    vectorMemWritesPerKiloInst = (vectorMemWrites / numInstrExecuted) * 1000;
    vectorMemInstsPerKiloInst =
        ((vectorMemReads + vectorMemWrites) / numInstrExecuted) * 1000;
    scalarMemReadsPerKiloInst = (scalarMemReads / numInstrExecuted) * 1000;
    scalarMemWritesPerKiloInst = (scalarMemWrites / numInstrExecuted) * 1000;
    scalarMemInstsPerKiloInst =
        ((scalarMemReads + scalarMemWrites) / numInstrExecuted) * 1000;

    globalMemInsts = globalReads + globalWrites;
    argMemInsts = argReads + argWrites;
    spillMemInsts = spillReads + spillWrites;
    groupMemInsts = groupReads + groupWrites;
    privMemInsts = privReads + privWrites;
    readonlyMemInsts = readonlyReads + readonlyWrites;
    kernargMemInsts = kernargReads + kernargWrites;

    tlbLatency = tlbCycles / tlbRequests;

    // fixed number of TLB levels
    for (int i = 0; i < 4; ++i) {
        if (!i)
            hitsPerTLBLevel.subname(i, "page_table");
        else
            hitsPerTLBLevel.subname(i, csprintf("L%d_TLB", i));
    }

    ipc = numInstrExecuted / totalCycles;
    vpc = numVecOpsExecuted / totalCycles;
    vpc_f16 = numVecOpsExecutedF16 / totalCycles;
    vpc_f32 = numVecOpsExecutedF32 / totalCycles;
    vpc_f64 = numVecOpsExecutedF64 / totalCycles;

    numALUInstsExecuted =
        numInstrExecuted - dynamicGMemInstrCnt - dynamicLMemInstrCnt;
}

} // namespace gem5
