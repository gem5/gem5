/*
 * Copyright (c) 2011-2015 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
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

#include "arch/x86/isa_traits.hh"
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
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/simple_pool_manager.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"
#include "sim/sim_exit.hh"

ComputeUnit::ComputeUnit(const Params *p) : ClockedObject(p),
    numVectorGlobalMemUnits(p->num_global_mem_pipes),
    numVectorSharedMemUnits(p->num_shared_mem_pipes),
    numScalarMemUnits(p->num_scalar_mem_pipes),
    numVectorALUs(p->num_SIMDs),
    numScalarALUs(p->num_scalar_cores),
    vrfToCoalescerBusWidth(p->vrf_to_coalescer_bus_width),
    coalescerToVrfBusWidth(p->coalescer_to_vrf_bus_width),
    registerManager(p->register_manager),
    fetchStage(p, *this),
    scoreboardCheckStage(p, *this, scoreboardCheckToSchedule),
    scheduleStage(p, *this, scoreboardCheckToSchedule, scheduleToExecute),
    execStage(p, *this, scheduleToExecute),
    globalMemoryPipe(p, *this),
    localMemoryPipe(p, *this),
    scalarMemoryPipe(p, *this),
    tickEvent([this]{ exec(); }, "Compute unit tick event",
          false, Event::CPU_Tick_Pri),
    cu_id(p->cu_id),
    vrf(p->vector_register_file), srf(p->scalar_register_file),
    simdWidth(p->simd_width),
    spBypassPipeLength(p->spbypass_pipe_length),
    dpBypassPipeLength(p->dpbypass_pipe_length),
    scalarPipeStages(p->scalar_pipe_length),
    operandNetworkLength(p->operand_network_length),
    issuePeriod(p->issue_period),
    vrf_gm_bus_latency(p->vrf_gm_bus_latency),
    srf_scm_bus_latency(p->srf_scm_bus_latency),
    vrf_lm_bus_latency(p->vrf_lm_bus_latency),
    perLaneTLB(p->perLaneTLB), prefetchDepth(p->prefetch_depth),
    prefetchStride(p->prefetch_stride), prefetchType(p->prefetch_prev_type),
    debugSegFault(p->debugSegFault),
    functionalTLB(p->functionalTLB), localMemBarrier(p->localMemBarrier),
    countPages(p->countPages),
    req_tick_latency(p->mem_req_latency * p->clk_domain->clockPeriod()),
    resp_tick_latency(p->mem_resp_latency * p->clk_domain->clockPeriod()),
    _requestorId(p->system->getRequestorId(this, "ComputeUnit")),
    lds(*p->localDataStore), gmTokenPort(name() + ".gmTokenPort", this),
    ldsPort(csprintf("%s-port", name()), this),
    scalarDataPort(csprintf("%s-port", name()), this),
    scalarDTLBPort(csprintf("%s-port", name()), this),
    sqcPort(csprintf("%s-port", name()), this),
    sqcTLBPort(csprintf("%s-port", name()), this),
    _cacheLineSize(p->system->cacheLineSize()),
    _numBarrierSlots(p->num_barrier_slots),
    globalSeqNum(0), wavefrontSize(p->wf_size),
    scoreboardCheckToSchedule(p),
    scheduleToExecute(p)
{
    /**
     * This check is necessary because std::bitset only provides conversion
     * to unsigned long or unsigned long long via to_ulong() or to_ullong().
     * there are a few places in the code where to_ullong() is used, however
     * if wavefrontSize is larger than a value the host can support then
     * bitset will throw a runtime exception. We should remove all use of
     * to_long() or to_ullong() so we can have wavefrontSize greater than 64b,
     * however until that is done this assert is required.
     */
    fatal_if(p->wf_size > std::numeric_limits<unsigned long long>::digits ||
             p->wf_size <= 0,
             "WF size is larger than the host can support");
    fatal_if(!isPowerOf2(wavefrontSize),
             "Wavefront size should be a power of 2");
    // calculate how many cycles a vector load or store will need to transfer
    // its data over the corresponding buses
    numCyclesPerStoreTransfer =
        (uint32_t)ceil((double)(wfSize() * sizeof(uint32_t)) /
                (double)vrfToCoalescerBusWidth);

    numCyclesPerLoadTransfer = (wfSize() * sizeof(uint32_t))
                               / coalescerToVrfBusWidth;

    // Initialization: all WF slots are assumed STOPPED
    idleWfs = p->n_wf * numVectorALUs;
    lastVaddrWF.resize(numVectorALUs);
    wfList.resize(numVectorALUs);

    wfBarrierSlots.resize(p->num_barrier_slots, WFBarrier());

    for (int i = 0; i < p->num_barrier_slots; ++i) {
        freeBarrierIds.insert(i);
    }

    for (int j = 0; j < numVectorALUs; ++j) {
        lastVaddrWF[j].resize(p->n_wf);

        for (int i = 0; i < p->n_wf; ++i) {
            lastVaddrWF[j][i].resize(wfSize());

            wfList[j].push_back(p->wavefronts[j * p->n_wf + i]);
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

    if (p->execPolicy == "OLDEST-FIRST") {
        exec_policy = EXEC_POLICY::OLDEST;
    } else if (p->execPolicy == "ROUND-ROBIN") {
        exec_policy = EXEC_POLICY::RR;
    } else {
        fatal("Invalid WF execution policy (CU)\n");
    }

    for (int i = 0; i < p->port_memory_port_connection_count; ++i) {
        memPort.emplace_back(csprintf("%s-port%d", name(), i), this, i);
    }

    for (int i = 0; i < p->port_translation_port_connection_count; ++i) {
        tlbPort.emplace_back(csprintf("%s-port%d", name(), i), this, i);
    }

    // Setup tokens for response ports. The number of tokens in memPortTokens
    // is the total token count for the entire vector port (i.e., this CU).
    memPortTokens = new TokenManager(p->max_cu_tokens);

    registerExitCallback([this]() { exitCallback(); });

    lastExecCycle.resize(numVectorALUs, 0);

    for (int i = 0; i < vrf.size(); ++i) {
        vrf[i]->setParent(this);
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
        w->workItemId[1][k] = ((k + waveId * wfSize()) / w->actualWgSz[0]) %
                             w->actualWgSz[1];
        w->workItemId[2][k] = (k + waveId * wfSize()) /
                              (w->actualWgSz[0] * w->actualWgSz[1]);

        w->workItemFlatId[k] = w->workItemId[2][k] * w->actualWgSz[0] *
            w->actualWgSz[1] + w->workItemId[1][k] * w->actualWgSz[0] +
            w->workItemId[0][k];
    }

    // WG state
    w->wgId = task->globalWgId();
    w->dispatchId = task->dispatchId();
    w->workGroupId[0] = w->wgId % task->numWg(0);
    w->workGroupId[1] = (w->wgId / task->numWg(0)) % task->numWg(1);
    w->workGroupId[2] = w->wgId / (task->numWg(0) * task->numWg(1));

    // set the wavefront context to have a pointer to this section of the LDS
    w->ldsChunk = ldsChunk;

    int32_t refCount M5_VAR_USED =
                lds.increaseRefCounter(w->dispatchId, w->wgId);
    DPRINTF(GPUDisp, "CU%d: increase ref ctr wg[%d] to [%d]\n",
                    cu_id, w->wgId, refCount);

    w->instructionBuffer.clear();

    if (w->pendingFetch)
        w->dropFetch = true;

    DPRINTF(GPUDisp, "Scheduling wfDynId/barrier_id %d/%d on CU%d: "
            "WF[%d][%d]. Ref cnt:%d\n", _n_wave, w->barrierId(), cu_id,
            w->simdId, w->wfSlotId, refCount);

    w->initRegState(task, w->actualWgSzTotal);
    w->start(_n_wave++, task->codeAddr());

    waveLevelParallelism.sample(activeWaves);
    activeWaves++;
}

/**
 * trigger invalidate operation in the cu
 *
 * req: request initialized in shader, carrying the invlidate flags
 */
void
ComputeUnit::doInvalidate(RequestPtr req, int kernId){
    GPUDynInstPtr gpuDynInst
        = std::make_shared<GPUDynInst>(this, nullptr,
            new KernelLaunchStaticInst(), getAndIncSeqNum());

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
ComputeUnit::doFlush(GPUDynInstPtr gpuDynInst) {
    injectGlobalMemFence(gpuDynInst, true);
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
                                          task->globalWgId(),
                                          task->ldsSize());

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

        DPRINTF(GPUSync, "CU[%d] - Dispatching WG with barrier Id%d. "
                "%d waves using this barrier.\n", cu_id, barrier_id,
                num_wfs_in_wg);
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

                DPRINTF(GPURename, "SIMD[%d] wfSlotId[%d] WF[%d] "
                    "vregDemand[%d] sregDemand[%d]\n", i, j, w->wfDynId,
                    vregDemand, sregDemand);

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
        trueWgSize[d] = std::min(task->wgSize(d), task->gridSize(d) -
                                 task->wgId(d) * task->wgSize(d));

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
            vregAvail &= registerManager->
                canAllocateVgprs(j, numWfsToSched[j], vregDemandPerWI);
            // find if there are enough free SGPRs in the SIMD's SRF
            // to accomodate the WFs of the new WG that would be mapped
            // to this SIMD unit
            sregAvail &= registerManager->
                canAllocateSgprs(j, numWfsToSched[j], sregDemandPerWI);
        }
    }

    DPRINTF(GPUDisp, "Free WF slots =  %d, Mapped WFs = %d, \
            VGPR Availability = %d, SGPR Availability = %d\n",
            freeWfSlots, numMappedWfs, vregAvail, sregAvail);

    if (!vregAvail) {
        ++numTimesWgBlockedDueVgprAlloc;
    }

    if (!sregAvail) {
        ++numTimesWgBlockedDueSgprAlloc;
    }

    // Return true if enough WF slots to submit workgroup and if there are
    // enough VGPRs to schedule all WFs to their SIMD units
    bool ldsAvail = lds.canReserve(task->ldsSize());
    if (!ldsAvail) {
        wgBlockedDueLdsAllocation++;
    }

    if (!barrier_avail) {
        wgBlockedDueBarrierAllocation++;
    }

    // Return true if the following are all true:
    // (a) all WFs of the WG were mapped to free WF slots
    // (b) there are enough VGPRs to schedule all WFs to their SIMD units
    // (c) there are enough SGPRs on the CU to schedule all WFs
    // (d) there is enough space in LDS to allocate for all WFs
    bool can_dispatch = numMappedWfs == numWfs && vregAvail && sregAvail
                        && ldsAvail && barrier_avail;
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

    totalCycles++;

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
    // Ruby has completed the memory op. Schedule the mem_resp_event at the
    // appropriate cycle to process the timing memory response
    // This delay represents the pipeline delay
    SenderState *sender_state = safe_cast<SenderState*>(pkt->senderState);
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
            // and acquire
            assert(pkt->req->isKernel());
            assert(pkt->req->isAcquire());

            // one D-Cache inv is done, decrement counter
            dispatcher.updateInvCounter(gpuDynInst->kern_id);

            delete pkt->senderState;
            delete pkt;
            return true;
        }

        // retrieve wavefront from inst
        Wavefront *w = gpuDynInst->wavefront();

        // Check if we are waiting on Kernel End Release
        if (w->getStatus() == Wavefront::S_RETURNING
            && gpuDynInst->isEndOfKernel()) {
            // for kernel end, the original request must be both kernel-type
            // and release
            assert(pkt->req->isKernel());
            assert(pkt->req->isRelease());

            // one wb done, decrement counter, and return whether all wbs are
            // done for the kernel
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
                    computeUnit->cu_id, w->simdId, w->wfSlotId,
                    w->wfDynId, w->wgId);

            dispatcher.notifyWgCompl(w);
            w->setStatus(Wavefront::S_STOPPED);
        }

        if (!pkt->req->isKernel()) {
            w = computeUnit->wfList[gpuDynInst->simdId][gpuDynInst->wfSlotId];
            DPRINTF(GPUExec, "MemSyncResp: WF[%d][%d] WV%d %s decrementing "
                            "outstanding reqs %d => %d\n", gpuDynInst->simdId,
                            gpuDynInst->wfSlotId, gpuDynInst->wfDynId,
                            gpuDynInst->disassemble(), w->outstandingReqs,
                            w->outstandingReqs - 1);
            computeUnit->globalMemoryPipe.handleResponse(gpuDynInst);
        }

        delete pkt->senderState;
        delete pkt;
        return true;
    } else if (pkt->cmd == MemCmd::WriteCompleteResp) {
        // this is for writeComplete callback
        // we simply get decrement write-related wait counters
        assert(gpuDynInst);
        Wavefront *w M5_VAR_USED =
            computeUnit->wfList[gpuDynInst->simdId][gpuDynInst->wfSlotId];
        assert(w);
        DPRINTF(GPUExec, "WriteCompleteResp: WF[%d][%d] WV%d %s decrementing "
                        "outstanding reqs %d => %d\n", gpuDynInst->simdId,
                        gpuDynInst->wfSlotId, gpuDynInst->wfDynId,
                        gpuDynInst->disassemble(), w->outstandingReqs,
                        w->outstandingReqs - 1);
        if (gpuDynInst->allLanesZero()) {
            // ask gm pipe to decrement request counters, instead of directly
            // performing here, to avoid asynchronous counter update and
            // instruction retirement (which may hurt waincnt effects)
            computeUnit->globalMemoryPipe.handleResponse(gpuDynInst);

            DPRINTF(GPUMem, "CU%d: WF[%d][%d]: write totally complete\n",
                            computeUnit->cu_id, gpuDynInst->simdId,
                            gpuDynInst->wfSlotId);
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
    assert(!pkt->req->isKernel());

    // retrieve sender state
    SenderState *sender_state = safe_cast<SenderState*>(pkt->senderState);
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
                computeUnit->scalarMemoryPipe.getGMLdRespFIFO().push(
                                gpuDynInst);
        } else {
                computeUnit->scalarMemoryPipe.getGMStRespFIFO().push(
                                gpuDynInst);
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
        GPUDynInstPtr gpuDynInst M5_VAR_USED = retries.front().second;
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
    computeUnit->fetchStage.processFetchReturn(pkt);
    return true;
}

void
ComputeUnit::SQCPort::recvReqRetry()
{
    int len = retries.size();

    assert(len > 0);

    for (int i = 0; i < len; ++i) {
        PacketPtr pkt = retries.front().first;
        Wavefront *wavefront M5_VAR_USED = retries.front().second;
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
    BaseTLB::Mode TLB_mode;
    assert(pkt->isRead() || pkt->isWrite());

    // only do some things if actually accessing data
    bool isDataAccess = pkt->isWrite() || pkt->isRead();

    // Check write before read for atomic operations
    // since atomic operations should use BaseTLB::Write
    if (pkt->isWrite()) {
        TLB_mode = BaseTLB::Write;
    } else if (pkt->isRead()) {
        TLB_mode = BaseTLB::Read;
    } else {
        fatal("pkt is not a read nor a write\n");
    }

    tlbCycles -= curTick();
    ++tlbRequests;

    PortID tlbPort_index = perLaneTLB ? index : 0;

    if (shader->timingSim) {
        if (debugSegFault) {
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
                    panic("CU%d: WF[%d][%d]: Fault on addr %#x!\n",
                          cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                          vaddr);
                }
            }
        }

        // This is the SenderState needed upon return
        pkt->senderState = new DTLBPort::SenderState(gpuDynInst, index);

        // This is the senderState needed by the TLB hierarchy to function
        TheISA::GpuTLB::TranslationState *translation_state =
          new TheISA::GpuTLB::TranslationState(TLB_mode, shader->gpuTc, false,
                                               pkt->senderState);

        pkt->senderState = translation_state;

        if (functionalTLB) {
            tlbPort[tlbPort_index].sendFunctional(pkt);

            // update the hitLevel distribution
            int hit_level = translation_state->hitLevel;
            assert(hit_level != -1);
            hitsPerTLBLevel[hit_level]++;

            // New SenderState for the memory access
            X86ISA::GpuTLB::TranslationState *sender_state =
                safe_cast<X86ISA::GpuTLB::TranslationState*>(pkt->senderState);

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
            pkt->senderState =
                new ComputeUnit::DataPort::SenderState(gpuDynInst, index,
                    nullptr);

            gpuDynInst->memStatusVector[pkt->getAddr()].push_back(index);
            gpuDynInst->tlbHitLevel[index] = hit_level;

            // translation is done. Schedule the mem_req_event at the
            // appropriate cycle to send the timing memory request to ruby
            EventFunctionWrapper *mem_req_event =
                memPort[index].createMemReqEvent(pkt);

            DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x data "
                    "scheduled\n", cu_id, gpuDynInst->simdId,
                    gpuDynInst->wfSlotId, index, pkt->req->getPaddr());

            schedule(mem_req_event, curTick() + req_tick_latency);
        } else if (tlbPort[tlbPort_index].isStalled()) {
            assert(tlbPort[tlbPort_index].retries.size() > 0);

            DPRINTF(GPUTLB, "CU%d: WF[%d][%d]: Translation for addr %#x "
                    "failed!\n", cu_id, gpuDynInst->simdId,
                    gpuDynInst->wfSlotId, tmp_vaddr);

            tlbPort[tlbPort_index].retries.push_back(pkt);
        } else if (!tlbPort[tlbPort_index].sendTimingReq(pkt)) {
            // Stall the data port;
            // No more packet will be issued till
            // ruby indicates resources are freed by
            // a recvReqRetry() call back on this port.
            tlbPort[tlbPort_index].stallPort();

            DPRINTF(GPUTLB, "CU%d: WF[%d][%d]: Translation for addr %#x "
                    "failed!\n", cu_id, gpuDynInst->simdId,
                    gpuDynInst->wfSlotId, tmp_vaddr);

            tlbPort[tlbPort_index].retries.push_back(pkt);
        } else {
           DPRINTF(GPUTLB,
                   "CU%d: WF[%d][%d]: Translation for addr %#x sent!\n",
                   cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, tmp_vaddr);
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
        pkt->senderState = new TheISA::GpuTLB::TranslationState(TLB_mode,
                                                                shader->gpuTc);

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
        TheISA::GpuTLB::TranslationState *sender_state =
             safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

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

    BaseTLB::Mode tlb_mode = pkt->isRead() ? BaseTLB::Read : BaseTLB::Write;

    pkt->senderState =
        new ComputeUnit::ScalarDTLBPort::SenderState(gpuDynInst);

    pkt->senderState =
        new TheISA::GpuTLB::TranslationState(tlb_mode, shader->gpuTc, false,
                                             pkt->senderState);

    if (scalarDTLBPort.isStalled()) {
        assert(scalarDTLBPort.retries.size());
        scalarDTLBPort.retries.push_back(pkt);
    } else if (!scalarDTLBPort.sendTimingReq(pkt)) {
        scalarDTLBPort.stallPort();
        scalarDTLBPort.retries.push_back(pkt);
    } else {
        DPRINTF(GPUTLB, "sent scalar %s translation request for addr %#x\n",
                tlb_mode == BaseTLB::Read ? "read" : "write",
                pkt->req->getVaddr());
    }
}

void
ComputeUnit::injectGlobalMemFence(GPUDynInstPtr gpuDynInst,
                                  bool kernelMemSync,
                                  RequestPtr req)
{
    assert(gpuDynInst->isGlobalSeg() ||
           gpuDynInst->executedAs() == Enums::SC_GLOBAL);

    if (!req) {
        req = std::make_shared<Request>(
            0, 0, 0, requestorId(), 0, gpuDynInst->wfDynId);
    }

    // all mem sync requests have Paddr == 0
    req->setPaddr(0);

    PacketPtr pkt = nullptr;

    if (kernelMemSync) {
        if (gpuDynInst->isKernelLaunch()) {
            req->setCacheCoherenceFlags(Request::ACQUIRE);
            req->setReqInstSeqNum(gpuDynInst->seqNum());
            req->setFlags(Request::KERNEL);
            pkt = new Packet(req, MemCmd::MemSyncReq);
            pkt->pushSenderState(
               new ComputeUnit::DataPort::SenderState(gpuDynInst, 0, nullptr));

            EventFunctionWrapper *mem_req_event =
              memPort[0].createMemReqEvent(pkt);

            DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x scheduling "
                    "an acquire\n", cu_id, gpuDynInst->simdId,
                    gpuDynInst->wfSlotId, 0, pkt->req->getPaddr());

            schedule(mem_req_event, curTick() + req_tick_latency);
        } else {
          // kernel end release must be enabled
          assert(shader->impl_kern_end_rel);
          assert(gpuDynInst->isEndOfKernel());

          req->setCacheCoherenceFlags(Request::WB_L2);
          req->setReqInstSeqNum(gpuDynInst->seqNum());
          req->setFlags(Request::KERNEL);
          pkt = new Packet(req, MemCmd::MemSyncReq);
          pkt->pushSenderState(
             new ComputeUnit::DataPort::SenderState(gpuDynInst, 0, nullptr));

          EventFunctionWrapper *mem_req_event =
            memPort[0].createMemReqEvent(pkt);

          DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x scheduling "
                  "a release\n", cu_id, gpuDynInst->simdId,
                  gpuDynInst->wfSlotId, 0, pkt->req->getPaddr());

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
                "CU%d: WF[%d][%d]: index %d, addr %#x sync scheduled\n",
                cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, 0,
                pkt->req->getPaddr());

        schedule(mem_req_event, curTick() + req_tick_latency);
    }
}

void
ComputeUnit::DataPort::processMemRespEvent(PacketPtr pkt)
{
    DataPort::SenderState *sender_state =
        safe_cast<DataPort::SenderState*>(pkt->senderState);

    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    ComputeUnit *compute_unit = computeUnit;

    assert(gpuDynInst);

    DPRINTF(GPUPort, "CU%d: WF[%d][%d]: Response for addr %#x, index %d\n",
            compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
            pkt->req->getPaddr(), id);

    Addr paddr = pkt->req->getPaddr();

    // mem sync resp and write-complete callback must be handled already in
    // DataPort::recvTimingResp
    assert(pkt->cmd != MemCmd::MemSyncResp);
    assert(pkt->cmd != MemCmd::WriteCompleteResp);

    // this is for read, write and atomic
    int index = gpuDynInst->memStatusVector[paddr].back();

    DPRINTF(GPUMem, "Response for addr %#x, index %d\n",
            pkt->req->getPaddr(), id);

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
            compute_unit->headTailLatency.sample(curTick() - headTick);
            compute_unit->headTailMap.erase(gpuDynInst);
        }

        gpuDynInst->memStatusVector.clear();

        // note: only handle read response here; for write, the response
        // is separately handled when writeComplete callback is received
        if (pkt->isRead()) {
            gpuDynInst->
                profileRoundTripTime(curTick(), InstMemoryHop::GMEnqueue);
            compute_unit->globalMemoryPipe.handleResponse(gpuDynInst);

            DPRINTF(GPUMem, "CU%d: WF[%d][%d]: packet totally complete\n",
                    compute_unit->cu_id, gpuDynInst->simdId,
                    gpuDynInst->wfSlotId);
        }
    } else {
        if (pkt->isRead()) {
            if (!compute_unit->headTailMap.count(gpuDynInst)) {
                compute_unit->headTailMap
                    .insert(std::make_pair(gpuDynInst, curTick()));
            }
        }
    }

    delete pkt->senderState;
    delete pkt;
}

ComputeUnit*
ComputeUnitParams::create()
{
    return new ComputeUnit(this);
}

bool
ComputeUnit::DTLBPort::recvTimingResp(PacketPtr pkt)
{
    Addr line = pkt->req->getPaddr();

    DPRINTF(GPUTLB, "CU%d: DTLBPort received %#x->%#x\n", computeUnit->cu_id,
            pkt->req->getVaddr(), line);

    assert(pkt->senderState);
    computeUnit->tlbCycles += curTick();

    // pop off the TLB translation state
    TheISA::GpuTLB::TranslationState *translation_state =
               safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

    // no PageFaults are permitted for data accesses
    if (!translation_state->tlbEntry) {
        DTLBPort::SenderState *sender_state =
            safe_cast<DTLBPort::SenderState*>(translation_state->saved);

        Wavefront *w M5_VAR_USED =
            computeUnit->wfList[sender_state->_gpuDynInst->simdId]
            [sender_state->_gpuDynInst->wfSlotId];

        DPRINTFN("Wave %d couldn't tranlate vaddr %#x\n", w->wfDynId,
                 pkt->req->getVaddr());
    }

    // update the hitLevel distribution
    int hit_level = translation_state->hitLevel;
    computeUnit->hitsPerTLBLevel[hit_level]++;

    delete translation_state->tlbEntry;
    assert(!translation_state->ports.size());
    pkt->senderState = translation_state->saved;

    // for prefetch pkt
    BaseTLB::Mode TLB_mode = translation_state->tlbMode;

    delete translation_state;

    // use the original sender state to know how to close this transaction
    DTLBPort::SenderState *sender_state =
        safe_cast<DTLBPort::SenderState*>(pkt->senderState);

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

        switch(computeUnit->prefetchType) {
        case Enums::PF_CU:
            last = computeUnit->lastVaddrCU[mp_index];
            break;
        case Enums::PF_PHASE:
            last = computeUnit->lastVaddrSimd[simdId][mp_index];
            break;
        case Enums::PF_WF:
            last = computeUnit->lastVaddrWF[simdId][wfSlotId][mp_index];
        default:
            break;
        }

        DPRINTF(GPUPrefetch, "CU[%d][%d][%d][%d]: %#x was last\n",
                computeUnit->cu_id, simdId, wfSlotId, mp_index, last);

        int stride = last ? (roundDown(vaddr, TheISA::PageBytes) -
                     roundDown(last, TheISA::PageBytes)) >> TheISA::PageShift
                     : 0;

        DPRINTF(GPUPrefetch, "Stride is %d\n", stride);

        computeUnit->lastVaddrCU[mp_index] = vaddr;
        computeUnit->lastVaddrSimd[simdId][mp_index] = vaddr;
        computeUnit->lastVaddrWF[simdId][wfSlotId][mp_index] = vaddr;

        stride = (computeUnit->prefetchType == Enums::PF_STRIDE) ?
            computeUnit->prefetchStride: stride;

        DPRINTF(GPUPrefetch, "%#x to: CU[%d][%d][%d][%d]\n", vaddr,
                computeUnit->cu_id, simdId, wfSlotId, mp_index);

        DPRINTF(GPUPrefetch, "Prefetching from %#x:", vaddr);

        // Prefetch Next few pages atomically
        for (int pf = 1; pf <= computeUnit->prefetchDepth; ++pf) {
            DPRINTF(GPUPrefetch, "%d * %d: %#x\n", pf, stride,
                    vaddr+stride*pf*TheISA::PageBytes);

            if (!stride)
                break;

            RequestPtr prefetch_req = std::make_shared<Request>(
                vaddr + stride * pf * TheISA::PageBytes,
                sizeof(uint8_t), 0,
                computeUnit->requestorId(),
                0, 0, nullptr);

            PacketPtr prefetch_pkt = new Packet(prefetch_req, requestCmd);
            uint8_t foo = 0;
            prefetch_pkt->dataStatic(&foo);

            // Because it's atomic operation, only need TLB translation state
            prefetch_pkt->senderState =
                new TheISA::GpuTLB::TranslationState(TLB_mode,
                    computeUnit->shader->gpuTc, true);

            // Currently prefetches are zero-latency, hence the sendFunctional
            sendFunctional(prefetch_pkt);

            /* safe_cast the senderState */
            TheISA::GpuTLB::TranslationState *tlb_state =
                 safe_cast<TheISA::GpuTLB::TranslationState*>(
                         prefetch_pkt->senderState);


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
            new ComputeUnit::DataPort::SenderState(gpuDynInst, mp_index,
                                                   nullptr);

    // translation is done. Schedule the mem_req_event at the appropriate
    // cycle to send the timing memory request to ruby
    EventFunctionWrapper *mem_req_event =
        computeUnit->memPort[mp_index].createMemReqEvent(new_pkt);

    DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x data scheduled\n",
            computeUnit->cu_id, gpuDynInst->simdId,
            gpuDynInst->wfSlotId, mp_index, new_pkt->req->getPaddr());

    computeUnit->schedule(mem_req_event, curTick() +
                          computeUnit->req_tick_latency);

    return true;
}

EventFunctionWrapper*
ComputeUnit::DataPort::createMemReqEvent(PacketPtr pkt)
{
    return new EventFunctionWrapper(
        [this, pkt]{ processMemReqEvent(pkt); },
        "ComputeUnit memory request event", true);
}

EventFunctionWrapper*
ComputeUnit::DataPort::createMemRespEvent(PacketPtr pkt)
{
    return new EventFunctionWrapper(
        [this, pkt]{ processMemRespEvent(pkt); },
        "ComputeUnit memory response event", true);
}

void
ComputeUnit::DataPort::processMemReqEvent(PacketPtr pkt)
{
    SenderState *sender_state = safe_cast<SenderState*>(pkt->senderState);
    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    ComputeUnit *compute_unit M5_VAR_USED = computeUnit;

    if (!(sendTimingReq(pkt))) {
        retries.push_back(std::make_pair(pkt, gpuDynInst));

        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: index %d, addr %#x data req failed!\n",
                compute_unit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                id, pkt->req->getPaddr());
    } else {
        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: gpuDynInst: %d, index %d, addr %#x data "
                "req sent!\n", compute_unit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, gpuDynInst->seqNum(), id,
                pkt->req->getPaddr());
    }
}

const char*
ComputeUnit::ScalarDataPort::MemReqEvent::description() const
{
    return "ComputeUnit scalar memory request event";
}

void
ComputeUnit::ScalarDataPort::MemReqEvent::process()
{
    SenderState *sender_state = safe_cast<SenderState*>(pkt->senderState);
    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    ComputeUnit *compute_unit M5_VAR_USED = scalarDataPort.computeUnit;

    if (!(scalarDataPort.sendTimingReq(pkt))) {
        scalarDataPort.retries.push_back(pkt);

        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: addr %#x data req failed!\n",
                compute_unit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, pkt->req->getPaddr());
    } else {
        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: gpuDynInst: %d, addr %#x data "
                "req sent!\n", compute_unit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, gpuDynInst->seqNum(),
                pkt->req->getPaddr());
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
        Addr vaddr M5_VAR_USED = pkt->req->getVaddr();
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

    TheISA::GpuTLB::TranslationState *translation_state =
        safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

    // Page faults are not allowed
    fatal_if(!translation_state->tlbEntry,
            "Translation of vaddr %#x failed\n", pkt->req->getVaddr());

    delete translation_state->tlbEntry;
    assert(!translation_state->ports.size());

    pkt->senderState = translation_state->saved;
    delete translation_state;

    ScalarDTLBPort::SenderState *sender_state =
        safe_cast<ScalarDTLBPort::SenderState*>(pkt->senderState);

    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;
    delete pkt->senderState;

    Wavefront *w M5_VAR_USED = gpuDynInst->wavefront();

    DPRINTF(GPUTLB, "CU%d: WF[%d][%d][wv=%d]: scalar DTLB port received "
        "translation: PA %#x -> %#x\n", computeUnit->cu_id, w->simdId,
        w->wfSlotId, w->kernId, pkt->req->getVaddr(), pkt->req->getPaddr());

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

    if (!computeUnit->scalarDataPort.sendTimingReq(req_pkt)) {
        computeUnit->scalarDataPort.retries.push_back(req_pkt);
        DPRINTF(GPUMem, "send scalar req failed for: %s\n",
                gpuDynInst->disassemble());
    } else {
        DPRINTF(GPUMem, "send scalar req for: %s\n",
                gpuDynInst->disassemble());
    }

    return true;
}

bool
ComputeUnit::ITLBPort::recvTimingResp(PacketPtr pkt)
{
    Addr line M5_VAR_USED = pkt->req->getPaddr();
    DPRINTF(GPUTLB, "CU%d: ITLBPort received %#x->%#x\n",
            computeUnit->cu_id, pkt->req->getVaddr(), line);

    assert(pkt->senderState);

    // pop off the TLB translation state
    TheISA::GpuTLB::TranslationState *translation_state
        = safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

    bool success = translation_state->tlbEntry != nullptr;
    delete translation_state->tlbEntry;
    assert(!translation_state->ports.size());
    pkt->senderState = translation_state->saved;
    delete translation_state;

    // use the original sender state to know how to close this transaction
    ITLBPort::SenderState *sender_state =
        safe_cast<ITLBPort::SenderState*>(pkt->senderState);

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
        Addr vaddr M5_VAR_USED = pkt->req->getVaddr();
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
ComputeUnit::regStats()
{
    ClockedObject::regStats();

    vALUInsts
        .name(name() + ".valu_insts")
        .desc("Number of vector ALU insts issued.")
        ;
    vALUInstsPerWF
        .name(name() + ".valu_insts_per_wf")
        .desc("The avg. number of vector ALU insts issued per-wavefront.")
        ;
    sALUInsts
        .name(name() + ".salu_insts")
        .desc("Number of scalar ALU insts issued.")
        ;
    sALUInstsPerWF
        .name(name() + ".salu_insts_per_wf")
        .desc("The avg. number of scalar ALU insts issued per-wavefront.")
        ;
    instCyclesVALU
        .name(name() + ".inst_cycles_valu")
        .desc("Number of cycles needed to execute VALU insts.")
        ;
    instCyclesSALU
        .name(name() + ".inst_cycles_salu")
        .desc("Number of cycles needed to execute SALU insts.")
        ;
    threadCyclesVALU
        .name(name() + ".thread_cycles_valu")
        .desc("Number of thread cycles used to execute vector ALU ops. "
              "Similar to instCyclesVALU but multiplied by the number of "
              "active threads.")
        ;
    vALUUtilization
        .name(name() + ".valu_utilization")
        .desc("Percentage of active vector ALU threads in a wave.")
        ;
    ldsNoFlatInsts
        .name(name() + ".lds_no_flat_insts")
        .desc("Number of LDS insts issued, not including FLAT "
              "accesses that resolve to LDS.")
        ;
    ldsNoFlatInstsPerWF
        .name(name() + ".lds_no_flat_insts_per_wf")
        .desc("The avg. number of LDS insts (not including FLAT "
              "accesses that resolve to LDS) per-wavefront.")
        ;
    flatVMemInsts
        .name(name() + ".flat_vmem_insts")
        .desc("The number of FLAT insts that resolve to vmem issued.")
        ;
    flatVMemInstsPerWF
        .name(name() + ".flat_vmem_insts_per_wf")
        .desc("The average number of FLAT insts that resolve to vmem "
              "issued per-wavefront.")
        ;
    flatLDSInsts
        .name(name() + ".flat_lds_insts")
        .desc("The number of FLAT insts that resolve to LDS issued.")
        ;
    flatLDSInstsPerWF
        .name(name() + ".flat_lds_insts_per_wf")
        .desc("The average number of FLAT insts that resolve to LDS "
              "issued per-wavefront.")
        ;
    vectorMemWrites
        .name(name() + ".vector_mem_writes")
        .desc("Number of vector mem write insts (excluding FLAT insts).")
        ;
    vectorMemWritesPerWF
        .name(name() + ".vector_mem_writes_per_wf")
        .desc("The average number of vector mem write insts "
              "(excluding FLAT insts) per-wavefront.")
        ;
    vectorMemReads
        .name(name() + ".vector_mem_reads")
        .desc("Number of vector mem read insts (excluding FLAT insts).")
        ;
    vectorMemReadsPerWF
        .name(name() + ".vector_mem_reads_per_wf")
        .desc("The avg. number of vector mem read insts (excluding "
              "FLAT insts) per-wavefront.")
        ;
    scalarMemWrites
        .name(name() + ".scalar_mem_writes")
        .desc("Number of scalar mem write insts.")
        ;
    scalarMemWritesPerWF
        .name(name() + ".scalar_mem_writes_per_wf")
        .desc("The average number of scalar mem write insts per-wavefront.")
        ;
    scalarMemReads
        .name(name() + ".scalar_mem_reads")
        .desc("Number of scalar mem read insts.")
        ;
    scalarMemReadsPerWF
        .name(name() + ".scalar_mem_reads_per_wf")
        .desc("The average number of scalar mem read insts per-wavefront.")
        ;

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

    vectorMemReadsPerKiloInst
        .name(name() + ".vector_mem_reads_per_kilo_inst")
        .desc("Number of vector mem reads per kilo-instruction")
        ;
    vectorMemReadsPerKiloInst = (vectorMemReads / numInstrExecuted) * 1000;
    vectorMemWritesPerKiloInst
        .name(name() + ".vector_mem_writes_per_kilo_inst")
        .desc("Number of vector mem writes per kilo-instruction")
        ;
    vectorMemWritesPerKiloInst = (vectorMemWrites / numInstrExecuted) * 1000;
    vectorMemInstsPerKiloInst
        .name(name() + ".vector_mem_insts_per_kilo_inst")
        .desc("Number of vector mem insts per kilo-instruction")
        ;
    vectorMemInstsPerKiloInst =
        ((vectorMemReads + vectorMemWrites) / numInstrExecuted) * 1000;
    scalarMemReadsPerKiloInst
        .name(name() + ".scalar_mem_reads_per_kilo_inst")
        .desc("Number of scalar mem reads per kilo-instruction")
    ;
    scalarMemReadsPerKiloInst = (scalarMemReads / numInstrExecuted) * 1000;
    scalarMemWritesPerKiloInst
        .name(name() + ".scalar_mem_writes_per_kilo_inst")
        .desc("Number of scalar mem writes per kilo-instruction")
    ;
    scalarMemWritesPerKiloInst = (scalarMemWrites / numInstrExecuted) * 1000;
    scalarMemInstsPerKiloInst
        .name(name() + ".scalar_mem_insts_per_kilo_inst")
        .desc("Number of scalar mem insts per kilo-instruction")
        ;
    scalarMemInstsPerKiloInst =
        ((scalarMemReads + scalarMemWrites) / numInstrExecuted) * 1000;

    instCyclesVMemPerSimd
       .init(numVectorALUs)
       .name(name() + ".inst_cycles_vector_memory")
       .desc("Number of cycles to send address, command, data from VRF to "
             "vector memory unit, per SIMD")
       ;

    instCyclesScMemPerSimd
       .init(numVectorALUs)
       .name(name() + ".inst_cycles_scalar_memory")
       .desc("Number of cycles to send address, command, data from SRF to "
             "scalar memory unit, per SIMD")
       ;

    instCyclesLdsPerSimd
       .init(numVectorALUs)
       .name(name() + ".inst_cycles_lds")
       .desc("Number of cycles to send address, command, data from VRF to "
             "LDS unit, per SIMD")
       ;

    globalReads
        .name(name() + ".global_mem_reads")
        .desc("Number of reads to the global segment")
    ;
    globalWrites
        .name(name() + ".global_mem_writes")
        .desc("Number of writes to the global segment")
    ;
    globalMemInsts
        .name(name() + ".global_mem_insts")
        .desc("Number of memory instructions sent to the global segment")
    ;
    globalMemInsts = globalReads + globalWrites;
    argReads
        .name(name() + ".arg_reads")
        .desc("Number of reads to the arg segment")
    ;
    argWrites
        .name(name() + ".arg_writes")
        .desc("NUmber of writes to the arg segment")
    ;
    argMemInsts
        .name(name() + ".arg_mem_insts")
        .desc("Number of memory instructions sent to the arg segment")
    ;
    argMemInsts = argReads + argWrites;
    spillReads
        .name(name() + ".spill_reads")
        .desc("Number of reads to the spill segment")
    ;
    spillWrites
        .name(name() + ".spill_writes")
        .desc("Number of writes to the spill segment")
    ;
    spillMemInsts
        .name(name() + ".spill_mem_insts")
        .desc("Number of memory instructions sent to the spill segment")
    ;
    spillMemInsts = spillReads + spillWrites;
    groupReads
        .name(name() + ".group_reads")
        .desc("Number of reads to the group segment")
    ;
    groupWrites
        .name(name() + ".group_writes")
        .desc("Number of writes to the group segment")
    ;
    groupMemInsts
        .name(name() + ".group_mem_insts")
        .desc("Number of memory instructions sent to the group segment")
    ;
    groupMemInsts = groupReads + groupWrites;
    privReads
        .name(name() + ".private_reads")
        .desc("Number of reads to the private segment")
    ;
    privWrites
        .name(name() + ".private_writes")
        .desc("Number of writes to the private segment")
    ;
    privMemInsts
        .name(name() + ".private_mem_insts")
        .desc("Number of memory instructions sent to the private segment")
    ;
    privMemInsts = privReads + privWrites;
    readonlyReads
        .name(name() + ".readonly_reads")
        .desc("Number of reads to the readonly segment")
    ;
    readonlyWrites
        .name(name() + ".readonly_writes")
        .desc("Number of memory instructions sent to the readonly segment")
    ;
    readonlyMemInsts
        .name(name() + ".readonly_mem_insts")
        .desc("Number of memory instructions sent to the readonly segment")
    ;
    readonlyMemInsts = readonlyReads + readonlyWrites;
    kernargReads
        .name(name() + ".kernarg_reads")
        .desc("Number of reads sent to the kernarg segment")
    ;
    kernargWrites
        .name(name() + ".kernarg_writes")
        .desc("Number of memory instructions sent to the kernarg segment")
    ;
    kernargMemInsts
        .name(name() + ".kernarg_mem_insts")
        .desc("Number of memory instructions sent to the kernarg segment")
    ;
    kernargMemInsts = kernargReads + kernargWrites;

    tlbCycles
        .name(name() + ".tlb_cycles")
        .desc("total number of cycles for all uncoalesced requests")
        ;

    tlbRequests
        .name(name() + ".tlb_requests")
        .desc("number of uncoalesced requests")
        ;

    tlbLatency
        .name(name() + ".avg_translation_latency")
        .desc("Avg. translation latency for data translations")
        ;

    tlbLatency = tlbCycles / tlbRequests;

    hitsPerTLBLevel
       .init(4)
       .name(name() + ".TLB_hits_distribution")
       .desc("TLB hits distribution (0 for page table, x for Lx-TLB")
       ;

    // fixed number of TLB levels
    for (int i = 0; i < 4; ++i) {
        if (!i)
            hitsPerTLBLevel.subname(i,"page_table");
        else
            hitsPerTLBLevel.subname(i, csprintf("L%d_TLB",i));
    }

    execRateDist
        .init(0, 10, 2)
        .name(name() + ".inst_exec_rate")
        .desc("Instruction Execution Rate: Number of executed vector "
              "instructions per cycle")
        ;

    ldsBankConflictDist
       .init(0, wfSize(), 2)
       .name(name() + ".lds_bank_conflicts")
       .desc("Number of bank conflicts per LDS memory packet")
       ;

    ldsBankAccesses
        .name(name() + ".lds_bank_access_cnt")
        .desc("Total number of LDS bank accesses")
        ;

    pageDivergenceDist
        // A wavefront can touch up to N pages per memory instruction where
        // N is equal to the wavefront size
        // The number of pages per bin can be configured (here it's 4).
       .init(1, wfSize(), 4)
       .name(name() + ".page_divergence_dist")
       .desc("pages touched per wf (over all mem. instr.)")
       ;

    controlFlowDivergenceDist
        .init(1, wfSize(), 4)
        .name(name() + ".warp_execution_dist")
        .desc("number of lanes active per instruction (oval all instructions)")
        ;

    activeLanesPerGMemInstrDist
        .init(1, wfSize(), 4)
        .name(name() + ".gmem_lanes_execution_dist")
        .desc("number of active lanes per global memory instruction")
        ;

    activeLanesPerLMemInstrDist
        .init(1, wfSize(), 4)
        .name(name() + ".lmem_lanes_execution_dist")
        .desc("number of active lanes per local memory instruction")
        ;

    numInstrExecuted
        .name(name() + ".num_instr_executed")
        .desc("number of instructions executed")
        ;

    numVecOpsExecuted
        .name(name() + ".num_vec_ops_executed")
        .desc("number of vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedF16
        .name(name() + ".num_vec_ops_f16_executed")
        .desc("number of f16 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedF32
        .name(name() + ".num_vec_ops_f32_executed")
        .desc("number of f32 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedF64
        .name(name() + ".num_vec_ops_f64_executed")
        .desc("number of f64 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedFMA16
        .name(name() + ".num_vec_ops_fma16_executed")
        .desc("number of fma16 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedFMA32
        .name(name() + ".num_vec_ops_fma32_executed")
        .desc("number of fma32 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedFMA64
        .name(name() + ".num_vec_ops_fma64_executed")
        .desc("number of fma64 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedMAD16
        .name(name() + ".num_vec_ops_mad16_executed")
        .desc("number of mad16 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedMAD32
        .name(name() + ".num_vec_ops_mad32_executed")
        .desc("number of mad32 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedMAD64
        .name(name() + ".num_vec_ops_mad64_executed")
        .desc("number of mad64 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedMAC16
        .name(name() + ".num_vec_ops_mac16_executed")
        .desc("number of mac16 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedMAC32
        .name(name() + ".num_vec_ops_mac32_executed")
        .desc("number of mac32 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedMAC64
        .name(name() + ".num_vec_ops_mac64_executed")
        .desc("number of mac64 vec ops executed (e.g. WF size/inst)")
        ;

    numVecOpsExecutedTwoOpFP
        .name(name() + ".num_vec_ops_two_op_fp_executed")
        .desc("number of two op FP vec ops executed (e.g. WF size/inst)")
        ;

    totalCycles
        .name(name() + ".num_total_cycles")
        .desc("number of cycles the CU ran for")
        ;

    ipc
        .name(name() + ".ipc")
        .desc("Instructions per cycle (this CU only)")
        ;

    vpc
        .name(name() + ".vpc")
        .desc("Vector Operations per cycle (this CU only)")
        ;

    vpc_f16
        .name(name() + ".vpc_f16")
        .desc("F16 Vector Operations per cycle (this CU only)")
        ;

    vpc_f32
        .name(name() + ".vpc_f32")
        .desc("F32 Vector Operations per cycle (this CU only)")
        ;

    vpc_f64
        .name(name() + ".vpc_f64")
        .desc("F64 Vector Operations per cycle (this CU only)")
        ;

    numALUInstsExecuted
        .name(name() + ".num_alu_insts_executed")
        .desc("Number of dynamic non-GM memory insts executed")
        ;

    wgBlockedDueBarrierAllocation
        .name(name() + ".wg_blocked_due_barrier_alloc")
        .desc("WG dispatch was blocked due to lack of barrier resources")
        ;

    wgBlockedDueLdsAllocation
        .name(name() + ".wg_blocked_due_lds_alloc")
        .desc("Workgroup blocked due to LDS capacity")
        ;

    ipc = numInstrExecuted / totalCycles;
    vpc = numVecOpsExecuted / totalCycles;
    vpc_f16 = numVecOpsExecutedF16 / totalCycles;
    vpc_f32 = numVecOpsExecutedF32 / totalCycles;
    vpc_f64 = numVecOpsExecutedF64 / totalCycles;

    numTimesWgBlockedDueVgprAlloc
        .name(name() + ".times_wg_blocked_due_vgpr_alloc")
        .desc("Number of times WGs are blocked due to VGPR allocation per "
              "SIMD")
        ;

    numTimesWgBlockedDueSgprAlloc
        .name(name() + ".times_wg_blocked_due_sgpr_alloc")
        .desc("Number of times WGs are blocked due to SGPR allocation per "
              "SIMD")
        ;

    dynamicGMemInstrCnt
        .name(name() + ".global_mem_instr_cnt")
        .desc("dynamic non-flat global memory instruction count")
        ;

    dynamicFlatMemInstrCnt
        .name(name() + ".flat_global_mem_instr_cnt")
        .desc("dynamic flat global memory instruction count")
        ;

    dynamicLMemInstrCnt
        .name(name() + ".local_mem_instr_cnt")
        .desc("dynamic local memory intruction count")
        ;

    numALUInstsExecuted = numInstrExecuted - dynamicGMemInstrCnt -
        dynamicLMemInstrCnt;

    completedWfs
        .name(name() + ".num_completed_wfs")
        .desc("number of completed wavefronts")
        ;

    completedWGs
        .name(name() + ".num_completed_wgs")
        .desc("number of completed workgroups")
        ;

    numCASOps
        .name(name() + ".num_CAS_ops")
        .desc("number of compare and swap operations")
        ;

    numFailedCASOps
        .name(name() + ".num_failed_CAS_ops")
        .desc("number of compare and swap operations that failed")
        ;

    headTailLatency
        .init(0, 1000000, 10000)
        .name(name() + ".head_tail_latency")
        .desc("ticks between first and last cache block arrival at coalescer")
        .flags(Stats::pdf | Stats::oneline)
        ;

    waveLevelParallelism
        .init(0, shader->n_wf * numVectorALUs, 1)
        .name(name() + ".wlp")
        .desc("wave level parallelism: count of active waves at wave launch")
        ;

    instInterleave
        .init(numVectorALUs, 0, 20, 1)
        .name(name() + ".interleaving")
        .desc("Measure of instruction interleaving per SIMD")
        ;

    // register stats of pipeline stages
    fetchStage.regStats();
    scoreboardCheckStage.regStats();
    scheduleStage.regStats();
    execStage.regStats();

    // register stats of memory pipelines
    globalMemoryPipe.regStats();
    localMemoryPipe.regStats();
    scalarMemoryPipe.regStats();

    registerManager->regStats();
}

void
ComputeUnit::updateInstStats(GPUDynInstPtr gpuDynInst)
{
    if (gpuDynInst->isScalar()) {
        if (gpuDynInst->isALU() && !gpuDynInst->isWaitcnt()) {
            sALUInsts++;
            instCyclesSALU++;
        } else if (gpuDynInst->isLoad()) {
            scalarMemReads++;
        } else if (gpuDynInst->isStore()) {
            scalarMemWrites++;
        }
    } else {
        if (gpuDynInst->isALU()) {
            shader->total_valu_insts++;
            if (shader->total_valu_insts == shader->max_valu_insts) {
                exitSimLoop("max vALU insts");
            }
            vALUInsts++;
            instCyclesVALU++;
            threadCyclesVALU += gpuDynInst->wavefront()->execMask().count();
        } else if (gpuDynInst->isFlat()) {
            if (gpuDynInst->isLocalMem()) {
                flatLDSInsts++;
            } else {
                flatVMemInsts++;
            }
        } else if (gpuDynInst->isLocalMem()) {
            ldsNoFlatInsts++;
        } else if (gpuDynInst->isLoad()) {
            vectorMemReads++;
        } else if (gpuDynInst->isStore()) {
            vectorMemWrites++;
        }

        if (gpuDynInst->isLoad()) {
            switch (gpuDynInst->executedAs()) {
              case Enums::SC_SPILL:
                spillReads++;
                break;
              case Enums::SC_GLOBAL:
                globalReads++;
                break;
              case Enums::SC_GROUP:
                groupReads++;
                break;
              case Enums::SC_PRIVATE:
                privReads++;
                break;
              case Enums::SC_READONLY:
                readonlyReads++;
                break;
              case Enums::SC_KERNARG:
                kernargReads++;
                break;
              case Enums::SC_ARG:
                argReads++;
                break;
              case Enums::SC_NONE:
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
              case Enums::SC_SPILL:
                spillWrites++;
                break;
              case Enums::SC_GLOBAL:
                globalWrites++;
                break;
              case Enums::SC_GROUP:
                groupWrites++;
                break;
              case Enums::SC_PRIVATE:
                privWrites++;
                break;
              case Enums::SC_READONLY:
                readonlyWrites++;
                break;
              case Enums::SC_KERNARG:
                kernargWrites++;
                break;
              case Enums::SC_ARG:
                argWrites++;
                break;
              case Enums::SC_NONE:
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
    Addr virt_page_addr = roundDown(addr, TheISA::PageBytes);

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

        *page_stat_file << "page, wavefront accesses, workitem accesses" <<
            std::endl;

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

    if (!globalMemoryPipe.isGMReqFIFOWrRdy()
        || !localMemoryPipe.isLMReqFIFOWrRdy()
        || !localMemoryPipe.isLMRespFIFOWrRdy() || !locMemToVrfBus.rdy() ||
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

    for (int i_wf = 0; i_wf < shader->n_wf; ++i_wf){
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
            dynamic_cast<ComputeUnit::LDSPort::SenderState*>(pkt->senderState);
    fatal_if(!sender_state, "packet without a valid sender state");

    GPUDynInstPtr gpuDynInst M5_VAR_USED = sender_state->getMemInst();

    if (isStalled()) {
        fatal_if(retries.empty(), "must have retries waiting to be stalled");

        retries.push(pkt);

        DPRINTF(GPUPort, "CU%d: WF[%d][%d]: LDS send failed!\n",
                        computeUnit->cu_id, gpuDynInst->simdId,
                        gpuDynInst->wfSlotId);
        return false;
    } else if (!RequestPort::sendTimingReq(pkt)) {
        // need to stall the LDS port until a recvReqRetry() is received
        // this indicates that there is more space
        stallPort();
        retries.push(pkt);

        DPRINTF(GPUPort, "CU%d: WF[%d][%d]: addr %#x lds req failed!\n",
                computeUnit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, pkt->req->getPaddr());
        return false;
    } else {
        DPRINTF(GPUPort, "CU%d: WF[%d][%d]: addr %#x lds req sent!\n",
                computeUnit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, pkt->req->getPaddr());
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
