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

#include "base/output.hh"
#include "debug/GPUDisp.hh"
#include "debug/GPUExec.hh"
#include "debug/GPUFetch.hh"
#include "debug/GPUMem.hh"
#include "debug/GPUPort.hh"
#include "debug/GPUPrefetch.hh"
#include "debug/GPUSync.hh"
#include "debug/GPUTLB.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/ndrange.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/simple_pool_manager.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"
#include "mem/page_table.hh"
#include "sim/process.hh"

ComputeUnit::ComputeUnit(const Params *p) : ClockedObject(p), fetchStage(p),
    scoreboardCheckStage(p), scheduleStage(p), execStage(p),
    globalMemoryPipe(p), localMemoryPipe(p), rrNextMemID(0), rrNextALUWp(0),
    cu_id(p->cu_id), vrf(p->vector_register_file), numSIMDs(p->num_SIMDs),
    spBypassPipeLength(p->spbypass_pipe_length),
    dpBypassPipeLength(p->dpbypass_pipe_length),
    issuePeriod(p->issue_period),
    numGlbMemUnits(p->num_global_mem_pipes),
    numLocMemUnits(p->num_shared_mem_pipes),
    perLaneTLB(p->perLaneTLB), prefetchDepth(p->prefetch_depth),
    prefetchStride(p->prefetch_stride), prefetchType(p->prefetch_prev_type),
    xact_cas_mode(p->xactCasMode), debugSegFault(p->debugSegFault),
    functionalTLB(p->functionalTLB), localMemBarrier(p->localMemBarrier),
    countPages(p->countPages), barrier_id(0),
    vrfToCoalescerBusWidth(p->vrf_to_coalescer_bus_width),
    coalescerToVrfBusWidth(p->coalescer_to_vrf_bus_width),
    req_tick_latency(p->mem_req_latency * p->clk_domain->clockPeriod()),
    resp_tick_latency(p->mem_resp_latency * p->clk_domain->clockPeriod()),
    _masterId(p->system->getMasterId(this, "ComputeUnit")),
    lds(*p->localDataStore), _cacheLineSize(p->system->cacheLineSize()),
    globalSeqNum(0), wavefrontSize(p->wfSize),
    kernelLaunchInst(new KernelLaunchStaticInst())
{
    /**
     * This check is necessary because std::bitset only provides conversion
     * to unsigned long or unsigned long long via to_ulong() or to_ullong().
     * there are * a few places in the code where to_ullong() is used, however
     * if VSZ is larger than a value the host can support then bitset will
     * throw a runtime exception. we should remove all use of to_long() or
     * to_ullong() so we can have VSZ greater than 64b, however until that is
     * done this assert is required.
     */
    fatal_if(p->wfSize > std::numeric_limits<unsigned long long>::digits ||
             p->wfSize <= 0,
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

    lastVaddrWF.resize(numSIMDs);
    wfList.resize(numSIMDs);

    for (int j = 0; j < numSIMDs; ++j) {
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

    lastVaddrSimd.resize(numSIMDs);

    for (int i = 0; i < numSIMDs; ++i) {
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

    memPort.resize(wfSize());

    // resize the tlbPort vectorArray
    int tlbPort_width = perLaneTLB ? wfSize() : 1;
    tlbPort.resize(tlbPort_width);

    cuExitCallback = new CUExitCallback(this);
    registerExitCallback(cuExitCallback);

    xactCasLoadMap.clear();
    lastExecCycle.resize(numSIMDs, 0);

    for (int i = 0; i < vrf.size(); ++i) {
        vrf[i]->setParent(this);
    }

    numVecRegsPerSimd = vrf[0]->numRegs();
}

ComputeUnit::~ComputeUnit()
{
    // Delete wavefront slots
    for (int j = 0; j < numSIMDs; ++j) {
        for (int i = 0; i < shader->n_wf; ++i) {
            delete wfList[j][i];
        }
        lastVaddrSimd[j].clear();
    }
    lastVaddrCU.clear();
    readyList.clear();
    waveStatusList.clear();
    dispatchList.clear();
    vectorAluInstAvail.clear();
    delete cuExitCallback;
    delete ldsPort;
}

void
ComputeUnit::fillKernelState(Wavefront *w, NDRange *ndr)
{
    w->resizeRegFiles(ndr->q.cRegCount, ndr->q.sRegCount, ndr->q.dRegCount);

    w->workGroupSz[0] = ndr->q.wgSize[0];
    w->workGroupSz[1] = ndr->q.wgSize[1];
    w->workGroupSz[2] = ndr->q.wgSize[2];
    w->wgSz = w->workGroupSz[0] * w->workGroupSz[1] * w->workGroupSz[2];
    w->gridSz[0] = ndr->q.gdSize[0];
    w->gridSz[1] = ndr->q.gdSize[1];
    w->gridSz[2] = ndr->q.gdSize[2];
    w->kernelArgs = ndr->q.args;
    w->privSizePerItem = ndr->q.privMemPerItem;
    w->spillSizePerItem = ndr->q.spillMemPerItem;
    w->roBase = ndr->q.roMemStart;
    w->roSize = ndr->q.roMemTotal;
    w->computeActualWgSz(ndr);
}

void
ComputeUnit::updateEvents() {

    if (!timestampVec.empty()) {
        uint32_t vecSize = timestampVec.size();
        uint32_t i = 0;
        while (i < vecSize) {
            if (timestampVec[i] <= shader->tick_cnt) {
                std::pair<uint32_t, uint32_t> regInfo = regIdxVec[i];
                vrf[regInfo.first]->markReg(regInfo.second, sizeof(uint32_t),
                                            statusVec[i]);
                timestampVec.erase(timestampVec.begin() + i);
                regIdxVec.erase(regIdxVec.begin() + i);
                statusVec.erase(statusVec.begin() + i);
                --vecSize;
                --i;
            }
            ++i;
        }
    }

    for (int i = 0; i< numSIMDs; ++i) {
        vrf[i]->updateEvents();
    }
}


void
ComputeUnit::startWavefront(Wavefront *w, int waveId, LdsChunk *ldsChunk,
                            NDRange *ndr)
{
    static int _n_wave = 0;

    VectorMask init_mask;
    init_mask.reset();

    for (int k = 0; k < wfSize(); ++k) {
        if (k + waveId * wfSize() < w->actualWgSzTotal)
            init_mask[k] = 1;
    }

    w->kernId = ndr->dispatchId;
    w->wfId = waveId;
    w->initMask = init_mask.to_ullong();

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

    w->barrierSlots = divCeil(w->actualWgSzTotal, wfSize());

    w->barCnt.resize(wfSize(), 0);

    w->maxBarCnt = 0;
    w->oldBarrierCnt = 0;
    w->barrierCnt = 0;

    w->privBase = ndr->q.privMemStart;
    ndr->q.privMemStart += ndr->q.privMemPerItem * wfSize();

    w->spillBase = ndr->q.spillMemStart;
    ndr->q.spillMemStart += ndr->q.spillMemPerItem * wfSize();

    w->pushToReconvergenceStack(0, UINT32_MAX, init_mask.to_ulong());

    // WG state
    w->wgId = ndr->globalWgId;
    w->dispatchId = ndr->dispatchId;
    w->workGroupId[0] = w->wgId % ndr->numWg[0];
    w->workGroupId[1] = (w->wgId / ndr->numWg[0]) % ndr->numWg[1];
    w->workGroupId[2] = w->wgId / (ndr->numWg[0] * ndr->numWg[1]);

    w->barrierId = barrier_id;
    w->stalledAtBarrier = false;

    // set the wavefront context to have a pointer to this section of the LDS
    w->ldsChunk = ldsChunk;

    int32_t refCount M5_VAR_USED =
                    lds.increaseRefCounter(w->dispatchId, w->wgId);
    DPRINTF(GPUDisp, "CU%d: increase ref ctr wg[%d] to [%d]\n",
                    cu_id, w->wgId, refCount);

    w->instructionBuffer.clear();

    if (w->pendingFetch)
        w->dropFetch = true;

    // is this the last wavefront in the workgroup
    // if set the spillWidth to be the remaining work-items
    // so that the vector access is correct
    if ((waveId + 1) * wfSize() >= w->actualWgSzTotal) {
        w->spillWidth = w->actualWgSzTotal - (waveId * wfSize());
    } else {
        w->spillWidth = wfSize();
    }

    DPRINTF(GPUDisp, "Scheduling wfDynId/barrier_id %d/%d on CU%d: "
            "WF[%d][%d]\n", _n_wave, barrier_id, cu_id, w->simdId, w->wfSlotId);

    w->start(++_n_wave, ndr->q.code_ptr);
}

void
ComputeUnit::StartWorkgroup(NDRange *ndr)
{
    // reserve the LDS capacity allocated to the work group
    // disambiguated by the dispatch ID and workgroup ID, which should be
    // globally unique
    LdsChunk *ldsChunk = lds.reserveSpace(ndr->dispatchId, ndr->globalWgId,
                                          ndr->q.ldsSize);

    // Send L1 cache acquire
    // isKernel + isAcquire = Kernel Begin
    if (shader->impl_kern_boundary_sync) {
        GPUDynInstPtr gpuDynInst =
            std::make_shared<GPUDynInst>(this, nullptr, kernelLaunchInst,
                                         getAndIncSeqNum());

        gpuDynInst->useContinuation = false;
        injectGlobalMemFence(gpuDynInst, true);
    }

    // calculate the number of 32-bit vector registers required by wavefront
    int vregDemand = ndr->q.sRegCount + (2 * ndr->q.dRegCount);
    int wave_id = 0;

    // Assign WFs by spreading them across SIMDs, 1 WF per SIMD at a time
    for (int m = 0; m < shader->n_wf * numSIMDs; ++m) {
        Wavefront *w = wfList[m % numSIMDs][m / numSIMDs];
        // Check if this wavefront slot is available:
        // It must be stopped and not waiting
        // for a release to complete S_RETURNING
        if (w->status == Wavefront::S_STOPPED) {
            fillKernelState(w, ndr);
            // if we have scheduled all work items then stop
            // scheduling wavefronts
            if (wave_id * wfSize() >= w->actualWgSzTotal)
                break;

            // reserve vector registers for the scheduled wavefront
            assert(vectorRegsReserved[m % numSIMDs] <= numVecRegsPerSimd);
            uint32_t normSize = 0;

            w->startVgprIndex = vrf[m % numSIMDs]->manager->
                                    allocateRegion(vregDemand, &normSize);

            w->reservedVectorRegs = normSize;
            vectorRegsReserved[m % numSIMDs] += w->reservedVectorRegs;

            startWavefront(w, wave_id, ldsChunk, ndr);
            ++wave_id;
        }
    }
    ++barrier_id;
}

int
ComputeUnit::ReadyWorkgroup(NDRange *ndr)
{
    // Get true size of workgroup (after clamping to grid size)
    int trueWgSize[3];
    int trueWgSizeTotal = 1;

    for (int d = 0; d < 3; ++d) {
        trueWgSize[d] = std::min(ndr->q.wgSize[d], ndr->q.gdSize[d] -
                                 ndr->wgId[d] * ndr->q.wgSize[d]);

        trueWgSizeTotal *= trueWgSize[d];
        DPRINTF(GPUDisp, "trueWgSize[%d] =  %d\n", d, trueWgSize[d]);
    }

    DPRINTF(GPUDisp, "trueWgSizeTotal =  %d\n", trueWgSizeTotal);

    // calculate the number of 32-bit vector registers required by each
    // work item of the work group
    int vregDemandPerWI = ndr->q.sRegCount + (2 * ndr->q.dRegCount);
    bool vregAvail = true;
    int numWfs = (trueWgSizeTotal + wfSize() - 1) / wfSize();
    int freeWfSlots = 0;
    // check if the total number of VGPRs required by all WFs of the WG
    // fit in the VRFs of all SIMD units
    assert((numWfs * vregDemandPerWI) <= (numSIMDs * numVecRegsPerSimd));
    int numMappedWfs = 0;
    std::vector<int> numWfsPerSimd;
    numWfsPerSimd.resize(numSIMDs, 0);
    // find how many free WF slots we have across all SIMDs
    for (int j = 0; j < shader->n_wf; ++j) {
        for (int i = 0; i < numSIMDs; ++i) {
            if (wfList[i][j]->status == Wavefront::S_STOPPED) {
                // count the number of free WF slots
                ++freeWfSlots;
                if (numMappedWfs < numWfs) {
                    // count the WFs to be assigned per SIMD
                    numWfsPerSimd[i]++;
                }
                numMappedWfs++;
            }
        }
    }

    // if there are enough free WF slots then find if there are enough
    // free VGPRs per SIMD based on the WF->SIMD mapping
    if (freeWfSlots >= numWfs) {
        for (int j = 0; j < numSIMDs; ++j) {
            // find if there are enough free VGPR regions in the SIMD's VRF
            // to accommodate the WFs of the new WG that would be mapped to
            // this SIMD unit
            vregAvail = vrf[j]->manager->canAllocate(numWfsPerSimd[j],
                                                     vregDemandPerWI);

            // stop searching if there is at least one SIMD
            // whose VRF does not have enough free VGPR pools.
            // This is because a WG is scheduled only if ALL
            // of its WFs can be scheduled
            if (!vregAvail)
                break;
        }
    }

    DPRINTF(GPUDisp, "Free WF slots =  %d, VGPR Availability = %d\n",
            freeWfSlots, vregAvail);

    if (!vregAvail) {
        ++numTimesWgBlockedDueVgprAlloc;
    }

    // Return true if enough WF slots to submit workgroup and if there are
    // enough VGPRs to schedule all WFs to their SIMD units
    if (!lds.canReserve(ndr->q.ldsSize)) {
        wgBlockedDueLdsAllocation++;
    }

    // Return true if (a) there are enough free WF slots to submit
    // workgrounp and (b) if there are enough VGPRs to schedule all WFs to their
    // SIMD units and (c) if there is enough space in LDS
    return freeWfSlots >= numWfs && vregAvail && lds.canReserve(ndr->q.ldsSize);
}

int
ComputeUnit::AllAtBarrier(uint32_t _barrier_id, uint32_t bcnt, uint32_t bslots)
{
    DPRINTF(GPUSync, "CU%d: Checking for All At Barrier\n", cu_id);
    int ccnt = 0;

    for (int i_simd = 0; i_simd < numSIMDs; ++i_simd) {
        for (int i_wf = 0; i_wf < shader->n_wf; ++i_wf) {
            Wavefront *w = wfList[i_simd][i_wf];

            if (w->status == Wavefront::S_RUNNING) {
                DPRINTF(GPUSync, "Checking WF[%d][%d]\n", i_simd, i_wf);

                DPRINTF(GPUSync, "wf->barrier_id = %d, _barrier_id = %d\n",
                        w->barrierId, _barrier_id);

                DPRINTF(GPUSync, "wf->barrier_cnt %d, bcnt = %d\n",
                        w->barrierCnt, bcnt);
            }

            if (w->status == Wavefront::S_RUNNING &&
                w->barrierId == _barrier_id && w->barrierCnt == bcnt &&
                !w->outstandingReqs) {
                ++ccnt;

                DPRINTF(GPUSync, "WF[%d][%d] at barrier, increment ccnt to "
                        "%d\n", i_simd, i_wf, ccnt);
            }
        }
    }

    DPRINTF(GPUSync, "CU%d: returning allAtBarrier ccnt = %d, bslots = %d\n",
            cu_id, ccnt, bslots);

    return ccnt == bslots;
}

//  Check if the current wavefront is blocked on additional resources.
bool
ComputeUnit::cedeSIMD(int simdId, int wfSlotId)
{
    bool cede = false;

    // If --xact-cas-mode option is enabled in run.py, then xact_cas_ld
    // magic instructions will impact the scheduling of wavefronts
    if (xact_cas_mode) {
        /*
         * When a wavefront calls xact_cas_ld, it adds itself to a per address
         * queue. All per address queues are managed by the xactCasLoadMap.
         *
         * A wavefront is not blocked if: it is not in ANY per address queue or
         * if it is at the head of a per address queue.
         */
        for (auto itMap : xactCasLoadMap) {
            std::list<waveIdentifier> curWaveIDQueue = itMap.second.waveIDQueue;

            if (!curWaveIDQueue.empty()) {
                for (auto it : curWaveIDQueue) {
                    waveIdentifier cur_wave = it;

                    if (cur_wave.simdId == simdId &&
                        cur_wave.wfSlotId == wfSlotId) {
                        // 2 possibilities
                        // 1: this WF has a green light
                        // 2: another WF has a green light
                        waveIdentifier owner_wave = curWaveIDQueue.front();

                        if (owner_wave.simdId != cur_wave.simdId ||
                            owner_wave.wfSlotId != cur_wave.wfSlotId) {
                            // possibility 2
                            cede = true;
                            break;
                        } else {
                            // possibility 1
                            break;
                        }
                    }
                }
            }
        }
    }

    return cede;
}

// Execute one clock worth of work on the ComputeUnit.
void
ComputeUnit::exec()
{
    updateEvents();
    // Execute pipeline stages in reverse order to simulate
    // the pipeline latency
    globalMemoryPipe.exec();
    localMemoryPipe.exec();
    execStage.exec();
    scheduleStage.exec();
    scoreboardCheckStage.exec();
    fetchStage.exec();

    totalCycles++;
}

void
ComputeUnit::init()
{
    // Initialize CU Bus models
    glbMemToVrfBus.init(&shader->tick_cnt, shader->ticks(1));
    locMemToVrfBus.init(&shader->tick_cnt, shader->ticks(1));
    nextGlbMemBus = 0;
    nextLocMemBus = 0;
    fatal_if(numGlbMemUnits > 1,
             "No support for multiple Global Memory Pipelines exists!!!");
    vrfToGlobalMemPipeBus.resize(numGlbMemUnits);
    for (int j = 0; j < numGlbMemUnits; ++j) {
        vrfToGlobalMemPipeBus[j] = WaitClass();
        vrfToGlobalMemPipeBus[j].init(&shader->tick_cnt, shader->ticks(1));
    }

    fatal_if(numLocMemUnits > 1,
             "No support for multiple Local Memory Pipelines exists!!!");
    vrfToLocalMemPipeBus.resize(numLocMemUnits);
    for (int j = 0; j < numLocMemUnits; ++j) {
        vrfToLocalMemPipeBus[j] = WaitClass();
        vrfToLocalMemPipeBus[j].init(&shader->tick_cnt, shader->ticks(1));
    }
    vectorRegsReserved.resize(numSIMDs, 0);
    aluPipe.resize(numSIMDs);
    wfWait.resize(numSIMDs + numLocMemUnits + numGlbMemUnits);

    for (int i = 0; i < numSIMDs + numLocMemUnits + numGlbMemUnits; ++i) {
        wfWait[i] = WaitClass();
        wfWait[i].init(&shader->tick_cnt, shader->ticks(1));
    }

    for (int i = 0; i < numSIMDs; ++i) {
        aluPipe[i] = WaitClass();
        aluPipe[i].init(&shader->tick_cnt, shader->ticks(1));
    }

    // Setup space for call args
    for (int j = 0; j < numSIMDs; ++j) {
        for (int i = 0; i < shader->n_wf; ++i) {
            wfList[j][i]->initCallArgMem(shader->funcargs_size, wavefrontSize);
        }
    }

    // Initializing pipeline resources
    readyList.resize(numSIMDs + numGlbMemUnits + numLocMemUnits);
    waveStatusList.resize(numSIMDs);

    for (int j = 0; j < numSIMDs; ++j) {
        for (int i = 0; i < shader->n_wf; ++i) {
            waveStatusList[j].push_back(
                std::make_pair(wfList[j][i], BLOCKED));
        }
    }

    for (int j = 0; j < (numSIMDs + numGlbMemUnits + numLocMemUnits); ++j) {
        dispatchList.push_back(std::make_pair((Wavefront*)nullptr, EMPTY));
    }

    fetchStage.init(this);
    scoreboardCheckStage.init(this);
    scheduleStage.init(this);
    execStage.init(this);
    globalMemoryPipe.init(this);
    localMemoryPipe.init(this);
    // initialize state for statistics calculation
    vectorAluInstAvail.resize(numSIMDs, false);
    shrMemInstAvail = 0;
    glbMemInstAvail = 0;
}

bool
ComputeUnit::DataPort::recvTimingResp(PacketPtr pkt)
{
    // Ruby has completed the memory op. Schedule the mem_resp_event at the
    // appropriate cycle to process the timing memory response
    // This delay represents the pipeline delay
    SenderState *sender_state = safe_cast<SenderState*>(pkt->senderState);
    int index = sender_state->port_index;
    GPUDynInstPtr gpuDynInst = sender_state->_gpuDynInst;

    // Is the packet returned a Kernel End or Barrier
    if (pkt->req->isKernel() && pkt->req->isRelease()) {
        Wavefront *w =
            computeUnit->wfList[gpuDynInst->simdId][gpuDynInst->wfSlotId];

        // Check if we are waiting on Kernel End Release
        if (w->status == Wavefront::S_RETURNING) {
            DPRINTF(GPUDisp, "CU%d: WF[%d][%d][wv=%d]: WG id completed %d\n",
                    computeUnit->cu_id, w->simdId, w->wfSlotId,
                    w->wfDynId, w->kernId);

            computeUnit->shader->dispatcher->notifyWgCompl(w);
            w->status = Wavefront::S_STOPPED;
        } else {
            w->outstandingReqs--;
        }

        DPRINTF(GPUSync, "CU%d: WF[%d][%d]: barrier_cnt = %d\n",
                computeUnit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, w->barrierCnt);

        if (gpuDynInst->useContinuation) {
            assert(!gpuDynInst->isNoScope());
            gpuDynInst->execContinuation(gpuDynInst->staticInstruction(),
                                           gpuDynInst);
        }

        delete pkt->senderState;
        delete pkt;
        return true;
    } else if (pkt->req->isKernel() && pkt->req->isAcquire()) {
        if (gpuDynInst->useContinuation) {
            assert(!gpuDynInst->isNoScope());
            gpuDynInst->execContinuation(gpuDynInst->staticInstruction(),
                                           gpuDynInst);
        }

        delete pkt->senderState;
        delete pkt;
        return true;
    }

    EventFunctionWrapper *mem_resp_event =
        computeUnit->memPort[index]->createMemRespEvent(pkt);

    DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x received!\n",
            computeUnit->cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
            index, pkt->req->getPaddr());

    computeUnit->schedule(mem_resp_event,
                          curTick() + computeUnit->resp_tick_latency);
    return true;
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
ComputeUnit::sendRequest(GPUDynInstPtr gpuDynInst, int index, PacketPtr pkt)
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

    // Check write before read for atomic operations
    // since atomic operations should use BaseTLB::Write
    if (pkt->isWrite()){
        TLB_mode = BaseTLB::Write;
    } else if (pkt->isRead()) {
        TLB_mode = BaseTLB::Read;
    } else {
        fatal("pkt is not a read nor a write\n");
    }

    tlbCycles -= curTick();
    ++tlbRequests;

    int tlbPort_index = perLaneTLB ? index : 0;

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
            tlbPort[tlbPort_index]->sendFunctional(pkt);

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

            uint8_t *tmpData = pkt->getPtr<uint8_t>();

            // this is necessary because the GPU TLB receives packets instead
            // of requests. when the translation is complete, all relevent
            // fields in the request will be populated, but not in the packet.
            // here we create the new packet so we can set the size, addr,
            // and proper flags.
            PacketPtr oldPkt = pkt;
            pkt = new Packet(oldPkt->req, oldPkt->cmd);
            delete oldPkt;
            pkt->dataStatic(tmpData);


            // New SenderState for the memory access
            pkt->senderState = new ComputeUnit::DataPort::SenderState(gpuDynInst,
                                                             index, nullptr);

            gpuDynInst->memStatusVector[pkt->getAddr()].push_back(index);
            gpuDynInst->tlbHitLevel[index] = hit_level;


            // translation is done. Schedule the mem_req_event at the
            // appropriate cycle to send the timing memory request to ruby
            EventFunctionWrapper *mem_req_event =
                memPort[index]->createMemReqEvent(pkt);

            DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x data "
                    "scheduled\n", cu_id, gpuDynInst->simdId,
                    gpuDynInst->wfSlotId, index, pkt->req->getPaddr());

            schedule(mem_req_event, curTick() + req_tick_latency);
        } else if (tlbPort[tlbPort_index]->isStalled()) {
            assert(tlbPort[tlbPort_index]->retries.size() > 0);

            DPRINTF(GPUTLB, "CU%d: WF[%d][%d]: Translation for addr %#x "
                    "failed!\n", cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                    tmp_vaddr);

            tlbPort[tlbPort_index]->retries.push_back(pkt);
        } else if (!tlbPort[tlbPort_index]->sendTimingReq(pkt)) {
            // Stall the data port;
            // No more packet will be issued till
            // ruby indicates resources are freed by
            // a recvReqRetry() call back on this port.
            tlbPort[tlbPort_index]->stallPort();

            DPRINTF(GPUTLB, "CU%d: WF[%d][%d]: Translation for addr %#x "
                    "failed!\n", cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId,
                    tmp_vaddr);

            tlbPort[tlbPort_index]->retries.push_back(pkt);
        } else {
           DPRINTF(GPUTLB,
                   "CU%d: WF[%d][%d]: Translation for addr %#x sent!\n",
                   cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, tmp_vaddr);
        }
    } else {
        if (pkt->cmd == MemCmd::MemFenceReq) {
            gpuDynInst->statusBitVector = VectorMask(0);
        } else {
            gpuDynInst->statusBitVector &= (~(1ll << index));
        }

        // New SenderState for the memory access
        delete pkt->senderState;

        // Because it's atomic operation, only need TLB translation state
        pkt->senderState = new TheISA::GpuTLB::TranslationState(TLB_mode,
                                                                shader->gpuTc);

        tlbPort[tlbPort_index]->sendFunctional(pkt);

        // the addr of the packet is not modified, so we need to create a new
        // packet, or otherwise the memory access will have the old virtual
        // address sent in the translation packet, instead of the physical
        // address returned by the translation.
        PacketPtr new_pkt = new Packet(pkt->req, pkt->cmd);
        new_pkt->dataStatic(pkt->getPtr<uint8_t>());

        // Translation is done. It is safe to send the packet to memory.
        memPort[0]->sendFunctional(new_pkt);

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
ComputeUnit::sendSyncRequest(GPUDynInstPtr gpuDynInst, int index, PacketPtr pkt)
{
    EventFunctionWrapper *mem_req_event =
        memPort[index]->createMemReqEvent(pkt);


    // New SenderState for the memory access
    pkt->senderState = new ComputeUnit::DataPort::SenderState(gpuDynInst, index,
                                                              nullptr);

    DPRINTF(GPUPort, "CU%d: WF[%d][%d]: index %d, addr %#x sync scheduled\n",
            cu_id, gpuDynInst->simdId, gpuDynInst->wfSlotId, index,
            pkt->req->getPaddr());

    schedule(mem_req_event, curTick() + req_tick_latency);
}

void
ComputeUnit::injectGlobalMemFence(GPUDynInstPtr gpuDynInst, bool kernelLaunch,
                                  RequestPtr req)
{
    assert(gpuDynInst->isGlobalSeg());

    if (!req) {
        req = std::make_shared<Request>(
            0, 0, 0, masterId(), 0, gpuDynInst->wfDynId);
    }
    req->setPaddr(0);
    if (kernelLaunch) {
        req->setFlags(Request::KERNEL);
    }

    // for non-kernel MemFence operations, memorder flags are set depending
    // on which type of request is currently being sent, so this
    // should be set by the caller (e.g. if an inst has acq-rel
    // semantics, it will send one acquire req an one release req)
    gpuDynInst->setRequestFlags(req, kernelLaunch);

    // a mem fence must correspond to an acquire/release request
    assert(req->isAcquire() || req->isRelease());

    // create packet
    PacketPtr pkt = new Packet(req, MemCmd::MemFenceReq);

    // set packet's sender state
    pkt->senderState =
        new ComputeUnit::DataPort::SenderState(gpuDynInst, 0, nullptr);

    // send the packet
    sendSyncRequest(gpuDynInst, 0, pkt);
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
            pkt->req->getPaddr(), index);

    Addr paddr = pkt->req->getPaddr();

    if (pkt->cmd != MemCmd::MemFenceResp) {
        int index = gpuDynInst->memStatusVector[paddr].back();

        DPRINTF(GPUMem, "Response for addr %#x, index %d\n",
                pkt->req->getPaddr(), index);

        gpuDynInst->memStatusVector[paddr].pop_back();
        gpuDynInst->pAddr = pkt->req->getPaddr();

        if (pkt->isRead() || pkt->isWrite()) {

            if (gpuDynInst->n_reg <= MAX_REGS_FOR_NON_VEC_MEM_INST) {
                gpuDynInst->statusBitVector &= (~(1ULL << index));
            } else {
                assert(gpuDynInst->statusVector[index] > 0);
                gpuDynInst->statusVector[index]--;

                if (!gpuDynInst->statusVector[index])
                    gpuDynInst->statusBitVector &= (~(1ULL << index));
            }

            DPRINTF(GPUMem, "bitvector is now %#x\n",
                    gpuDynInst->statusBitVector);

            if (gpuDynInst->statusBitVector == VectorMask(0)) {
                auto iter = gpuDynInst->memStatusVector.begin();
                auto end = gpuDynInst->memStatusVector.end();

                while (iter != end) {
                    assert(iter->second.empty());
                    ++iter;
                }

                gpuDynInst->memStatusVector.clear();

                if (gpuDynInst->n_reg > MAX_REGS_FOR_NON_VEC_MEM_INST)
                    gpuDynInst->statusVector.clear();

                compute_unit->globalMemoryPipe.handleResponse(gpuDynInst);

                DPRINTF(GPUMem, "CU%d: WF[%d][%d]: packet totally complete\n",
                        compute_unit->cu_id, gpuDynInst->simdId,
                        gpuDynInst->wfSlotId);

                // after clearing the status vectors,
                // see if there is a continuation to perform
                // the continuation may generate more work for
                // this memory request
                if (gpuDynInst->useContinuation) {
                    assert(!gpuDynInst->isNoScope());
                    gpuDynInst->execContinuation(
                        gpuDynInst->staticInstruction(),
                        gpuDynInst);
                }
            }
        }
    } else {
        gpuDynInst->statusBitVector = VectorMask(0);

        if (gpuDynInst->useContinuation) {
            assert(!gpuDynInst->isNoScope());
            gpuDynInst->execContinuation(gpuDynInst->staticInstruction(),
                                         gpuDynInst);
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
    int mp_index = sender_state->portIndex;
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
                computeUnit->masterId(),
                0, 0, nullptr);

            PacketPtr prefetch_pkt = new Packet(prefetch_req, requestCmd);
            uint8_t foo = 0;
            prefetch_pkt->dataStatic(&foo);

            // Because it's atomic operation, only need TLB translation state
            prefetch_pkt->senderState =
                new TheISA::GpuTLB::TranslationState(TLB_mode,
                                                     computeUnit->shader->gpuTc,
                                                     true);

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
    // the request can be sent through the cu's master port
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
        computeUnit->memPort[mp_index]->createMemReqEvent(new_pkt);

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
                compute_unit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, index,
                pkt->req->getPaddr());
    } else {
        DPRINTF(GPUPort,
                "CU%d: WF[%d][%d]: index %d, addr %#x data req sent!\n",
                compute_unit->cu_id, gpuDynInst->simdId,
                gpuDynInst->wfSlotId, index,
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
ComputeUnit::ITLBPort::recvTimingResp(PacketPtr pkt)
{
    Addr line M5_VAR_USED = pkt->req->getPaddr();
    DPRINTF(GPUTLB, "CU%d: ITLBPort received %#x->%#x\n",
            computeUnit->cu_id, pkt->req->getVaddr(), line);

    assert(pkt->senderState);

    // pop off the TLB translation state
    TheISA::GpuTLB::TranslationState *translation_state =
                 safe_cast<TheISA::GpuTLB::TranslationState*>(pkt->senderState);

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
        // the cu's master port
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

    numALUInstsExecuted
        .name(name() + ".num_alu_insts_executed")
        .desc("Number of dynamic non-GM memory insts executed")
        ;

    wgBlockedDueLdsAllocation
        .name(name() + ".wg_blocked_due_lds_alloc")
        .desc("Workgroup blocked due to LDS capacity")
        ;

    ipc = numInstrExecuted / totalCycles;
    vpc = numVecOpsExecuted / totalCycles;

    numTimesWgBlockedDueVgprAlloc
        .name(name() + ".times_wg_blocked_due_vgpr_alloc")
        .desc("Number of times WGs are blocked due to VGPR allocation per SIMD")
        ;

    dynamicGMemInstrCnt
        .name(name() + ".global_mem_instr_cnt")
        .desc("dynamic global memory instructions count")
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

    numCASOps
        .name(name() + ".num_CAS_ops")
        .desc("number of compare and swap operations")
        ;

    numFailedCASOps
        .name(name() + ".num_failed_CAS_ops")
        .desc("number of compare and swap operations that failed")
        ;

    // register stats of pipeline stages
    fetchStage.regStats();
    scoreboardCheckStage.regStats();
    scheduleStage.regStats();
    execStage.regStats();

    // register stats of memory pipeline
    globalMemoryPipe.regStats();
    localMemoryPipe.regStats();
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
ComputeUnit::CUExitCallback::process()
{
    if (computeUnit->countPages) {
        std::ostream *page_stat_file =
            simout.create(computeUnit->name().c_str())->stream();

        *page_stat_file << "page, wavefront accesses, workitem accesses" <<
            std::endl;

        for (auto iter : computeUnit->pageAccesses) {
            *page_stat_file << std::hex << iter.first << ",";
            *page_stat_file << std::dec << iter.second.first << ",";
            *page_stat_file << std::dec << iter.second.second << std::endl;
        }
    }
 }

bool
ComputeUnit::isDone() const
{
    for (int i = 0; i < numSIMDs; ++i) {
        if (!isSimdDone(i)) {
            return false;
        }
    }

    bool glbMemBusRdy = true;
    for (int j = 0; j < numGlbMemUnits; ++j) {
        glbMemBusRdy &= vrfToGlobalMemPipeBus[j].rdy();
    }
    bool locMemBusRdy = true;
    for (int j = 0; j < numLocMemUnits; ++j) {
        locMemBusRdy &= vrfToLocalMemPipeBus[j].rdy();
    }

    if (!globalMemoryPipe.isGMLdRespFIFOWrRdy() ||
        !globalMemoryPipe.isGMStRespFIFOWrRdy() ||
        !globalMemoryPipe.isGMReqFIFOWrRdy() || !localMemoryPipe.isLMReqFIFOWrRdy()
        || !localMemoryPipe.isLMRespFIFOWrRdy() || !locMemToVrfBus.rdy() ||
        !glbMemToVrfBus.rdy() || !locMemBusRdy || !glbMemBusRdy) {
        return false;
    }

    return true;
}

int32_t
ComputeUnit::getRefCounter(const uint32_t dispatchId, const uint32_t wgId) const
{
    return lds.getRefCounter(dispatchId, wgId);
}

bool
ComputeUnit::isSimdDone(uint32_t simdId) const
{
    assert(simdId < numSIMDs);

    for (int i=0; i < numGlbMemUnits; ++i) {
        if (!vrfToGlobalMemPipeBus[i].rdy())
            return false;
    }
    for (int i=0; i < numLocMemUnits; ++i) {
        if (!vrfToLocalMemPipeBus[i].rdy())
            return false;
    }
    if (!aluPipe[simdId].rdy()) {
        return false;
    }

    for (int i_wf = 0; i_wf < shader->n_wf; ++i_wf){
        if (wfList[simdId][i_wf]->status != Wavefront::S_STOPPED) {
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

    return ldsPort->sendTimingReq(newPacket);
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
    } else if (!MasterPort::sendTimingReq(pkt)) {
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

        if (!MasterPort::sendTimingReq(packet)) {
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
