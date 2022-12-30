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

#include "gpu-compute/wavefront.hh"

#include "base/bitfield.hh"
#include "debug/GPUExec.hh"
#include "debug/GPUInitAbi.hh"
#include "debug/WavefrontStack.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/simple_pool_manager.hh"
#include "gpu-compute/vector_register_file.hh"

namespace gem5
{

Wavefront::Wavefront(const Params &p)
  : SimObject(p), wfSlotId(p.wf_slot_id), simdId(p.simdId),
    maxIbSize(p.max_ib_size), _gpuISA(*this),
    vmWaitCnt(-1), expWaitCnt(-1), lgkmWaitCnt(-1),
    vmemInstsIssued(0), expInstsIssued(0), lgkmInstsIssued(0),
    sleepCnt(0), barId(WFBarrier::InvalidID), stats(this)
{
    lastTrace = 0;
    execUnitId = -1;
    status = S_STOPPED;
    reservedVectorRegs = 0;
    reservedScalarRegs = 0;
    startVgprIndex = 0;
    startSgprIndex = 0;
    outstandingReqs = 0;
    outstandingReqsWrGm = 0;
    outstandingReqsWrLm = 0;
    outstandingReqsRdGm = 0;
    outstandingReqsRdLm = 0;
    rdLmReqsInPipe = 0;
    rdGmReqsInPipe = 0;
    wrLmReqsInPipe = 0;
    wrGmReqsInPipe = 0;
    scalarRdGmReqsInPipe = 0;
    scalarWrGmReqsInPipe = 0;
    scalarOutstandingReqsRdGm = 0;
    scalarOutstandingReqsWrGm = 0;
    lastNonIdleTick = 0;
    ldsChunk = nullptr;

    memTraceBusy = 0;
    oldVgprTcnt = 0xffffffffffffffffll;
    oldDgprTcnt = 0xffffffffffffffffll;
    oldVgpr.resize(p.wf_size);

    pendingFetch = false;
    dropFetch = false;
    maxVgprs = 0;
    maxSgprs = 0;

    lastAddr.resize(p.wf_size);
    workItemFlatId.resize(p.wf_size);
    oldDgpr.resize(p.wf_size);
    for (int i = 0; i < 3; ++i) {
        workItemId[i].resize(p.wf_size);
    }

    _execMask.set();
    rawDist.clear();
    lastInstExec = 0;
    vecReads.clear();
}

void
Wavefront::init()
{
    reservedVectorRegs = 0;
    reservedScalarRegs = 0;
    startVgprIndex = 0;
    startSgprIndex = 0;

    scalarAlu = computeUnit->mapWaveToScalarAlu(this);
    scalarAluGlobalIdx = computeUnit->mapWaveToScalarAluGlobalIdx(this);
    globalMem = computeUnit->mapWaveToGlobalMem(this);
    localMem = computeUnit->mapWaveToLocalMem(this);
    scalarMem = computeUnit->mapWaveToScalarMem(this);
}

void
Wavefront::initRegState(HSAQueueEntry *task, int wgSizeInWorkItems)
{
    int regInitIdx = 0;

    // iterate over all the init fields and check which
    // bits are enabled
    for (int en_bit = 0; en_bit < NumScalarInitFields; ++en_bit) {

        if (task->sgprBitEnabled(en_bit)) {
            int physSgprIdx = 0;
            uint32_t wiCount = 0;
            uint32_t firstWave = 0;
            int orderedAppendTerm = 0;
            int numWfsInWg = 0;
            uint32_t finalValue = 0;
            Addr host_disp_pkt_addr = task->hostDispPktAddr();
            Addr kernarg_addr = task->kernargAddr();
            Addr hidden_priv_base(0);

            switch (en_bit) {
              case PrivateSegBuf:
                    physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[0]);
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting PrivateSegBuffer: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[0]);

                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[1]);
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting PrivateSegBuffer: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[1]);

                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[2]);
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting PrivateSegBuffer: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[2]);

                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[3]);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting PrivateSegBuffer: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        task->amdQueue.scratch_resource_descriptor[3]);
                break;
              case DispatchPtr:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        bits(host_disp_pkt_addr, 31, 0));
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting DispatchPtr: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        bits(host_disp_pkt_addr, 31, 0));

                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        bits(host_disp_pkt_addr, 63, 32));
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting DispatchPtr: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        bits(host_disp_pkt_addr, 63, 32));

                ++regInitIdx;
                break;
              case QueuePtr:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        bits(task->hostAMDQueueAddr, 31, 0));
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting QueuePtr: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        bits(task->hostAMDQueueAddr, 31, 0));

                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        bits(task->hostAMDQueueAddr, 63, 32));
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting QueuePtr: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        bits(task->hostAMDQueueAddr, 63, 32));

                ++regInitIdx;
                break;
              case KernargSegPtr:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        bits(kernarg_addr, 31, 0));
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting KernargSegPtr: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        bits(kernarg_addr, 31, 0));

                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        bits(kernarg_addr, 63, 32));
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting KernargSegPtr: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        bits(kernarg_addr, 63, 32));

                ++regInitIdx;
                break;
              case DispatchId:
                physSgprIdx
                    = computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        task->dispatchId());
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting DispatchId: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        task->dispatchId());
                break;
              case FlatScratchInit:
                physSgprIdx
                    = computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                    (TheGpuISA::ScalarRegU32)(task->amdQueue
                        .scratch_backing_memory_location & 0xffffffff));
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting FlatScratch Addr: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        (TheGpuISA::ScalarRegU32)(task->amdQueue
                        .scratch_backing_memory_location & 0xffffffff));

                physSgprIdx =
                       computeUnit->registerManager->mapSgpr(this, regInitIdx);
                // This vallue should be sizeof(DWORD) aligned, that is
                // 4 byte aligned
                computeUnit->srf[simdId]->write(physSgprIdx,
                    task->amdQueue.scratch_workitem_byte_size);
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting FlatScratch size: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        task->amdQueue.scratch_workitem_byte_size);
                /**
                 * Since flat scratch init is needed for this kernel, this
                 * kernel is going to have flat memory instructions and we
                 * need to initialize the hidden private base for this queue.
                 * scratch_resource_descriptor[0] has this queue's scratch
                 * base address. scratch_backing_memory_location has the
                 * offset to this queue's scratch base address from the
                 * SH_HIDDEN_PRIVATE_BASE_VMID. Ideally, we only require this
                 * queue's scratch base address for address calculation
                 * (stored in scratch_resource_descriptor[0]). But that
                 * address calculation shoule be done by first finding the
                 * queue's scratch base address using the calculation
                 * "SH_HIDDEN_PRIVATE_BASE_VMID + offset". So, we initialize
                 * SH_HIDDEN_PRIVATE_BASE_VMID.
                 *
                 * For more details see:
                 *     http://rocm-documentation.readthedocs.io/en/latest/
                 *     ROCm_Compiler_SDK/ROCm-Native-ISA.html#flat-scratch
                 *
                 *     https://github.com/ROCm-Developer-Tools/
                 *     ROCm-ComputeABI-Doc/blob/master/AMDGPU-ABI.md
                 *     #flat-addressing
                 */
                hidden_priv_base =
                    (uint64_t)task->amdQueue.scratch_resource_descriptor[0] |
                    (((uint64_t)task->amdQueue.scratch_resource_descriptor[1]
                    & 0x000000000000ffff) << 32);
                computeUnit->shader->initShHiddenPrivateBase(
                       hidden_priv_base,
                       task->amdQueue.scratch_backing_memory_location);
                break;
              case PrivateSegSize:
                physSgprIdx
                    = computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                        task->privMemPerItem());
                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting private segment size: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        task->privMemPerItem());
                break;
              case GridWorkgroupCountX:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                wiCount = ((task->gridSize(0) +
                           task->wgSize(0) - 1) /
                           task->wgSize(0));
                computeUnit->srf[simdId]->write(physSgprIdx, wiCount);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting num WG X: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx, wiCount);
                break;
              case GridWorkgroupCountY:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                wiCount = ((task->gridSize(1) +
                           task->wgSize(1) - 1) /
                           task->wgSize(1));
                computeUnit->srf[simdId]->write(physSgprIdx, wiCount);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting num WG Y: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx, wiCount);
                break;
              case GridWorkgroupCountZ:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                wiCount = ((task->gridSize(2) +
                           task->wgSize(2) - 1) /
                           task->wgSize(2));
                computeUnit->srf[simdId]->write(physSgprIdx, wiCount);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting num WG Z: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx, wiCount);
                break;
              case WorkgroupIdX:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                                                     workGroupId[0]);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting WG ID X: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx, workGroupId[0]);
                break;
              case WorkgroupIdY:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                                                     workGroupId[1]);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting WG ID Y: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx, workGroupId[1]);
                break;
              case WorkgroupIdZ:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->write(physSgprIdx,
                                                     workGroupId[2]);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting WG ID Z: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx, workGroupId[2]);
                break;
              case PrivSegWaveByteOffset:
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                /**
                  * the compute_tmpring_size_wavesize specifies the number of
                  * kB allocated per wavefront, hence the multiplication by
                  * 1024.
                  *
                  * to get the per wavefront offset into the scratch
                  * memory, we also multiply this by the wfId. the wfId stored
                  * in the Wavefront class, however, is the wave ID within the
                  * WG, whereas here we need the global WFID because the
                  * scratch space will be divided amongst all waves in the
                  * kernel. to get the global ID we multiply the WGID by
                  * the WG size, then add the WFID of the wave within its WG.
                  */
                computeUnit->srf[simdId]->write(physSgprIdx, 1024 *
                    (wgId * (wgSz / 64) + wfId) *
                    task->amdQueue.compute_tmpring_size_wavesize);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting Private Seg Offset: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx,
                        1024 * (wgId * (wgSz / 64) + wfId) *
                        task->amdQueue.compute_tmpring_size_wavesize);
                break;
              case WorkgroupInfo:
                firstWave = (wfId == 0) ? 1 : 0;
                numWfsInWg = divCeil(wgSizeInWorkItems,
                                         computeUnit->wfSize());
                finalValue = firstWave << ((sizeof(uint32_t) * 8) - 1);
                finalValue |= (orderedAppendTerm << 6);
                finalValue |= numWfsInWg;
                physSgprIdx =
                    computeUnit->registerManager->mapSgpr(this, regInitIdx);
                computeUnit->srf[simdId]->
                    write(physSgprIdx, finalValue);

                ++regInitIdx;
                DPRINTF(GPUInitAbi, "CU%d: WF[%d][%d]: wave[%d] "
                        "Setting WG Info: s[%d] = %x\n",
                        computeUnit->cu_id, simdId,
                        wfSlotId, wfDynId, physSgprIdx, finalValue);
                break;
              default:
                fatal("SGPR enable bit %i not supported\n", en_bit);
                break;
            }
        }
    }

    regInitIdx = 0;

    // iterate over all the init fields and check which
    // bits are enabled
    for (int en_bit = 0; en_bit < NumVectorInitFields; ++en_bit) {
        if (task->vgprBitEnabled(en_bit)) {
            uint32_t physVgprIdx = 0;
            TheGpuISA::VecRegContainerU32 raw_vgpr;

            switch (en_bit) {
              case WorkitemIdX:
                {
                    physVgprIdx = computeUnit->registerManager
                        ->mapVgpr(this, regInitIdx);
                    TheGpuISA::VecElemU32 *vgpr_x
                        = raw_vgpr.as<TheGpuISA::VecElemU32>();

                    for (int lane = 0; lane < workItemId[0].size(); ++lane) {
                        vgpr_x[lane] = workItemId[0][lane];
                    }

                    computeUnit->vrf[simdId]->write(physVgprIdx, raw_vgpr);
                    rawDist[regInitIdx] = 0;
                    ++regInitIdx;
                }
                break;
              case WorkitemIdY:
                {
                    physVgprIdx = computeUnit->registerManager
                        ->mapVgpr(this, regInitIdx);
                    TheGpuISA::VecElemU32 *vgpr_y
                        = raw_vgpr.as<TheGpuISA::VecElemU32>();

                    for (int lane = 0; lane < workItemId[1].size(); ++lane) {
                        vgpr_y[lane] = workItemId[1][lane];
                    }

                    computeUnit->vrf[simdId]->write(physVgprIdx, raw_vgpr);
                    rawDist[regInitIdx] = 0;
                    ++regInitIdx;
                }
                break;
              case WorkitemIdZ:
                {
                    physVgprIdx = computeUnit->registerManager->
                        mapVgpr(this, regInitIdx);
                    TheGpuISA::VecElemU32 *vgpr_z
                        = raw_vgpr.as<TheGpuISA::VecElemU32>();

                    for (int lane = 0; lane < workItemId[2].size(); ++lane) {
                        vgpr_z[lane] = workItemId[2][lane];
                    }

                    computeUnit->vrf[simdId]->write(physVgprIdx, raw_vgpr);
                    rawDist[regInitIdx] = 0;
                    ++regInitIdx;
                }
                break;
            }
        }
    }
}

void
Wavefront::resizeRegFiles(int num_vregs, int num_sregs)
{
    maxVgprs = num_vregs;
    maxSgprs = num_sregs;
}

Wavefront::~Wavefront()
{
}

void
Wavefront::setStatus(status_e newStatus)
{
    if (computeUnit->idleCUTimeout > 0) {
        // Wavefront's status transitions to stalled or stopped
        if ((newStatus == S_STOPPED || newStatus == S_STALLED ||
             newStatus == S_WAITCNT || newStatus == S_BARRIER) &&
            (status != newStatus)) {
            computeUnit->idleWfs++;
            assert(computeUnit->idleWfs <=
                   (computeUnit->shader->n_wf * computeUnit->numVectorALUs));
            if (computeUnit->idleWfs ==
                (computeUnit->shader->n_wf * computeUnit->numVectorALUs)) {
                lastNonIdleTick = curTick();
            }
            // Wavefront's status transitions to an active state (from
            // a stopped or stalled state)
        } else if ((status == S_STOPPED || status == S_STALLED ||
                    status == S_WAITCNT || status == S_BARRIER) &&
                   (status != newStatus)) {
            // if all WFs in the CU were idle then check if the idleness
            // period exceeded the timeout threshold
            if (computeUnit->idleWfs ==
                (computeUnit->shader->n_wf * computeUnit->numVectorALUs)) {
                panic_if((curTick() - lastNonIdleTick) >=
                         computeUnit->idleCUTimeout,
                         "CU%d has been idle for %d ticks at tick %d",
                         computeUnit->cu_id, computeUnit->idleCUTimeout,
                         curTick());
            }
            computeUnit->idleWfs--;
            assert(computeUnit->idleWfs >= 0);
        }
    }
    status = newStatus;
}

void
Wavefront::start(uint64_t _wf_dyn_id, Addr init_pc)
{
    wfDynId = _wf_dyn_id;
    _pc = init_pc;

    status = S_RUNNING;

    vecReads.resize(maxVgprs, 0);
}

bool
Wavefront::isGmInstruction(GPUDynInstPtr ii)
{
    if (ii->isGlobalMem() ||
        (ii->isFlat() && ii->executedAs() == enums::SC_GLOBAL)) {
        return true;
    }

    return false;
}

bool
Wavefront::isLmInstruction(GPUDynInstPtr ii)
{
    if (ii->isLocalMem() ||
        (ii->isFlat() && ii->executedAs() == enums::SC_GROUP)) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstSleep()
{
    if (instructionBuffer.empty())
        return false;

    GPUDynInstPtr ii = instructionBuffer.front();

    if (ii->isSleep()) {
        return true;
    }
    return false;
}

bool
Wavefront::isOldestInstWaitcnt()
{
    if (instructionBuffer.empty())
        return false;

    GPUDynInstPtr ii = instructionBuffer.front();

    if (ii->isWaitcnt()) {
        // waitcnt is a scalar
        assert(ii->isScalar());
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstScalarALU()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isScalar() && (ii->isNop() || ii->isReturn()
        || ii->isEndOfKernel() || ii->isBranch() || ii->isALU() ||
        (ii->isKernArgSeg() && ii->isLoad()))) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstVectorALU()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && !ii->isScalar() && (ii->isNop() ||
        ii->isReturn() || ii->isBranch() || ii->isALU() || ii->isEndOfKernel()
        || (ii->isKernArgSeg() && ii->isLoad()))) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstBarrier()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isBarrier()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstGMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && !ii->isScalar() && ii->isGlobalMem()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstScalarMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isScalar() && ii->isGlobalMem()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstLMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isLocalMem()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstPrivMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isPrivateSeg()) {
        return true;
    }

    return false;
}

bool
Wavefront::isOldestInstFlatMem()
{
    assert(!instructionBuffer.empty());
    GPUDynInstPtr ii = instructionBuffer.front();

    if (status != S_STOPPED && ii->isFlat()) {
        return true;
    }

    return false;
}

bool
Wavefront::stopFetch()
{
    for (auto it : instructionBuffer) {
        GPUDynInstPtr ii = it;
        if (ii->isReturn() || ii->isBranch() ||
            ii->isEndOfKernel()) {
            return true;
        }
    }

    return false;
}

void
Wavefront::freeResources()
{
    execUnitId = -1;
}

void Wavefront::validateRequestCounters()
{
    panic_if(wrGmReqsInPipe < 0 || rdGmReqsInPipe < 0 ||
             wrLmReqsInPipe < 0 || rdLmReqsInPipe < 0 ||
             outstandingReqs < 0,
             "Negative requests in pipe for WF%d for slot%d"
             " and SIMD%d: Rd GlobalMem Reqs=%d, Wr GlobalMem Reqs=%d,"
             " Rd LocalMem Reqs=%d, Wr LocalMem Reqs=%d,"
             " Outstanding Reqs=%d\n",
             wfDynId, wfSlotId, simdId, rdGmReqsInPipe, wrGmReqsInPipe,
             rdLmReqsInPipe, wrLmReqsInPipe, outstandingReqs);
}

void
Wavefront::reserveGmResource(GPUDynInstPtr ii)
{
    if (!ii->isScalar()) {
        if (ii->isLoad()) {
            rdGmReqsInPipe++;
        } else if (ii->isStore()) {
            wrGmReqsInPipe++;
        } else if (ii->isAtomic() || ii->isMemSync()) {
            rdGmReqsInPipe++;
            wrGmReqsInPipe++;
        } else {
            panic("Invalid memory operation!\n");
        }
        execUnitId = globalMem;
    } else {
        if (ii->isLoad()) {
            scalarRdGmReqsInPipe++;
        } else if (ii->isStore()) {
            scalarWrGmReqsInPipe++;
        } else if (ii->isAtomic() || ii->isMemSync()) {
            scalarWrGmReqsInPipe++;
            scalarRdGmReqsInPipe++;
        } else {
            panic("Invalid memory operation!\n");
        }
        execUnitId = scalarMem;
    }
}

void
Wavefront::reserveLmResource(GPUDynInstPtr ii)
{
    fatal_if(ii->isScalar(),
             "Scalar instructions can not access Shared memory!!!");
    if (ii->isLoad()) {
        rdLmReqsInPipe++;
    } else if (ii->isStore()) {
        wrLmReqsInPipe++;
    } else if (ii->isAtomic() || ii->isMemSync()) {
        wrLmReqsInPipe++;
        rdLmReqsInPipe++;
    } else {
        panic("Invalid memory operation!\n");
    }
    execUnitId = localMem;
}

std::vector<int>
Wavefront::reserveResources()
{
    // vector of execution unit IDs to return to schedule stage
    // this return is only used for debugging and an assertion...
    std::vector<int> execUnitIds;

    // Get current instruction
    GPUDynInstPtr ii = instructionBuffer.front();
    assert(ii);

    // Single precision ALU or Branch or Return or Special instruction
    if (ii->isALU() || ii->isSpecialOp() ||
        ii->isBranch() || ii->isNop() ||
        (ii->isKernArgSeg() && ii->isLoad()) || ii->isArgSeg() ||
        ii->isReturn() || ii->isEndOfKernel()) {
        if (!ii->isScalar()) {
            execUnitId = simdId;
        } else {
            execUnitId = scalarAluGlobalIdx;
        }
        // this is to enforce a fixed number of cycles per issue slot per SIMD
    } else if (ii->isBarrier()) {
        execUnitId = ii->isScalar() ? scalarAluGlobalIdx : simdId;
    } else if (ii->isFlat()) {
        assert(!ii->isScalar());
        reserveLmResource(ii);
        // add execUnitId, reserved by reserveLmResource, list before it is
        // overwriten by reserveGmResource
        execUnitIds.push_back(execUnitId);
        flatLmUnitId = execUnitId;
        reserveGmResource(ii);
        flatGmUnitId = execUnitId;
        execUnitIds.push_back(flatGmUnitId);
        execUnitId = -1;
    } else if (ii->isGlobalMem()) {
        reserveGmResource(ii);
    } else if (ii->isLocalMem()) {
        reserveLmResource(ii);
    } else if (ii->isPrivateSeg()) {
        fatal_if(ii->isScalar(),
                 "Scalar instructions can not access Private memory!!!");
        reserveGmResource(ii);
    } else {
        panic("reserveResources -> Couldn't process op!\n");
    }

    if (execUnitId != -1) {
        execUnitIds.push_back(execUnitId);
    }
    assert(execUnitIds.size());
    return execUnitIds;
}

void
Wavefront::exec()
{
    // ---- Exit if wavefront is inactive ----------------------------- //

    if (status == S_STOPPED || status == S_RETURNING ||
        status==S_STALLED || instructionBuffer.empty()) {
        return;
    }

    if (status == S_WAITCNT) {
        /**
         * if this wave is in S_WAITCNT state, then
         * it should enter exec() precisely one time
         * before the waitcnts are satisfied, in order
         * to execute the waitcnt instruction itself
         * thus we assert that the waitcnt is the
         * oldest instruction. if we enter exec() with
         * active waitcnts, and we're not executing
         * the waitcnt instruction, something must be
         * wrong
         */
        assert(isOldestInstWaitcnt());
    }

    // Get current instruction

    GPUDynInstPtr ii = instructionBuffer.front();

    const Addr old_pc = pc();
    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: wave[%d] Executing inst: %s "
            "(pc: %#x; seqNum: %d)\n", computeUnit->cu_id, simdId, wfSlotId,
            wfDynId, ii->disassemble(), old_pc, ii->seqNum());

    ii->execute(ii);
    // delete the dynamic instruction from the pipeline map
    computeUnit->deleteFromPipeMap(this);
    // update the instruction stats in the CU
    computeUnit->updateInstStats(ii);

    // inform VRF of instruction execution to schedule write-back
    // and scoreboard ready for registers
    if (!ii->isScalar()) {
        computeUnit->vrf[simdId]->waveExecuteInst(this, ii);
    }
    computeUnit->srf[simdId]->waveExecuteInst(this, ii);

    computeUnit->shader->incVectorInstSrcOperand(ii->numSrcVecRegOperands());
    computeUnit->shader->incVectorInstDstOperand(ii->numDstVecRegOperands());
    computeUnit->stats.numInstrExecuted++;
    stats.numInstrExecuted++;
    computeUnit->instExecPerSimd[simdId]++;
    computeUnit->stats.execRateDist.sample(
                                    computeUnit->stats.totalCycles.value() -
                                    computeUnit->lastExecCycle[simdId]);
    computeUnit->lastExecCycle[simdId] =
        computeUnit->stats.totalCycles.value();

    if (lastInstExec) {
        computeUnit->stats.instInterleave[simdId].
            sample(computeUnit->instExecPerSimd[simdId] - lastInstExec);
    }
    lastInstExec = computeUnit->instExecPerSimd[simdId];

    // want to track:
    // number of reads that occur per value written

    // vector RAW dependency tracking
    for (const auto& srcVecOp : ii->srcVecRegOperands()) {
        for (const auto& virtIdx : srcVecOp.virtIndices()) {
            // This check should never fail, but to be safe we check
            if (rawDist.find(virtIdx) != rawDist.end()) {
                stats.vecRawDistance.sample(stats.numInstrExecuted.value() -
                                      rawDist[virtIdx]);
            }
            // increment number of reads to this register
            vecReads[virtIdx]++;
        }
    }

    for (const auto& dstVecOp : ii->dstVecRegOperands()) {
        for (const auto& virtIdx : dstVecOp.virtIndices()) {
            // rawDist is set on writes, but will not be set for the first
            // write to each physical register
            if (rawDist.find(virtIdx) != rawDist.end()) {
                // Sample the number of reads that were performed
                stats.readsPerWrite.sample(vecReads[virtIdx]);
            }
            // on a write, reset count of reads to 0
            vecReads[virtIdx] = 0;

            rawDist[virtIdx] = stats.numInstrExecuted.value();
        }
    }

    if (pc() == old_pc) {
        // PC not modified by instruction, proceed to next
        _gpuISA.advancePC(ii);
        instructionBuffer.pop_front();
    } else {
        DPRINTF(GPUExec, "CU%d: WF[%d][%d]: wave%d %s taken branch\n",
                computeUnit->cu_id, simdId, wfSlotId, wfDynId,
                ii->disassemble());
        discardFetch();
    }
    DPRINTF(GPUExec, "CU%d: WF[%d][%d]: wave[%d] (pc: %#x)\n",
            computeUnit->cu_id, simdId, wfSlotId, wfDynId, pc());

    if (computeUnit->shader->hsail_mode==Shader::SIMT) {
        const int num_active_lanes = execMask().count();
        computeUnit->stats.controlFlowDivergenceDist.sample(num_active_lanes);
        computeUnit->stats.numVecOpsExecuted += num_active_lanes;

        if (ii->isF16() && ii->isALU()) {
            if (ii->isF32() || ii->isF64()) {
                fatal("Instruction is tagged as both (1) F16, and (2)"
                       "either F32 or F64.");
            }
            computeUnit->stats.numVecOpsExecutedF16 += num_active_lanes;
            if (ii->isFMA()) {
                computeUnit->stats.numVecOpsExecutedFMA16 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
            else if (ii->isMAC()) {
                computeUnit->stats.numVecOpsExecutedMAC16 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
            else if (ii->isMAD()) {
                computeUnit->stats.numVecOpsExecutedMAD16 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
        }
        if (ii->isF32() && ii->isALU()) {
            if (ii->isF16() || ii->isF64()) {
                fatal("Instruction is tagged as both (1) F32, and (2)"
                       "either F16 or F64.");
            }
            computeUnit->stats.numVecOpsExecutedF32 += num_active_lanes;
            if (ii->isFMA()) {
                computeUnit->stats.numVecOpsExecutedFMA32 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
            else if (ii->isMAC()) {
                computeUnit->stats.numVecOpsExecutedMAC32 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
            else if (ii->isMAD()) {
                computeUnit->stats.numVecOpsExecutedMAD32 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
        }
        if (ii->isF64() && ii->isALU()) {
            if (ii->isF16() || ii->isF32()) {
                fatal("Instruction is tagged as both (1) F64, and (2)"
                       "either F16 or F32.");
            }
            computeUnit->stats.numVecOpsExecutedF64 += num_active_lanes;
            if (ii->isFMA()) {
                computeUnit->stats.numVecOpsExecutedFMA64 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
            else if (ii->isMAC()) {
                computeUnit->stats.numVecOpsExecutedMAC64 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
            else if (ii->isMAD()) {
                computeUnit->stats.numVecOpsExecutedMAD64 += num_active_lanes;
                computeUnit->stats.numVecOpsExecutedTwoOpFP
                    += num_active_lanes;
            }
        }
        if (isGmInstruction(ii)) {
            computeUnit->stats.activeLanesPerGMemInstrDist.sample(
                                                            num_active_lanes);
        } else if (isLmInstruction(ii)) {
            computeUnit->stats.activeLanesPerLMemInstrDist.sample(
                                                            num_active_lanes);
        }
    }

    /**
     * we return here to avoid spurious errors related to flat insts
     * and their address segment resolution.
     */
    if (execMask().none() && ii->isFlat()) {
        computeUnit->getTokenManager()->recvTokens(1);
        return;
    }

    // Update Vector ALU pipeline and other resources
    bool flat_as_gm = false;
    bool flat_as_lm = false;
    if (ii->isFlat()) {
        flat_as_gm = (ii->executedAs() == enums::SC_GLOBAL) ||
                     (ii->executedAs() == enums::SC_PRIVATE);
        flat_as_lm = (ii->executedAs() == enums::SC_GROUP);
    }

    // Single precision ALU or Branch or Return or Special instruction
    // Note, we use the same timing regardless of SP or DP ALU operation.
    if (ii->isALU() || ii->isSpecialOp() ||
        ii->isBranch() || ii->isNop() ||
        (ii->isKernArgSeg() && ii->isLoad()) ||
        ii->isArgSeg() || ii->isEndOfKernel() || ii->isReturn()) {
        // this is to enforce a fixed number of cycles per issue slot per SIMD
        if (!ii->isScalar()) {
            computeUnit->vectorALUs[simdId].set(computeUnit->
                cyclesToTicks(computeUnit->issuePeriod));
        } else {
            computeUnit->scalarALUs[scalarAlu].set(computeUnit->
                cyclesToTicks(computeUnit->issuePeriod));
        }
    // Barrier on Scalar ALU
    } else if (ii->isBarrier()) {
        computeUnit->scalarALUs[scalarAlu].set(computeUnit->
            cyclesToTicks(computeUnit->issuePeriod));
    // GM or Flat as GM Load
    } else if (ii->isLoad() && (ii->isGlobalMem() || flat_as_gm)) {
        if (!ii->isScalar()) {
            computeUnit->vrfToGlobalMemPipeBus.set(
                computeUnit->cyclesToTicks(computeUnit->vrf_gm_bus_latency));
            computeUnit->vectorGlobalMemUnit.
                set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
            computeUnit->stats.instCyclesVMemPerSimd[simdId] +=
                computeUnit->vrf_gm_bus_latency;
        } else {
            computeUnit->srfToScalarMemPipeBus.set(computeUnit->
                cyclesToTicks(computeUnit->srf_scm_bus_latency));
            computeUnit->scalarMemUnit.
                set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
            computeUnit->stats.instCyclesScMemPerSimd[simdId] +=
                computeUnit->srf_scm_bus_latency;
        }
    // GM or Flat as GM Store
    } else if (ii->isStore() && (ii->isGlobalMem() || flat_as_gm)) {
        if (!ii->isScalar()) {
            computeUnit->vrfToGlobalMemPipeBus.set(computeUnit->
                cyclesToTicks(Cycles(2 * computeUnit->vrf_gm_bus_latency)));
            computeUnit->vectorGlobalMemUnit.
                set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
            computeUnit->stats.instCyclesVMemPerSimd[simdId] +=
                (2 * computeUnit->vrf_gm_bus_latency);
        } else {
            computeUnit->srfToScalarMemPipeBus.set(computeUnit->
                cyclesToTicks(Cycles(2 * computeUnit->srf_scm_bus_latency)));
            computeUnit->scalarMemUnit.
                set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
            computeUnit->stats.instCyclesScMemPerSimd[simdId] +=
                (2 * computeUnit->srf_scm_bus_latency);
        }
    } else if ((ii->isAtomic() || ii->isMemSync()) &&
               (ii->isGlobalMem() || flat_as_gm)) {
        if (!ii->isScalar()) {
            computeUnit->vrfToGlobalMemPipeBus.set(computeUnit->
                cyclesToTicks(Cycles(2 * computeUnit->vrf_gm_bus_latency)));
            computeUnit->vectorGlobalMemUnit.
                set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
            computeUnit->stats.instCyclesVMemPerSimd[simdId] +=
                (2 * computeUnit->vrf_gm_bus_latency);
        } else {
            computeUnit->srfToScalarMemPipeBus.set(computeUnit->
                cyclesToTicks(Cycles(2 * computeUnit->srf_scm_bus_latency)));
            computeUnit->scalarMemUnit.
                set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
            computeUnit->stats.instCyclesScMemPerSimd[simdId] +=
                (2 * computeUnit->srf_scm_bus_latency);
        }
    // LM or Flat as LM Load
    } else if (ii->isLoad() && (ii->isLocalMem() || flat_as_lm)) {
        computeUnit->vrfToLocalMemPipeBus.set(computeUnit->
            cyclesToTicks(computeUnit->vrf_lm_bus_latency));
        computeUnit->vectorSharedMemUnit.
            set(computeUnit->shader->cyclesToTicks(computeUnit->issuePeriod));
        computeUnit->stats.instCyclesLdsPerSimd[simdId] +=
            computeUnit->vrf_lm_bus_latency;
    // LM or Flat as LM Store
    } else if (ii->isStore() && (ii->isLocalMem() || flat_as_lm)) {
        computeUnit->vrfToLocalMemPipeBus.set(computeUnit->
            cyclesToTicks(Cycles(2 * computeUnit->vrf_lm_bus_latency)));
        computeUnit->vectorSharedMemUnit.
            set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
        computeUnit->stats.instCyclesLdsPerSimd[simdId] +=
            (2 * computeUnit->vrf_lm_bus_latency);
    // LM or Flat as LM, Atomic or MemFence
    } else if ((ii->isAtomic() || ii->isMemSync()) &&
               (ii->isLocalMem() || flat_as_lm)) {
        computeUnit->vrfToLocalMemPipeBus.set(computeUnit->
            cyclesToTicks(Cycles(2 * computeUnit->vrf_lm_bus_latency)));
        computeUnit->vectorSharedMemUnit.
            set(computeUnit->cyclesToTicks(computeUnit->issuePeriod));
        computeUnit->stats.instCyclesLdsPerSimd[simdId] +=
            (2 * computeUnit->vrf_lm_bus_latency);
    } else {
        panic("Bad instruction type!\n");
    }
}

GPUDynInstPtr
Wavefront::nextInstr()
{
    // Read next instruction from instruction buffer
    GPUDynInstPtr ii = instructionBuffer.front();
    // if the WF has been dispatched in the schedule stage then
    // check the next oldest instruction for readiness
    if (computeUnit->pipeMap.find(ii->seqNum()) !=
        computeUnit->pipeMap.end()) {
        if (instructionBuffer.size() > 1) {
            auto it = instructionBuffer.begin() + 1;
            return *it;
        } else { // No new instructions to check
            return nullptr;
        }
    }
    return ii;
}

void
Wavefront::discardFetch()
{
    instructionBuffer.clear();
    dropFetch |= pendingFetch;

    /**
     * clear the fetch buffer for this wave in order to
     * remove any stale inst data
     */
    computeUnit->fetchStage.fetchUnit(simdId).flushBuf(wfSlotId);
}

bool
Wavefront::waitCntsSatisfied()
{
    // Both vmWaitCnt && lgkmWaitCnt uninitialized means
    // waitCnt instruction has been dispatched but not executed yet: next
    // instruction should be blocked until waitCnt is executed.
    if (vmWaitCnt == -1 && expWaitCnt == -1 && lgkmWaitCnt == -1) {
        return false;
    }

    /**
     * If we reach here, that means an s_waitcnt instruction was executed
     * and the waitcnts are set by the execute method. Check if waitcnts
     * are satisfied.
     */
    if (vmWaitCnt != -1) {
        if (vmemInstsIssued > vmWaitCnt) {
            // vmWaitCnt not satisfied
            return false;
        }
    }

    if (expWaitCnt != -1) {
        if (expInstsIssued > expWaitCnt) {
            // expWaitCnt not satisfied
            return false;
        }
    }

    if (lgkmWaitCnt != -1) {
        if (lgkmInstsIssued > lgkmWaitCnt) {
            // lgkmWaitCnt not satisfied
            return false;
        }
    }

    // if we get here all outstanding waitcnts must
    // be satisfied, so we resume normal operation
    clearWaitCnts();

    return true;
}

bool
Wavefront::sleepDone()
{
    assert(status == S_STALLED_SLEEP);

    // if the sleep count has not been set, then the sleep instruction has not
    // been executed yet, so we will return true without setting the wavefront
    // status
    if (sleepCnt == 0)
        return false;

    sleepCnt--;
    if (sleepCnt != 0)
        return false;

    status = S_RUNNING;
    return true;
}

void
Wavefront::setSleepTime(int sleep_time)
{
    assert(sleepCnt == 0);
    sleepCnt = sleep_time;
}

void
Wavefront::setWaitCnts(int vm_wait_cnt, int exp_wait_cnt, int lgkm_wait_cnt)
{
    // the scoreboard should have set the status
    // to S_WAITCNT once a waitcnt instruction
    // was marked as ready
    assert(status == S_WAITCNT);

    // waitcnt instruction shouldn't be sending
    // negative counts
    assert(vm_wait_cnt >= 0);
    assert(exp_wait_cnt >= 0);
    assert(lgkm_wait_cnt >= 0);
    // waitcnts are a max of 15 because we have
    // only 1 nibble (4 bits) to set the counts
    assert(vm_wait_cnt <= 0xf);
    assert(exp_wait_cnt <= 0x7);
    assert(lgkm_wait_cnt <= 0x1f);

    /**
     * prior waitcnts should be satisfied,
     * at which time the WF resets them
     * back to -1, indicating they are no
     * longer active
     */
    assert(vmWaitCnt == -1);
    assert(expWaitCnt == -1);
    assert(lgkmWaitCnt == -1);

    /**
     * if the instruction encoding
     * indicates a waitcnt of 0xf,
     * that means the waitcnt is
     * not being used
     */
    if (vm_wait_cnt != 0xf)
        vmWaitCnt = vm_wait_cnt;

    if (exp_wait_cnt != 0x7)
        expWaitCnt = exp_wait_cnt;

    if (lgkm_wait_cnt != 0x1f)
        lgkmWaitCnt = lgkm_wait_cnt;
}

void
Wavefront::clearWaitCnts()
{
    // reset the waitcnts back to
    // -1, indicating they are no
    // longer valid
    vmWaitCnt = -1;
    expWaitCnt = -1;
    lgkmWaitCnt = -1;

    // resume running normally
    status = S_RUNNING;
}

void
Wavefront::incVMemInstsIssued()
{
    ++vmemInstsIssued;
}

void
Wavefront::incExpInstsIssued()
{
    ++expInstsIssued;
}

void
Wavefront::incLGKMInstsIssued()
{
    ++lgkmInstsIssued;
}

void
Wavefront::decVMemInstsIssued()
{
    --vmemInstsIssued;
}

void
Wavefront::decExpInstsIssued()
{
    --expInstsIssued;
}

void
Wavefront::decLGKMInstsIssued()
{
    --lgkmInstsIssued;
}

Addr
Wavefront::pc() const
{
    return _pc;
}

void
Wavefront::pc(Addr new_pc)
{
    _pc = new_pc;
}

VectorMask&
Wavefront::execMask()
{
    return _execMask;
}

bool
Wavefront::execMask(int lane) const
{
    return _execMask[lane];
}

void
Wavefront::freeRegisterFile()
{
    /* clear busy registers */
    for (int i=0; i < maxVgprs; i++) {
        int vgprIdx = computeUnit->registerManager->mapVgpr(this, i);
        computeUnit->vrf[simdId]->markReg(vgprIdx, false);
    }

    /* Free registers used by this wavefront */
    uint32_t endIndex = (startVgprIndex + reservedVectorRegs - 1) %
                         computeUnit->vrf[simdId]->numRegs();
    computeUnit->registerManager->vrfPoolMgrs[simdId]->
        freeRegion(startVgprIndex, endIndex);
}

void
Wavefront::computeActualWgSz(HSAQueueEntry *task)
{
    actualWgSzTotal = 1;
    for (int d = 0; d < HSAQueueEntry::MAX_DIM; ++d) {
        actualWgSz[d] = std::min(workGroupSz[d], gridSz[d]
                                 - task->wgId(d) * workGroupSz[d]);
        actualWgSzTotal *= actualWgSz[d];
    }
}

void
Wavefront::barrierId(int bar_id)
{
    assert(bar_id >= WFBarrier::InvalidID);
    assert(bar_id < computeUnit->numBarrierSlots());
    barId = bar_id;
}

int
Wavefront::barrierId() const
{
    return barId;
}

bool
Wavefront::hasBarrier() const
{
    return barId > WFBarrier::InvalidID;
}

void
Wavefront::releaseBarrier()
{
    barId = WFBarrier::InvalidID;
}

Wavefront::WavefrontStats::WavefrontStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numInstrExecuted,
               "number of instructions executed by this WF slot"),
      ADD_STAT(schCycles, "number of cycles spent in schedule stage"),
      ADD_STAT(schStalls, "number of cycles WF is stalled in SCH stage"),
      ADD_STAT(schRfAccessStalls, "number of cycles wave selected in SCH but "
               "RF denied adding instruction"),
      ADD_STAT(schResourceStalls, "number of cycles stalled in sch by resource"
               " not available"),
      ADD_STAT(schOpdNrdyStalls, "number of cycles stalled in sch waiting for "
               "RF reads to complete"),
      ADD_STAT(schLdsArbStalls,
               "number of cycles wave stalled due to LDS-VRF arbitration"),
      // FIXME: the name of the WF needs to be unique
      ADD_STAT(numTimesBlockedDueWAXDependencies, "number of times the wf's "
               "instructions are blocked due to WAW or WAR dependencies"),
      // FIXME: the name of the WF needs to be unique
      ADD_STAT(numTimesBlockedDueRAWDependencies, "number of times the wf's "
               "instructions are blocked due to RAW dependencies"),
      ADD_STAT(vecRawDistance,
               "Count of RAW distance in dynamic instructions for this WF"),
      ADD_STAT(readsPerWrite, "Count of Vector reads per write for this WF")
{
    vecRawDistance.init(0, 20, 1);
    readsPerWrite.init(0, 4, 1);
}

} // namespace gem5
