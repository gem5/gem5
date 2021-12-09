/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
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

#include "gpu-compute/static_register_manager_policy.hh"

#include "config/the_gpu_isa.hh"
#include "debug/GPURename.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/pool_manager.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

StaticRegisterManagerPolicy::StaticRegisterManagerPolicy()
{
}

void
StaticRegisterManagerPolicy::exec()
{
}

int
StaticRegisterManagerPolicy::mapVgpr(Wavefront* w, int vgprIndex)
{
    panic_if((vgprIndex >= w->reservedVectorRegs)
             || (w->reservedVectorRegs < 0),
             "VGPR index %d is out of range: VGPR range=[0,%d]",
             vgprIndex, w->reservedVectorRegs);

    // add the offset from where the VGPRs of the wavefront have been assigned
    int physicalVgprIndex = w->startVgprIndex + vgprIndex;

    panic_if(!((w->startVgprIndex <= physicalVgprIndex) &&
             (w->startVgprIndex + w->reservedVectorRegs - 1)
             >= physicalVgprIndex),
             "Invalid VGPR index %d\n", physicalVgprIndex);

    // calculate physical VGPR index
    return physicalVgprIndex % w->computeUnit->vrf[w->simdId]->numRegs();
}

int
StaticRegisterManagerPolicy::mapSgpr(Wavefront* w, int sgprIndex)
{
    panic_if(!((sgprIndex < w->reservedScalarRegs)
             && (w->reservedScalarRegs > 0)),
             "SGPR index %d is out of range: SGPR range=[0,%d]\n",
             sgprIndex, w->reservedScalarRegs);

    // add the offset from where the SGPRs of the wavefront have been assigned
    int physicalSgprIndex = w->startSgprIndex + sgprIndex;

    panic_if(!((w->startSgprIndex <= physicalSgprIndex) &&
             (w->startSgprIndex + w->reservedScalarRegs - 1)
             >= physicalSgprIndex),
             "Invalid SGPR index %d\n", physicalSgprIndex);

    // calculate physical SGPR index
    return physicalSgprIndex % w->computeUnit->srf[w->simdId]->numRegs();
}

bool
StaticRegisterManagerPolicy::canAllocateVgprs(int simdId, int nWfs,
                                              int demandPerWf)
{
    return cu->registerManager->vrfPoolMgrs[simdId]->
        canAllocate(nWfs, demandPerWf);
}

bool
StaticRegisterManagerPolicy::canAllocateSgprs(int simdId, int nWfs,
                                              int demandPerWf)
{
    return cu->registerManager->srfPoolMgrs[simdId]->
        canAllocate(nWfs, demandPerWf);
}

void
StaticRegisterManagerPolicy::allocateRegisters(Wavefront *w, int vectorDemand,
                                               int scalarDemand)
{
    uint32_t allocatedSize = 0;
    w->startVgprIndex = cu->registerManager->vrfPoolMgrs[w->simdId]->
        allocateRegion(vectorDemand, &allocatedSize);
    w->reservedVectorRegs = allocatedSize;
    cu->vectorRegsReserved[w->simdId] += w->reservedVectorRegs;
    panic_if(cu->vectorRegsReserved[w->simdId] > cu->numVecRegsPerSimd,
             "VRF[%d] has been overallocated %d > %d\n",
             w->simdId, cu->vectorRegsReserved[w->simdId],
             cu->numVecRegsPerSimd);

    if (scalarDemand) {
        w->startSgprIndex = cu->registerManager->srfPoolMgrs[w->simdId]->
            allocateRegion(scalarDemand, &allocatedSize);
        w->reservedScalarRegs = allocatedSize;
        cu->scalarRegsReserved[w->simdId] += w->reservedScalarRegs;
        panic_if(cu->scalarRegsReserved[w->simdId] > cu->numScalarRegsPerSimd,
                 "SRF[%d] has been overallocated %d > %d\n",
                 w->simdId, cu->scalarRegsReserved[w->simdId],
                 cu->numScalarRegsPerSimd);
    }
}

void
StaticRegisterManagerPolicy::freeRegisters(Wavefront *w)
{
    // free the vector registers of the completed wavefront
    w->computeUnit->vectorRegsReserved[w->simdId] -= w->reservedVectorRegs;
    // free the scalar registers of the completed wavefront
    w->computeUnit->scalarRegsReserved[w->simdId] -= w->reservedScalarRegs;

    panic_if(w->computeUnit->vectorRegsReserved[w->simdId] < 0,
             "Freeing VRF[%d] registers left %d registers reserved\n",
             w->simdId,
             w->computeUnit->vectorRegsReserved[w->simdId]);
    panic_if(w->computeUnit->scalarRegsReserved[w->simdId] < 0,
             "Freeing SRF[%d] registers left %d registers reserved\n",
             w->simdId,
             w->computeUnit->scalarRegsReserved[w->simdId]);

    // Current dynamic register allocation does not handle wraparound
    int endIndex = w->startVgprIndex + w->reservedVectorRegs;

    w->computeUnit->registerManager->vrfPoolMgrs[w->simdId]->
        freeRegion(w->startVgprIndex, endIndex);

    // mark/pre-mark all registers are not busy
    for (int i = 0; i < w->reservedVectorRegs; i++) {
        uint32_t physVgprIdx = mapVgpr(w, i);
        w->computeUnit->vrf[w->simdId]->markReg(physVgprIdx, false);
    }

    w->reservedVectorRegs = 0;
    w->startVgprIndex = 0;

    endIndex = w->startSgprIndex + w->reservedScalarRegs;
    w->computeUnit->registerManager->srfPoolMgrs[w->simdId]->
        freeRegion(w->startSgprIndex, endIndex);

    // mark/pre-mark all registers are not busy
    for (int i = 0; i < w->reservedScalarRegs; i++) {
        uint32_t physSgprIdx = mapSgpr(w, i);
        w->computeUnit->srf[w->simdId]->markReg(physSgprIdx, false);
    }

    w->reservedScalarRegs = 0;
    w->startSgprIndex = 0;
}

} // namespace gem5
