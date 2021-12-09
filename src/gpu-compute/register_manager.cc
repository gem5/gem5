/*
 * Copyright (c) 2016, 2017 Advanced Micro Devices, Inc.
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
 *
 * Author: Mark Wyse
 */

#include "gpu-compute/register_manager.hh"

#include "config/the_gpu_isa.hh"
#include "debug/GPURename.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/static_register_manager_policy.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"
#include "params/RegisterManager.hh"

namespace gem5
{

RegisterManager::RegisterManager(const RegisterManagerParams &p)
    : SimObject(p), srfPoolMgrs(p.srf_pool_managers),
      vrfPoolMgrs(p.vrf_pool_managers)
{
    if (p.policy == "static") {
        policy = new StaticRegisterManagerPolicy();
    } else {
        fatal("Unimplemented Register Manager Policy");
    }

}

RegisterManager::~RegisterManager()
{
    for (auto mgr : srfPoolMgrs) {
        delete mgr;
    }
    for (auto mgr : vrfPoolMgrs) {
        delete mgr;
    }
}

void
RegisterManager::exec()
{
    policy->exec();
}

void
RegisterManager::setParent(ComputeUnit *cu)
{
    computeUnit = cu;
    policy->setParent(computeUnit);
    for (int i = 0; i < srfPoolMgrs.size(); i++) {
        fatal_if(computeUnit->srf[i]->numRegs() %
                 srfPoolMgrs[i]->minAllocation(),
                 "Min SGPR allocation is not multiple of VRF size\n");
    }
    for (int i = 0; i < vrfPoolMgrs.size(); i++) {
        fatal_if(computeUnit->vrf[i]->numRegs() %
                 vrfPoolMgrs[i]->minAllocation(),
                 "Min VGPG allocation is not multiple of VRF size\n");
    }
}

// compute mapping for vector register
int
RegisterManager::mapVgpr(Wavefront* w, int vgprIndex)
{
    return policy->mapVgpr(w, vgprIndex);
}

// compute mapping for scalar register
int
RegisterManager::mapSgpr(Wavefront* w, int sgprIndex)
{
    return policy->mapSgpr(w, sgprIndex);
}

// check if we can allocate registers
bool
RegisterManager::canAllocateVgprs(int simdId, int nWfs, int demandPerWf)
{
    return policy->canAllocateVgprs(simdId, nWfs, demandPerWf);
}

bool
RegisterManager::canAllocateSgprs(int simdId, int nWfs, int demandPerWf)
{
    return policy->canAllocateSgprs(simdId, nWfs, demandPerWf);
}

// allocate registers
void
RegisterManager::allocateRegisters(Wavefront *w, int vectorDemand,
                                   int scalarDemand)
{
    policy->allocateRegisters(w, vectorDemand, scalarDemand);
}

void
RegisterManager::freeRegisters(Wavefront* w)
{
    policy->freeRegisters(w);
}

} // namespace gem5
