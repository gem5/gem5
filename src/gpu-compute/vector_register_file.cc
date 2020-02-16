/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
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

#include "gpu-compute/vector_register_file.hh"

#include <string>

#include "base/logging.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/simple_pool_manager.hh"
#include "gpu-compute/wavefront.hh"
#include "params/VectorRegisterFile.hh"

VectorRegisterFile::VectorRegisterFile(const VectorRegisterFileParams *p)
    : SimObject(p),
      manager(new SimplePoolManager(p->min_alloc, p->num_regs_per_simd)),
      simdId(p->simd_id), numRegsPerSimd(p->num_regs_per_simd),
      vgprState(new VecRegisterState())
{
    fatal_if(numRegsPerSimd % 2, "VRF size is illegal\n");
    fatal_if(simdId < 0, "Illegal SIMD id for VRF");

    fatal_if(numRegsPerSimd % p->min_alloc, "Min VGPR region allocation is not "
             "multiple of VRF size\n");

    busy.clear();
    busy.resize(numRegsPerSimd, 0);
    nxtBusy.clear();
    nxtBusy.resize(numRegsPerSimd, 0);

    vgprState->init(numRegsPerSimd, p->wfSize);
}

void
VectorRegisterFile::setParent(ComputeUnit *_computeUnit)
{
    computeUnit = _computeUnit;
    vgprState->setParent(computeUnit);
}

uint8_t
VectorRegisterFile::regNxtBusy(int idx, uint32_t operandSize) const
{
    uint8_t status = nxtBusy.at(idx);

    if (operandSize > 4) {
        status = status | (nxtBusy.at((idx + 1) % numRegs()));
    }

    return status;
}

uint8_t
VectorRegisterFile::regBusy(int idx, uint32_t operandSize) const
{
    uint8_t status = busy.at(idx);

    if (operandSize > 4) {
        status = status | (busy.at((idx + 1) % numRegs()));
    }

    return status;
}

void
VectorRegisterFile::preMarkReg(int regIdx, uint32_t operandSize, uint8_t value)
{
    nxtBusy.at(regIdx) = value;

    if (operandSize > 4) {
        nxtBusy.at((regIdx + 1) % numRegs()) = value;
    }
}

void
VectorRegisterFile::markReg(int regIdx, uint32_t operandSize, uint8_t value)
{
    busy.at(regIdx) = value;

    if (operandSize > 4) {
        busy.at((regIdx + 1) % numRegs()) = value;
    }
}

bool
VectorRegisterFile::operandsReady(Wavefront *w, GPUDynInstPtr ii) const
{
    for (int i = 0; i < ii->getNumOperands(); ++i) {
        if (ii->isVectorRegister(i)) {
            uint32_t vgprIdx = ii->getRegisterIndex(i, ii);
            uint32_t pVgpr = w->remap(vgprIdx, ii->getOperandSize(i), 1);

            if (regBusy(pVgpr, ii->getOperandSize(i)) == 1) {
                if (ii->isDstOperand(i)) {
                    w->numTimesBlockedDueWAXDependencies++;
                } else if (ii->isSrcOperand(i)) {
                    w->numTimesBlockedDueRAWDependencies++;
                }

                return false;
            }

            if (regNxtBusy(pVgpr, ii->getOperandSize(i)) == 1) {
                if (ii->isDstOperand(i)) {
                    w->numTimesBlockedDueWAXDependencies++;
                } else if (ii->isSrcOperand(i)) {
                    w->numTimesBlockedDueRAWDependencies++;
                }

                return false;
            }
        }
    }

    return true;
}

void
VectorRegisterFile::exec(GPUDynInstPtr ii, Wavefront *w)
{
    bool loadInstr = ii->isLoad();
    bool atomicInstr = ii->isAtomic() || ii->isMemFence();

    bool loadNoArgInstr = loadInstr && !ii->isArgLoad();

    // iterate over all register destination operands
    for (int i = 0; i < ii->getNumOperands(); ++i) {
        if (ii->isVectorRegister(i) && ii->isDstOperand(i)) {
            uint32_t physReg = w->remap(ii->getRegisterIndex(i, ii),
                                        ii->getOperandSize(i), 1);

            // mark the destination vector register as busy
            markReg(physReg, ii->getOperandSize(i), 1);
            // clear the in-flight status of the destination vector register
            preMarkReg(physReg, ii->getOperandSize(i), 0);

            // FIXME: if we ever model correct timing behavior
            // for load argument instructions then we should not
            // set the destination register as busy now but when
            // the data returns. Loads and Atomics should free
            // their destination registers when the data returns,
            // not now
            if (!atomicInstr && !loadNoArgInstr) {
                uint32_t pipeLen = ii->getOperandSize(i) <= 4 ?
                    computeUnit->spBypassLength() :
                    computeUnit->dpBypassLength();

                // schedule an event for marking the register as ready
                computeUnit->registerEvent(w->simdId, physReg,
                                           ii->getOperandSize(i),
                                           computeUnit->shader->tick_cnt +
                                           computeUnit->shader->ticks(pipeLen),
                                           0);
            }
        }
    }
}

int
VectorRegisterFile::exec(uint64_t dynamic_id, Wavefront *w,
                         std::vector<uint32_t> &regVec, uint32_t operandSize,
                         uint64_t timestamp)
{
    int delay = 0;

    panic_if(regVec.size() <= 0, "Illegal VGPR vector size=%d\n",
             regVec.size());

    for (int i = 0; i < regVec.size(); ++i) {
        // mark the destination VGPR as free when the timestamp expires
        computeUnit->registerEvent(w->simdId, regVec[i], operandSize,
                                   computeUnit->shader->tick_cnt + timestamp +
                                   computeUnit->shader->ticks(delay), 0);
    }

    return delay;
}

void
VectorRegisterFile::updateResources(Wavefront *w, GPUDynInstPtr ii)
{
    // iterate over all register destination operands
    for (int i = 0; i < ii->getNumOperands(); ++i) {
        if (ii->isVectorRegister(i) && ii->isDstOperand(i)) {
            uint32_t physReg = w->remap(ii->getRegisterIndex(i, ii),
                                        ii->getOperandSize(i), 1);
            // set the in-flight status of the destination vector register
            preMarkReg(physReg, ii->getOperandSize(i), 1);
        }
    }
}

bool
VectorRegisterFile::vrfOperandAccessReady(uint64_t dynamic_id, Wavefront *w,
                                          GPUDynInstPtr ii,
                                          VrfAccessType accessType)
{
    bool ready = true;

    return ready;
}

bool
VectorRegisterFile::vrfOperandAccessReady(Wavefront *w, GPUDynInstPtr ii,
                                          VrfAccessType accessType)
{
    bool ready = true;

    return ready;
}

VectorRegisterFile*
VectorRegisterFileParams::create()
{
    return new VectorRegisterFile(this);
}
