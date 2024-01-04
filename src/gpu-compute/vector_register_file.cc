/*
 * Copyright (c) 2015-2017 Advanced Micro Devices, Inc.
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

#include "gpu-compute/vector_register_file.hh"

#include <string>

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/GPUVRF.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/register_file_cache.hh"
#include "gpu-compute/simple_pool_manager.hh"
#include "gpu-compute/wavefront.hh"
#include "params/VectorRegisterFile.hh"

namespace gem5
{

VectorRegisterFile::VectorRegisterFile(const VectorRegisterFileParams &p)
    : RegisterFile(p)
{
    regFile.resize(numRegs());

    for (auto &reg : regFile) {
        reg.zero();
    }
}

bool
VectorRegisterFile::operandsReady(Wavefront *w, GPUDynInstPtr ii) const
{
    bool src_ready = true, dst_ready=true;
    for (const auto& srcVecOp : ii->srcVecRegOperands()) {
        for (const auto& physIdx : srcVecOp.physIndices()) {
            if (regBusy(physIdx) &&
                    !computeUnit->rfc[simdId]->inRFC(physIdx)) {
                DPRINTF(GPUVRF, "RAW stall: WV[%d]: %s: physReg[%d]\n",
                        w->wfDynId, ii->disassemble(), physIdx);
                w->stats.numTimesBlockedDueRAWDependencies++;
                src_ready = false;
                break;
            }
        }
        if (!src_ready) {
            break;
        }
    }

    for (const auto& dstVecOp : ii->dstVecRegOperands()) {
        for (const auto& physIdx : dstVecOp.physIndices()) {
            if (regBusy(physIdx) &&
                    !computeUnit->rfc[simdId]->inRFC(physIdx)) {
                DPRINTF(GPUVRF, "WAX stall: WV[%d]: %s: physReg[%d]\n",
                        w->wfDynId, ii->disassemble(), physIdx);
                w->stats.numTimesBlockedDueWAXDependencies++;
                dst_ready = false;
                break;
            }
        }
        if (!dst_ready) {
            break;
        }
    }

    return src_ready && dst_ready;
}

void
VectorRegisterFile::scheduleWriteOperands(Wavefront *w, GPUDynInstPtr ii)
{
    for (const auto& dstVecOp : ii->dstVecRegOperands()) {
        for (const auto& physIdx : dstVecOp.physIndices()) {
            // If the instruction is atomic instruciton and the atomics do
            // not return value, then do not mark this reg as busy.
            if (!(ii->isAtomic() && !ii->isAtomicRet())) {
                /**
                 * if the instruction is a load with EXEC = 0, then we do not
                 * mark the reg. We do this to avoid a deadlock that can
                 * occur because a load reserves its destination regs before
                 * checking its exec mask, and in the cas it is 0, it will not
                 * send/recv any packets, and therefore it will never free its
                 * dst reg(s)
                 */
                if (ii->exec_mask.any()) {
                    markReg(physIdx, true);
                }
            }
        }
    }
}

void
VectorRegisterFile::waveExecuteInst(Wavefront *w, GPUDynInstPtr ii)
{
    // increment count of number of DWords read from VRF
    int DWords = ii->numSrcVecDWords();
    stats.registerReads += (DWords * w->execMask().count());

    for (const auto& dstVecOp : ii->dstVecRegOperands()) {
        for (const auto& physIdx : dstVecOp.physIndices()) {
            if (computeUnit->rfc[simdId]->inRFC(physIdx)) {
                stats.rfc_cache_write_hits += w->execMask().count();
            }
        }
    }

    for (const auto& srcVecOp : ii->srcVecRegOperands()) {
        for (const auto& physIdx : srcVecOp.physIndices()) {
            if (computeUnit->rfc[simdId]->inRFC(physIdx)) {
                stats.rfc_cache_read_hits += w->execMask().count();
            }
        }
    }

    uint64_t mask = w->execMask().to_ullong();
    int srams = w->execMask().size() / 4;
    for (int i = 0; i < srams; i++) {
        if (mask & 0xF) {
            stats.sramReads += DWords;
        }
        mask = mask >> 4;
    }

    if (!ii->isLoad()
        && !(ii->isAtomic() || ii->isMemSync())) {
        // TODO: compute proper delay
        // For now, it is based on largest operand size
        int opSize = ii->maxOperandSize();
        Cycles delay(opSize <= 4 ? computeUnit->spBypassLength()
            : computeUnit->dpBypassLength());
        Tick tickDelay = computeUnit->cyclesToTicks(delay);

        for (const auto& dstVecOp : ii->dstVecRegOperands()) {
            for (const auto& physIdx : dstVecOp.physIndices()) {
                enqRegFreeEvent(physIdx, tickDelay);
            }
        }
        // increment count of number of DWords written to VRF
        DWords = ii->numDstVecDWords();
        stats.registerWrites += (DWords * w->execMask().count());

        mask = w->execMask().to_ullong();
        srams = w->execMask().size() / 4;
        for (int i = 0; i < srams; i++) {
            if (mask & 0xF) {
                stats.sramWrites += DWords;
            }
            mask = mask >> 4;
        }
    }
}

void
VectorRegisterFile::scheduleWriteOperandsFromLoad(
    Wavefront *w, GPUDynInstPtr ii)
{
    assert(ii->isLoad() || ii->isAtomicRet());
    for (const auto& dstVecOp : ii->dstVecRegOperands()) {
        for (const auto& physIdx : dstVecOp.physIndices()) {
            enqRegFreeEvent(physIdx, computeUnit->clockPeriod());
        }
    }
    // increment count of number of DWords written to VRF
    int DWords = ii->numDstVecDWords();
    stats.registerWrites += (DWords * ii->exec_mask.count());

    uint64_t mask = ii->exec_mask.to_ullong();
    int srams = ii->exec_mask.size() / 4;
    for (int i = 0; i < srams; i++) {
        if (mask & 0xF) {
            stats.sramWrites += DWords;
        }
        mask = mask >> 4;
    }
}

} // namespace gem5
