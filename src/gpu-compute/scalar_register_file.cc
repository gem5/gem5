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

#include "gpu-compute/scalar_register_file.hh"

#include "base/logging.hh"
#include "debug/GPUSRF.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"
#include "params/ScalarRegisterFile.hh"

namespace gem5
{

ScalarRegisterFile::ScalarRegisterFile(const ScalarRegisterFileParams &p)
    : RegisterFile(p)
{
    regFile.resize(numRegs(), 0);
}

bool
ScalarRegisterFile::operandsReady(Wavefront *w, GPUDynInstPtr ii) const
{
    for (const auto &srcScalarOp : ii->srcScalarRegOperands()) {
        for (const auto &physIdx : srcScalarOp.physIndices()) {
            if (regBusy(physIdx)) {
                DPRINTF(GPUSRF, "RAW stall: WV[%d]: %s: physReg[%d]\n",
                        w->wfDynId, ii->disassemble(), physIdx);
                w->stats.numTimesBlockedDueRAWDependencies++;
                return false;
            }
        }
    }

    for (const auto &dstScalarOp : ii->dstScalarRegOperands()) {
        for (const auto &physIdx : dstScalarOp.physIndices()) {
            if (regBusy(physIdx)) {
                DPRINTF(GPUSRF, "WAX stall: WV[%d]: %s: physReg[%d]\n",
                        w->wfDynId, ii->disassemble(), physIdx);
                w->stats.numTimesBlockedDueWAXDependencies++;
                return false;
            }
        }
    }

    return true;
}

void
ScalarRegisterFile::scheduleWriteOperands(Wavefront *w, GPUDynInstPtr ii)
{
    for (const auto &dstScalarOp : ii->dstScalarRegOperands()) {
        for (const auto &physIdx : dstScalarOp.physIndices()) {
            // mark the destination scalar register as busy
            markReg(physIdx, true);
        }
    }
}

void
ScalarRegisterFile::waveExecuteInst(Wavefront *w, GPUDynInstPtr ii)
{
    stats.registerReads += ii->numSrcScalarDWords();

    if (!ii->isLoad() && !(ii->isAtomic() || ii->isMemSync())) {
        Cycles delay(computeUnit->scalarPipeLength());
        Tick tickDelay = computeUnit->cyclesToTicks(delay);

        for (const auto &dstScalarOp : ii->dstScalarRegOperands()) {
            for (const auto &physIdx : dstScalarOp.physIndices()) {
                enqRegFreeEvent(physIdx, tickDelay);
            }
        }

        stats.registerWrites += ii->numDstScalarDWords();
    }
}

void
ScalarRegisterFile::scheduleWriteOperandsFromLoad(Wavefront *w,
                                                  GPUDynInstPtr ii)
{
    assert(ii->isLoad() || ii->isAtomicRet());
    for (const auto &dstScalarOp : ii->dstScalarRegOperands()) {
        for (const auto &physIdx : dstScalarOp.physIndices()) {
            enqRegFreeEvent(physIdx, computeUnit->clockPeriod());
        }
    }

    stats.registerWrites += ii->numDstScalarDWords();
}

} // namespace gem5
