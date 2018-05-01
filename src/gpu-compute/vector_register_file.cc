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
#include "base/trace.hh"
#include "debug/GPUVRF.hh"
#include "gpu-compute/compute_unit.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/simple_pool_manager.hh"
#include "gpu-compute/wavefront.hh"
#include "params/VectorRegisterFile.hh"

VectorRegisterFile::VectorRegisterFile(const VectorRegisterFileParams *p)
    : RegisterFile(p)
{
    regFile.resize(numRegs(), VecRegContainer());

    for (auto &reg : regFile) {
        reg.zero();
    }
}

bool
VectorRegisterFile::operandsReady(Wavefront *w, GPUDynInstPtr ii) const
{
    for (int i = 0; i < ii->getNumOperands(); ++i) {
        if (ii->isVectorRegister(i) && ii->isSrcOperand(i)) {
            int vgprIdx = ii->getRegisterIndex(i, ii);

            // determine number of registers
            int nRegs =
                ii->getOperandSize(i) <= 4 ? 1 : ii->getOperandSize(i) / 4;
            for (int j = 0; j < nRegs; j++) {
                int pVgpr = computeUnit->registerManager
                    ->mapVgpr(w, vgprIdx + j);
                if (regBusy(pVgpr)) {
                    if (ii->isDstOperand(i)) {
                        w->numTimesBlockedDueWAXDependencies++;
                    } else if (ii->isSrcOperand(i)) {
                        DPRINTF(GPUVRF, "RAW stall: WV[%d]: %s: physReg[%d]\n",
                                w->wfDynId, ii->disassemble(), pVgpr);
                        w->numTimesBlockedDueRAWDependencies++;
                    }
                    return false;
                }
            }
        }
    }
    return true;
}

void
VectorRegisterFile::scheduleWriteOperands(Wavefront *w, GPUDynInstPtr ii)
{
    // iterate over all register destination operands
    for (int i = 0; i < ii->getNumOperands(); ++i) {
        if (ii->isVectorRegister(i) && ii->isDstOperand(i)) {
            int vgprIdx = ii->getRegisterIndex(i, ii);
            int nRegs = ii->getOperandSize(i) <= 4 ? 1 :
                ii->getOperandSize(i) / 4;

            for (int j = 0; j < nRegs; ++j) {
                int physReg = computeUnit->registerManager
                    ->mapVgpr(w, vgprIdx + j);

                // If instruction is atomic instruction and
                // the atomics do not return value, then
                // do not mark this reg as busy.
                if (!(ii->isAtomic() && !ii->isAtomicRet())) {
                    /**
                     * if the instruction is a load with EXEC = 0, then
                     * we do not mark the reg. we do this to avoid a
                     * deadlock that can occur because a load reserves
                     * its destination regs before checking its exec mask,
                     * and in the case it is 0, it will not send/recv any
                     * packets, and therefore it will never free its dest
                     * reg(s).
                     */
                    if (!ii->isLoad() || (ii->isLoad()
                        && ii->exec_mask.any())) {
                        markReg(physReg, true);
                    }
                }
            }
        }
    }
}

void
VectorRegisterFile::waveExecuteInst(Wavefront *w, GPUDynInstPtr ii)
{
    // increment count of number of DWORDs read from VRF
    int DWORDs = ii->numSrcVecDWORDs();
    registerReads += (DWORDs * w->execMask().count());

    uint64_t mask = w->execMask().to_ullong();
    int srams = w->execMask().size() / 4;
    for (int i = 0; i < srams; i++) {
        if (mask & 0xF) {
            sramReads += DWORDs;
        }
        mask = mask >> 4;
    }

    if (!ii->isLoad()
        && !(ii->isAtomic() || ii->isMemSync())) {
        int opSize = 4;
        for (int i = 0; i < ii->getNumOperands(); i++) {
            if (ii->getOperandSize(i) > opSize) {
                opSize = ii->getOperandSize(i);
            }
        }
        Cycles delay(opSize <= 4 ? computeUnit->spBypassLength()
            : computeUnit->dpBypassLength());
        Tick tickDelay = computeUnit->cyclesToTicks(delay);

        for (int i = 0; i < ii->getNumOperands(); i++) {
            if (ii->isVectorRegister(i) && ii->isDstOperand(i)) {
                int vgprIdx = ii->getRegisterIndex(i, ii);
                int nRegs = ii->getOperandSize(i) <= 4 ? 1
                    : ii->getOperandSize(i) / 4;
                for (int j = 0; j < nRegs; j++) {
                    int physReg = computeUnit->registerManager
                        ->mapVgpr(w, vgprIdx + j);
                    enqRegFreeEvent(physReg, tickDelay);
                }
            }
        }

        // increment count of number of DWORDs written to VRF
        DWORDs = ii->numDstVecDWORDs();
        registerWrites += (DWORDs * w->execMask().count());

        mask = w->execMask().to_ullong();
        srams = w->execMask().size() / 4;
        for (int i = 0; i < srams; i++) {
            if (mask & 0xF) {
                sramWrites += DWORDs;
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
    for (int i = 0; i < ii->getNumOperands(); ++i) {
        if (ii->isVectorRegister(i) && ii->isDstOperand(i)) {
            int vgprIdx = ii->getRegisterIndex(i, ii);
            int nRegs = ii->getOperandSize(i) <= 4 ? 1 :
                ii->getOperandSize(i) / 4;

            for (int j = 0; j < nRegs; ++j) {
                int physReg = computeUnit->registerManager
                    ->mapVgpr(w, vgprIdx + j);
                enqRegFreeEvent(physReg, computeUnit->clockPeriod());
            }
        }
    }
    // increment count of number of DWORDs written to VRF
    int DWORDs = ii->numDstVecDWORDs();
    registerWrites += (DWORDs * ii->exec_mask.count());

    uint64_t mask = ii->exec_mask.to_ullong();
    int srams = ii->exec_mask.size() / 4;
    for (int i = 0; i < srams; i++) {
        if (mask & 0xF) {
            sramWrites += DWORDs;
        }
        mask = mask >> 4;
    }
}

VectorRegisterFile*
VectorRegisterFileParams::create()
{
    return new VectorRegisterFile(this);
}
