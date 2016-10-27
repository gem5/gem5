/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * Author: Steve Reinhardt
 */

#include "arch/hsail/insts/decl.hh"
#include "debug/GPUExec.hh"
#include "gpu-compute/dispatcher.hh"
#include "gpu-compute/simple_pool_manager.hh"

namespace HsailISA
{
    template<> const char *B1::label = "b1";
    template<> const char *B8::label = "b8";
    template<> const char *B16::label = "b16";
    template<> const char *B32::label = "b32";
    template<> const char *B64::label = "b64";

    template<> const char *S8::label = "s8";
    template<> const char *S16::label = "s16";
    template<> const char *S32::label = "s32";
    template<> const char *S64::label = "s64";

    template<> const char *U8::label = "u8";
    template<> const char *U16::label = "u16";
    template<> const char *U32::label = "u32";
    template<> const char *U64::label = "u64";

    template<> const char *F32::label = "f32";
    template<> const char *F64::label = "f64";

    const char*
    cmpOpToString(Brig::BrigCompareOperation cmpOp)
    {
        using namespace Brig;

        switch (cmpOp) {
          case BRIG_COMPARE_EQ:
            return "eq";
          case BRIG_COMPARE_NE:
            return "ne";
          case BRIG_COMPARE_LT:
            return "lt";
          case BRIG_COMPARE_LE:
            return "le";
          case BRIG_COMPARE_GT:
            return "gt";
          case BRIG_COMPARE_GE:
            return "ge";
          case BRIG_COMPARE_EQU:
            return "equ";
          case BRIG_COMPARE_NEU:
            return "neu";
          case BRIG_COMPARE_LTU:
            return "ltu";
          case BRIG_COMPARE_LEU:
            return "leu";
          case BRIG_COMPARE_GTU:
            return "gtu";
          case BRIG_COMPARE_GEU:
            return "geu";
          case BRIG_COMPARE_NUM:
            return "num";
          case BRIG_COMPARE_NAN:
            return "nan";
          case BRIG_COMPARE_SEQ:
            return "seq";
          case BRIG_COMPARE_SNE:
            return "sne";
          case BRIG_COMPARE_SLT:
            return "slt";
          case BRIG_COMPARE_SLE:
            return "sle";
          case BRIG_COMPARE_SGT:
            return "sgt";
          case BRIG_COMPARE_SGE:
            return "sge";
          case BRIG_COMPARE_SGEU:
            return "sgeu";
          case BRIG_COMPARE_SEQU:
            return "sequ";
          case BRIG_COMPARE_SNEU:
            return "sneu";
          case BRIG_COMPARE_SLTU:
            return "sltu";
          case BRIG_COMPARE_SLEU:
            return "sleu";
          case BRIG_COMPARE_SNUM:
            return "snum";
          case BRIG_COMPARE_SNAN:
            return "snan";
          case BRIG_COMPARE_SGTU:
            return "sgtu";
          default:
            return "unknown";
        }
    }

    void
    Ret::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        const VectorMask &mask = w->getPred();

        // mask off completed work-items
        for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
            if (mask[lane]) {
                w->initMask[lane] = 0;
            }

        }

        // delete extra instructions fetched for completed work-items
        w->instructionBuffer.erase(w->instructionBuffer.begin() + 1,
                                   w->instructionBuffer.end());
        if (w->pendingFetch) {
            w->dropFetch = true;
        }

        // if all work-items have completed, then wave-front is done
        if (w->initMask.none()) {
            w->status = Wavefront::S_STOPPED;

            int32_t refCount = w->computeUnit->getLds().
                                   decreaseRefCounter(w->dispatchId, w->wgId);

            DPRINTF(GPUExec, "CU%d: decrease ref ctr WG[%d] to [%d]\n",
                            w->computeUnit->cu_id, w->wgId, refCount);

            // free the vector registers of the completed wavefront
            w->computeUnit->vectorRegsReserved[w->simdId] -=
                w->reservedVectorRegs;

            assert(w->computeUnit->vectorRegsReserved[w->simdId] >= 0);

            uint32_t endIndex = (w->startVgprIndex +
                                 w->reservedVectorRegs - 1) %
                w->computeUnit->vrf[w->simdId]->numRegs();

            w->computeUnit->vrf[w->simdId]->manager->
                freeRegion(w->startVgprIndex, endIndex);

            w->reservedVectorRegs = 0;
            w->startVgprIndex = 0;
            w->computeUnit->completedWfs++;

            DPRINTF(GPUExec, "Doing return for CU%d: WF[%d][%d][%d]\n",
                    w->computeUnit->cu_id, w->simdId, w->wfSlotId, w->wfDynId);

            if (!refCount) {
                setFlag(SystemScope);
                setFlag(Release);
                setFlag(GlobalSegment);
                // Notify Memory System of Kernel Completion
                // Kernel End = isKernel + isRelease
                w->status = Wavefront::S_RETURNING;
                GPUDynInstPtr local_mempacket = gpuDynInst;
                local_mempacket->useContinuation = false;
                local_mempacket->simdId = w->simdId;
                local_mempacket->wfSlotId = w->wfSlotId;
                local_mempacket->wfDynId = w->wfDynId;
                w->computeUnit->injectGlobalMemFence(local_mempacket, true);
            } else {
                w->computeUnit->shader->dispatcher->scheduleDispatch();
            }
        }
    }

    void
    Barrier::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        assert(w->barrierCnt == w->oldBarrierCnt);
        w->barrierCnt = w->oldBarrierCnt + 1;
        w->stalledAtBarrier = true;
    }
} // namespace HsailISA
