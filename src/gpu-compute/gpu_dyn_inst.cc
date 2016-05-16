/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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
 * Author: Anthony Gutierrez
 */

#include "gpu-compute/gpu_dyn_inst.hh"

#include "debug/GPUMem.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"

GPUDynInst::GPUDynInst(ComputeUnit *_cu, Wavefront *_wf,
                       GPUStaticInst *_staticInst, uint64_t instSeqNum)
    : GPUExecContext(_cu, _wf), m_op(Enums::MO_UNDEF),
      memoryOrder(Enums::MEMORY_ORDER_NONE), n_reg(0), useContinuation(false),
      statusBitVector(0), staticInst(_staticInst), _seqNum(instSeqNum)
{
    tlbHitLevel.assign(VSZ, -1);
}

void
GPUDynInst::execute()
{
    GPUDynInstPtr gpuDynInst = std::make_shared<GPUDynInst>(cu, wf, staticInst,
                                                            _seqNum);
    staticInst->execute(gpuDynInst);
}

int
GPUDynInst::numSrcRegOperands()
{
    return staticInst->numSrcRegOperands();
}

int
GPUDynInst::numDstRegOperands()
{
    return staticInst->numDstRegOperands();
}

int
GPUDynInst::getNumOperands()
{
    return staticInst->getNumOperands();
}

bool
GPUDynInst::isVectorRegister(int operandIdx)
{
    return staticInst->isVectorRegister(operandIdx);
}

bool
GPUDynInst::isScalarRegister(int operandIdx)
{
    return staticInst->isScalarRegister(operandIdx);
}

int
GPUDynInst::getRegisterIndex(int operandIdx)
{
    return staticInst->getRegisterIndex(operandIdx);
}

int
GPUDynInst::getOperandSize(int operandIdx)
{
    return staticInst->getOperandSize(operandIdx);
}

bool
GPUDynInst::isDstOperand(int operandIdx)
{
    return staticInst->isDstOperand(operandIdx);
}

bool
GPUDynInst::isSrcOperand(int operandIdx)
{
    return staticInst->isSrcOperand(operandIdx);
}

bool
GPUDynInst::isArgLoad()
{
    return staticInst->isArgLoad();
}

const std::string&
GPUDynInst::disassemble() const
{
    return staticInst->disassemble();
}

uint64_t
GPUDynInst::seqNum() const
{
    return _seqNum;
}

Enums::OpType
GPUDynInst::opType()
{
    return staticInst->o_type;
}

Enums::StorageClassType
GPUDynInst::executedAs()
{
    return staticInst->executed_as;
}

// Process a memory instruction and (if necessary) submit timing request
void
GPUDynInst::initiateAcc(GPUDynInstPtr gpuDynInst)
{
    DPRINTF(GPUMem, "CU%d: WF[%d][%d]: mempacket status bitvector=%#x\n",
            cu->cu_id, simdId, wfSlotId, exec_mask);

    staticInst->initiateAcc(gpuDynInst);
    time = 0;
}

bool
GPUDynInst::scalarOp() const
{
    return staticInst->scalarOp();
}

void
GPUDynInst::updateStats()
{
    if (staticInst->isLocalMem()) {
        // access to LDS (shared) memory
        cu->dynamicLMemInstrCnt++;
    } else {
        // access to global memory

        // update PageDivergence histogram
        int number_pages_touched = cu->pagesTouched.size();
        assert(number_pages_touched);
        cu->pageDivergenceDist.sample(number_pages_touched);

        std::pair<ComputeUnit::pageDataStruct::iterator, bool> ret;

        for (auto it : cu->pagesTouched) {
            // see if this page has been touched before. if not, this also
            // inserts the page into the table.
            ret = cu->pageAccesses
                .insert(ComputeUnit::pageDataStruct::value_type(it.first,
                        std::make_pair(1, it.second)));

            // if yes, then update the stats
            if (!ret.second) {
                ret.first->second.first++;
                ret.first->second.second += it.second;
            }
        }

        cu->pagesTouched.clear();

        // total number of memory instructions (dynamic)
        // Atomics are counted as a single memory instruction.
        // this is # memory instructions per wavefronts, not per workitem
        cu->dynamicGMemInstrCnt++;
    }
}
