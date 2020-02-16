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

#include "gpu-compute/gpu_dyn_inst.hh"

#include "debug/GPUMem.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"

GPUDynInst::GPUDynInst(ComputeUnit *_cu, Wavefront *_wf,
                       GPUStaticInst *static_inst, uint64_t instSeqNum)
    : GPUExecContext(_cu, _wf), addr(computeUnit()->wfSize(), (Addr)0),
      n_reg(0), useContinuation(false),
      statusBitVector(0), _staticInst(static_inst), _seqNum(instSeqNum)
{
    tlbHitLevel.assign(computeUnit()->wfSize(), -1);
    d_data = new uint8_t[computeUnit()->wfSize() * 16];
    a_data = new uint8_t[computeUnit()->wfSize() * 8];
    x_data = new uint8_t[computeUnit()->wfSize() * 8];
    for (int i = 0; i < (computeUnit()->wfSize() * 8); ++i) {
        a_data[i] = 0;
        x_data[i] = 0;
    }
    for (int i = 0; i < (computeUnit()->wfSize() * 16); ++i) {
        d_data[i] = 0;
    }
}

GPUDynInst::~GPUDynInst()
{
    delete[] d_data;
    delete[] a_data;
    delete[] x_data;
}

void
GPUDynInst::execute(GPUDynInstPtr gpuDynInst)
{
    _staticInst->execute(gpuDynInst);
}

int
GPUDynInst::numSrcRegOperands()
{
    return _staticInst->numSrcRegOperands();
}

int
GPUDynInst::numDstRegOperands()
{
    return _staticInst->numDstRegOperands();
}

int
GPUDynInst::getNumOperands()
{
    return _staticInst->getNumOperands();
}

bool
GPUDynInst::isVectorRegister(int operandIdx)
{
    return _staticInst->isVectorRegister(operandIdx);
}

bool
GPUDynInst::isScalarRegister(int operandIdx)
{
    return _staticInst->isScalarRegister(operandIdx);
}

bool
GPUDynInst::isCondRegister(int operandIdx)
{
    return _staticInst->isCondRegister(operandIdx);
}

int
GPUDynInst::getRegisterIndex(int operandIdx, GPUDynInstPtr gpuDynInst)
{
    return _staticInst->getRegisterIndex(operandIdx, gpuDynInst);
}

int
GPUDynInst::getOperandSize(int operandIdx)
{
    return _staticInst->getOperandSize(operandIdx);
}

bool
GPUDynInst::isDstOperand(int operandIdx)
{
    return _staticInst->isDstOperand(operandIdx);
}

bool
GPUDynInst::isSrcOperand(int operandIdx)
{
    return _staticInst->isSrcOperand(operandIdx);
}

const std::string&
GPUDynInst::disassemble() const
{
    return _staticInst->disassemble();
}

uint64_t
GPUDynInst::seqNum() const
{
    return _seqNum;
}

Enums::StorageClassType
GPUDynInst::executedAs()
{
    return _staticInst->executed_as;
}

// Process a memory instruction and (if necessary) submit timing request
void
GPUDynInst::initiateAcc(GPUDynInstPtr gpuDynInst)
{
    DPRINTF(GPUMem, "CU%d: WF[%d][%d]: mempacket status bitvector=%#x\n",
            cu->cu_id, simdId, wfSlotId, exec_mask);

    _staticInst->initiateAcc(gpuDynInst);
    time = 0;
}

void
GPUDynInst::completeAcc(GPUDynInstPtr gpuDynInst)
{
    _staticInst->completeAcc(gpuDynInst);
}

/**
 * accessor methods for the attributes of
 * the underlying GPU static instruction
 */
bool
GPUDynInst::isALU() const
{
    return _staticInst->isALU();
}

bool
GPUDynInst::isBranch() const
{
    return _staticInst->isBranch();
}

bool
GPUDynInst::isNop() const
{
    return _staticInst->isNop();
}

bool
GPUDynInst::isReturn() const
{
    return _staticInst->isReturn();
}

bool
GPUDynInst::isUnconditionalJump() const
{
    return _staticInst->isUnconditionalJump();
}

bool
GPUDynInst::isSpecialOp() const
{
    return _staticInst->isSpecialOp();
}

bool
GPUDynInst::isWaitcnt() const
{
    return _staticInst->isWaitcnt();
}

bool
GPUDynInst::isBarrier() const
{
    return _staticInst->isBarrier();
}

bool
GPUDynInst::isMemFence() const
{
    return _staticInst->isMemFence();
}

bool
GPUDynInst::isMemRef() const
{
    return _staticInst->isMemRef();
}

bool
GPUDynInst::isFlat() const
{
    return _staticInst->isFlat();
}

bool
GPUDynInst::isLoad() const
{
    return _staticInst->isLoad();
}

bool
GPUDynInst::isStore() const
{
    return _staticInst->isStore();
}

bool
GPUDynInst::isAtomic() const
{
    return _staticInst->isAtomic();
}

bool
GPUDynInst::isAtomicNoRet() const
{
    return _staticInst->isAtomicNoRet();
}

bool
GPUDynInst::isAtomicRet() const
{
    return _staticInst->isAtomicRet();
}

bool
GPUDynInst::isScalar() const
{
    return _staticInst->isScalar();
}

bool
GPUDynInst::readsSCC() const
{
    return _staticInst->readsSCC();
}

bool
GPUDynInst::writesSCC() const
{
    return _staticInst->writesSCC();
}

bool
GPUDynInst::readsVCC() const
{
    return _staticInst->readsVCC();
}

bool
GPUDynInst::writesVCC() const
{
    return _staticInst->writesVCC();
}

bool
GPUDynInst::isAtomicAnd() const
{
    return _staticInst->isAtomicAnd();
}

bool
GPUDynInst::isAtomicOr() const
{
    return _staticInst->isAtomicOr();
}

bool
GPUDynInst::isAtomicXor() const
{
    return _staticInst->isAtomicXor();
}

bool
GPUDynInst::isAtomicCAS() const
{
    return _staticInst->isAtomicCAS();
}

bool GPUDynInst::isAtomicExch() const
{
    return _staticInst->isAtomicExch();
}

bool
GPUDynInst::isAtomicAdd() const
{
    return _staticInst->isAtomicAdd();
}

bool
GPUDynInst::isAtomicSub() const
{
    return _staticInst->isAtomicSub();
}

bool
GPUDynInst::isAtomicInc() const
{
    return _staticInst->isAtomicInc();
}

bool
GPUDynInst::isAtomicDec() const
{
    return _staticInst->isAtomicDec();
}

bool
GPUDynInst::isAtomicMax() const
{
    return _staticInst->isAtomicMax();
}

bool
GPUDynInst::isAtomicMin() const
{
    return _staticInst->isAtomicMin();
}

bool
GPUDynInst::isArgLoad() const
{
    return _staticInst->isArgLoad();
}

bool
GPUDynInst::isGlobalMem() const
{
    return _staticInst->isGlobalMem();
}

bool
GPUDynInst::isLocalMem() const
{
    return _staticInst->isLocalMem();
}

bool
GPUDynInst::isArgSeg() const
{
    return _staticInst->isArgSeg();
}

bool
GPUDynInst::isGlobalSeg() const
{
    return _staticInst->isGlobalSeg();
}

bool
GPUDynInst::isGroupSeg() const
{
    return _staticInst->isGroupSeg();
}

bool
GPUDynInst::isKernArgSeg() const
{
    return _staticInst->isKernArgSeg();
}

bool
GPUDynInst::isPrivateSeg() const
{
    return _staticInst->isPrivateSeg();
}

bool
GPUDynInst::isReadOnlySeg() const
{
    return _staticInst->isReadOnlySeg();
}

bool
GPUDynInst::isSpillSeg() const
{
    return _staticInst->isSpillSeg();
}

bool
GPUDynInst::isWorkitemScope() const
{
    return _staticInst->isWorkitemScope();
}

bool
GPUDynInst::isWavefrontScope() const
{
    return _staticInst->isWavefrontScope();
}

bool
GPUDynInst::isWorkgroupScope() const
{
    return _staticInst->isWorkgroupScope();
}

bool
GPUDynInst::isDeviceScope() const
{
    return _staticInst->isDeviceScope();
}

bool
GPUDynInst::isSystemScope() const
{
    return _staticInst->isSystemScope();
}

bool
GPUDynInst::isNoScope() const
{
    return _staticInst->isNoScope();
}

bool
GPUDynInst::isRelaxedOrder() const
{
    return _staticInst->isRelaxedOrder();
}

bool
GPUDynInst::isAcquire() const
{
    return _staticInst->isAcquire();
}

bool
GPUDynInst::isRelease() const
{
    return _staticInst->isRelease();
}

bool
GPUDynInst::isAcquireRelease() const
{
    return _staticInst->isAcquireRelease();
}

bool
GPUDynInst::isNoOrder() const
{
    return _staticInst->isNoOrder();
}

bool
GPUDynInst::isGloballyCoherent() const
{
    return _staticInst->isGloballyCoherent();
}

bool
GPUDynInst::isSystemCoherent() const
{
    return _staticInst->isSystemCoherent();
}

void
GPUDynInst::updateStats()
{
    if (_staticInst->isLocalMem()) {
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
