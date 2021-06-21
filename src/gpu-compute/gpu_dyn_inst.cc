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

#include "debug/GPUInst.hh"
#include "debug/GPUMem.hh"
#include "gpu-compute/gpu_static_inst.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/shader.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

GPUDynInst::GPUDynInst(ComputeUnit *_cu, Wavefront *_wf,
                       GPUStaticInst *static_inst, InstSeqNum instSeqNum)
    : GPUExecContext(_cu, _wf), scalarAddr(0), addr(computeUnit()->wfSize(),
      (Addr)0), numScalarReqs(0), isSaveRestore(false),
      _staticInst(static_inst), _seqNum(instSeqNum),
      maxSrcVecRegOpSize(-1), maxSrcScalarRegOpSize(-1)
{
    _staticInst->initOperandInfo();
    statusVector.assign(TheGpuISA::NumVecElemPerVecReg, 0);
    tlbHitLevel.assign(computeUnit()->wfSize(), -1);
    // vector instructions can have up to 4 source/destination operands
    d_data = new uint8_t[computeUnit()->wfSize() * 4 * sizeof(double)];
    a_data = new uint8_t[computeUnit()->wfSize() * 8];
    x_data = new uint8_t[computeUnit()->wfSize() * 8];
    // scalar loads can read up to 16 Dwords of data (see publicly
    // available GCN3 ISA manual)
    scalar_data = new uint8_t[16 * sizeof(uint32_t)];
    for (int i = 0; i < (16 * sizeof(uint32_t)); ++i) {
        scalar_data[i] = 0;
    }
    for (int i = 0; i < (computeUnit()->wfSize() * 8); ++i) {
        a_data[i] = 0;
        x_data[i] = 0;
    }
    for (int i = 0; i < (computeUnit()->wfSize() * 4 * sizeof(double)); ++i) {
        d_data[i] = 0;
    }
    time = 0;

    cu_id = _cu->cu_id;
    if (_wf) {
        simdId = _wf->simdId;
        wfDynId = _wf->wfDynId;
        kern_id = _wf->kernId;
        wg_id = _wf->wgId;
        wfSlotId = _wf->wfSlotId;
    } else {
        simdId = -1;
        wfDynId = -1;
        kern_id = -1;
        wg_id = -1;
        wfSlotId = -1;
    }


    DPRINTF(GPUInst, "%s: generating operand info for %d operands\n",
            disassemble(), getNumOperands());

    _staticInst->initDynOperandInfo(wavefront(), computeUnit());

}

GPUDynInst::~GPUDynInst()
{
    delete[] d_data;
    delete[] a_data;
    delete[] x_data;
    delete[] scalar_data;
    delete _staticInst;
}

void
GPUDynInst::execute(GPUDynInstPtr gpuDynInst)
{
    _staticInst->execute(gpuDynInst);
}

const std::vector<OperandInfo>&
GPUDynInst::srcVecRegOperands() const
{
    return _staticInst->srcVecRegOperands();
}

const std::vector<OperandInfo>&
GPUDynInst::dstVecRegOperands() const
{
    return _staticInst->dstVecRegOperands();
}

const std::vector<OperandInfo>&
GPUDynInst::srcScalarRegOperands() const
{
    return _staticInst->srcScalarRegOperands();
}

const std::vector<OperandInfo>&
GPUDynInst::dstScalarRegOperands() const
{
    return _staticInst->dstScalarRegOperands();
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
GPUDynInst::numSrcVecRegOperands() const
{
    return _staticInst->numSrcVecOperands();
}

int
GPUDynInst::numDstVecRegOperands() const
{
    return _staticInst->numDstVecOperands();
}

int
GPUDynInst::maxSrcVecRegOperandSize()
{
    if (maxSrcVecRegOpSize != -1)
        return maxSrcVecRegOpSize;

    maxSrcVecRegOpSize = 0;
    for (const auto& srcVecOp : srcVecRegOperands())
        if (srcVecOp.sizeInDWords() > maxSrcVecRegOpSize)
            maxSrcVecRegOpSize = srcVecOp.sizeInDWords();

    return maxSrcVecRegOpSize;
}

int
GPUDynInst::numSrcVecDWords()
{
    return _staticInst->numSrcVecDWords();
}

int
GPUDynInst::numDstVecDWords()
{
    return _staticInst->numDstVecDWords();
}

int
GPUDynInst::numSrcScalarRegOperands() const
{
    return _staticInst->numSrcScalarOperands();
}

int
GPUDynInst::numDstScalarRegOperands() const
{
    return _staticInst->numDstScalarOperands();
}

int
GPUDynInst::maxSrcScalarRegOperandSize()
{
    if (maxSrcScalarRegOpSize != -1)
        return maxSrcScalarRegOpSize;

    maxSrcScalarRegOpSize = 0;
    for (const auto& srcScOp : srcScalarRegOperands())
        if (srcScOp.sizeInDWords() > maxSrcScalarRegOpSize)
            maxSrcScalarRegOpSize = srcScOp.sizeInDWords();

    return maxSrcScalarRegOpSize;
}

int
GPUDynInst::numSrcScalarDWords()
{
    return _staticInst->numSrcScalarDWords();
}

int
GPUDynInst::numDstScalarDWords()
{
    return _staticInst->numDstScalarDWords();
}

int
GPUDynInst::maxOperandSize()
{
    return _staticInst->maxOperandSize();
}

int
GPUDynInst::getNumOperands() const
{
    return _staticInst->getNumOperands();
}

bool
GPUDynInst::hasSourceVgpr() const
{
    return !srcVecRegOperands().empty();
}

bool
GPUDynInst::hasDestinationVgpr() const
{
    return !dstVecRegOperands().empty();
}

bool
GPUDynInst::hasSourceSgpr() const
{
    return !srcScalarRegOperands().empty();
}

bool
GPUDynInst::hasDestinationSgpr() const
{
    return !dstScalarRegOperands().empty();
}

bool
GPUDynInst::isOpcode(const std::string& opcodeStr,
                     const std::string& extStr) const
{
    return _staticInst->opcode().find(opcodeStr) != std::string::npos &&
        _staticInst->opcode().find(extStr) != std::string::npos;
}

bool
GPUDynInst::isOpcode(const std::string& opcodeStr) const
{
    return _staticInst->opcode().find(opcodeStr) != std::string::npos;
}

const std::string&
GPUDynInst::disassemble() const
{
    return _staticInst->disassemble();
}

InstSeqNum
GPUDynInst::seqNum() const
{
    return _seqNum;
}

Addr
GPUDynInst::pc()
{
    return wavefront()->pc();
}

void
GPUDynInst::pc(Addr _pc)
{
    wavefront()->pc(_pc);
}

enums::StorageClassType
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
}

void
GPUDynInst::completeAcc(GPUDynInstPtr gpuDynInst)
{
    DPRINTF(GPUMem, "CU%d: WF[%d][%d]: mempacket status bitvector="
            "%#x\n complete",
            cu->cu_id, simdId, wfSlotId, exec_mask);

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
GPUDynInst::isCondBranch() const
{
    return _staticInst->isCondBranch();
}

bool
GPUDynInst::isNop() const
{
    return _staticInst->isNop();
}

bool
GPUDynInst::isEndOfKernel() const
{
    return _staticInst->isEndOfKernel();
}

bool
GPUDynInst::isKernelLaunch() const
{
    return _staticInst->isKernelLaunch();
}

bool
GPUDynInst::isSDWAInst() const
{
    return _staticInst->isSDWAInst();
}

bool
GPUDynInst::isDPPInst() const
{
    return _staticInst->isDPPInst();
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
GPUDynInst::isSleep() const
{
    return _staticInst->isSleep();
}

bool
GPUDynInst::isBarrier() const
{
    return _staticInst->isBarrier();
}

bool
GPUDynInst::isMemSync() const
{
    return _staticInst->isMemSync();
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
GPUDynInst::isFlatGlobal() const
{
    return _staticInst->isFlatGlobal();
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
GPUDynInst::isVector() const
{
    return !_staticInst->isScalar();
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
    for (const auto& srcOp : _staticInst->srcOperands())
        if (srcOp.isVcc())
            return true;

    return _staticInst->readsVCC();
}

bool
GPUDynInst::writesVCC() const
{
    for (const auto& dstOp : _staticInst->dstOperands())
        if (dstOp.isVcc())
            return true;

    return _staticInst->writesVCC();
}

bool
GPUDynInst::readsMode() const
{
    return _staticInst->readsMode();
}

bool
GPUDynInst::writesMode() const
{
    return _staticInst->writesMode();
}

bool
GPUDynInst::readsExec() const
{
    return _staticInst->readsEXEC();
}

bool
GPUDynInst::writesExec() const
{
    return _staticInst->writesEXEC();
}

bool
GPUDynInst::ignoreExec() const
{
    return _staticInst->ignoreExec();
}

bool
GPUDynInst::writesExecMask() const
{
    for (const auto& dstOp : _staticInst->dstOperands())
        if (dstOp.isExec())
            return true;

    return _staticInst->writesEXEC();
}

bool
GPUDynInst::readsExecMask() const
{
    for (const auto& srcOp : _staticInst->srcOperands())
        if (srcOp.isExec())
            return true;

    return _staticInst->readsEXEC();
}

bool
GPUDynInst::writesFlatScratch() const
{
    for (const auto& dstScalarOp : dstScalarRegOperands())
        if (dstScalarOp.isFlatScratch())
            return true;

    return false;
}

bool
GPUDynInst::readsFlatScratch() const
{
    for (const auto& srcScalarOp : srcScalarRegOperands())
        if (srcScalarOp.isFlatScratch())
            return true;

    return false;
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
GPUDynInst::isGloballyCoherent() const
{
    return _staticInst->isGloballyCoherent();
}

bool
GPUDynInst::isSystemCoherent() const
{
    return _staticInst->isSystemCoherent();
}

bool
GPUDynInst::isF16() const
{
    return _staticInst->isF16();
}

bool
GPUDynInst::isF32() const
{
    return _staticInst->isF32();
}

bool
GPUDynInst::isF64() const
{
    return _staticInst->isF64();
}

bool
GPUDynInst::isFMA() const
{
    return _staticInst->isFMA();
}

bool
GPUDynInst::isMAC() const
{
    return _staticInst->isMAC();
}

bool
GPUDynInst::isMAD() const
{
    return _staticInst->isMAD();
}

void
GPUDynInst::doApertureCheck(const VectorMask &mask)
{
    assert(mask.any());
    // find the segment of the first active address, after
    // that we check that all other active addresses also
    // fall within the same APE
    for (int lane = 0; lane < computeUnit()->wfSize(); ++lane) {
        if (mask[lane]) {
            if (computeUnit()->shader->isLdsApe(addr[lane])) {
                // group segment
                staticInstruction()->executed_as = enums::SC_GROUP;
                break;
            } else if (computeUnit()->shader->isScratchApe(addr[lane])) {
                // private segment
                staticInstruction()->executed_as = enums::SC_PRIVATE;
                break;
            } else if (computeUnit()->shader->isGpuVmApe(addr[lane])) {
                // we won't support GPUVM
                fatal("flat access is in GPUVM APE\n");
            } else if (bits(addr[lane], 63, 47) != 0x1FFFF &&
                       bits(addr[lane], 63, 47)) {
                // we are in the "hole", this is a memory violation
                fatal("flat access at addr %#x has a memory violation\n",
                      addr[lane]);
            } else {
                // global memory segment
                staticInstruction()->executed_as = enums::SC_GLOBAL;
                break;
            }
        }
    }

    // we should have found the segment
    assert(executedAs() != enums::SC_NONE);

    // flat accesses should not straddle multiple APEs so we
    // must check that all addresses fall within the same APE
    if (executedAs() == enums::SC_GROUP) {
        for (int lane = 0; lane < computeUnit()->wfSize(); ++lane) {
            if (mask[lane]) {
                // if the first valid addr we found above was LDS,
                // all the rest should be
                assert(computeUnit()->shader->isLdsApe(addr[lane]));
            }
        }
    } else if (executedAs() == enums::SC_PRIVATE) {
        for (int lane = 0; lane < computeUnit()->wfSize(); ++lane) {
            if (mask[lane]) {
                // if the first valid addr we found above was private,
                // all the rest should be
                assert(computeUnit()->shader->isScratchApe(addr[lane]));
            }
        }
    } else {
        for (int lane = 0; lane < computeUnit()->wfSize(); ++lane) {
            if (mask[lane]) {
                // if the first valid addr we found above was global,
                // all the rest should be. because we don't have an
                // explicit range of the global segment, we just make
                // sure that the address fall in no other APE and that
                // it is not a memory violation
                assert(!computeUnit()->shader->isLdsApe(addr[lane]));
                assert(!computeUnit()->shader->isScratchApe(addr[lane]));
                assert(!computeUnit()->shader->isGpuVmApe(addr[lane]));
                assert(!(bits(addr[lane], 63, 47) != 0x1FFFF
                       && bits(addr[lane], 63, 47)));
            }
        }
    }
}

void
GPUDynInst::resolveFlatSegment(const VectorMask &mask)
{
    doApertureCheck(mask);


    // Now that we know the aperature, do the following:
    // 1. Transform the flat address to its segmented equivalent.
    // 2. Set the execUnitId based an the aperture check.
    // 3. Decrement any extra resources that were reserved. Other
    //    resources are released as normal, below.
    if (executedAs() == enums::SC_GLOBAL) {
        // no transormation for global segment
        wavefront()->execUnitId =  wavefront()->flatGmUnitId;
        if (isLoad()) {
            wavefront()->rdLmReqsInPipe--;
        } else if (isStore()) {
            wavefront()->wrLmReqsInPipe--;
        } else if (isAtomic() || isMemSync()) {
            wavefront()->wrLmReqsInPipe--;
            wavefront()->rdLmReqsInPipe--;
        } else {
            panic("Invalid memory operation!\n");
        }
    } else if (executedAs() == enums::SC_GROUP) {
        for (int lane = 0; lane < wavefront()->computeUnit->wfSize(); ++lane) {
            if (mask[lane]) {
                // flat address calculation goes here.
                // addr[lane] = segmented address
                addr[lane] = addr[lane] -
                    wavefront()->computeUnit->shader->ldsApe().base;
                assert(addr[lane] <
                  wavefront()->computeUnit->getLds().getAddrRange().size());
            }
        }
        wavefront()->execUnitId =  wavefront()->flatLmUnitId;
        wavefront()->decVMemInstsIssued();
        if (isLoad()) {
            wavefront()->rdGmReqsInPipe--;
        } else if (isStore()) {
            wavefront()->wrGmReqsInPipe--;
        } else if (isAtomic() || isMemSync()) {
            wavefront()->rdGmReqsInPipe--;
            wavefront()->wrGmReqsInPipe--;
        } else {
            panic("Invalid memory operation!\n");
        }
    } else if (executedAs() == enums::SC_PRIVATE) {
        /**
         * Flat instructions may resolve to the private segment (scratch),
         * which is backed by main memory and provides per-lane scratch
         * memory. Flat addressing uses apertures - registers that specify
         * the address range in the VA space where LDS/private memory is
         * mapped. The value of which is set by the kernel mode driver.
         * These apertures use addresses that are not used by x86 CPUs.
         * When the address of a Flat operation falls into one of the
         * apertures, the Flat operation is redirected to either LDS or
         * to the private memory segment.
         *
         * For private memory the SW runtime will allocate some space in
         * the VA space for each AQL queue. The base address of which is
         * stored in scalar registers per the AMD GPU ABI. The amd_queue_t
         * scratch_backing_memory_location provides the base address in
         * memory for the queue's private segment. Various other fields
         * loaded into register state during kernel launch specify per-WF
         * and per-work-item offsets so that individual lanes may access
         * their private segment allocation.
         *
         * For more details about flat addressing see:
         *     http://rocm-documentation.readthedocs.io/en/latest/
         *     ROCm_Compiler_SDK/ROCm-Native-ISA.html#flat-scratch
         *
         *     https://github.com/ROCm-Developer-Tools/
         *     ROCm-ComputeABI-Doc/blob/master/AMDGPU-ABI.md
         *     #flat-addressing
         */

        uint32_t numSgprs = wavefront()->maxSgprs;
        uint32_t physSgprIdx =
            wavefront()->computeUnit->registerManager->mapSgpr(wavefront(),
                                                          numSgprs - 3);
        uint32_t offset =
            wavefront()->computeUnit->srf[simdId]->read(physSgprIdx);
        physSgprIdx =
            wavefront()->computeUnit->registerManager->mapSgpr(wavefront(),
                                                          numSgprs - 4);
        uint32_t size =
            wavefront()->computeUnit->srf[simdId]->read(physSgprIdx);
        for (int lane = 0; lane < wavefront()->computeUnit->wfSize(); ++lane) {
            if (mask[lane]) {
                addr[lane] = addr[lane] + lane * size + offset +
                    wavefront()->computeUnit->shader->getHiddenPrivateBase() -
                    wavefront()->computeUnit->shader->getScratchBase();
            }
        }
        wavefront()->execUnitId =  wavefront()->flatLmUnitId;
        wavefront()->decLGKMInstsIssued();
        if (isLoad()) {
            wavefront()->rdGmReqsInPipe--;
        } else if (isStore()) {
            wavefront()->wrGmReqsInPipe--;
        } else if (isAtomic() || isMemSync()) {
            wavefront()->rdGmReqsInPipe--;
            wavefront()->wrGmReqsInPipe--;
        } else {
            panic("Invalid memory operation!\n");
        }
    } else {
        for (int lane = 0; lane < wavefront()->computeUnit->wfSize(); ++lane) {
            if (mask[lane]) {
                panic("flat addr %#llx maps to bad segment %d\n",
                      addr[lane], executedAs());
            }
        }
    }
}

TheGpuISA::ScalarRegU32
GPUDynInst::srcLiteral() const
{
    return _staticInst->srcLiteral();
}

void
GPUDynInst::updateStats()
{
    if (_staticInst->isLocalMem()) {
        // access to LDS (shared) memory
        cu->stats.dynamicLMemInstrCnt++;
    } else if (_staticInst->isFlat()) {
        cu->stats.dynamicFlatMemInstrCnt++;
    } else {
        // access to global memory

        // update PageDivergence histogram
        int number_pages_touched = cu->pagesTouched.size();
        assert(number_pages_touched);
        cu->stats.pageDivergenceDist.sample(number_pages_touched);

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
        cu->stats.dynamicGMemInstrCnt++;
    }
}

void
GPUDynInst::profileRoundTripTime(Tick currentTime, int hopId)
{
    // Only take the first measurement in the case of coalescing
    if (roundTripTime.size() > hopId)
        return;

    roundTripTime.push_back(currentTime);
}

void
GPUDynInst::profileLineAddressTime(Addr addr, Tick currentTime, int hopId)
{
    if (lineAddressTime.count(addr)) {
        if (lineAddressTime[addr].size() > hopId) {
            return;
        }

        lineAddressTime[addr].push_back(currentTime);
    } else if (hopId == 0) {
        auto addressTimeVec = std::vector<Tick> { currentTime };
        lineAddressTime.insert(std::make_pair(addr, addressTimeVec));
    }
}

} // namespace gem5
