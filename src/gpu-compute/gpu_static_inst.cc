/*
 * Copyright (c) 2015 Advanced Micro Devices, Inc.
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

#include "gpu-compute/gpu_static_inst.hh"

#include "debug/GPUInst.hh"

namespace gem5
{

GPUStaticInst::GPUStaticInst(const std::string &opcode)
    : executed_as(enums::SC_NONE), _opcode(opcode),
      _instNum(0), _instAddr(0), srcVecDWords(-1), dstVecDWords(-1),
      srcScalarDWords(-1), dstScalarDWords(-1), maxOpSize(-1)
{
}

const std::string&
GPUStaticInst::disassemble()
{
    if (disassembly.empty()) {
        generateDisassembly();
        assert(!disassembly.empty());
    }

    return disassembly;
}


void
GPUStaticInst::generateVirtToPhysMap(Wavefront *wf, ComputeUnit *cu,
                                     OperandInfo& op,
                                     std::vector<OperandInfo>& opVec,
                                     OpType opType)
{
    std::vector<int> virt_idxs;
    std::vector<int> phys_idxs;

    int num_dwords = op.sizeInDWords();
    int virt_idx = op.registerIndex(wf->reservedScalarRegs);

    int phys_idx = -1;
    for (int i = 0; i < num_dwords; i++) {
        if (opType == OpType::SRC_VEC || opType == OpType::DST_VEC) {
            phys_idx = cu->registerManager->mapVgpr(wf, virt_idx + i);
        } else {
            assert(opType == OpType::SRC_SCALAR ||
                   opType == OpType::DST_SCALAR);
            phys_idx = cu->registerManager->mapSgpr(wf, virt_idx + i);
        }
        virt_idxs.push_back(virt_idx + i);
        phys_idxs.push_back(phys_idx);
    }
    DPRINTF(GPUInst, "%s adding %s %s (%d->%d) operand that uses "
            "%d registers.\n", disassemble(),
            (opType == OpType::SRC_VEC || opType == OpType::DST_VEC) ?
            "vector" : "scalar",
            (opType == OpType::SRC_VEC || opType == OpType::SRC_SCALAR) ?
            "src" : "dst", virt_idxs[0], phys_idxs[0], num_dwords);

    op.setVirtToPhysMapping(virt_idxs, phys_idxs);

    opVec.emplace_back(op);
}

void
GPUStaticInst::initDynOperandInfo(Wavefront *wf, ComputeUnit *cu)
{
    for (auto& srcOp : srcOps) {
        if (srcOp.isVectorReg()) {
            generateVirtToPhysMap(wf, cu, srcOp, srcVecRegOps,
                                  OpType::SRC_VEC);
        } else if (srcOp.isScalarReg()) {
            generateVirtToPhysMap(wf, cu, srcOp, srcScalarRegOps,
                                  OpType::SRC_SCALAR);
        }
    }

    for (auto& dstOp : dstOps) {
        if (dstOp.isVectorReg()) {
            generateVirtToPhysMap(wf, cu, dstOp, dstVecRegOps,
                                  OpType::DST_VEC);
        } else if (dstOp.isScalarReg()) {
            generateVirtToPhysMap(wf, cu, dstOp, dstScalarRegOps,
                                  OpType::DST_SCALAR);
        }
    }
}

int
GPUStaticInst::numSrcVecOperands()
{
    return srcVecRegOps.size();
}

int
GPUStaticInst::numDstVecOperands()
{
    return dstVecRegOps.size();
}

int
GPUStaticInst::numSrcVecDWords()
{
    if (srcVecDWords != -1) {
        return srcVecDWords;
    }

    srcVecDWords = 0;

    for (const auto& srcOp : srcOps)
        if (srcOp.isVectorReg())
            srcVecDWords += srcOp.sizeInDWords();

    return srcVecDWords;
}

int
GPUStaticInst::numDstVecDWords()
{
    if (dstVecDWords != -1) {
        return dstVecDWords;
    }

    dstVecDWords = 0;

    for (const auto& dstOp : dstOps)
        if (dstOp.isVectorReg())
            dstVecDWords += dstOp.sizeInDWords();

    return dstVecDWords;
}

int
GPUStaticInst::numSrcScalarOperands()
{
    return srcScalarRegOps.size();
}

int
GPUStaticInst::numDstScalarOperands()
{
    return dstScalarRegOps.size();
}

int
GPUStaticInst::numSrcScalarDWords()
{
    if (srcScalarDWords != -1)
        return srcScalarDWords;

    srcScalarDWords = 0;

    for (const auto& srcOp : srcOps)
        if (srcOp.isScalarReg())
            srcScalarDWords += srcOp.sizeInDWords();

    return srcScalarDWords;
}

int
GPUStaticInst::numDstScalarDWords()
{
    if (dstScalarDWords != -1)
        return dstScalarDWords;

    dstScalarDWords = 0;

    for (const auto& dstOp : dstOps)
        if (dstOp.isScalarReg())
            dstScalarDWords += dstOp.sizeInDWords();

    return dstScalarDWords;
}

int
GPUStaticInst::maxOperandSize()
{
    if (maxOpSize != -1)
        return maxOpSize;

    maxOpSize = 0;

    for (const auto& dstOp : dstOps)
        if (dstOp.size() > maxOpSize)
            maxOpSize = dstOp.size();

    for (const auto& srcOp : srcOps)
        if (srcOp.size() > maxOpSize)
            maxOpSize = srcOp.size();

    return maxOpSize;
}

} // namespace gem5
