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

GPUStaticInst::GPUStaticInst(const std::string &opcode)
    : executed_as(Enums::SC_NONE), _opcode(opcode),
      _instNum(0), _instAddr(0), srcVecOperands(-1), dstVecOperands(-1),
      srcVecDWORDs(-1), dstVecDWORDs(-1)
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

int
GPUStaticInst::numSrcVecOperands()
{
    if (srcVecOperands > -1)
        return srcVecOperands;

    srcVecOperands = 0;
    if (!isScalar()) {
        for (int k = 0; k < getNumOperands(); ++k) {
            if (isVectorRegister(k) && isSrcOperand(k))
                srcVecOperands++;
        }
    }
    return srcVecOperands;
}

int
GPUStaticInst::numDstVecOperands()
{
    if (dstVecOperands > -1)
        return dstVecOperands;

    dstVecOperands = 0;
    if (!isScalar()) {
        for (int k = 0; k < getNumOperands(); ++k) {
            if (isVectorRegister(k) && isDstOperand(k))
                dstVecOperands++;
        }
    }
    return dstVecOperands;
}

int
GPUStaticInst::numSrcVecDWORDs()
{
    if (srcVecDWORDs > -1) {
        return srcVecDWORDs;
    }

    srcVecDWORDs = 0;
    if (!isScalar()) {
        for (int i = 0; i < getNumOperands(); i++) {
            if (isVectorRegister(i) && isSrcOperand(i)) {
                int dwords = numOpdDWORDs(i);
                srcVecDWORDs += dwords;
            }
        }
    }
    return srcVecDWORDs;
}

int
GPUStaticInst::numDstVecDWORDs()
{
    if (dstVecDWORDs > -1) {
        return dstVecDWORDs;
    }

    dstVecDWORDs = 0;
    if (!isScalar()) {
        for (int i = 0; i < getNumOperands(); i++) {
            if (isVectorRegister(i) && isDstOperand(i)) {
                int dwords = numOpdDWORDs(i);
                dstVecDWORDs += dwords;
            }
        }
    }
    return dstVecDWORDs;
}

int
GPUStaticInst::numOpdDWORDs(int operandIdx)
{
    return getOperandSize(operandIdx) <= 4 ? 1
        : getOperandSize(operandIdx) / 4;
}
