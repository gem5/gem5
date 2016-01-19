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

#ifndef __GPU_STATIC_INST_HH__
#define __GPU_STATIC_INST_HH__

/*
 * @file gpu_static_inst.hh
 *
 * Defines the base class representing static instructions for the GPU. The
 * instructions are "static" because they contain no dynamic instruction
 * information. GPUStaticInst corresponds to the StaticInst class for the CPU
 * models.
 */

#include <cstdint>
#include <string>

#include "enums/OpType.hh"
#include "enums/StorageClassType.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/misc.hh"

class BaseOperand;
class BaseRegOperand;
class Wavefront;

class GPUStaticInst
{
  public:
    GPUStaticInst(const std::string &opcode);

    void instNum(int num) { _instNum = num; }

    int instNum() { return _instNum;  }

    void ipdInstNum(int num) { _ipdInstNum = num; }

    int ipdInstNum() const { return _ipdInstNum; }

    virtual void execute(GPUDynInstPtr gpuDynInst) = 0;
    virtual void generateDisassembly() = 0;
    virtual const std::string &disassemble() = 0;
    virtual int getNumOperands() = 0;
    virtual bool isCondRegister(int operandIndex) = 0;
    virtual bool isScalarRegister(int operandIndex) = 0;
    virtual bool isVectorRegister(int operandIndex) = 0;
    virtual bool isSrcOperand(int operandIndex) = 0;
    virtual bool isDstOperand(int operandIndex) = 0;
    virtual int getOperandSize(int operandIndex) = 0;
    virtual int getRegisterIndex(int operandIndex) = 0;
    virtual int numDstRegOperands() = 0;
    virtual int numSrcRegOperands() = 0;

    /*
     * Most instructions (including all HSAIL instructions)
     * are vector ops, so _scalarOp will be false by default.
     * Derived instruction objects that are scalar ops must
     * set _scalarOp to true in their constructors.
     */
    bool scalarOp() const { return _scalarOp; }

    virtual bool isLocalMem() const
    {
        fatal("calling isLocalMem() on non-memory instruction.\n");

        return false;
    }

    bool isArgLoad() { return false; }
    virtual uint32_t instSize() = 0;

    // only used for memory instructions
    virtual void
    initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        fatal("calling initiateAcc() on a non-memory instruction.\n");
    }

    virtual uint32_t getTargetPc() { return 0; }

    /**
     * Query whether the instruction is an unconditional jump i.e., the jump
     * is always executed because there is no condition to be evaluated.
     *
     * If the instruction is not of branch type, the result is always false.
     *
     * @return True if the instruction is an unconditional jump.
     */
    virtual bool unconditionalJumpInstruction() { return false; }

    static uint64_t dynamic_id_count;

    Enums::OpType o_type;
    // For flat memory accesses
    Enums::StorageClassType executed_as;

  protected:
    virtual void
    execLdAcq(GPUDynInstPtr gpuDynInst)
    {
        fatal("calling execLdAcq() on a non-load instruction.\n");
    }

    virtual void
    execSt(GPUDynInstPtr gpuDynInst)
    {
        fatal("calling execLdAcq() on a non-load instruction.\n");
    }

    virtual void
    execAtomic(GPUDynInstPtr gpuDynInst)
    {
        fatal("calling execAtomic() on a non-atomic instruction.\n");
    }

    virtual void
    execAtomicAcq(GPUDynInstPtr gpuDynInst)
    {
        fatal("calling execAtomicAcq() on a non-atomic instruction.\n");
    }

    const std::string opcode;
    std::string disassembly;
    int _instNum;
    /**
     * Identifier of the immediate post-dominator instruction.
     */
    int _ipdInstNum;

    bool _scalarOp;
};

#endif // __GPU_STATIC_INST_HH__
