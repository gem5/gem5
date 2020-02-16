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

#include "enums/GPUStaticInstFlags.hh"
#include "enums/StorageClassType.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/misc.hh"

class BaseOperand;
class BaseRegOperand;
class Wavefront;

class GPUStaticInst : public GPUStaticInstFlags
{
  public:
    GPUStaticInst(const std::string &opcode);
    void instAddr(int inst_addr) { _instAddr = inst_addr; }
    int instAddr() const { return _instAddr; }
    int nextInstAddr() const { return _instAddr + instSize(); }

    void instNum(int num) { _instNum = num; }

    int instNum() { return _instNum;  }

    void ipdInstNum(int num) { _ipdInstNum = num; }

    int ipdInstNum() const { return _ipdInstNum; }

    virtual void execute(GPUDynInstPtr gpuDynInst) = 0;
    virtual void generateDisassembly() = 0;
    const std::string& disassemble();
    virtual int getNumOperands() = 0;
    virtual bool isCondRegister(int operandIndex) = 0;
    virtual bool isScalarRegister(int operandIndex) = 0;
    virtual bool isVectorRegister(int operandIndex) = 0;
    virtual bool isSrcOperand(int operandIndex) = 0;
    virtual bool isDstOperand(int operandIndex) = 0;
    virtual int getOperandSize(int operandIndex) = 0;

    virtual int getRegisterIndex(int operandIndex,
                                 GPUDynInstPtr gpuDynInst) = 0;

    virtual int numDstRegOperands() = 0;
    virtual int numSrcRegOperands() = 0;

    virtual bool isValid() const = 0;

    bool isALU() const { return _flags[ALU]; }
    bool isBranch() const { return _flags[Branch]; }
    bool isNop() const { return _flags[Nop]; }
    bool isReturn() const { return _flags[Return]; }

    bool
    isUnconditionalJump() const
    {
        return _flags[UnconditionalJump];
    }

    bool isSpecialOp() const { return _flags[SpecialOp]; }
    bool isWaitcnt() const { return _flags[Waitcnt]; }

    bool isBarrier() const { return _flags[MemBarrier]; }
    bool isMemFence() const { return _flags[MemFence]; }
    bool isMemRef() const { return _flags[MemoryRef]; }
    bool isFlat() const { return _flags[Flat]; }
    bool isLoad() const { return _flags[Load]; }
    bool isStore() const { return _flags[Store]; }

    bool
    isAtomic() const
    {
        return _flags[AtomicReturn] || _flags[AtomicNoReturn];
    }

    bool isAtomicNoRet() const { return _flags[AtomicNoReturn]; }
    bool isAtomicRet() const { return _flags[AtomicReturn]; }

    bool isScalar() const { return _flags[Scalar]; }
    bool readsSCC() const { return _flags[ReadsSCC]; }
    bool writesSCC() const { return _flags[WritesSCC]; }
    bool readsVCC() const { return _flags[ReadsVCC]; }
    bool writesVCC() const { return _flags[WritesVCC]; }

    bool isAtomicAnd() const { return _flags[AtomicAnd]; }
    bool isAtomicOr() const { return _flags[AtomicOr]; }
    bool isAtomicXor() const { return _flags[AtomicXor]; }
    bool isAtomicCAS() const { return _flags[AtomicCAS]; }
    bool isAtomicExch() const { return _flags[AtomicExch]; }
    bool isAtomicAdd() const { return _flags[AtomicAdd]; }
    bool isAtomicSub() const { return _flags[AtomicSub]; }
    bool isAtomicInc() const { return _flags[AtomicInc]; }
    bool isAtomicDec() const { return _flags[AtomicDec]; }
    bool isAtomicMax() const { return _flags[AtomicMax]; }
    bool isAtomicMin() const { return _flags[AtomicMin]; }

    bool
    isArgLoad() const
    {
        return (_flags[KernArgSegment] || _flags[ArgSegment]) && _flags[Load];
    }

    bool
    isGlobalMem() const
    {
        return _flags[MemoryRef] && (_flags[GlobalSegment] ||
               _flags[PrivateSegment] || _flags[ReadOnlySegment] ||
               _flags[SpillSegment]);
    }

    bool
    isLocalMem() const
    {
        return _flags[MemoryRef] && _flags[GroupSegment];
    }

    bool isArgSeg() const { return _flags[ArgSegment]; }
    bool isGlobalSeg() const { return _flags[GlobalSegment]; }
    bool isGroupSeg() const { return _flags[GroupSegment]; }
    bool isKernArgSeg() const { return _flags[KernArgSegment]; }
    bool isPrivateSeg() const { return _flags[PrivateSegment]; }
    bool isReadOnlySeg() const { return _flags[ReadOnlySegment]; }
    bool isSpillSeg() const { return _flags[SpillSegment]; }

    bool isWorkitemScope() const { return _flags[WorkitemScope]; }
    bool isWavefrontScope() const { return _flags[WavefrontScope]; }
    bool isWorkgroupScope() const { return _flags[WorkgroupScope]; }
    bool isDeviceScope() const { return _flags[DeviceScope]; }
    bool isSystemScope() const { return _flags[SystemScope]; }
    bool isNoScope() const { return _flags[NoScope]; }

    bool isRelaxedOrder() const { return _flags[RelaxedOrder]; }
    bool isAcquire() const { return _flags[Acquire]; }
    bool isRelease() const { return _flags[Release]; }
    bool isAcquireRelease() const { return _flags[AcquireRelease]; }
    bool isNoOrder() const { return _flags[NoOrder]; }

    /**
     * Coherence domain of a memory instruction. Only valid for
     * machine ISA. The coherence domain specifies where it is
     * possible to perform memory synchronization, e.g., acquire
     * or release, from the shader kernel.
     *
     * isGloballyCoherent(): returns true if kernel is sharing memory
     * with other work-items on the same device (GPU)
     *
     * isSystemCoherent(): returns true if kernel is sharing memory
     * with other work-items on a different device (GPU) or the host (CPU)
     */
    bool isGloballyCoherent() const { return _flags[GloballyCoherent]; }
    bool isSystemCoherent() const { return _flags[SystemCoherent]; }

    virtual int instSize() const = 0;

    // only used for memory instructions
    virtual void
    initiateAcc(GPUDynInstPtr gpuDynInst)
    {
        fatal("calling initiateAcc() on a non-memory instruction.\n");
    }

    // only used for memory instructions
    virtual void
    completeAcc(GPUDynInstPtr gpuDynInst)
    {
        fatal("calling completeAcc() on a non-memory instruction.\n");
    }

    virtual uint32_t getTargetPc() { return 0; }

    static uint64_t dynamic_id_count;

    // For flat memory accesses
    Enums::StorageClassType executed_as;

    void setFlag(Flags flag) { _flags[flag] = true; }

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

  protected:
    const std::string opcode;
    std::string disassembly;
    int _instNum;
    int _instAddr;
    /**
     * Identifier of the immediate post-dominator instruction.
     */
    int _ipdInstNum;

    std::bitset<Num_Flags> _flags;
};

class KernelLaunchStaticInst : public GPUStaticInst
{
  public:
    KernelLaunchStaticInst() : GPUStaticInst("kernel_launch")
    {
        setFlag(Nop);
        setFlag(Scalar);
        setFlag(Acquire);
        setFlag(SystemScope);
        setFlag(GlobalSegment);
    }

    void
    execute(GPUDynInstPtr gpuDynInst) override
    {
        fatal("kernel launch instruction should not be executed\n");
    }

    void
    generateDisassembly() override
    {
        disassembly = opcode;
    }

    int getNumOperands() override { return 0; }
    bool isCondRegister(int operandIndex) override { return false; }
    bool isScalarRegister(int operandIndex) override { return false; }
    bool isVectorRegister(int operandIndex) override { return false; }
    bool isSrcOperand(int operandIndex) override { return false; }
    bool isDstOperand(int operandIndex) override { return false; }
    int getOperandSize(int operandIndex) override { return 0; }

    int
    getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
    {
        return 0;
    }

    int numDstRegOperands() override { return 0; }
    int numSrcRegOperands() override { return 0; }
    bool isValid() const override { return true; }
    int instSize() const override { return 0; }
};

#endif // __GPU_STATIC_INST_HH__
