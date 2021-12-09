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
#include <vector>

#include "enums/GPUStaticInstFlags.hh"
#include "enums/StorageClassType.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/misc.hh"
#include "gpu-compute/operand_info.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

class BaseOperand;
class BaseRegOperand;

class GPUStaticInst : public GPUStaticInstFlags
{
  public:
    GPUStaticInst(const std::string &opcode);
    virtual ~GPUStaticInst() { }
    void instAddr(int inst_addr) { _instAddr = inst_addr; }
    int instAddr() const { return _instAddr; }
    int nextInstAddr() const { return _instAddr + instSize(); }

    void instNum(int num) { _instNum = num; }

    int instNum() { return _instNum;  }

    void ipdInstNum(int num) { _ipdInstNum = num; }

    int ipdInstNum() const { return _ipdInstNum; }

    virtual TheGpuISA::ScalarRegU32 srcLiteral() const { return 0; }

    void initDynOperandInfo(Wavefront *wf, ComputeUnit *cu);

    virtual void initOperandInfo() = 0;
    virtual void execute(GPUDynInstPtr gpuDynInst) = 0;
    virtual void generateDisassembly() = 0;
    const std::string& disassemble();
    virtual int getNumOperands() = 0;
    virtual bool isFlatScratchRegister(int opIdx) = 0;
    virtual bool isExecMaskRegister(int opIdx) = 0;
    virtual int getOperandSize(int operandIndex) = 0;

    virtual int numDstRegOperands() = 0;
    virtual int numSrcRegOperands() = 0;

    int numSrcVecOperands();
    int numDstVecOperands();
    int numSrcVecDWords();
    int numDstVecDWords();

    int numSrcScalarOperands();
    int numDstScalarOperands();
    int numSrcScalarDWords();
    int numDstScalarDWords();

    int maxOperandSize();

    virtual int coalescerTokenCount() const { return 0; }

    bool isALU() const { return _flags[ALU]; }
    bool isBranch() const { return _flags[Branch]; }
    bool isCondBranch() const { return _flags[CondBranch]; }
    bool isNop() const { return _flags[Nop]; }
    bool isReturn() const { return _flags[Return]; }
    bool isEndOfKernel() const { return _flags[EndOfKernel]; }
    bool isKernelLaunch() const { return _flags[KernelLaunch]; }
    bool isSDWAInst() const { return _flags[IsSDWA]; }
    bool isDPPInst() const { return _flags[IsDPP]; }

    bool
    isUnconditionalJump() const
    {
        return _flags[UnconditionalJump];
    }

    bool isSpecialOp() const { return _flags[SpecialOp]; }
    bool isWaitcnt() const { return _flags[Waitcnt]; }
    bool isSleep() const { return _flags[Sleep]; }

    bool isBarrier() const { return _flags[MemBarrier]; }
    bool isMemSync() const { return _flags[MemSync]; }
    bool isMemRef() const { return _flags[MemoryRef]; }
    bool isFlat() const { return _flags[Flat]; }
    bool isFlatGlobal() const { return _flags[FlatGlobal]; }
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
    // Identify instructions that implicitly read the Execute mask
    // as a source operand but not to dictate which threads execute.
    bool readsEXEC() const { return _flags[ReadsEXEC]; }
    bool writesEXEC() const { return _flags[WritesEXEC]; }
    bool readsMode() const { return _flags[ReadsMode]; }
    bool writesMode() const { return _flags[WritesMode]; }
    bool ignoreExec() const { return _flags[IgnoreExec]; }

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
               _flags[SpillSegment] || _flags[FlatGlobal]);
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

    /**
     * Coherence domain of a memory instruction. The coherence domain
     * specifies where it is possible to perform memory synchronization
     * (e.g., acquire or release) from the shader kernel.
     *
     * isGloballyCoherent(): returns true if WIs share same device
     * isSystemCoherent(): returns true if WIs or threads in different
     *                     devices share memory
     *
     */
    bool isGloballyCoherent() const { return _flags[GloballyCoherent]; }
    bool isSystemCoherent() const { return _flags[SystemCoherent]; }

    // Floating-point instructions
    bool isF16() const { return _flags[F16]; }
    bool isF32() const { return _flags[F32]; }
    bool isF64() const { return _flags[F64]; }

    // FMA, MAC, MAD instructions
    bool isFMA() const { return _flags[FMA]; }
    bool isMAC() const { return _flags[MAC]; }
    bool isMAD() const { return _flags[MAD]; }

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
    enums::StorageClassType executed_as;

    void setFlag(Flags flag) {
        _flags[flag] = true;

        if (isGroupSeg()) {
            executed_as = enums::SC_GROUP;
        } else if (isGlobalSeg()) {
            executed_as = enums::SC_GLOBAL;
        } else if (isPrivateSeg()) {
            executed_as = enums::SC_PRIVATE;
        } else if (isSpillSeg()) {
            executed_as = enums::SC_SPILL;
        } else if (isReadOnlySeg()) {
            executed_as = enums::SC_READONLY;
        } else if (isKernArgSeg()) {
            executed_as = enums::SC_KERNARG;
        } else if (isArgSeg()) {
            executed_as = enums::SC_ARG;
        }
    }
    const std::string& opcode() const { return _opcode; }

    const std::vector<OperandInfo>& srcOperands() const { return srcOps; }
    const std::vector<OperandInfo>& dstOperands() const { return dstOps; }

    const std::vector<OperandInfo>&
    srcVecRegOperands() const
    {
        return srcVecRegOps;
    }

    const std::vector<OperandInfo>&
    dstVecRegOperands() const
    {
        return dstVecRegOps;
    }

    const std::vector<OperandInfo>&
    srcScalarRegOperands() const
    {
        return srcScalarRegOps;
    }

    const std::vector<OperandInfo>&
    dstScalarRegOperands() const
    {
        return dstScalarRegOps;
    }

    // These next 2 lines are used in initDynOperandInfo to let the lambda
    // function work
    typedef int (RegisterManager::*MapRegFn)(Wavefront *, int);
    enum OpType { SRC_VEC, SRC_SCALAR, DST_VEC, DST_SCALAR };

  protected:
    const std::string _opcode;
    std::string disassembly;
    int _instNum;
    int _instAddr;
    std::vector<OperandInfo> srcOps;
    std::vector<OperandInfo> dstOps;

  private:
    int srcVecDWords;
    int dstVecDWords;
    int srcScalarDWords;
    int dstScalarDWords;
    int maxOpSize;

    std::vector<OperandInfo> srcVecRegOps;
    std::vector<OperandInfo> dstVecRegOps;
    std::vector<OperandInfo> srcScalarRegOps;
    std::vector<OperandInfo> dstScalarRegOps;

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
        setFlag(KernelLaunch);
        setFlag(MemSync);
        setFlag(Scalar);
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
        disassembly = _opcode;
    }

    void initOperandInfo() override { return; }
    int getNumOperands() override { return 0; }
    bool isFlatScratchRegister(int opIdx) override { return false; }
    // return true if the Execute mask is explicitly used as a source
    // register operand
    bool isExecMaskRegister(int opIdx) override { return false; }
    int getOperandSize(int operandIndex) override { return 0; }

    int numDstRegOperands() override { return 0; }
    int numSrcRegOperands() override { return 0; }
    int instSize() const override { return 0; }
};

} // namespace gem5

#endif // __GPU_STATIC_INST_HH__
