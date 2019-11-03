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

#ifndef __ARCH_HSAIL_INSTS_BRANCH_HH__
#define __ARCH_HSAIL_INSTS_BRANCH_HH__

#include "arch/hsail/insts/gpu_static_inst.hh"
#include "arch/hsail/operand.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/wavefront.hh"

namespace HsailISA
{

    // The main difference between a direct branch and an indirect branch
    // is whether the target is a register or a label, so we can share a
    // lot of code if we template the base implementation on that type.
    template<typename TargetType>
    class BrnInstBase : public HsailGPUStaticInst
    {
    public:
        void generateDisassembly() override;

        Brig::BrigWidth8_t width;
        TargetType target;

        BrnInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj)
           : HsailGPUStaticInst(obj, "brn")
        {
            setFlag(Branch);
            setFlag(UnconditionalJump);
            width = ((Brig::BrigInstBr*)ib)->width;
            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            target.init(op_offs, obj);
        }

        uint32_t getTargetPc()  override { return target.getTarget(0, 0); }

        bool isVectorRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.isScalarRegister();
        }

        bool isSrcOperand(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return true;
        }

        bool isDstOperand(int operandIndex) override {
            return false;
        }

        int getOperandSize(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.opSize();
        }

        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.regIndex();
        }

        int getNumOperands() override {
            return 1;
        }

        void execute(GPUDynInstPtr gpuDynInst) override;
    };

    template<typename TargetType>
    void
    BrnInstBase<TargetType>::generateDisassembly()
    {
        std::string widthClause;

        if (width != 1) {
            widthClause = csprintf("_width(%d)", width);
        }

        disassembly = csprintf("%s%s %s", opcode, widthClause,
                               target.disassemble());
    }

    template<typename TargetType>
    void
    BrnInstBase<TargetType>::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        if (getTargetPc() == w->rpc()) {
            w->popFromReconvergenceStack();
        } else {
            // Rpc and execution mask remain the same
            w->pc(getTargetPc());
        }
    }

    class BrnDirectInst : public BrnInstBase<LabelOperand>
    {
      public:
        BrnDirectInst(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : BrnInstBase<LabelOperand>(ib, obj)
        {
        }
        int numSrcRegOperands() { return 0; }
        int numDstRegOperands() { return 0; }
    };

    class BrnIndirectInst : public BrnInstBase<SRegOperand>
    {
      public:
        BrnIndirectInst(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : BrnInstBase<SRegOperand>(ib, obj)
        {
        }
        int numSrcRegOperands() { return target.isVectorRegister(); }
        int numDstRegOperands() { return 0; }
    };

    GPUStaticInst* decodeBrn(const Brig::BrigInstBase *ib,
                             const BrigObject *obj);

    template<typename TargetType>
    class CbrInstBase : public HsailGPUStaticInst
    {
      public:
        void generateDisassembly() override;

        Brig::BrigWidth8_t width;
        CRegOperand cond;
        TargetType target;

        CbrInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj)
           : HsailGPUStaticInst(obj, "cbr")
        {
            setFlag(Branch);
            width = ((Brig::BrigInstBr *)ib)->width;
            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            cond.init(op_offs, obj);
            op_offs = obj->getOperandPtr(ib->operands, 1);
            target.init(op_offs, obj);
        }

        uint32_t getTargetPc() override { return target.getTarget(0, 0); }

        void execute(GPUDynInstPtr gpuDynInst) override;
        // Assumption: Target is operand 0, Condition Register is operand 1
        bool isVectorRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            if (!operandIndex)
                return target.isVectorRegister();
            else
                return false;
        }
        bool isCondRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            if (!operandIndex)
                return target.isCondRegister();
            else
                return true;
        }
        bool isScalarRegister(int operandIndex) override {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return target.isScalarRegister();
            else
                return false;
        }
        bool isSrcOperand(int operandIndex) override {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex == 0)
                return true;
            return false;
        }
        // both Condition Register and Target are source operands
        bool isDstOperand(int operandIndex) override {
            return false;
        }
        int getOperandSize(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            if (!operandIndex)
                return target.opSize();
            else
                return 1;
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            if (!operandIndex)
                return target.regIndex();
            else
                return -1;
         }

        // Operands = Target, Condition Register
        int getNumOperands() override {
            return 2;
        }
    };

    template<typename TargetType>
    void
    CbrInstBase<TargetType>::generateDisassembly()
    {
        std::string widthClause;

        if (width != 1) {
            widthClause = csprintf("_width(%d)", width);
        }

        disassembly = csprintf("%s%s %s,%s", opcode, widthClause,
                               cond.disassemble(), target.disassemble());
    }

    template<typename TargetType>
    void
    CbrInstBase<TargetType>::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        const uint32_t curr_pc M5_VAR_USED = w->pc();
        const uint32_t curr_rpc = w->rpc();
        const VectorMask curr_mask = w->execMask();

        /**
         * TODO: can we move this pop outside the instruction, and
         * into the wavefront?
         */
        w->popFromReconvergenceStack();

        // immediate post-dominator instruction
        const uint32_t rpc = static_cast<uint32_t>(ipdInstNum());
        if (curr_rpc != rpc) {
            w->pushToReconvergenceStack(rpc, curr_rpc, curr_mask);
        }

        // taken branch
        const uint32_t true_pc = getTargetPc();
        VectorMask true_mask;
        for (unsigned int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
            true_mask[lane] = cond.get<bool>(w, lane) & curr_mask[lane];
        }

        // not taken branch
        const uint32_t false_pc = nextInstAddr();
        assert(true_pc != false_pc);
        if (false_pc != rpc && true_mask.count() < curr_mask.count()) {
            VectorMask false_mask = curr_mask & ~true_mask;
            w->pushToReconvergenceStack(false_pc, rpc, false_mask);
        }

        if (true_pc != rpc && true_mask.count()) {
            w->pushToReconvergenceStack(true_pc, rpc, true_mask);
        }
        assert(w->pc() != curr_pc);
    }


    class CbrDirectInst : public CbrInstBase<LabelOperand>
    {
      public:
        CbrDirectInst(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : CbrInstBase<LabelOperand>(ib, obj)
        {
        }
        // the source operand of a conditional branch is a Condition
        // Register which is not stored in the VRF
        // so we do not count it as a source-register operand
        // even though, formally, it is one.
        int numSrcRegOperands() { return 0; }
        int numDstRegOperands() { return 0; }
    };

    class CbrIndirectInst : public CbrInstBase<SRegOperand>
    {
      public:
        CbrIndirectInst(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : CbrInstBase<SRegOperand>(ib, obj)
        {
        }
        // one source operand of the conditional indirect branch is a Condition
        // register which is not stored in the VRF so we do not count it
        // as a source-register operand even though, formally, it is one.
        int numSrcRegOperands() { return target.isVectorRegister(); }
        int numDstRegOperands() { return 0; }
    };

    GPUStaticInst* decodeCbr(const Brig::BrigInstBase *ib,
                             const BrigObject *obj);

    template<typename TargetType>
    class BrInstBase : public HsailGPUStaticInst
    {
      public:
        void generateDisassembly() override;

        ImmOperand<uint32_t> width;
        TargetType target;

        BrInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj)
           : HsailGPUStaticInst(obj, "br")
        {
            setFlag(Branch);
            setFlag(UnconditionalJump);
            width.init(((Brig::BrigInstBr *)ib)->width, obj);
            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            target.init(op_offs, obj);
        }

        uint32_t getTargetPc() override { return target.getTarget(0, 0); }

        void execute(GPUDynInstPtr gpuDynInst) override;
        bool isVectorRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex) override {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return true;
        }
        bool isDstOperand(int operandIndex) override { return false; }
        int getOperandSize(int operandIndex) override {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.opSize();
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return target.regIndex();
        }
        int getNumOperands() override { return 1; }
    };

    template<typename TargetType>
    void
    BrInstBase<TargetType>::generateDisassembly()
    {
        std::string widthClause;

        if (width.bits != 1) {
            widthClause = csprintf("_width(%d)", width.bits);
        }

        disassembly = csprintf("%s%s %s", opcode, widthClause,
                               target.disassemble());
    }

    template<typename TargetType>
    void
    BrInstBase<TargetType>::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        if (getTargetPc() == w->rpc()) {
            w->popFromReconvergenceStack();
        } else {
            // Rpc and execution mask remain the same
            w->pc(getTargetPc());
        }
    }

    class BrDirectInst : public BrInstBase<LabelOperand>
    {
      public:
        BrDirectInst(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : BrInstBase<LabelOperand>(ib, obj)
        {
        }

        int numSrcRegOperands() { return 0; }
        int numDstRegOperands() { return 0; }
    };

    class BrIndirectInst : public BrInstBase<SRegOperand>
    {
      public:
        BrIndirectInst(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : BrInstBase<SRegOperand>(ib, obj)
        {
        }
        int numSrcRegOperands() { return target.isVectorRegister(); }
        int numDstRegOperands() { return 0; }
    };

    GPUStaticInst* decodeBr(const Brig::BrigInstBase *ib,
                            const BrigObject *obj);
} // namespace HsailISA

#endif // __ARCH_HSAIL_INSTS_BRANCH_HH__
