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

#ifndef __ARCH_HSAIL_INSTS_MEM_HH__
#define __ARCH_HSAIL_INSTS_MEM_HH__

#include <type_traits>

#include "arch/hsail/insts/decl.hh"
#include "arch/hsail/insts/gpu_static_inst.hh"
#include "arch/hsail/operand.hh"
#include "gpu-compute/compute_unit.hh"

namespace HsailISA
{
    class MemInst
    {
      public:
        MemInst() : size(0), addr_operand(nullptr) { }

        MemInst(Enums::MemType m_type)
        {
            if (m_type == Enums::M_U64 ||
                m_type == Enums::M_S64 ||
                m_type == Enums::M_F64) {
                size = 8;
            } else if (m_type == Enums::M_U32 ||
                       m_type == Enums::M_S32 ||
                       m_type == Enums::M_F32) {
                size = 4;
            } else if (m_type == Enums::M_U16 ||
                       m_type == Enums::M_S16 ||
                       m_type == Enums::M_F16) {
                size = 2;
            } else {
                size = 1;
            }

            addr_operand = nullptr;
        }

        void
        init_addr(AddrOperandBase *_addr_operand)
        {
            addr_operand = _addr_operand;
        }

      private:
        int size;
        AddrOperandBase *addr_operand;

      public:
        int getMemOperandSize() { return size; }
        AddrOperandBase *getAddressOperand() { return addr_operand; }
    };

    template<typename DestOperandType, typename AddrOperandType>
    class LdaInstBase : public HsailGPUStaticInst
    {
      public:
        typename DestOperandType::DestOperand dest;
        AddrOperandType addr;

        LdaInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj,
                    const char *_opcode)
           : HsailGPUStaticInst(obj, _opcode)
        {
            using namespace Brig;

            setFlag(ALU);

            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            dest.init(op_offs, obj);
            op_offs = obj->getOperandPtr(ib->operands, 1);
            addr.init(op_offs, obj);
        }

        int numSrcRegOperands() override
        { return(this->addr.isVectorRegister()); }
        int numDstRegOperands() override
        { return dest.isVectorRegister(); }
        bool isVectorRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.isVectorRegister() :
                   this->addr.isVectorRegister());
        }
        bool isCondRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.isCondRegister() :
                   this->addr.isCondRegister());
        }
        bool isScalarRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.isScalarRegister() :
                   this->addr.isScalarRegister());
        }
        bool isSrcOperand(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex > 0)
                return(this->addr.isVectorRegister());
            return false;
        }
        bool isDstOperand(int operandIndex) override {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return(operandIndex == 0);
        }
        int getOperandSize(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.opSize() :
                   this->addr.opSize());
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.regIndex() :
                   this->addr.regIndex());
        }
        int getNumOperands() override
        {
            if (this->addr.isVectorRegister())
                return 2;
            return 1;
        }
    };

    template<typename DestDataType, typename AddrOperandType>
    class LdaInst :
        public LdaInstBase<typename DestDataType::OperandType, AddrOperandType>,
        public MemInst
    {
      public:
        void generateDisassembly();

        LdaInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                        const char *_opcode)
            : LdaInstBase<typename DestDataType::OperandType,
                          AddrOperandType>(ib, obj, _opcode)
        {
            init_addr(&this->addr);
        }

        void execute(GPUDynInstPtr gpuDynInst);
    };

    template<typename DataType>
    GPUStaticInst*
    decodeLda(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        unsigned op_offs = obj->getOperandPtr(ib->operands, 1);
        BrigRegOperandInfo regDataType = findRegDataType(op_offs, obj);

        if (regDataType.kind == Brig::BRIG_KIND_OPERAND_ADDRESS) {
            return new LdaInst<DataType, NoRegAddrOperand>(ib, obj, "ldas");
        } else if (regDataType.kind == Brig::BRIG_KIND_OPERAND_REGISTER) {
            // V2/V4 not allowed
            switch (regDataType.regKind) {
              case Brig::BRIG_REGISTER_KIND_SINGLE:
                return new LdaInst<DataType, SRegAddrOperand>(ib, obj, "ldas");
              case Brig::BRIG_REGISTER_KIND_DOUBLE:
                return new LdaInst<DataType, DRegAddrOperand>(ib, obj, "ldas");
              default:
                fatal("Bad ldas register operand type %d\n", regDataType.type);
            }
        } else {
            fatal("Bad ldas register operand kind %d\n", regDataType.kind);
        }
    }

    template<typename MemOperandType, typename DestOperandType,
             typename AddrOperandType>
    class LdInstBase : public HsailGPUStaticInst
    {
      public:
        Brig::BrigWidth8_t width;
        typename DestOperandType::DestOperand dest;
        AddrOperandType addr;

        Brig::BrigSegment segment;
        Brig::BrigMemoryOrder memoryOrder;
        Brig::BrigMemoryScope memoryScope;
        unsigned int equivClass;

        LdInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj,
                   const char *_opcode)
           : HsailGPUStaticInst(obj, _opcode)
        {
            using namespace Brig;

            setFlag(MemoryRef);
            setFlag(Load);

            if (ib->opcode == BRIG_OPCODE_LD) {
                const BrigInstMem *ldst = (const BrigInstMem*)ib;

                segment = (BrigSegment)ldst->segment;
                memoryOrder = BRIG_MEMORY_ORDER_NONE;
                memoryScope = BRIG_MEMORY_SCOPE_NONE;
                equivClass = ldst->equivClass;

                width = ldst->width;
                unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
                const Brig::BrigOperand *brigOp = obj->getOperand(op_offs);
                if (brigOp->kind == BRIG_KIND_OPERAND_REGISTER)
                    dest.init(op_offs, obj);

                op_offs = obj->getOperandPtr(ib->operands, 1);
                addr.init(op_offs, obj);
            } else {
                const BrigInstAtomic *at = (const BrigInstAtomic*)ib;

                segment = (BrigSegment)at->segment;
                memoryOrder = (BrigMemoryOrder)at->memoryOrder;
                memoryScope = (BrigMemoryScope)at->memoryScope;
                equivClass = 0;

                width = BRIG_WIDTH_1;
                unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
                const Brig::BrigOperand *brigOp = obj->getOperand(op_offs);

                if (brigOp->kind == BRIG_KIND_OPERAND_REGISTER)
                    dest.init(op_offs, obj);

                op_offs = obj->getOperandPtr(ib->operands,1);
                addr.init(op_offs, obj);
            }

            switch (memoryOrder) {
              case BRIG_MEMORY_ORDER_NONE:
                setFlag(NoOrder);
                break;
              case BRIG_MEMORY_ORDER_RELAXED:
                setFlag(RelaxedOrder);
                break;
              case BRIG_MEMORY_ORDER_SC_ACQUIRE:
                setFlag(Acquire);
                break;
              case BRIG_MEMORY_ORDER_SC_RELEASE:
                setFlag(Release);
                break;
              case BRIG_MEMORY_ORDER_SC_ACQUIRE_RELEASE:
                setFlag(AcquireRelease);
                break;
              default:
                fatal("LdInst has bad memory order type\n");
            }

            switch (memoryScope) {
              case BRIG_MEMORY_SCOPE_NONE:
                setFlag(NoScope);
                break;
              case BRIG_MEMORY_SCOPE_WORKITEM:
                setFlag(WorkitemScope);
                break;
              case BRIG_MEMORY_SCOPE_WORKGROUP:
                setFlag(WorkgroupScope);
                break;
              case BRIG_MEMORY_SCOPE_AGENT:
                setFlag(DeviceScope);
                break;
              case BRIG_MEMORY_SCOPE_SYSTEM:
                setFlag(SystemScope);
                break;
              default:
                fatal("LdInst has bad memory scope type\n");
            }

            switch (segment) {
              case BRIG_SEGMENT_GLOBAL:
                setFlag(GlobalSegment);
                break;
              case BRIG_SEGMENT_GROUP:
                setFlag(GroupSegment);
                break;
              case BRIG_SEGMENT_PRIVATE:
                setFlag(PrivateSegment);
                break;
              case BRIG_SEGMENT_READONLY:
                setFlag(ReadOnlySegment);
                break;
              case BRIG_SEGMENT_SPILL:
                setFlag(SpillSegment);
                break;
              case BRIG_SEGMENT_FLAT:
                setFlag(Flat);
                break;
              case BRIG_SEGMENT_KERNARG:
                setFlag(KernArgSegment);
                break;
              case BRIG_SEGMENT_ARG:
                setFlag(ArgSegment);
                break;
              default:
                panic("Ld: segment %d not supported\n", segment);
            }
        }

        int numSrcRegOperands() override
        { return(this->addr.isVectorRegister()); }
        int numDstRegOperands() override { return dest.isVectorRegister(); }
        int getNumOperands() override
        {
            if (this->addr.isVectorRegister())
                return 2;
            else
                return 1;
        }
        bool isVectorRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.isVectorRegister() :
                   this->addr.isVectorRegister());
        }
        bool isCondRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.isCondRegister() :
                   this->addr.isCondRegister());
        }
        bool isScalarRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.isScalarRegister() :
                   this->addr.isScalarRegister());
        }
        bool isSrcOperand(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex > 0)
                return(this->addr.isVectorRegister());
            return false;
        }
        bool isDstOperand(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return(operandIndex == 0);
        }
        int getOperandSize(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.opSize() :
                   this->addr.opSize());
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return((operandIndex == 0) ? dest.regIndex() :
                   this->addr.regIndex());
        }
    };

    template<typename MemDataType, typename DestDataType,
             typename AddrOperandType>
    class LdInst :
        public LdInstBase<typename MemDataType::CType,
                          typename DestDataType::OperandType, AddrOperandType>,
        public MemInst
    {
        typename DestDataType::OperandType::DestOperand dest_vect[4];
        uint16_t num_dest_operands;
        void generateDisassembly() override;

      public:
        LdInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
               const char *_opcode)
            : LdInstBase<typename MemDataType::CType,
                         typename DestDataType::OperandType,
                         AddrOperandType>(ib, obj, _opcode),
              MemInst(MemDataType::memType)
        {
            init_addr(&this->addr);

            unsigned op_offs = obj->getOperandPtr(ib->operands,0);
            const Brig::BrigOperand *brigOp = obj->getOperand(op_offs);

            if (brigOp->kind == Brig::BRIG_KIND_OPERAND_OPERAND_LIST) {
                const Brig::BrigOperandOperandList *brigRegVecOp =
                    (const Brig::BrigOperandOperandList*)brigOp;

                num_dest_operands =
                    *((unsigned*)obj->getData(brigRegVecOp->elements)) / 4;

                assert(num_dest_operands <= 4);
            } else {
                num_dest_operands = 1;
            }

            if (num_dest_operands > 1) {
                assert(brigOp->kind == Brig::BRIG_KIND_OPERAND_OPERAND_LIST);

                for (int i = 0; i < num_dest_operands; ++i) {
                    dest_vect[i].init_from_vect(op_offs, obj, i);
                }
            }
        }

        void
        initiateAcc(GPUDynInstPtr gpuDynInst) override
        {
            typedef typename MemDataType::CType c0;

            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            if (num_dest_operands > 1) {
                for (int i = 0; i < gpuDynInst->computeUnit()->wfSize(); ++i)
                    if (gpuDynInst->exec_mask[i])
                        gpuDynInst->statusVector.push_back(num_dest_operands);
                    else
                        gpuDynInst->statusVector.push_back(0);
            }

            for (int k = 0; k < num_dest_operands; ++k) {

                c0 *d = &((c0*)gpuDynInst->d_data)
                    [k * gpuDynInst->computeUnit()->wfSize()];

                for (int i = 0; i < gpuDynInst->computeUnit()->wfSize(); ++i) {
                    if (gpuDynInst->exec_mask[i]) {
                        Addr vaddr = gpuDynInst->addr[i] + k * sizeof(c0);

                        if (this->isLocalMem()) {
                            // load from shared memory
                            *d = gpuDynInst->wavefront()->ldsChunk->
                                read<c0>(vaddr);
                        } else {
                            RequestPtr req = std::make_shared<Request>(0,
                                vaddr, sizeof(c0), 0,
                                gpuDynInst->computeUnit()->masterId(),
                                0, gpuDynInst->wfDynId);

                            gpuDynInst->setRequestFlags(req);
                            PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
                            pkt->dataStatic(d);

                            if (gpuDynInst->computeUnit()->shader->
                                separate_acquire_release &&
                                gpuDynInst->isAcquire()) {
                                // if this load has acquire semantics,
                                // set the response continuation function
                                // to perform an Acquire request
                                gpuDynInst->execContinuation =
                                    &GPUStaticInst::execLdAcq;

                                gpuDynInst->useContinuation = true;
                            } else {
                                // the request will be finished when
                                // the load completes
                                gpuDynInst->useContinuation = false;
                            }
                            // translation is performed in sendRequest()
                            gpuDynInst->computeUnit()->sendRequest(gpuDynInst,
                                                                   i, pkt);
                        }
                    }
                    ++d;
                }
            }

            gpuDynInst->updateStats();
        }

        void
        completeAcc(GPUDynInstPtr gpuDynInst) override
        {
            typedef typename MemDataType::CType c1;

            constexpr bool is_vt_32 = DestDataType::vgprType == VT_32;

            /**
              * this code essentially replaces the long if-else chain
              * that was in used GlobalMemPipeline::exec() to infer the
              * size (single/double) and type (floating point/integer) of
              * the destination register. this is needed for load
              * instructions because the loaded value and the
              * destination type can be of different sizes, and we also
              * need to know if the value we're writing back is floating
              * point and signed/unsigned, so we can properly cast the
              * writeback value
              */
            typedef typename std::conditional<is_vt_32,
                typename std::conditional<std::is_floating_point<c1>::value,
                    float, typename std::conditional<std::is_signed<c1>::value,
                    int32_t, uint32_t>::type>::type,
                typename std::conditional<std::is_floating_point<c1>::value,
                    double, typename std::conditional<std::is_signed<c1>::value,
                    int64_t, uint64_t>::type>::type>::type c0;


            Wavefront *w = gpuDynInst->wavefront();

            std::vector<uint32_t> regVec;
            // iterate over number of destination register operands since
            // this is a load
            for (int k = 0; k < num_dest_operands; ++k) {
                assert((sizeof(c1) * num_dest_operands)
                       <= MAX_WIDTH_FOR_MEM_INST);

                int dst = this->dest.regIndex() + k;
                if (num_dest_operands > MAX_REGS_FOR_NON_VEC_MEM_INST)
                    dst = dest_vect[k].regIndex();
                // virtual->physical VGPR mapping
                int physVgpr = w->remap(dst, sizeof(c0), 1);
                // save the physical VGPR index
                regVec.push_back(physVgpr);

                c1 *p1 =
                    &((c1*)gpuDynInst->d_data)[k * w->computeUnit->wfSize()];

                for (int i = 0; i < w->computeUnit->wfSize(); ++i) {
                    if (gpuDynInst->exec_mask[i]) {
                        DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: "
                                "$%s%d <- %d global ld done (src = wavefront "
                                "ld inst)\n", w->computeUnit->cu_id, w->simdId,
                                w->wfSlotId, i, sizeof(c0) == 4 ? "s" : "d",
                                dst, *p1);
                        // write the value into the physical VGPR. This is a
                        // purely functional operation. No timing is modeled.
                        w->computeUnit->vrf[w->simdId]->write<c0>(physVgpr,
                                                                    *p1, i);
                    }
                    ++p1;
                }
            }

            // Schedule the write operation of the load data on the VRF.
            // This simply models the timing aspect of the VRF write operation.
            // It does not modify the physical VGPR.
            int loadVrfBankConflictCycles = gpuDynInst->computeUnit()->
                vrf[w->simdId]->exec(gpuDynInst->seqNum(), w, regVec,
                                     sizeof(c0), gpuDynInst->time);

            if (this->isGlobalMem()) {
                gpuDynInst->computeUnit()->globalMemoryPipe
                    .incLoadVRFBankConflictCycles(loadVrfBankConflictCycles);
            } else {
                assert(this->isLocalMem());
                gpuDynInst->computeUnit()->localMemoryPipe
                    .incLoadVRFBankConflictCycles(loadVrfBankConflictCycles);
            }
        }

      private:
        void
        execLdAcq(GPUDynInstPtr gpuDynInst) override
        {
            // after the load has complete and if the load has acquire
            // semantics, issue an acquire request.
            if (!this->isLocalMem()) {
                if (gpuDynInst->computeUnit()->shader->separate_acquire_release
                    && gpuDynInst->isAcquire()) {
                    gpuDynInst->statusBitVector = VectorMask(1);
                    gpuDynInst->useContinuation = false;
                    // create request
                    RequestPtr req = std::make_shared<Request>(0, 0, 0, 0,
                                  gpuDynInst->computeUnit()->masterId(),
                                  0, gpuDynInst->wfDynId);
                    req->setFlags(Request::ACQUIRE);
                    gpuDynInst->computeUnit()->injectGlobalMemFence(gpuDynInst, false, req);
                }
            }
        }

      public:
        bool isVectorRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if ((num_dest_operands != getNumOperands()) &&
                (operandIndex == (getNumOperands()-1)))
                return(this->addr.isVectorRegister());
            if (num_dest_operands > 1) {
                return dest_vect[operandIndex].isVectorRegister();
            }
            else if (num_dest_operands == 1) {
                return LdInstBase<typename MemDataType::CType,
                       typename DestDataType::OperandType,
                       AddrOperandType>::dest.isVectorRegister();
            }
            return false;
        }
        bool isCondRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if ((num_dest_operands != getNumOperands()) &&
                (operandIndex == (getNumOperands()-1)))
                return(this->addr.isCondRegister());
            if (num_dest_operands > 1)
                return dest_vect[operandIndex].isCondRegister();
            else if (num_dest_operands == 1)
                return LdInstBase<typename MemDataType::CType,
                       typename DestDataType::OperandType,
                       AddrOperandType>::dest.isCondRegister();
            return false;
        }
        bool isScalarRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if ((num_dest_operands != getNumOperands()) &&
                (operandIndex == (getNumOperands()-1)))
                return(this->addr.isScalarRegister());
            if (num_dest_operands > 1)
                return dest_vect[operandIndex].isScalarRegister();
            else if (num_dest_operands == 1)
                return LdInstBase<typename MemDataType::CType,
                       typename DestDataType::OperandType,
                       AddrOperandType>::dest.isScalarRegister();
            return false;
        }
        bool isSrcOperand(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if ((num_dest_operands != getNumOperands()) &&
                (operandIndex == (getNumOperands()-1)))
                return(this->addr.isVectorRegister());
            return false;
        }
        bool isDstOperand(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if ((num_dest_operands != getNumOperands()) &&
                (operandIndex == (getNumOperands()-1)))
                return false;
            return true;
        }
        int getOperandSize(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if ((num_dest_operands != getNumOperands()) &&
                (operandIndex == (getNumOperands()-1)))
                return(this->addr.opSize());
            if (num_dest_operands > 1)
                return(dest_vect[operandIndex].opSize());
            else if (num_dest_operands == 1)
                return(LdInstBase<typename MemDataType::CType,
                       typename DestDataType::OperandType,
                       AddrOperandType>::dest.opSize());
            return 0;
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if ((num_dest_operands != getNumOperands()) &&
                (operandIndex == (getNumOperands()-1)))
                return(this->addr.regIndex());
            if (num_dest_operands > 1)
                return(dest_vect[operandIndex].regIndex());
            else if (num_dest_operands == 1)
                return(LdInstBase<typename MemDataType::CType,
                       typename DestDataType::OperandType,
                       AddrOperandType>::dest.regIndex());
            return -1;
        }
        int getNumOperands() override
        {
            if (this->addr.isVectorRegister() || this->addr.isScalarRegister())
                return(num_dest_operands+1);
            else
                return(num_dest_operands);
        }
        void execute(GPUDynInstPtr gpuDynInst) override;
    };

    template<typename MemDT, typename DestDT>
    GPUStaticInst*
    decodeLd2(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        unsigned op_offs = obj->getOperandPtr(ib->operands,1);
        BrigRegOperandInfo tmp = findRegDataType(op_offs, obj);

        if (tmp.kind == Brig::BRIG_KIND_OPERAND_ADDRESS) {
            return new LdInst<MemDT, DestDT, NoRegAddrOperand>(ib, obj, "ld");
        } else if (tmp.kind == Brig::BRIG_KIND_OPERAND_REGISTER ||
                   tmp.kind == Brig::BRIG_KIND_OPERAND_OPERAND_LIST) {
            switch (tmp.regKind) {
              case Brig::BRIG_REGISTER_KIND_SINGLE:
                return new LdInst<MemDT, DestDT,
                                  SRegAddrOperand>(ib, obj, "ld");
              case Brig::BRIG_REGISTER_KIND_DOUBLE:
                return new LdInst<MemDT, DestDT,
                                  DRegAddrOperand>(ib, obj, "ld");
              default:
                fatal("Bad ld register operand type %d\n", tmp.regKind);
            }
        } else {
            fatal("Bad ld register operand kind %d\n", tmp.kind);
        }
    }

    template<typename MemDT>
    GPUStaticInst*
    decodeLd(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        unsigned op_offs = obj->getOperandPtr(ib->operands,0);
        BrigRegOperandInfo dest = findRegDataType(op_offs, obj);

        assert(dest.kind == Brig::BRIG_KIND_OPERAND_REGISTER ||
               dest.kind == Brig::BRIG_KIND_OPERAND_OPERAND_LIST);
        switch(dest.regKind) {
          case Brig::BRIG_REGISTER_KIND_SINGLE:
            switch (ib->type) {
              case Brig::BRIG_TYPE_B8:
              case Brig::BRIG_TYPE_B16:
              case Brig::BRIG_TYPE_B32:
                return decodeLd2<MemDT, B32>(ib, obj);
              case Brig::BRIG_TYPE_U8:
              case Brig::BRIG_TYPE_U16:
              case Brig::BRIG_TYPE_U32:
                return decodeLd2<MemDT, U32>(ib, obj);
              case Brig::BRIG_TYPE_S8:
              case Brig::BRIG_TYPE_S16:
              case Brig::BRIG_TYPE_S32:
                return decodeLd2<MemDT, S32>(ib, obj);
              case Brig::BRIG_TYPE_F16:
              case Brig::BRIG_TYPE_F32:
                return decodeLd2<MemDT, U32>(ib, obj);
              default:
                fatal("Bad ld register operand type %d, %d\n",
                      dest.regKind, ib->type);
            };
          case Brig::BRIG_REGISTER_KIND_DOUBLE:
            switch (ib->type) {
              case Brig::BRIG_TYPE_B64:
                return decodeLd2<MemDT, B64>(ib, obj);
              case Brig::BRIG_TYPE_U64:
                return decodeLd2<MemDT, U64>(ib, obj);
              case Brig::BRIG_TYPE_S64:
                return decodeLd2<MemDT, S64>(ib, obj);
              case Brig::BRIG_TYPE_F64:
                return decodeLd2<MemDT, U64>(ib, obj);
              default:
                fatal("Bad ld register operand type %d, %d\n",
                      dest.regKind, ib->type);
            };
          default:
            fatal("Bad ld register operand type %d, %d\n", dest.regKind,
                  ib->type);
        }
    }

    template<typename MemDataType, typename SrcOperandType,
             typename AddrOperandType>
    class StInstBase : public HsailGPUStaticInst
    {
      public:
        typename SrcOperandType::SrcOperand src;
        AddrOperandType addr;

        Brig::BrigSegment segment;
        Brig::BrigMemoryScope memoryScope;
        Brig::BrigMemoryOrder memoryOrder;
        unsigned int equivClass;

        StInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj,
                   const char *_opcode)
           : HsailGPUStaticInst(obj, _opcode)
        {
            using namespace Brig;

            setFlag(MemoryRef);
            setFlag(Store);

            if (ib->opcode == BRIG_OPCODE_ST) {
                const BrigInstMem *ldst = (const BrigInstMem*)ib;

                segment = (BrigSegment)ldst->segment;
                memoryOrder = BRIG_MEMORY_ORDER_NONE;
                memoryScope = BRIG_MEMORY_SCOPE_NONE;
                equivClass = ldst->equivClass;

                unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
                const BrigOperand *baseOp = obj->getOperand(op_offs);

                if ((baseOp->kind == BRIG_KIND_OPERAND_CONSTANT_BYTES) ||
                    (baseOp->kind == BRIG_KIND_OPERAND_REGISTER)) {
                    src.init(op_offs, obj);
                }

                op_offs = obj->getOperandPtr(ib->operands, 1);
                addr.init(op_offs, obj);
            } else {
                const BrigInstAtomic *at = (const BrigInstAtomic*)ib;

                segment = (BrigSegment)at->segment;
                memoryScope = (BrigMemoryScope)at->memoryScope;
                memoryOrder = (BrigMemoryOrder)at->memoryOrder;
                equivClass = 0;

                unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
                addr.init(op_offs, obj);

                op_offs = obj->getOperandPtr(ib->operands, 1);
                src.init(op_offs, obj);
            }

            switch (memoryOrder) {
              case BRIG_MEMORY_ORDER_NONE:
                setFlag(NoOrder);
                break;
              case BRIG_MEMORY_ORDER_RELAXED:
                setFlag(RelaxedOrder);
                break;
              case BRIG_MEMORY_ORDER_SC_ACQUIRE:
                setFlag(Acquire);
                break;
              case BRIG_MEMORY_ORDER_SC_RELEASE:
                setFlag(Release);
                break;
              case BRIG_MEMORY_ORDER_SC_ACQUIRE_RELEASE:
                setFlag(AcquireRelease);
                break;
              default:
                fatal("StInst has bad memory order type\n");
            }

            switch (memoryScope) {
              case BRIG_MEMORY_SCOPE_NONE:
                setFlag(NoScope);
                break;
              case BRIG_MEMORY_SCOPE_WORKITEM:
                setFlag(WorkitemScope);
                break;
              case BRIG_MEMORY_SCOPE_WORKGROUP:
                setFlag(WorkgroupScope);
                break;
              case BRIG_MEMORY_SCOPE_AGENT:
                setFlag(DeviceScope);
                break;
              case BRIG_MEMORY_SCOPE_SYSTEM:
                setFlag(SystemScope);
                break;
              default:
                fatal("StInst has bad memory scope type\n");
            }

            switch (segment) {
              case BRIG_SEGMENT_GLOBAL:
                setFlag(GlobalSegment);
                break;
              case BRIG_SEGMENT_GROUP:
                setFlag(GroupSegment);
                break;
              case BRIG_SEGMENT_PRIVATE:
                setFlag(PrivateSegment);
                break;
              case BRIG_SEGMENT_READONLY:
                setFlag(ReadOnlySegment);
                break;
              case BRIG_SEGMENT_SPILL:
                setFlag(SpillSegment);
                break;
              case BRIG_SEGMENT_FLAT:
                setFlag(Flat);
                break;
              case BRIG_SEGMENT_ARG:
                setFlag(ArgSegment);
                break;
              default:
                panic("St: segment %d not supported\n", segment);
            }
        }

        int numDstRegOperands() override { return 0; }
        int numSrcRegOperands() override
        {
            return src.isVectorRegister() + this->addr.isVectorRegister();
        }
        int getNumOperands() override
        {
            if (this->addr.isVectorRegister() || this->addr.isScalarRegister())
                return 2;
            else
                return 1;
        }
        bool isVectorRegister(int operandIndex) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return !operandIndex ? src.isVectorRegister() :
                   this->addr.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return !operandIndex ? src.isCondRegister() :
                   this->addr.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return !operandIndex ? src.isScalarRegister() :
                   this->addr.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return true;
        }
        bool isDstOperand(int operandIndex) override { return false; }
        int getOperandSize(int operandIndex) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return !operandIndex ? src.opSize() : this->addr.opSize();
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());
            return !operandIndex ? src.regIndex() : this->addr.regIndex();
        }
    };


    template<typename MemDataType, typename SrcDataType,
             typename AddrOperandType>
    class StInst :
        public StInstBase<MemDataType, typename SrcDataType::OperandType,
                          AddrOperandType>,
        public MemInst
    {
      public:
        typename SrcDataType::OperandType::SrcOperand src_vect[4];
        uint16_t num_src_operands;
        void generateDisassembly() override;

        StInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                        const char *_opcode, int srcIdx)
            : StInstBase<MemDataType, typename SrcDataType::OperandType,
                         AddrOperandType>(ib, obj, _opcode),
              MemInst(SrcDataType::memType)
        {
            init_addr(&this->addr);

            BrigRegOperandInfo rinfo;
            unsigned op_offs = obj->getOperandPtr(ib->operands,srcIdx);
            const Brig::BrigOperand *baseOp = obj->getOperand(op_offs);

            if (baseOp->kind == Brig::BRIG_KIND_OPERAND_CONSTANT_BYTES) {
                const Brig::BrigOperandConstantBytes *op =
                    (Brig::BrigOperandConstantBytes*)baseOp;

                rinfo = BrigRegOperandInfo((Brig::BrigKind16_t)op->base.kind,
                                           Brig::BRIG_TYPE_NONE);
            } else {
                rinfo = findRegDataType(op_offs, obj);
            }

            if (baseOp->kind == Brig::BRIG_KIND_OPERAND_OPERAND_LIST) {
                const Brig::BrigOperandOperandList *brigRegVecOp =
                    (const Brig::BrigOperandOperandList*)baseOp;

                num_src_operands =
                    *((unsigned*)obj->getData(brigRegVecOp->elements)) / 4;

                assert(num_src_operands <= 4);
            } else {
                num_src_operands = 1;
            }

            if (num_src_operands > 1) {
                assert(baseOp->kind == Brig::BRIG_KIND_OPERAND_OPERAND_LIST);

                for (int i = 0; i < num_src_operands; ++i) {
                    src_vect[i].init_from_vect(op_offs, obj, i);
                }
            }
        }

        void
        initiateAcc(GPUDynInstPtr gpuDynInst) override
        {
            // before performing a store, check if this store has
            // release semantics, and if so issue a release first
            if (!this->isLocalMem()) {
                if (gpuDynInst->computeUnit()->shader->separate_acquire_release
                    && gpuDynInst->isRelease()) {

                    gpuDynInst->statusBitVector = VectorMask(1);
                    gpuDynInst->execContinuation = &GPUStaticInst::execSt;
                    gpuDynInst->useContinuation = true;
                    // create request
                    RequestPtr req = std::make_shared<Request>(0, 0, 0, 0,
                                  gpuDynInst->computeUnit()->masterId(),
                                  0, gpuDynInst->wfDynId);
                    req->setFlags(Request::RELEASE);
                    gpuDynInst->computeUnit()->injectGlobalMemFence(gpuDynInst, false, req);

                    return;
                }
            }

            // if there is no release semantic, perform stores immediately
            execSt(gpuDynInst);
        }

        // stores don't write anything back, so there is nothing
        // to do here. we only override this method to avoid the
        // fatal in the base class implementation
        void completeAcc(GPUDynInstPtr gpuDynInst) override { }

      private:
        // execSt may be called through a continuation
        // if the store had release semantics. see comment for
        // execSt in gpu_static_inst.hh
        void
        execSt(GPUDynInstPtr gpuDynInst) override
        {
            typedef typename MemDataType::CType c0;

            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            if (num_src_operands > 1) {
                for (int i = 0; i < gpuDynInst->computeUnit()->wfSize(); ++i)
                    if (gpuDynInst->exec_mask[i])
                        gpuDynInst->statusVector.push_back(num_src_operands);
                    else
                        gpuDynInst->statusVector.push_back(0);
            }

            for (int k = 0; k < num_src_operands; ++k) {
                c0 *d = &((c0*)gpuDynInst->d_data)
                    [k * gpuDynInst->computeUnit()->wfSize()];

                for (int i = 0; i < gpuDynInst->computeUnit()->wfSize(); ++i) {
                    if (gpuDynInst->exec_mask[i]) {
                        Addr vaddr = gpuDynInst->addr[i] + k * sizeof(c0);

                        if (this->isLocalMem()) {
                            //store to shared memory
                            gpuDynInst->wavefront()->ldsChunk->write<c0>(vaddr,
                                                                         *d);
                        } else {
                            RequestPtr req = std::make_shared<Request>(
                                0, vaddr, sizeof(c0), 0,
                                gpuDynInst->computeUnit()->masterId(),
                                0, gpuDynInst->wfDynId);

                            gpuDynInst->setRequestFlags(req);
                            PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                            pkt->dataStatic<c0>(d);

                            // translation is performed in sendRequest()
                            // the request will be finished when the store completes
                            gpuDynInst->useContinuation = false;
                            gpuDynInst->computeUnit()->sendRequest(gpuDynInst,
                                                                   i, pkt);

                        }
                    }
                    ++d;
                }
            }

            gpuDynInst->updateStats();
        }

      public:
        bool isVectorRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex == num_src_operands)
                return this->addr.isVectorRegister();
            if (num_src_operands > 1)
                return src_vect[operandIndex].isVectorRegister();
            else if (num_src_operands == 1)
                return StInstBase<MemDataType,
                       typename SrcDataType::OperandType,
                       AddrOperandType>::src.isVectorRegister();
            return false;
        }
        bool isCondRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex == num_src_operands)
                return this->addr.isCondRegister();
            if (num_src_operands > 1)
                return src_vect[operandIndex].isCondRegister();
            else if (num_src_operands == 1)
                return StInstBase<MemDataType,
                       typename SrcDataType::OperandType,
                       AddrOperandType>::src.isCondRegister();
            return false;
        }
        bool isScalarRegister(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex == num_src_operands)
                return this->addr.isScalarRegister();
            if (num_src_operands > 1)
                return src_vect[operandIndex].isScalarRegister();
            else if (num_src_operands == 1)
                return StInstBase<MemDataType,
                       typename SrcDataType::OperandType,
                       AddrOperandType>::src.isScalarRegister();
            return false;
        }
        bool isSrcOperand(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return true;
        }
        bool isDstOperand(int operandIndex) override { return false; }
        int getOperandSize(int operandIndex) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex == num_src_operands)
                return this->addr.opSize();
            if (num_src_operands > 1)
                return src_vect[operandIndex].opSize();
            else if (num_src_operands == 1)
                return StInstBase<MemDataType,
                       typename SrcDataType::OperandType,
                       AddrOperandType>::src.opSize();
            return 0;
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst) override
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex == num_src_operands)
                return this->addr.regIndex();
            if (num_src_operands > 1)
                return src_vect[operandIndex].regIndex();
            else if (num_src_operands == 1)
                return StInstBase<MemDataType,
                       typename SrcDataType::OperandType,
                       AddrOperandType>::src.regIndex();
            return -1;
        }
        int getNumOperands() override
        {
            if (this->addr.isVectorRegister() || this->addr.isScalarRegister())
                return num_src_operands + 1;
            else
                return num_src_operands;
        }
        void execute(GPUDynInstPtr gpuDynInst) override;
    };

    template<typename DataType, typename SrcDataType>
    GPUStaticInst*
    decodeSt(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        int srcIdx = 0;
        int destIdx = 1;
        if (ib->opcode == Brig::BRIG_OPCODE_ATOMIC ||
            ib->opcode == Brig::BRIG_OPCODE_ATOMICNORET) {
            srcIdx = 1;
            destIdx = 0;
        }
        unsigned op_offs = obj->getOperandPtr(ib->operands,destIdx);

        BrigRegOperandInfo tmp = findRegDataType(op_offs, obj);

        if (tmp.kind == Brig::BRIG_KIND_OPERAND_ADDRESS) {
            return new StInst<DataType, SrcDataType,
                              NoRegAddrOperand>(ib, obj, "st", srcIdx);
        } else if (tmp.kind == Brig::BRIG_KIND_OPERAND_REGISTER) {
            // V2/V4 not allowed
            switch (tmp.regKind) {
              case Brig::BRIG_REGISTER_KIND_SINGLE:
                return new StInst<DataType, SrcDataType,
                                  SRegAddrOperand>(ib, obj, "st", srcIdx);
              case Brig::BRIG_REGISTER_KIND_DOUBLE:
                return new StInst<DataType, SrcDataType,
                                  DRegAddrOperand>(ib, obj, "st", srcIdx);
              default:
                fatal("Bad st register operand type %d\n", tmp.type);
            }
        } else {
            fatal("Bad st register operand kind %d\n", tmp.kind);
        }
    }

    template<typename OperandType, typename AddrOperandType, int NumSrcOperands,
             bool HasDst>
    class AtomicInstBase : public HsailGPUStaticInst
    {
      public:
        typename OperandType::DestOperand dest;
        typename OperandType::SrcOperand src[NumSrcOperands];
        AddrOperandType addr;

        Brig::BrigSegment segment;
        Brig::BrigMemoryOrder memoryOrder;
        Brig::BrigAtomicOperation atomicOperation;
        Brig::BrigMemoryScope memoryScope;
        Brig::BrigOpcode opcode;

        AtomicInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj,
                       const char *_opcode)
           : HsailGPUStaticInst(obj, _opcode)
        {
            using namespace Brig;

            const BrigInstAtomic *at = (const BrigInstAtomic*)ib;

            segment = (BrigSegment)at->segment;
            memoryScope = (BrigMemoryScope)at->memoryScope;
            memoryOrder = (BrigMemoryOrder)at->memoryOrder;
            atomicOperation = (BrigAtomicOperation)at->atomicOperation;
            opcode = (BrigOpcode)ib->opcode;

            assert(opcode == Brig::BRIG_OPCODE_ATOMICNORET ||
                   opcode == Brig::BRIG_OPCODE_ATOMIC);

            setFlag(MemoryRef);

            if (opcode == Brig::BRIG_OPCODE_ATOMIC) {
                setFlag(AtomicReturn);
            } else {
                setFlag(AtomicNoReturn);
            }

            switch (memoryOrder) {
              case BRIG_MEMORY_ORDER_NONE:
                setFlag(NoOrder);
                break;
              case BRIG_MEMORY_ORDER_RELAXED:
                setFlag(RelaxedOrder);
                break;
              case BRIG_MEMORY_ORDER_SC_ACQUIRE:
                setFlag(Acquire);
                break;
              case BRIG_MEMORY_ORDER_SC_RELEASE:
                setFlag(Release);
                break;
              case BRIG_MEMORY_ORDER_SC_ACQUIRE_RELEASE:
                setFlag(AcquireRelease);
                break;
              default:
                fatal("AtomicInst has bad memory order type\n");
            }

            switch (memoryScope) {
              case BRIG_MEMORY_SCOPE_NONE:
                setFlag(NoScope);
                break;
              case BRIG_MEMORY_SCOPE_WORKITEM:
                setFlag(WorkitemScope);
                break;
              case BRIG_MEMORY_SCOPE_WORKGROUP:
                setFlag(WorkgroupScope);
                break;
              case BRIG_MEMORY_SCOPE_AGENT:
                setFlag(DeviceScope);
                break;
              case BRIG_MEMORY_SCOPE_SYSTEM:
                setFlag(SystemScope);
                break;
              default:
                fatal("AtomicInst has bad memory scope type\n");
            }

            switch (atomicOperation) {
              case Brig::BRIG_ATOMIC_AND:
                setFlag(AtomicAnd);
                break;
              case Brig::BRIG_ATOMIC_OR:
                setFlag(AtomicOr);
                break;
              case Brig::BRIG_ATOMIC_XOR:
                setFlag(AtomicXor);
                break;
              case Brig::BRIG_ATOMIC_CAS:
                setFlag(AtomicCAS);
                break;
              case Brig::BRIG_ATOMIC_EXCH:
                setFlag(AtomicExch);
                break;
              case Brig::BRIG_ATOMIC_ADD:
                setFlag(AtomicAdd);
                break;
              case Brig::BRIG_ATOMIC_WRAPINC:
                setFlag(AtomicInc);
                break;
              case Brig::BRIG_ATOMIC_WRAPDEC:
                setFlag(AtomicDec);
                break;
              case Brig::BRIG_ATOMIC_MIN:
                setFlag(AtomicMin);
                break;
              case Brig::BRIG_ATOMIC_MAX:
                setFlag(AtomicMax);
                break;
              case Brig::BRIG_ATOMIC_SUB:
                setFlag(AtomicSub);
                break;
              default:
                fatal("Bad BrigAtomicOperation code %d\n", atomicOperation);
            }

            switch (segment) {
              case BRIG_SEGMENT_GLOBAL:
                setFlag(GlobalSegment);
                break;
              case BRIG_SEGMENT_GROUP:
                setFlag(GroupSegment);
                break;
              case BRIG_SEGMENT_FLAT:
                setFlag(Flat);
                break;
              default:
                panic("Atomic: segment %d not supported\n", segment);
            }

            if (HasDst) {
                unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
                dest.init(op_offs, obj);

                op_offs = obj->getOperandPtr(ib->operands, 1);
                addr.init(op_offs, obj);

                for (int i = 0; i < NumSrcOperands; ++i) {
                    op_offs = obj->getOperandPtr(ib->operands, i + 2);
                    src[i].init(op_offs, obj);
                }
            } else {

                unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
                addr.init(op_offs, obj);

                for (int i = 0; i < NumSrcOperands; ++i) {
                    op_offs = obj->getOperandPtr(ib->operands, i + 1);
                    src[i].init(op_offs, obj);
                }
            }
        }

        int numSrcRegOperands()
        {
            int operands = 0;
            for (int i = 0; i < NumSrcOperands; i++) {
                if (src[i].isVectorRegister()) {
                    operands++;
                }
            }
            if (addr.isVectorRegister())
                operands++;
            return operands;
        }
        int numDstRegOperands() { return dest.isVectorRegister(); }
        int getNumOperands()
        {
            if (addr.isVectorRegister())
                return(NumSrcOperands + 2);
            return(NumSrcOperands + 1);
        }
        bool isVectorRegister(int operandIndex)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return src[operandIndex].isVectorRegister();
            else if (operandIndex == NumSrcOperands)
                return(addr.isVectorRegister());
            else
                return dest.isVectorRegister();
        }
        bool isCondRegister(int operandIndex)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return src[operandIndex].isCondRegister();
            else if (operandIndex == NumSrcOperands)
                return(addr.isCondRegister());
            else
                return dest.isCondRegister();
        }
        bool isScalarRegister(int operandIndex)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return src[operandIndex].isScalarRegister();
            else if (operandIndex == NumSrcOperands)
                return(addr.isScalarRegister());
            else
                return dest.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return true;
            else if (operandIndex == NumSrcOperands)
                return(addr.isVectorRegister());
            else
                return false;
        }
        bool isDstOperand(int operandIndex)
        {
            if (operandIndex <= NumSrcOperands)
                return false;
            else
                return true;
        }
        int getOperandSize(int operandIndex)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return(src[operandIndex].opSize());
            else if (operandIndex == NumSrcOperands)
                return(addr.opSize());
            else
                return(dest.opSize());
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return(src[operandIndex].regIndex());
            else if (operandIndex == NumSrcOperands)
                return(addr.regIndex());
            else
                return(dest.regIndex());
            return -1;
        }
    };

    template<typename MemDataType, typename AddrOperandType, int NumSrcOperands,
             bool HasDst>
    class AtomicInst :
        public AtomicInstBase<typename MemDataType::OperandType,
                              AddrOperandType, NumSrcOperands, HasDst>,
        public MemInst
    {
      public:
        void generateDisassembly() override;

        AtomicInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                   const char *_opcode)
            : AtomicInstBase<typename MemDataType::OperandType, AddrOperandType,
                             NumSrcOperands, HasDst>
                (ib, obj, _opcode),
              MemInst(MemDataType::memType)
        {
            init_addr(&this->addr);
        }

        void
        initiateAcc(GPUDynInstPtr gpuDynInst) override
        {
            // before doing the RMW, check if this atomic has
            // release semantics, and if so issue a release first
            if (!this->isLocalMem()) {
                if (gpuDynInst->computeUnit()->shader->separate_acquire_release
                    && (gpuDynInst->isRelease()
                    || gpuDynInst->isAcquireRelease())) {

                    gpuDynInst->statusBitVector = VectorMask(1);

                    gpuDynInst->execContinuation = &GPUStaticInst::execAtomic;
                    gpuDynInst->useContinuation = true;

                    // create request
                    RequestPtr req = std::make_shared<Request>(0, 0, 0, 0,
                                  gpuDynInst->computeUnit()->masterId(),
                                  0, gpuDynInst->wfDynId);
                    req->setFlags(Request::RELEASE);
                    gpuDynInst->computeUnit()->injectGlobalMemFence(gpuDynInst, false, req);

                    return;
                }
            }

            // if there is no release semantic, execute the RMW immediately
            execAtomic(gpuDynInst);

        }

        void
        completeAcc(GPUDynInstPtr gpuDynInst) override
        {
            // if this is not an atomic return op, then we
            // have nothing more to do.
            if (this->isAtomicRet()) {
                // the size of the src operands and the
                // memory being operated on must match
                // for HSAIL atomics - this assumption may
                // not apply to all ISAs
                typedef typename MemDataType::CType CType;

                Wavefront *w = gpuDynInst->wavefront();
                int dst = this->dest.regIndex();
                std::vector<uint32_t> regVec;
                // virtual->physical VGPR mapping
                int physVgpr = w->remap(dst, sizeof(CType), 1);
                regVec.push_back(physVgpr);
                CType *p1 = &((CType*)gpuDynInst->d_data)[0];

                for (int i = 0; i < w->computeUnit->wfSize(); ++i) {
                    if (gpuDynInst->exec_mask[i]) {
                        DPRINTF(GPUReg, "CU%d, WF[%d][%d], lane %d: "
                                "$%s%d <- %d global ld done (src = wavefront "
                                "ld inst)\n", w->computeUnit->cu_id, w->simdId,
                                w->wfSlotId, i, sizeof(CType) == 4 ? "s" : "d",
                                dst, *p1);
                        // write the value into the physical VGPR. This is a
                        // purely functional operation. No timing is modeled.
                        w->computeUnit->vrf[w->simdId]->write<CType>(physVgpr, *p1, i);
                    }
                    ++p1;
                }

                // Schedule the write operation of the load data on the VRF.
                // This simply models the timing aspect of the VRF write operation.
                // It does not modify the physical VGPR.
                int loadVrfBankConflictCycles = gpuDynInst->computeUnit()->
                    vrf[w->simdId]->exec(gpuDynInst->seqNum(), w, regVec,
                                         sizeof(CType), gpuDynInst->time);

                if (this->isGlobalMem()) {
                    gpuDynInst->computeUnit()->globalMemoryPipe
                        .incLoadVRFBankConflictCycles(loadVrfBankConflictCycles);
                } else {
                    assert(this->isLocalMem());
                    gpuDynInst->computeUnit()->localMemoryPipe
                        .incLoadVRFBankConflictCycles(loadVrfBankConflictCycles);
                }
            }
        }

        void execute(GPUDynInstPtr gpuDynInst) override;

      private:
        // execAtomic may be called through a continuation
        // if the RMW had release semantics. see comment for
        // execContinuation in gpu_dyn_inst.hh
        void
        execAtomic(GPUDynInstPtr gpuDynInst) override
        {
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            typedef typename MemDataType::CType c0;

            c0 *d = &((c0*) gpuDynInst->d_data)[0];
            c0 *e = &((c0*) gpuDynInst->a_data)[0];
            c0 *f = &((c0*) gpuDynInst->x_data)[0];

            for (int i = 0; i < gpuDynInst->computeUnit()->wfSize(); ++i) {
                if (gpuDynInst->exec_mask[i]) {
                    Addr vaddr = gpuDynInst->addr[i];

                    if (this->isLocalMem()) {
                        Wavefront *wavefront = gpuDynInst->wavefront();
                        *d = wavefront->ldsChunk->read<c0>(vaddr);

                        if (this->isAtomicAdd()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            wavefront->ldsChunk->read<c0>(vaddr) + (*e));
                        } else if (this->isAtomicSub()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            wavefront->ldsChunk->read<c0>(vaddr) - (*e));
                        } else if (this->isAtomicMax()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            std::max(wavefront->ldsChunk->read<c0>(vaddr),
                            (*e)));
                        } else if (this->isAtomicMin()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            std::min(wavefront->ldsChunk->read<c0>(vaddr),
                            (*e)));
                        } else if (this->isAtomicAnd()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            wavefront->ldsChunk->read<c0>(vaddr) & (*e));
                        } else if (this->isAtomicOr()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            wavefront->ldsChunk->read<c0>(vaddr) | (*e));
                        } else if (this->isAtomicXor()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            wavefront->ldsChunk->read<c0>(vaddr) ^ (*e));
                        } else if (this->isAtomicInc()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            wavefront->ldsChunk->read<c0>(vaddr) + 1);
                        } else if (this->isAtomicDec()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            wavefront->ldsChunk->read<c0>(vaddr) - 1);
                        } else if (this->isAtomicExch()) {
                            wavefront->ldsChunk->write<c0>(vaddr, (*e));
                        } else if (this->isAtomicCAS()) {
                            wavefront->ldsChunk->write<c0>(vaddr,
                            (wavefront->ldsChunk->read<c0>(vaddr) == (*e)) ?
                            (*f) : wavefront->ldsChunk->read<c0>(vaddr));
                        } else {
                            fatal("Unrecognized or invalid HSAIL atomic op "
                                  "type.\n");
                        }
                    } else {
                        RequestPtr req =
                            std::make_shared<Request>(0, vaddr, sizeof(c0), 0,
                                        gpuDynInst->computeUnit()->masterId(),
                                        0, gpuDynInst->wfDynId,
                                        gpuDynInst->makeAtomicOpFunctor<c0>(e,
                                        f));

                        gpuDynInst->setRequestFlags(req);
                        PacketPtr pkt = new Packet(req, MemCmd::SwapReq);
                        pkt->dataStatic(d);

                        if (gpuDynInst->computeUnit()->shader->
                            separate_acquire_release &&
                            (gpuDynInst->isAcquire())) {
                            // if this atomic has acquire semantics,
                            // schedule the continuation to perform an
                            // acquire after the RMW completes
                            gpuDynInst->execContinuation =
                                &GPUStaticInst::execAtomicAcq;

                            gpuDynInst->useContinuation = true;
                        } else {
                            // the request will be finished when the RMW completes
                            gpuDynInst->useContinuation = false;
                        }
                        // translation is performed in sendRequest()
                        gpuDynInst->computeUnit()->sendRequest(gpuDynInst, i,
                                                               pkt);
                    }
                }

                ++d;
                ++e;
                ++f;
            }

            gpuDynInst->updateStats();
        }

        // execAtomicACq will always be called through a continuation.
        // see comment for execContinuation in gpu_dyn_inst.hh
        void
        execAtomicAcq(GPUDynInstPtr gpuDynInst) override
        {
            // after performing the RMW, check to see if this instruction
            // has acquire semantics, and if so, issue an acquire
            if (!this->isLocalMem()) {
                if (gpuDynInst->computeUnit()->shader->separate_acquire_release
                     && gpuDynInst->isAcquire()) {
                    gpuDynInst->statusBitVector = VectorMask(1);

                    // the request will be finished when
                    // the acquire completes
                    gpuDynInst->useContinuation = false;
                    // create request
                    RequestPtr req = std::make_shared<Request>(0, 0, 0, 0,
                                  gpuDynInst->computeUnit()->masterId(),
                                  0, gpuDynInst->wfDynId);
                    req->setFlags(Request::ACQUIRE);
                    gpuDynInst->computeUnit()->injectGlobalMemFence(gpuDynInst, false, req);
                }
            }
        }
    };

    template<typename DataType, typename AddrOperandType, int NumSrcOperands>
    GPUStaticInst*
    constructAtomic(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        const Brig::BrigInstAtomic *at = (const Brig::BrigInstAtomic*)ib;

        if (at->atomicOperation == Brig::BRIG_ATOMIC_LD) {
            return decodeLd<DataType>(ib, obj);
        } else if (at->atomicOperation == Brig::BRIG_ATOMIC_ST) {
            switch (ib->type) {
              case Brig::BRIG_TYPE_B8:
                return decodeSt<S8,S8>(ib, obj);
              case Brig::BRIG_TYPE_B16:
                return decodeSt<S16,S16>(ib, obj);
              case Brig::BRIG_TYPE_B32:
                return decodeSt<S32,S32>(ib, obj);
              case Brig::BRIG_TYPE_B64:
                return decodeSt<S64,S64>(ib, obj);
              default: fatal("AtomicSt: Operand type mismatch %d\n", ib->type);
            }
        } else {
            if ((Brig::BrigOpcode)ib->opcode == Brig::BRIG_OPCODE_ATOMICNORET)
                return new AtomicInst<DataType, AddrOperandType,
                    NumSrcOperands, false>(ib, obj, "atomicnoret");
            else
                return new AtomicInst<DataType, AddrOperandType,
                    NumSrcOperands, true>(ib, obj, "atomic");
        }
    }

    template<typename DataType, int NumSrcOperands>
    GPUStaticInst*
    decodeAtomicHelper(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        unsigned addrIndex = (Brig::BrigOpcode)ib->opcode ==
            Brig::BRIG_OPCODE_ATOMICNORET ? 0 : 1;

        unsigned op_offs = obj->getOperandPtr(ib->operands,addrIndex);

        BrigRegOperandInfo tmp = findRegDataType(op_offs, obj);

        if (tmp.kind == Brig::BRIG_KIND_OPERAND_ADDRESS) {
            return constructAtomic<DataType, NoRegAddrOperand,
                                   NumSrcOperands>(ib, obj);
        } else if (tmp.kind == Brig::BRIG_KIND_OPERAND_REGISTER) {
            // V2/V4 not allowed
            switch (tmp.regKind) {
              case Brig::BRIG_REGISTER_KIND_SINGLE:
                  return constructAtomic<DataType, SRegAddrOperand,
                                         NumSrcOperands>(ib, obj);
              case Brig::BRIG_REGISTER_KIND_DOUBLE:
                return constructAtomic<DataType, DRegAddrOperand,
                                       NumSrcOperands>(ib, obj);
              default:
                fatal("Bad atomic register operand type %d\n", tmp.type);
            }
        } else {
            fatal("Bad atomic register operand kind %d\n", tmp.kind);
        }
    }


    template<typename DataType>
    GPUStaticInst*
    decodeAtomic(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        const Brig::BrigInstAtomic *at = (const Brig::BrigInstAtomic*)ib;

        if (at->atomicOperation == Brig::BRIG_ATOMIC_CAS) {
            return decodeAtomicHelper<DataType, 2>(ib, obj);
        } else {
            return decodeAtomicHelper<DataType, 1>(ib, obj);
        }
    }

    template<typename DataType>
    GPUStaticInst*
    decodeAtomicNoRet(const Brig::BrigInstBase *ib, const BrigObject *obj)
    {
        const Brig::BrigInstAtomic *at = (const Brig::BrigInstAtomic*)ib;
        if (at->atomicOperation == Brig::BRIG_ATOMIC_CAS) {
            return decodeAtomicHelper<DataType, 2>(ib, obj);
        } else {
            return decodeAtomicHelper<DataType, 1>(ib, obj);
        }
    }
} // namespace HsailISA

#endif // __ARCH_HSAIL_INSTS_MEM_HH__
