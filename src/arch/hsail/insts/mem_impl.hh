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

#include "gpu-compute/hsail_code.hh"

// defined in code.cc, but not worth sucking in all of code.h for this
// at this point
extern const char *segmentNames[];

namespace HsailISA
{
    template<typename DestDataType, typename AddrRegOperandType>
    void
    LdaInst<DestDataType, AddrRegOperandType>::generateDisassembly()
    {
        this->disassembly = csprintf("%s_%s %s,%s", this->opcode,
                                     DestDataType::label,
                                     this->dest.disassemble(),
                                     this->addr.disassemble());
    }

    template<typename DestDataType, typename AddrRegOperandType>
    void
    LdaInst<DestDataType, AddrRegOperandType>::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        typedef typename DestDataType::CType CType M5_VAR_USED;
        const VectorMask &mask = w->getPred();
        std::vector<Addr> addr_vec;
        addr_vec.resize(w->computeUnit->wfSize(), (Addr)0);
        this->addr.calcVector(w, addr_vec);

        for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
            if (mask[lane]) {
                this->dest.set(w, lane, addr_vec[lane]);
            }
        }
        addr_vec.clear();
    }

    template<typename MemDataType, typename DestDataType,
             typename AddrRegOperandType>
    void
    LdInst<MemDataType, DestDataType, AddrRegOperandType>::generateDisassembly()
    {
        switch (num_dest_operands) {
          case 1:
            this->disassembly = csprintf("%s_%s_%s %s,%s", this->opcode,
                                         segmentNames[this->segment],
                                         MemDataType::label,
                                         this->dest.disassemble(),
                                         this->addr.disassemble());
            break;
          case 2:
            this->disassembly = csprintf("%s_%s_%s (%s,%s), %s", this->opcode,
                                         segmentNames[this->segment],
                                         MemDataType::label,
                                         this->dest_vect[0].disassemble(),
                                         this->dest_vect[1].disassemble(),
                                         this->addr.disassemble());
            break;
          case 3:
            this->disassembly = csprintf("%s_%s_%s (%s,%s,%s), %s", this->opcode,
                                         segmentNames[this->segment],
                                         MemDataType::label,
                                         this->dest_vect[0].disassemble(),
                                         this->dest_vect[1].disassemble(),
                                         this->dest_vect[2].disassemble(),
                                         this->addr.disassemble());
            break;
          case 4:
            this->disassembly = csprintf("%s_%s_%s (%s,%s,%s,%s), %s",
                                         this->opcode,
                                         segmentNames[this->segment],
                                         MemDataType::label,
                                         this->dest_vect[0].disassemble(),
                                         this->dest_vect[1].disassemble(),
                                         this->dest_vect[2].disassemble(),
                                         this->dest_vect[3].disassemble(),
                                         this->addr.disassemble());
            break;
          default:
            fatal("Bad ld register dest operand, num vector operands: %d \n",
                  num_dest_operands);
            break;
        }
    }

    static Addr
    calcPrivAddr(Addr addr, Wavefront *w, int lane, GPUStaticInst *i)
    {
        // what is the size of the object we are accessing??
        // NOTE: the compiler doesn't generate enough information
        // to do this yet..have to just line up all the private
        // work-item spaces back to back for now
        /*
        StorageElement* se =
            i->parent->findSymbol(Brig::BrigPrivateSpace, addr);
        assert(se);

        return w->wfSlotId * w->privSizePerItem * w->computeUnit->wfSize() +
            se->offset * w->computeUnit->wfSize() +
            lane * se->size;
        */

        // addressing strategy: interleave the private spaces of
        // work-items in a wave-front on 8 byte granularity.
        // this won't be perfect coalescing like the spill space
        // strategy, but it's better than nothing. The spill space
        // strategy won't work with private because the same address
        // may be accessed by different sized loads/stores.

        // Note: I'm assuming that the largest load/store to private
        // is 8 bytes. If it is larger, the stride will have to increase

        Addr addr_div8 = addr / 8;
        Addr addr_mod8 = addr % 8;

        Addr ret = addr_div8 * 8 * w->computeUnit->wfSize() + lane * 8 +
            addr_mod8 + w->privBase;

        assert(ret < w->privBase +
               (w->privSizePerItem * w->computeUnit->wfSize()));

        return ret;
    }

    template<typename MemDataType, typename DestDataType,
             typename AddrRegOperandType>
    void
    LdInst<MemDataType, DestDataType,
           AddrRegOperandType>::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        typedef typename MemDataType::CType MemCType;
        const VectorMask &mask = w->getPred();

        // Kernarg references are handled uniquely for now (no Memory Request
        // is used), so special-case them up front.  Someday we should
        // make this more realistic, at which we should get rid of this
        // block and fold this case into the switch below.
        if (this->segment == Brig::BRIG_SEGMENT_KERNARG) {
            MemCType val;

            // I assume no vector ld for kernargs
            assert(num_dest_operands == 1);

            // assuming for the moment that we'll never do register
            // offsets into kernarg space... just to make life simpler
            uint64_t address = this->addr.calcUniform();

            val = *(MemCType*)&w->kernelArgs[address];

            DPRINTF(HSAIL, "ld_kernarg [%d] -> %d\n", address, val);

            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                if (mask[lane]) {
                    this->dest.set(w, lane, val);
                }
            }

            return;
        } else if (this->segment == Brig::BRIG_SEGMENT_ARG) {
            uint64_t address = this->addr.calcUniform();
            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                if (mask[lane]) {
                    MemCType val = w->readCallArgMem<MemCType>(lane, address);

                    DPRINTF(HSAIL, "ld_arg [%d] -> %llu\n", address,
                            (unsigned long long)val);

                    this->dest.set(w, lane, val);
                }
            }

            return;
        }

        GPUDynInstPtr m = gpuDynInst;

        this->addr.calcVector(w, m->addr);

        m->m_type = MemDataType::memType;
        m->v_type = DestDataType::vgprType;

        m->exec_mask = w->execMask();
        m->statusBitVector = 0;
        m->equiv = this->equivClass;

        if (num_dest_operands == 1) {
            m->dst_reg = this->dest.regIndex();
            m->n_reg = 1;
        } else {
            m->n_reg = num_dest_operands;
            for (int i = 0; i < num_dest_operands; ++i) {
                m->dst_reg_vec[i] = this->dest_vect[i].regIndex();
            }
        }

        m->simdId = w->simdId;
        m->wfSlotId = w->wfSlotId;
        m->wfDynId = w->wfDynId;
        m->kern_id = w->kernId;
        m->cu_id = w->computeUnit->cu_id;
        m->latency.init(&w->computeUnit->shader->tick_cnt);

        switch (this->segment) {
          case Brig::BRIG_SEGMENT_GLOBAL:
            m->pipeId = GLBMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(1));

            // this is a complete hack to get around a compiler bug
            // (the compiler currently generates global access for private
            //  addresses (starting from 0). We need to add the private offset)
            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                if (m->addr[lane] < w->privSizePerItem) {
                    if (mask[lane]) {
                        // what is the size of the object we are accessing?
                        // find base for for this wavefront

                        // calcPrivAddr will fail if accesses are unaligned
                        assert(!((sizeof(MemCType) - 1) & m->addr[lane]));

                        Addr privAddr = calcPrivAddr(m->addr[lane], w, lane,
                                                     this);

                        m->addr[lane] = privAddr;
                    }
                }
            }

            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsRdGm++;
            w->rdGmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_SPILL:
            assert(num_dest_operands == 1);
            m->pipeId = GLBMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(1));
            {
                for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                    //  note: this calculation will NOT WORK if the compiler
                    //  ever generates loads/stores to the same address with
                    //  different widths (e.g., a ld_u32 addr and a ld_u16 addr)
                    if (mask[lane]) {
                        assert(m->addr[lane] < w->spillSizePerItem);

                        m->addr[lane] = m->addr[lane] * w->spillWidth +
                                        lane * sizeof(MemCType) + w->spillBase;

                        w->lastAddr[lane] = m->addr[lane];
                    }
                }
            }

            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsRdGm++;
            w->rdGmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_GROUP:
            m->pipeId = LDSMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(24));
            w->computeUnit->localMemoryPipe.getLMReqFIFO().push(m);
            w->outstandingReqsRdLm++;
            w->rdLmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_READONLY:
            m->pipeId = GLBMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(1));

            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                if (mask[lane]) {
                    assert(m->addr[lane] + sizeof(MemCType) <= w->roSize);
                    m->addr[lane] += w->roBase;
                }
            }

            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsRdGm++;
            w->rdGmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_PRIVATE:
            m->pipeId = GLBMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(1));
            {
                for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                    if (mask[lane]) {
                        assert(m->addr[lane] < w->privSizePerItem);

                        m->addr[lane] = m->addr[lane] +
                            lane * sizeof(MemCType) + w->privBase;
                    }
                }
            }
            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsRdGm++;
            w->rdGmReqsInPipe--;
            break;

          default:
            fatal("Load to unsupported segment %d %llxe\n", this->segment,
                  m->addr[0]);
        }

        w->outstandingReqs++;
        w->memReqsInPipe--;
    }

    template<typename OperationType, typename SrcDataType,
             typename AddrRegOperandType>
    void
    StInst<OperationType, SrcDataType,
           AddrRegOperandType>::execute(GPUDynInstPtr gpuDynInst)
    {
        Wavefront *w = gpuDynInst->wavefront();

        typedef typename OperationType::CType CType;

        const VectorMask &mask = w->getPred();

        // arg references are handled uniquely for now (no Memory Request
        // is used), so special-case them up front.  Someday we should
        // make this more realistic, at which we should get rid of this
        // block and fold this case into the switch below.
        if (this->segment == Brig::BRIG_SEGMENT_ARG) {
            uint64_t address = this->addr.calcUniform();

            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                if (mask[lane]) {
                    CType data = this->src.template get<CType>(w, lane);
                    DPRINTF(HSAIL, "st_arg [%d] <- %d\n", address, data);
                    w->writeCallArgMem<CType>(lane, address, data);
                }
            }

            return;
        }

        GPUDynInstPtr m = gpuDynInst;

        m->exec_mask = w->execMask();

        this->addr.calcVector(w, m->addr);

        if (num_src_operands == 1) {
            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                if (mask[lane]) {
                    ((CType*)m->d_data)[lane] =
                        this->src.template get<CType>(w, lane);
                }
            }
        } else {
            for (int k= 0; k < num_src_operands; ++k) {
                for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                    if (mask[lane]) {
                        ((CType*)m->d_data)[k * w->computeUnit->wfSize() + lane] =
                            this->src_vect[k].template get<CType>(w, lane);
                    }
                }
            }
        }

        m->m_type = OperationType::memType;
        m->v_type = OperationType::vgprType;

        m->statusBitVector = 0;
        m->equiv = this->equivClass;

        if (num_src_operands == 1) {
            m->n_reg = 1;
        } else {
            m->n_reg = num_src_operands;
        }

        m->simdId = w->simdId;
        m->wfSlotId = w->wfSlotId;
        m->wfDynId = w->wfDynId;
        m->kern_id = w->kernId;
        m->cu_id = w->computeUnit->cu_id;
        m->latency.init(&w->computeUnit->shader->tick_cnt);

        switch (this->segment) {
          case Brig::BRIG_SEGMENT_GLOBAL:
            m->pipeId = GLBMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(1));

            // this is a complete hack to get around a compiler bug
            // (the compiler currently generates global access for private
            //  addresses (starting from 0). We need to add the private offset)
            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                if (mask[lane]) {
                    if (m->addr[lane] < w->privSizePerItem) {

                        // calcPrivAddr will fail if accesses are unaligned
                        assert(!((sizeof(CType)-1) & m->addr[lane]));

                        Addr privAddr = calcPrivAddr(m->addr[lane], w, lane,
                                                     this);

                        m->addr[lane] = privAddr;
                    }
                }
            }

            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsWrGm++;
            w->wrGmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_SPILL:
            assert(num_src_operands == 1);
            m->pipeId = GLBMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(1));
            {
                for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                    if (mask[lane]) {
                        assert(m->addr[lane] < w->spillSizePerItem);

                        m->addr[lane] = m->addr[lane] * w->spillWidth +
                                        lane * sizeof(CType) + w->spillBase;
                    }
                }
            }

            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsWrGm++;
            w->wrGmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_GROUP:
            m->pipeId = LDSMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(24));
            w->computeUnit->localMemoryPipe.getLMReqFIFO().push(m);
            w->outstandingReqsWrLm++;
            w->wrLmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_PRIVATE:
            m->pipeId = GLBMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(1));
            {
                for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                    if (mask[lane]) {
                        assert(m->addr[lane] < w->privSizePerItem);
                        m->addr[lane] = m->addr[lane] + lane *
                            sizeof(CType)+w->privBase;
                    }
                }
            }

            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsWrGm++;
            w->wrGmReqsInPipe--;
            break;

          default:
            fatal("Store to unsupported segment %d\n", this->segment);
        }

        w->outstandingReqs++;
        w->memReqsInPipe--;
    }

    template<typename OperationType, typename SrcDataType,
             typename AddrRegOperandType>
    void
    StInst<OperationType, SrcDataType,
           AddrRegOperandType>::generateDisassembly()
    {
        switch (num_src_operands) {
          case 1:
            this->disassembly = csprintf("%s_%s_%s %s,%s", this->opcode,
                                         segmentNames[this->segment],
                                         OperationType::label,
                                         this->src.disassemble(),
                                         this->addr.disassemble());
            break;
          case 2:
            this->disassembly = csprintf("%s_%s_%s (%s,%s), %s", this->opcode,
                                         segmentNames[this->segment],
                                         OperationType::label,
                                         this->src_vect[0].disassemble(),
                                         this->src_vect[1].disassemble(),
                                         this->addr.disassemble());
            break;
          case 4:
            this->disassembly = csprintf("%s_%s_%s (%s,%s,%s,%s), %s",
                                         this->opcode,
                                         segmentNames[this->segment],
                                         OperationType::label,
                                         this->src_vect[0].disassemble(),
                                         this->src_vect[1].disassemble(),
                                         this->src_vect[2].disassemble(),
                                         this->src_vect[3].disassemble(),
                                         this->addr.disassemble());
            break;
          default: fatal("Bad ld register src operand, num vector operands: "
                         "%d \n", num_src_operands);
            break;
        }
    }

    template<typename DataType, typename AddrRegOperandType, int NumSrcOperands,
             bool HasDst>
    void
    AtomicInst<DataType, AddrRegOperandType, NumSrcOperands,
        HasDst>::execute(GPUDynInstPtr gpuDynInst)
    {
        typedef typename DataType::CType CType;

        Wavefront *w = gpuDynInst->wavefront();

        GPUDynInstPtr m = gpuDynInst;

        this->addr.calcVector(w, m->addr);

        for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
            ((CType *)m->a_data)[lane] =
                this->src[0].template get<CType>(w, lane);
        }

        // load second source operand for CAS
        if (NumSrcOperands > 1) {
            for (int lane = 0; lane < w->computeUnit->wfSize(); ++lane) {
                ((CType*)m->x_data)[lane] =
                    this->src[1].template get<CType>(w, lane);
            }
        }

        assert(NumSrcOperands <= 2);

        m->m_type = DataType::memType;
        m->v_type = DataType::vgprType;

        m->exec_mask = w->execMask();
        m->statusBitVector = 0;
        m->equiv = 0;  // atomics don't have an equivalence class operand
        m->n_reg = 1;

        if (HasDst) {
            m->dst_reg = this->dest.regIndex();
        }

        m->simdId = w->simdId;
        m->wfSlotId = w->wfSlotId;
        m->wfDynId = w->wfDynId;
        m->kern_id = w->kernId;
        m->cu_id = w->computeUnit->cu_id;
        m->latency.init(&w->computeUnit->shader->tick_cnt);

        switch (this->segment) {
          case Brig::BRIG_SEGMENT_GLOBAL:
            m->latency.set(w->computeUnit->shader->ticks(64));
            m->pipeId = GLBMEM_PIPE;

            w->computeUnit->globalMemoryPipe.issueRequest(m);
            w->outstandingReqsWrGm++;
            w->wrGmReqsInPipe--;
            w->outstandingReqsRdGm++;
            w->rdGmReqsInPipe--;
            break;

          case Brig::BRIG_SEGMENT_GROUP:
            m->pipeId = LDSMEM_PIPE;
            m->latency.set(w->computeUnit->shader->ticks(24));
            w->computeUnit->localMemoryPipe.getLMReqFIFO().push(m);
            w->outstandingReqsWrLm++;
            w->wrLmReqsInPipe--;
            w->outstandingReqsRdLm++;
            w->rdLmReqsInPipe--;
            break;

          default:
            fatal("Atomic op to unsupported segment %d\n",
                  this->segment);
        }

        w->outstandingReqs++;
        w->memReqsInPipe--;
    }

    const char* atomicOpToString(Brig::BrigAtomicOperation atomicOp);

    template<typename DataType, typename AddrRegOperandType, int NumSrcOperands,
             bool HasDst>
    void
    AtomicInst<DataType, AddrRegOperandType, NumSrcOperands,
               HasDst>::generateDisassembly()
    {
        if (HasDst) {
            this->disassembly =
                csprintf("%s_%s_%s_%s %s,%s", this->opcode,
                         atomicOpToString(this->atomicOperation),
                         segmentNames[this->segment],
                         DataType::label, this->dest.disassemble(),
                         this->addr.disassemble());
        } else {
            this->disassembly =
                csprintf("%s_%s_%s_%s %s", this->opcode,
                         atomicOpToString(this->atomicOperation),
                         segmentNames[this->segment],
                         DataType::label, this->addr.disassemble());
        }

        for (int i = 0; i < NumSrcOperands; ++i) {
            this->disassembly += ",";
            this->disassembly += this->src[i].disassemble();
        }
    }
} // namespace HsailISA
