/*
 * Copyright (c) 2016-2017 Advanced Micro Devices, Inc.
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
 *
 * Authors: Anthony Gutierrez
 */

#ifndef __ARCH_GCN3_INSTS_OP_ENCODINGS_HH__
#define __ARCH_GCN3_INSTS_OP_ENCODINGS_HH__

#include "arch/gcn3/gpu_decoder.hh"
#include "arch/gcn3/insts/gpu_static_inst.hh"
#include "arch/gcn3/operand.hh"
#include "debug/GPUExec.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace Gcn3ISA
{
    // --- purely virtual instruction classes ---

    class Inst_SOP2 : public GCN3GPUStaticInst
    {
      public:
        Inst_SOP2(InFmt_SOP2*, const std::string &opcode);

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_SOP2 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_SOP2 *);
    }; // Inst_SOP2

    class Inst_SOPK : public GCN3GPUStaticInst
    {
      public:
        Inst_SOPK(InFmt_SOPK*, const std::string &opcode);
        ~Inst_SOPK();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_SOPK instData;
    }; // Inst_SOPK

    class Inst_SOP1 : public GCN3GPUStaticInst
    {
      public:
        Inst_SOP1(InFmt_SOP1*, const std::string &opcode);
        ~Inst_SOP1();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_SOP1 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_SOP1 *);
    }; // Inst_SOP1

    class Inst_SOPC : public GCN3GPUStaticInst
    {
      public:
        Inst_SOPC(InFmt_SOPC*, const std::string &opcode);
        ~Inst_SOPC();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_SOPC instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_SOPC *);
    }; // Inst_SOPC

    class Inst_SOPP : public GCN3GPUStaticInst
    {
      public:
        Inst_SOPP(InFmt_SOPP*, const std::string &opcode);
        ~Inst_SOPP();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_SOPP instData;
    }; // Inst_SOPP

    class Inst_SMEM : public GCN3GPUStaticInst
    {
      public:
        Inst_SMEM(InFmt_SMEM*, const std::string &opcode);
        ~Inst_SMEM();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        /**
         * initiate a memory read access for N dwords
         */
        template<int N>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            int block_size = gpuDynInst->computeUnit()->cacheLineSize();
            int req_size = N * sizeof(ScalarRegU32);
            Addr vaddr = gpuDynInst->scalarAddr;

            /**
             * the base address of the cache line where the the last byte of
             * the request will be stored.
             */
            Addr split_addr = roundDown(vaddr + req_size - 1, block_size);

            assert(split_addr <= vaddr || split_addr - vaddr < block_size);
            /**
             * if the base cache line address of the last byte is greater
             * than the address of the first byte then we have a misaligned
             * access.
             */
            bool misaligned_acc = split_addr > vaddr;

            RequestPtr req = new Request(0, vaddr, req_size, 0,
                    gpuDynInst->computeUnit()->masterId(), 0,
                    gpuDynInst->wfDynId);

            if (misaligned_acc) {
                RequestPtr req1, req2;
                req->splitOnVaddr(split_addr, req1, req2);
                gpuDynInst->numScalarReqs = 2;
                gpuDynInst->setRequestFlags(req1);
                gpuDynInst->setRequestFlags(req2);
                PacketPtr pkt1 = new Packet(req1, MemCmd::ReadReq);
                PacketPtr pkt2 = new Packet(req2, MemCmd::ReadReq);
                pkt1->dataStatic(gpuDynInst->scalar_data);
                pkt2->dataStatic(gpuDynInst->scalar_data + req1->getSize());
                gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt1);
                gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt2);
                delete req;
            } else {
                gpuDynInst->numScalarReqs = 1;
                gpuDynInst->setRequestFlags(req);
                PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
                pkt->dataStatic(gpuDynInst->scalar_data);
                gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt);
            }
        }

        /**
         * initiate a memory write access for N dwords
         */
        template<int N>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            int block_size = gpuDynInst->computeUnit()->cacheLineSize();
            int req_size = N * sizeof(ScalarRegU32);
            Addr vaddr = gpuDynInst->scalarAddr;

            /**
             * the base address of the cache line where the the last byte of
             * the request will be stored.
             */
            Addr split_addr = roundDown(vaddr + req_size - 1, block_size);

            assert(split_addr <= vaddr || split_addr - vaddr < block_size);
            /**
             * if the base cache line address of the last byte is greater
             * than the address of the first byte then we have a misaligned
             * access.
             */
            bool misaligned_acc = split_addr > vaddr;

            RequestPtr req = new Request(0, vaddr, req_size, 0,
                    gpuDynInst->computeUnit()->masterId(), 0,
                    gpuDynInst->wfDynId);

            if (misaligned_acc) {
                RequestPtr req1, req2;
                req->splitOnVaddr(split_addr, req1, req2);
                gpuDynInst->numScalarReqs = 2;
                gpuDynInst->setRequestFlags(req1);
                gpuDynInst->setRequestFlags(req2);
                PacketPtr pkt1 = new Packet(req1, MemCmd::WriteReq);
                PacketPtr pkt2 = new Packet(req2, MemCmd::WriteReq);
                pkt1->dataStatic(gpuDynInst->scalar_data);
                pkt2->dataStatic(gpuDynInst->scalar_data + req1->getSize());
                gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt1);
                gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt2);
                delete req;
            } else {
                gpuDynInst->numScalarReqs = 1;
                gpuDynInst->setRequestFlags(req);
                PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
                pkt->dataStatic(gpuDynInst->scalar_data);
                gpuDynInst->computeUnit()->sendScalarRequest(gpuDynInst, pkt);
            }
        }

        void
        calcAddr(GPUDynInstPtr gpuDynInst, ConstScalarOperandU64 &addr,
            ScalarRegU32 offset)
        {
            Addr vaddr = addr.rawData();
            vaddr += offset;
            vaddr &= ~0x3;
            gpuDynInst->scalarAddr = vaddr;
        }

        // first instruction DWORD
        InFmt_SMEM instData;
        // second instruction DWORD
        InFmt_SMEM_1 extData;
    }; // Inst_SMEM

    class Inst_VOP2 : public GCN3GPUStaticInst
    {
      public:
        Inst_VOP2(InFmt_VOP2*, const std::string &opcode);
        ~Inst_VOP2();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_VOP2 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_VOP2 *);
    }; // Inst_VOP2

    class Inst_VOP1 : public GCN3GPUStaticInst
    {
      public:
        Inst_VOP1(InFmt_VOP1*, const std::string &opcode);
        ~Inst_VOP1();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_VOP1 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_VOP1 *);
    }; // Inst_VOP1

    class Inst_VOPC : public GCN3GPUStaticInst
    {
      public:
        Inst_VOPC(InFmt_VOPC*, const std::string &opcode);
        ~Inst_VOPC();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_VOPC instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_VOPC *);
    }; // Inst_VOPC

    class Inst_VINTRP : public GCN3GPUStaticInst
    {
      public:
        Inst_VINTRP(InFmt_VINTRP*, const std::string &opcode);
        ~Inst_VINTRP();

        int instSize() const override;

      protected:
        // first instruction DWORD
        InFmt_VINTRP instData;
    }; // Inst_VINTRP

    class Inst_VOP3 : public GCN3GPUStaticInst
    {
      public:
        Inst_VOP3(InFmt_VOP3*, const std::string &opcode, bool sgpr_dst);
        ~Inst_VOP3();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_VOP3 instData;
        // second instruction DWORD
        InFmt_VOP3_1 extData;

      private:
        bool hasSecondDword(InFmt_VOP3 *);
        /**
         * the v_cmp and readlane instructions in the VOP3
         * encoding are unique because they are the only
         * instructions that use the VDST field to specify
         * a scalar register destination. for VOP3::V_CMP insts
         * VDST specifies the arbitrary SGPR pair used to write
         * VCC. for V_READLANE VDST specifies the SGPR to return
         * the value of the selected lane in the source VGPR
         * from which we are reading.
         */
        const bool sgprDst;
    }; // Inst_VOP3

    class Inst_VOP3_SDST_ENC : public GCN3GPUStaticInst
    {
      public:
        Inst_VOP3_SDST_ENC(InFmt_VOP3_SDST_ENC*, const std::string &opcode);
        ~Inst_VOP3_SDST_ENC();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        // first instruction DWORD
        InFmt_VOP3_SDST_ENC instData;
        // second instruction DWORD
        InFmt_VOP3_1 extData;

      private:
        bool hasSecondDword(InFmt_VOP3_SDST_ENC *);
    }; // Inst_VOP3_SDST_ENC

    class Inst_DS : public GCN3GPUStaticInst
    {
      public:
        Inst_DS(InFmt_DS*, const std::string &opcode);
        ~Inst_DS();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        template<typename T>
        void
        initMemRead(GPUDynInstPtr gpuDynInst, Addr offset)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane] + offset;

                    (reinterpret_cast<T*>(gpuDynInst->d_data))[lane]
                        = wf->ldsChunk->read<T>(vaddr);
                }
            }
        }

        template<typename T>
        void
        initDualMemRead(GPUDynInstPtr gpuDynInst, Addr offset0, Addr offset1)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr0 = gpuDynInst->addr[lane] + offset0;
                    Addr vaddr1 = gpuDynInst->addr[lane] + offset1;

                    (reinterpret_cast<T*>(gpuDynInst->d_data))[lane * 2]
                        = wf->ldsChunk->read<T>(vaddr0);
                    (reinterpret_cast<T*>(gpuDynInst->d_data))[lane * 2 + 1]
                        = wf->ldsChunk->read<T>(vaddr1);
                }
            }
        }

        template<typename T>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst, Addr offset)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane] + offset;
                    wf->ldsChunk->write<T>(vaddr,
                        (reinterpret_cast<T*>(gpuDynInst->d_data))[lane]);
                }
            }
        }

        template<typename T>
        void
        initDualMemWrite(GPUDynInstPtr gpuDynInst, Addr offset0, Addr offset1)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr0 = gpuDynInst->addr[lane] + offset0;
                    Addr vaddr1 = gpuDynInst->addr[lane] + offset1;
                    wf->ldsChunk->write<T>(vaddr0, (reinterpret_cast<T*>(
                        gpuDynInst->d_data))[lane * 2]);
                    wf->ldsChunk->write<T>(vaddr1, (reinterpret_cast<T*>(
                        gpuDynInst->d_data))[lane * 2 + 1]);
                }
            }
        }

        void
        calcAddr(GPUDynInstPtr gpuDynInst, ConstVecOperandU32 &addr)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (wf->execMask(lane)) {
                    gpuDynInst->addr.at(lane) = (Addr)addr[lane];
                }
            }
        }

        // first instruction DWORD
        InFmt_DS instData;
        // second instruction DWORD
        InFmt_DS_1 extData;
    }; // Inst_DS

    class Inst_MUBUF : public GCN3GPUStaticInst
    {
      public:
        Inst_MUBUF(InFmt_MUBUF*, const std::string &opcode);
        ~Inst_MUBUF();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        struct BufferRsrcDescriptor
        {
            uint64_t baseAddr : 48;
            uint32_t stride : 14;
            uint32_t cacheSwizzle : 1;
            uint32_t swizzleEn : 1;
            uint32_t numRecords : 32;
            uint32_t dstSelX : 3;
            uint32_t dstSelY : 3;
            uint32_t dstSelZ : 3;
            uint32_t dstSelW : 3;
            uint32_t numFmt : 3;
            uint32_t dataFmt : 4;
            uint32_t elemSize : 2;
            uint32_t idxStride : 2;
            uint32_t addTidEn : 1;
            uint32_t atc : 1;
            uint32_t hashEn : 1;
            uint32_t heap : 1;
            uint32_t mType : 3;
            uint32_t type : 2;
        };

        template<typename T>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane];

                    RequestPtr req = new Request(0, vaddr, sizeof(T), 0,
                        gpuDynInst->computeUnit()->masterId(), 0,
                        gpuDynInst->wfDynId);

                    gpuDynInst->setRequestFlags(req);

                    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
                    pkt->dataStatic(&(reinterpret_cast<T*>(
                        gpuDynInst->d_data))[lane]);

                    gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane,
                        pkt);
                }
            }
        }

        template<typename T>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane];

                    RequestPtr req = new Request(0, vaddr, sizeof(T), 0,
                        gpuDynInst->computeUnit()->masterId(),
                        0, gpuDynInst->wfDynId);

                    gpuDynInst->setRequestFlags(req);
                    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                    pkt->dataStatic(&(reinterpret_cast<T*>(
                        gpuDynInst->d_data))[lane]);
                    gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane,
                        pkt);
                }
            }
        }

        void
        injectGlobalMemFence(GPUDynInstPtr gpuDynInst)
        {
            // create request and set flags
            gpuDynInst->statusBitVector = VectorMask(1);
            Request *req = new Request(0, 0, 0, 0,
                                       gpuDynInst->computeUnit()->
                                       masterId(), 0,
                                       gpuDynInst->wfDynId);
            gpuDynInst->setRequestFlags(req);
            gpuDynInst->computeUnit()->
                injectGlobalMemFence(gpuDynInst, false, req);
        }

        /**
         * MUBUF insructions calculate their addresses as follows:
         *
         * index  = (IDXEN ? vgpr_idx : 0) + (const_add_tid_en ? TID : 0)
         * offset = (OFFEN ? vgpr_off : 0) + inst_off
         *
         * / ====================== LINEAR ADDRESSING ====================== /
         * VADDR = base + sgpr_off + offset + stride * index
         *
         * / ===================== SWIZZLED ADDRESSING ===================== /
         * index_msb  = index / const_index_stride
         * index_lsb  = index % const_index_stride
         * offset_msb = offset / const_element_size
         * offset_lsb = offset % const_element_size
         * buffer_offset = ((index_msb * stride + offset_msb *
         *                  const_element_size) * const_index_stride +
         *                  index_lsb * const_element_size + offset_lsb)
         *
         * VADDR = base + sgpr_off + buffer_offset
         */
        template<typename VOFF, typename VIDX, typename SRSRC, typename SOFF>
        void
        calcAddr(GPUDynInstPtr gpuDynInst, VOFF v_off, VIDX v_idx,
            SRSRC s_rsrc_desc, SOFF s_offset, int inst_offset)
        {
            Addr vaddr = 0;
            Addr base_addr = 0;
            Addr stride = 0;
            Addr buf_idx = 0;
            Addr buf_off = 0;
            BufferRsrcDescriptor rsrc_desc;

            std::memcpy((void*)&rsrc_desc, s_rsrc_desc.rawDataPtr(),
                sizeof(BufferRsrcDescriptor));

            base_addr = rsrc_desc.baseAddr;

            stride = rsrc_desc.addTidEn ? ((rsrc_desc.dataFmt << 14)
                + rsrc_desc.stride) : rsrc_desc.stride;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    vaddr = base_addr + s_offset.rawData();
                    /**
                     * first we calculate the buffer's index and offset.
                     * these will be used for either linear or swizzled
                     * buffers.
                     */
                    buf_idx = v_idx[lane] + (rsrc_desc.addTidEn ? lane : 0);

                    buf_off = v_off[lane] + inst_offset;

                    if (rsrc_desc.swizzleEn) {
                        Addr idx_stride = 8 << rsrc_desc.idxStride;
                        Addr elem_size = 2 << rsrc_desc.elemSize;
                        Addr idx_msb = buf_idx / idx_stride;
                        Addr idx_lsb = buf_idx % idx_stride;
                        Addr off_msb = buf_off / elem_size;
                        Addr off_lsb = buf_off % elem_size;

                        vaddr += ((idx_msb * stride + off_msb * elem_size)
                            * idx_stride + idx_lsb * elem_size + off_lsb);
                    } else {
                        vaddr += buf_off + stride * buf_idx;
                    }

                    gpuDynInst->addr.at(lane) = vaddr;
                }
            }
        }

        // first instruction DWORD
        InFmt_MUBUF instData;
        // second instruction DWORD
        InFmt_MUBUF_1 extData;
    }; // Inst_MUBUF

    class Inst_MTBUF : public GCN3GPUStaticInst
    {
      public:
        Inst_MTBUF(InFmt_MTBUF*, const std::string &opcode);
        ~Inst_MTBUF();

        int instSize() const override;

      protected:
        // first instruction DWORD
        InFmt_MTBUF instData;
        // second instruction DWORD
        InFmt_MTBUF_1 extData;

      private:
        bool hasSecondDword(InFmt_MTBUF *);
    }; // Inst_MTBUF

    class Inst_MIMG : public GCN3GPUStaticInst
    {
      public:
        Inst_MIMG(InFmt_MIMG*, const std::string &opcode);
        ~Inst_MIMG();

        int instSize() const override;

      protected:
        // first instruction DWORD
        InFmt_MIMG instData;
        // second instruction DWORD
        InFmt_MIMG_1 extData;
    }; // Inst_MIMG

    class Inst_EXP : public GCN3GPUStaticInst
    {
      public:
        Inst_EXP(InFmt_EXP*, const std::string &opcode);
        ~Inst_EXP();

        int instSize() const override;

      protected:
        // first instruction DWORD
        InFmt_EXP instData;
        // second instruction DWORD
        InFmt_EXP_1 extData;
    }; // Inst_EXP

    class Inst_FLAT : public GCN3GPUStaticInst
    {
      public:
        Inst_FLAT(InFmt_FLAT*, const std::string &opcode);
        ~Inst_FLAT();

        int instSize() const override;
        void generateDisassembly() override;

        bool isScalarRegister(int opIdx) override;
        bool isVectorRegister(int opIdx) override;
        int getRegisterIndex(int opIdx, GPUDynInstPtr gpuDynInst) override;

      protected:
        template<typename T>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane];

                    RequestPtr req = new Request(0, vaddr, sizeof(T), 0,
                            gpuDynInst->computeUnit()->masterId(), 0,
                            gpuDynInst->wfDynId);

                    gpuDynInst->setRequestFlags(req);
                    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
                    pkt->dataStatic(&(reinterpret_cast<T*>(
                        gpuDynInst->d_data))[lane]);
                    gpuDynInst->computeUnit()
                        ->sendRequest(gpuDynInst, lane, pkt);
                }
            }
        }

        template<int N>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            int req_size = N * sizeof(VecElemU32);
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane];

                    RequestPtr req = new Request(0, vaddr, req_size, 0,
                        gpuDynInst->computeUnit()->masterId(), 0,
                        gpuDynInst->wfDynId);

                   gpuDynInst->setRequestFlags(req);
                   PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
                   pkt->dataStatic(&(reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * N]);
                   gpuDynInst->computeUnit()
                        ->sendRequest(gpuDynInst, lane, pkt);
                }
            }
        }

        template<typename T>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane];

                    RequestPtr req = new Request(0, vaddr, sizeof(T), 0,
                        gpuDynInst->computeUnit()->masterId(),
                            0, gpuDynInst->wfDynId);

                    gpuDynInst->setRequestFlags(req);
                    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                    pkt->dataStatic(&(reinterpret_cast<T*>(
                        gpuDynInst->d_data))[lane]);
                    gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane,
                                                           pkt);
                }
            }
        }

        template<int N>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            int req_size = N * sizeof(VecElemU32);
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane];

                    RequestPtr req = new Request(0, vaddr, req_size, 0,
                        gpuDynInst->computeUnit()->masterId(),
                            0, gpuDynInst->wfDynId);

                    gpuDynInst->setRequestFlags(req);
                    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                    pkt->dataStatic(&(reinterpret_cast<VecElemU32*>(
                        gpuDynInst->d_data))[lane * N]);
                    gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane,
                        pkt);
                }
            }
        }

        template<typename T>
        void
        initAtomicAccess(GPUDynInstPtr gpuDynInst)
        {
            gpuDynInst->statusBitVector = gpuDynInst->exec_mask;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane];

                    RequestPtr req = new Request(0, vaddr, sizeof(T), 0,
                        gpuDynInst->computeUnit()->masterId(), 0,
                        gpuDynInst->wfDynId,
                        gpuDynInst->makeAtomicOpFunctor<T>(
                            &(reinterpret_cast<T*>(gpuDynInst->a_data))[lane],
                            &(reinterpret_cast<T*>(
                                gpuDynInst->x_data))[lane]));

                    gpuDynInst->setRequestFlags(req);

                    PacketPtr pkt = new Packet(req, MemCmd::SwapReq);
                    pkt->dataStatic(&(reinterpret_cast<T*>(
                        gpuDynInst->d_data))[lane]);

                    gpuDynInst->computeUnit()->sendRequest(gpuDynInst, lane,
                        pkt);
                }
            }
        }

        void
        calcAddr(GPUDynInstPtr gpuDynInst, ConstVecOperandU64 &addr)
        {
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    gpuDynInst->addr.at(lane) = addr[lane];
                }
            }
            gpuDynInst->resolveFlatSegment(gpuDynInst->exec_mask);
        }

        // first instruction DWORD
        InFmt_FLAT instData;
        // second instruction DWORD
        InFmt_FLAT_1 extData;
    }; // Inst_FLAT
} // namespace Gcn3ISA

#endif // __ARCH_GCN3_INSTS_OP_ENCODINGS_HH__
