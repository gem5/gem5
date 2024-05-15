/*
 * Copyright (c) 2016-2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_VEGA_INSTS_OP_ENCODINGS_HH__
#define __ARCH_VEGA_INSTS_OP_ENCODINGS_HH__

#include "arch/amdgpu/vega/gpu_decoder.hh"
#include "arch/amdgpu/vega/gpu_mem_helpers.hh"
#include "arch/amdgpu/vega/insts/gpu_static_inst.hh"
#include "arch/amdgpu/vega/operand.hh"
#include "debug/GPUExec.hh"
#include "debug/VEGA.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace VegaISA
{
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

    // --- purely virtual instruction classes ---

    class Inst_SOP2 : public VEGAGPUStaticInst
    {
      public:
        Inst_SOP2(InFmt_SOP2*, const std::string &opcode);

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_SOP2 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_SOP2 *);
    }; // Inst_SOP2

    class Inst_SOPK : public VEGAGPUStaticInst
    {
      public:
        Inst_SOPK(InFmt_SOPK*, const std::string &opcode);
        ~Inst_SOPK();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_SOPK instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_SOPK *);
    }; // Inst_SOPK

    class Inst_SOP1 : public VEGAGPUStaticInst
    {
      public:
        Inst_SOP1(InFmt_SOP1*, const std::string &opcode);
        ~Inst_SOP1();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_SOP1 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_SOP1 *);
    }; // Inst_SOP1

    class Inst_SOPC : public VEGAGPUStaticInst
    {
      public:
        Inst_SOPC(InFmt_SOPC*, const std::string &opcode);
        ~Inst_SOPC();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_SOPC instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_SOPC *);
    }; // Inst_SOPC

    class Inst_SOPP : public VEGAGPUStaticInst
    {
      public:
        Inst_SOPP(InFmt_SOPP*, const std::string &opcode);
        ~Inst_SOPP();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_SOPP instData;
    }; // Inst_SOPP

    class Inst_SMEM : public VEGAGPUStaticInst
    {
      public:
        Inst_SMEM(InFmt_SMEM*, const std::string &opcode);
        ~Inst_SMEM();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        /**
         * initiate a memory read access for N dwords
         */
        template<int N>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            initMemReqScalarHelper<ScalarRegU32, N>(gpuDynInst,
                                                    MemCmd::ReadReq);
        }

        /**
         * initiate a memory write access for N dwords
         */
        template<int N>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            initMemReqScalarHelper<ScalarRegU32, N>(gpuDynInst,
                                                    MemCmd::WriteReq);
        }

        /**
         * For normal s_load_dword/s_store_dword instruction addresses.
         */
        void
        calcAddr(GPUDynInstPtr gpu_dyn_inst, ConstScalarOperandU64 &addr,
                 ScalarRegU32 offset)
        {
            Addr vaddr = ((addr.rawData() + offset) & ~0x3);
            gpu_dyn_inst->scalarAddr = vaddr;
        }

        /**
         * For s_buffer_load_dword/s_buffer_store_dword instruction addresses.
         * The s_buffer instructions use the same buffer resource descriptor
         * as the MUBUF instructions.
         */
        void
        calcAddr(GPUDynInstPtr gpu_dyn_inst,
                 ConstScalarOperandU128 &s_rsrc_desc, ScalarRegU32 offset)
        {
            BufferRsrcDescriptor rsrc_desc;
            ScalarRegU32 clamped_offset(offset);
            std::memcpy((void*)&rsrc_desc, s_rsrc_desc.rawDataPtr(),
                        sizeof(BufferRsrcDescriptor));

            /**
             * The address is clamped if:
             *     Stride is zero: clamp if offset >= num_records
             *     Stride is non-zero: clamp if offset > (stride * num_records)
             */
            if (!rsrc_desc.stride && offset >= rsrc_desc.numRecords) {
                clamped_offset = rsrc_desc.numRecords;
            } else if (rsrc_desc.stride && offset
                       > (rsrc_desc.stride * rsrc_desc.numRecords)) {
                clamped_offset = (rsrc_desc.stride * rsrc_desc.numRecords);
            }

            Addr vaddr = ((rsrc_desc.baseAddr + clamped_offset) & ~0x3);
            gpu_dyn_inst->scalarAddr = vaddr;
        }

        // first instruction DWORD
        InFmt_SMEM instData;
        // second instruction DWORD
        InFmt_SMEM_1 extData;
    }; // Inst_SMEM

    class Inst_VOP2 : public VEGAGPUStaticInst
    {
      public:
        Inst_VOP2(InFmt_VOP2*, const std::string &opcode);
        ~Inst_VOP2();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_VOP2 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

        template<typename T>
        T sdwaSrcHelper(GPUDynInstPtr gpuDynInst, T & src1)
        {
            T src0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
            // use copies of original src0, src1, and dest during selecting
            T origSrc0_sdwa(gpuDynInst, extData.iFmt_VOP_SDWA.SRC0);
            T origSrc1(gpuDynInst, instData.VSRC1);

            src0_sdwa.read();
            origSrc0_sdwa.read();
            origSrc1.read();

            DPRINTF(VEGA, "Handling %s SRC SDWA. SRC0: register v[%d], "
                "DST_SEL: %d, DST_U: %d, CLMP: %d, SRC0_SEL: %d, SRC0_SEXT: "
                "%d, SRC0_NEG: %d, SRC0_ABS: %d, SRC1_SEL: %d, SRC1_SEXT: %d, "
                "SRC1_NEG: %d, SRC1_ABS: %d\n",
                opcode().c_str(), extData.iFmt_VOP_SDWA.SRC0,
                extData.iFmt_VOP_SDWA.DST_SEL, extData.iFmt_VOP_SDWA.DST_U,
                extData.iFmt_VOP_SDWA.CLMP, extData.iFmt_VOP_SDWA.SRC0_SEL,
                extData.iFmt_VOP_SDWA.SRC0_SEXT,
                extData.iFmt_VOP_SDWA.SRC0_NEG, extData.iFmt_VOP_SDWA.SRC0_ABS,
                extData.iFmt_VOP_SDWA.SRC1_SEL,
                extData.iFmt_VOP_SDWA.SRC1_SEXT,
                extData.iFmt_VOP_SDWA.SRC1_NEG,
                extData.iFmt_VOP_SDWA.SRC1_ABS);

            processSDWA_src(extData.iFmt_VOP_SDWA, src0_sdwa, origSrc0_sdwa,
                            src1, origSrc1);

            return src0_sdwa;
        }

        template<typename T>
        void sdwaDstHelper(GPUDynInstPtr gpuDynInst, T & vdst)
        {
            T origVdst(gpuDynInst, instData.VDST);

            Wavefront *wf = gpuDynInst->wavefront();
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (wf->execMask(lane)) {
                    origVdst[lane] = vdst[lane]; // keep copy consistent
                }
            }

            processSDWA_dst(extData.iFmt_VOP_SDWA, vdst, origVdst);
        }

        template<typename T>
        T dppHelper(GPUDynInstPtr gpuDynInst, T & src1)
        {
            T src0_dpp(gpuDynInst, extData.iFmt_VOP_DPP.SRC0);
            src0_dpp.read();

            DPRINTF(VEGA, "Handling %s SRC DPP. SRC0: register v[%d], "
                "DPP_CTRL: 0x%#x, SRC0_ABS: %d, SRC0_NEG: %d, SRC1_ABS: %d, "
                "SRC1_NEG: %d, BC: %d, BANK_MASK: %d, ROW_MASK: %d\n",
                opcode().c_str(), extData.iFmt_VOP_DPP.SRC0,
                extData.iFmt_VOP_DPP.DPP_CTRL, extData.iFmt_VOP_DPP.SRC0_ABS,
                extData.iFmt_VOP_DPP.SRC0_NEG, extData.iFmt_VOP_DPP.SRC1_ABS,
                extData.iFmt_VOP_DPP.SRC1_NEG, extData.iFmt_VOP_DPP.BC,
                extData.iFmt_VOP_DPP.BANK_MASK, extData.iFmt_VOP_DPP.ROW_MASK);

            processDPP(gpuDynInst, extData.iFmt_VOP_DPP, src0_dpp, src1);

            return src0_dpp;
        }

        template<typename ConstT, typename T>
        void vop2Helper(GPUDynInstPtr gpuDynInst,
                        void (*fOpImpl)(T&, T&, T&, Wavefront*))
        {
            Wavefront *wf = gpuDynInst->wavefront();
            T src0(gpuDynInst, instData.SRC0);
            T src1(gpuDynInst, instData.VSRC1);
            T vdst(gpuDynInst, instData.VDST);

            src0.readSrc();
            src1.read();

            if (isSDWAInst()) {
                T src0_sdwa = sdwaSrcHelper(gpuDynInst, src1);
                fOpImpl(src0_sdwa, src1, vdst, wf);
                sdwaDstHelper(gpuDynInst, vdst);
            } else if (isDPPInst()) {
                T src0_dpp = dppHelper(gpuDynInst, src1);
                fOpImpl(src0_dpp, src1, vdst, wf);
            } else {
                // src0 is unmodified. We need to use the const container
                // type to allow reading scalar operands from src0. Only
                // src0 can index scalar operands. We copy this to vdst
                // temporarily to pass to the lambda so the instruction
                // does not need to write two lambda functions (one for
                // a const src0 and one of a mutable src0).
                ConstT const_src0(gpuDynInst, instData.SRC0);
                const_src0.readSrc();

                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    vdst[lane] = const_src0[lane];
                }
                fOpImpl(vdst, src1, vdst, wf);
            }

            vdst.write();
        }

      private:
        bool hasSecondDword(InFmt_VOP2 *);
    }; // Inst_VOP2

    class Inst_VOP1 : public VEGAGPUStaticInst
    {
      public:
        Inst_VOP1(InFmt_VOP1*, const std::string &opcode);
        ~Inst_VOP1();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_VOP1 instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_VOP1 *);
    }; // Inst_VOP1

    class Inst_VOPC : public VEGAGPUStaticInst
    {
      public:
        Inst_VOPC(InFmt_VOPC*, const std::string &opcode);
        ~Inst_VOPC();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_VOPC instData;
        // possible second DWORD
        InstFormat extData;
        uint32_t varSize;

      private:
        bool hasSecondDword(InFmt_VOPC *);
    }; // Inst_VOPC

    class Inst_VINTRP : public VEGAGPUStaticInst
    {
      public:
        Inst_VINTRP(InFmt_VINTRP*, const std::string &opcode);
        ~Inst_VINTRP();

        int instSize() const override;

      protected:
        // first instruction DWORD
        InFmt_VINTRP instData;
    }; // Inst_VINTRP

    class Inst_VOP3A : public VEGAGPUStaticInst
    {
      public:
        Inst_VOP3A(InFmt_VOP3A*, const std::string &opcode, bool sgpr_dst);
        ~Inst_VOP3A();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_VOP3A instData;
        // second instruction DWORD
        InFmt_VOP3_1 extData;

      private:
        bool hasSecondDword(InFmt_VOP3A *);
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
    }; // Inst_VOP3A

    class Inst_VOP3B : public VEGAGPUStaticInst
    {
      public:
        Inst_VOP3B(InFmt_VOP3B*, const std::string &opcode);
        ~Inst_VOP3B();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_VOP3B instData;
        // second instruction DWORD
        InFmt_VOP3_1 extData;

      private:
        bool hasSecondDword(InFmt_VOP3B *);
    }; // Inst_VOP3B

    class Inst_VOP3P : public VEGAGPUStaticInst
    {
      public:
        Inst_VOP3P(InFmt_VOP3P*, const std::string &opcode);
        ~Inst_VOP3P();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_VOP3P instData;
        // second instruction DWORD
        InFmt_VOP3P_1 extData;

        template<typename T>
        void vop3pHelper(GPUDynInstPtr gpuDynInst,
                        T (*fOpImpl)(T, T, bool))
        {
            Wavefront *wf = gpuDynInst->wavefront();
            ConstVecOperandU32 S0(gpuDynInst, extData.SRC0);
            ConstVecOperandU32 S1(gpuDynInst, extData.SRC1);
            VecOperandU32 D(gpuDynInst, instData.VDST);

            S0.readSrc();
            S1.readSrc();

            int opLo = instData.OPSEL;
            int opHi = instData.OPSEL_HI2 << 2 | extData.OPSEL_HI;
            int negLo = extData.NEG;
            int negHi = instData.NEG_HI;
            bool clamp = instData.CLMP;
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (wf->execMask(lane)) {
                    T upper_val = fOpImpl(word<T>(S0[lane], opHi, negHi, 0),
                                          word<T>(S1[lane], opHi, negHi, 1),
                                          clamp);
                    T lower_val = fOpImpl(word<T>(S0[lane], opLo, negLo, 0),
                                          word<T>(S1[lane], opLo, negLo, 1),
                                          clamp);

                    uint16_t upper_raw =
                        *reinterpret_cast<uint16_t*>(&upper_val);
                    uint16_t lower_raw =
                        *reinterpret_cast<uint16_t*>(&lower_val);

                    D[lane] = upper_raw << 16 | lower_raw;
                }
            }

            D.write();
        }

        template<typename T>
        void vop3pHelper(GPUDynInstPtr gpuDynInst,
                        T (*fOpImpl)(T, T, T, bool))
        {
            Wavefront *wf = gpuDynInst->wavefront();
            ConstVecOperandU32 S0(gpuDynInst, extData.SRC0);
            ConstVecOperandU32 S1(gpuDynInst, extData.SRC1);
            ConstVecOperandU32 S2(gpuDynInst, extData.SRC2);
            VecOperandU32 D(gpuDynInst, instData.VDST);

            S0.readSrc();
            S1.readSrc();
            S2.readSrc();

            int opLo = instData.OPSEL;
            int opHi = instData.OPSEL_HI2 << 2 | extData.OPSEL_HI;
            int negLo = extData.NEG;
            int negHi = instData.NEG_HI;
            bool clamp = instData.CLMP;
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (wf->execMask(lane)) {
                    T upper_val = fOpImpl(word<T>(S0[lane], opHi, negHi, 0),
                                          word<T>(S1[lane], opHi, negHi, 1),
                                          word<T>(S2[lane], opHi, negHi, 2),
                                          clamp);
                    T lower_val = fOpImpl(word<T>(S0[lane], opLo, negLo, 0),
                                          word<T>(S1[lane], opLo, negLo, 1),
                                          word<T>(S2[lane], opLo, negLo, 2),
                                          clamp);

                    uint16_t upper_raw =
                        *reinterpret_cast<uint16_t*>(&upper_val);
                    uint16_t lower_raw =
                        *reinterpret_cast<uint16_t*>(&lower_val);

                    D[lane] = upper_raw << 16 | lower_raw;
                }
            }

            D.write();
        }

        void
        dotHelper(GPUDynInstPtr gpuDynInst,
                  uint32_t (*fOpImpl)(uint32_t, uint32_t, uint32_t, bool))
        {
            Wavefront *wf = gpuDynInst->wavefront();
            ConstVecOperandU32 S0(gpuDynInst, extData.SRC0);
            ConstVecOperandU32 S1(gpuDynInst, extData.SRC1);
            ConstVecOperandU32 S2(gpuDynInst, extData.SRC2);
            VecOperandU32 D(gpuDynInst, instData.VDST);

            S0.readSrc();
            S1.readSrc();
            S2.readSrc();

            // OPSEL[2] and OPSEL_HI2 are unused. Craft two dwords where:
            // dword1[15:0]  is upper/lower 16b of src0 based on opsel[0]
            // dword1[31:15] is upper/lower 16b of src0 based on opsel_hi[0]
            // dword2[15:0]  is upper/lower 16b of src1 based on opsel[1]
            // dword2[31:15] is upper/lower 16b of src1 based on opsel_hi[1]
            int opLo = instData.OPSEL;
            int opHi = extData.OPSEL_HI;
            int negLo = extData.NEG;
            int negHi = instData.NEG_HI;
            bool clamp = instData.CLMP;

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (wf->execMask(lane)) {
                    uint32_t dword1l =
                        word<uint16_t>(S0[lane], opLo, negLo, 0);
                    uint32_t dword1h =
                        word<uint16_t>(S0[lane], opHi, negHi, 0);
                    uint32_t dword2l =
                        word<uint16_t>(S1[lane], opLo, negLo, 1);
                    uint32_t dword2h =
                        word<uint16_t>(S1[lane], opHi, negHi, 1);

                    uint32_t dword1 = (dword1h << 16) | dword1l;
                    uint32_t dword2 = (dword2h << 16) | dword2l;

                    // Take in two uint32_t dwords and one src2 dword. The
                    // function will need to call bits to break up to the
                    // correct size and then reinterpret cast to the correct
                    // value.
                    D[lane] = fOpImpl(dword1, dword2, S2[lane], clamp);
                }
            }

            D.write();
        }

      private:
        bool hasSecondDword(InFmt_VOP3P *);

        template<typename T>
        T
        word(uint32_t data, int opSel, int neg, int opSelBit)
        {
            // This method assumes two words packed into a dword
            static_assert(sizeof(T) == 2);

            bool select = bits(opSel, opSelBit, opSelBit);
            uint16_t raw = select ? bits(data, 31, 16)
                                  : bits(data, 15, 0);

            // Apply input modifiers. This may seem odd, but the hardware
            // just flips the MSb instead of doing unary negation.
            bool negate = bits(neg, opSelBit, opSelBit);
            if (negate) {
                raw ^= 0x8000;
            }

            return *reinterpret_cast<T*>(&raw);
        }
    }; // Inst_VOP3P

    class Inst_VOP3P_MAI : public VEGAGPUStaticInst
    {
      public:
        Inst_VOP3P_MAI(InFmt_VOP3P_MAI*, const std::string &opcode);
        ~Inst_VOP3P_MAI();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_VOP3P_MAI instData;
        // second instruction DWORD
        InFmt_VOP3P_MAI_1 extData;

      private:
        bool hasSecondDword(InFmt_VOP3P_MAI *);
    }; // Inst_VOP3P

    class Inst_DS : public VEGAGPUStaticInst
    {
      public:
        Inst_DS(InFmt_DS*, const std::string &opcode);
        ~Inst_DS();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

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

        template<int N>
        void
        initMemRead(GPUDynInstPtr gpuDynInst, Addr offset)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane] + offset;
                    for (int i = 0; i < N; ++i) {
                        (reinterpret_cast<VecElemU32*>(
                            gpuDynInst->d_data))[lane * N + i]
                            = wf->ldsChunk->read<VecElemU32>(
                                vaddr + i*sizeof(VecElemU32));
                    }
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

        template<int N>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst, Addr offset)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane] + offset;
                    for (int i = 0; i < N; ++i) {
                        wf->ldsChunk->write<VecElemU32>(
                            vaddr + i*sizeof(VecElemU32),
                            (reinterpret_cast<VecElemU32*>(
                                gpuDynInst->d_data))[lane * N + i]);
                    }
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

        template<typename T>
        void
        initAtomicAccess(GPUDynInstPtr gpuDynInst, Addr offset)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    Addr vaddr = gpuDynInst->addr[lane] + offset;

                    AtomicOpFunctorPtr amo_op =
                        gpuDynInst->makeAtomicOpFunctor<T>(
                        &(reinterpret_cast<T*>(gpuDynInst->a_data))[lane],
                        &(reinterpret_cast<T*>(gpuDynInst->x_data))[lane]);

                    (reinterpret_cast<T*>(gpuDynInst->d_data))[lane]
                        = wf->ldsChunk->atomic<T>(vaddr, std::move(amo_op));
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

    class Inst_MUBUF : public VEGAGPUStaticInst
    {
      public:
        Inst_MUBUF(InFmt_MUBUF*, const std::string &opcode);
        ~Inst_MUBUF();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        template<typename T>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            // temporarily modify exec_mask to supress memory accesses to oob
            // regions.  Only issue memory requests for lanes that have their
            // exec_mask set and are not out of bounds.
            VectorMask old_exec_mask = gpuDynInst->exec_mask;
            gpuDynInst->exec_mask &= ~oobMask;
            initMemReqHelper<T, 1>(gpuDynInst, MemCmd::ReadReq);
            gpuDynInst->exec_mask = old_exec_mask;
        }


        template<int N>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            // temporarily modify exec_mask to supress memory accesses to oob
            // regions.  Only issue memory requests for lanes that have their
            // exec_mask set and are not out of bounds.
            VectorMask old_exec_mask = gpuDynInst->exec_mask;
            gpuDynInst->exec_mask &= ~oobMask;
            initMemReqHelper<VecElemU32, N>(gpuDynInst, MemCmd::ReadReq);
            gpuDynInst->exec_mask = old_exec_mask;
        }

        template<typename T>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            // temporarily modify exec_mask to supress memory accesses to oob
            // regions.  Only issue memory requests for lanes that have their
            // exec_mask set and are not out of bounds.
            VectorMask old_exec_mask = gpuDynInst->exec_mask;
            gpuDynInst->exec_mask &= ~oobMask;
            initMemReqHelper<T, 1>(gpuDynInst, MemCmd::WriteReq);
            gpuDynInst->exec_mask = old_exec_mask;
        }

        template<int N>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            // temporarily modify exec_mask to supress memory accesses to oob
            // regions.  Only issue memory requests for lanes that have their
            // exec_mask set and are not out of bounds.
            VectorMask old_exec_mask = gpuDynInst->exec_mask;
            gpuDynInst->exec_mask &= ~oobMask;
            initMemReqHelper<VecElemU32, N>(gpuDynInst, MemCmd::WriteReq);
            gpuDynInst->exec_mask = old_exec_mask;
        }

        template<typename T>
        void
        initAtomicAccess(GPUDynInstPtr gpuDynInst)
        {
            // temporarily modify exec_mask to supress memory accesses to oob
            // regions.  Only issue memory requests for lanes that have their
            // exec_mask set and are not out of bounds.
            VectorMask old_exec_mask = gpuDynInst->exec_mask;
            gpuDynInst->exec_mask &= ~oobMask;
            initMemReqHelper<T, 1>(gpuDynInst, MemCmd::SwapReq, true);
            gpuDynInst->exec_mask = old_exec_mask;
        }

        void
        injectGlobalMemFence(GPUDynInstPtr gpuDynInst)
        {
            // create request and set flags
            gpuDynInst->resetEntireStatusVector();
            gpuDynInst->setStatusVector(0, 1);
            RequestPtr req = std::make_shared<Request>(0, 0, 0,
                                       gpuDynInst->computeUnit()->
                                       requestorId(), 0,
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
            Addr buffer_offset = 0;
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
                        DPRINTF(VEGA, "mubuf swizzled lane %d: "
                                "idx_stride = %llx, elem_size = %llx, "
                                "idx_msb = %llx, idx_lsb = %llx, "
                                "off_msb = %llx, off_lsb = %llx\n",
                                lane, idx_stride, elem_size, idx_msb, idx_lsb,
                                off_msb, off_lsb);

                        buffer_offset =(idx_msb * stride + off_msb * elem_size)
                            * idx_stride + idx_lsb * elem_size + off_lsb;
                    } else {
                        buffer_offset = buf_off + stride * buf_idx;
                    }


                    /**
                     * Range check behavior causes out of range accesses to
                     * to be treated differently. Out of range accesses return
                     * 0 for loads and are ignored for stores. For
                     * non-formatted accesses, this is done on a per-lane
                     * basis.
                     */
                    if (rsrc_desc.stride == 0 || !rsrc_desc.swizzleEn) {
                        if (buffer_offset >=
                            rsrc_desc.numRecords - s_offset.rawData()) {
                            DPRINTF(VEGA, "mubuf out-of-bounds condition 1: "
                                    "lane = %d, buffer_offset = %llx, "
                                    "const_stride = %llx, "
                                    "const_num_records = %llx\n",
                                    lane, buf_off + stride * buf_idx,
                                    stride, rsrc_desc.numRecords);
                            oobMask.set(lane);
                            continue;
                        }
                    }

                    if (rsrc_desc.stride != 0 && rsrc_desc.swizzleEn) {
                        if (buf_idx >= rsrc_desc.numRecords ||
                            buf_off >= stride) {
                            DPRINTF(VEGA, "mubuf out-of-bounds condition 2: "
                                    "lane = %d, offset = %llx, "
                                    "index = %llx, "
                                    "const_num_records = %llx\n",
                                    lane, buf_off, buf_idx,
                                    rsrc_desc.numRecords);
                            oobMask.set(lane);
                            continue;
                        }
                    }

                    vaddr += buffer_offset;

                    DPRINTF(VEGA, "Calculating mubuf address for lane %d: "
                            "vaddr = %llx, base_addr = %llx, "
                            "stride = %llx, buf_idx = %llx, buf_off = %llx\n",
                            lane, vaddr, base_addr, stride,
                            buf_idx, buf_off);
                    gpuDynInst->addr.at(lane) = vaddr;
                }
            }
        }

        // first instruction DWORD
        InFmt_MUBUF instData;
        // second instruction DWORD
        InFmt_MUBUF_1 extData;
        // Mask of lanes with out-of-bounds accesses.  Needs to be tracked
        // seperately from the exec_mask so that we remember to write zero
        // to the registers associated with out of bounds lanes.
        VectorMask oobMask;
    }; // Inst_MUBUF

    class Inst_MTBUF : public VEGAGPUStaticInst
    {
      public:
        Inst_MTBUF(InFmt_MTBUF*, const std::string &opcode);
        ~Inst_MTBUF();

        int instSize() const override;
        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_MTBUF instData;
        // second instruction DWORD
        InFmt_MTBUF_1 extData;

      private:
        bool hasSecondDword(InFmt_MTBUF *);
    }; // Inst_MTBUF

    class Inst_MIMG : public VEGAGPUStaticInst
    {
      public:
        Inst_MIMG(InFmt_MIMG*, const std::string &opcode);
        ~Inst_MIMG();

        int instSize() const override;
        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_MIMG instData;
        // second instruction DWORD
        InFmt_MIMG_1 extData;
    }; // Inst_MIMG

    class Inst_EXP : public VEGAGPUStaticInst
    {
      public:
        Inst_EXP(InFmt_EXP*, const std::string &opcode);
        ~Inst_EXP();

        int instSize() const override;
        void initOperandInfo() override;

      protected:
        // first instruction DWORD
        InFmt_EXP instData;
        // second instruction DWORD
        InFmt_EXP_1 extData;
    }; // Inst_EXP

    class Inst_FLAT : public VEGAGPUStaticInst
    {
      public:
        Inst_FLAT(InFmt_FLAT*, const std::string &opcode);
        ~Inst_FLAT();

        int instSize() const override;
        void generateDisassembly() override;

        void initOperandInfo() override;

      protected:
        template<typename T>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            if (gpuDynInst->executedAs() == enums::SC_GLOBAL ||
                gpuDynInst->executedAs() == enums::SC_PRIVATE) {
                initMemReqHelper<T, 1>(gpuDynInst, MemCmd::ReadReq);
            } else if (gpuDynInst->executedAs() == enums::SC_GROUP) {
                Wavefront *wf = gpuDynInst->wavefront();
                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        Addr vaddr = gpuDynInst->addr[lane];
                        (reinterpret_cast<T*>(gpuDynInst->d_data))[lane]
                            = wf->ldsChunk->read<T>(vaddr);
                    }
                }
            }
        }

        template<int N>
        void
        initMemRead(GPUDynInstPtr gpuDynInst)
        {
            if (gpuDynInst->executedAs() == enums::SC_GLOBAL ||
                gpuDynInst->executedAs() == enums::SC_PRIVATE) {
                initMemReqHelper<VecElemU32, N>(gpuDynInst, MemCmd::ReadReq);
            } else if (gpuDynInst->executedAs() == enums::SC_GROUP) {
                Wavefront *wf = gpuDynInst->wavefront();
                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        Addr vaddr = gpuDynInst->addr[lane];
                        for (int i = 0; i < N; ++i) {
                            (reinterpret_cast<VecElemU32*>(
                                gpuDynInst->d_data))[lane * N + i]
                                = wf->ldsChunk->read<VecElemU32>(
                                        vaddr + i*sizeof(VecElemU32));
                        }
                    }
                }
            }
        }

        template<typename T>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            if (gpuDynInst->executedAs() == enums::SC_GLOBAL ||
                gpuDynInst->executedAs() == enums::SC_PRIVATE) {
                initMemReqHelper<T, 1>(gpuDynInst, MemCmd::WriteReq);
            } else if (gpuDynInst->executedAs() == enums::SC_GROUP) {
                Wavefront *wf = gpuDynInst->wavefront();
                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        Addr vaddr = gpuDynInst->addr[lane];
                        wf->ldsChunk->write<T>(vaddr,
                            (reinterpret_cast<T*>(gpuDynInst->d_data))[lane]);
                    }
                }
            }
        }

        template<int N>
        void
        initMemWrite(GPUDynInstPtr gpuDynInst)
        {
            if (gpuDynInst->executedAs() == enums::SC_GLOBAL ||
                gpuDynInst->executedAs() == enums::SC_PRIVATE) {
                initMemReqHelper<VecElemU32, N>(gpuDynInst, MemCmd::WriteReq);
            } else if (gpuDynInst->executedAs() == enums::SC_GROUP) {
                Wavefront *wf = gpuDynInst->wavefront();
                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        Addr vaddr = gpuDynInst->addr[lane];
                        for (int i = 0; i < N; ++i) {
                            wf->ldsChunk->write<VecElemU32>(
                                vaddr + i*sizeof(VecElemU32),
                                (reinterpret_cast<VecElemU32*>(
                                    gpuDynInst->d_data))[lane * N + i]);
                        }
                    }
                }
            }
        }

        template<typename T>
        void
        initAtomicAccess(GPUDynInstPtr gpuDynInst)
        {
            // Flat scratch requests may not be atomic according to ISA manual
            // up to MI200. See MI200 manual Table 45.
            assert(gpuDynInst->executedAs() != enums::SC_PRIVATE);

            if (gpuDynInst->executedAs() == enums::SC_GLOBAL) {
                initMemReqHelper<T, 1>(gpuDynInst, MemCmd::SwapReq, true);
            } else if (gpuDynInst->executedAs() == enums::SC_GROUP) {
                Wavefront *wf = gpuDynInst->wavefront();
                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        Addr vaddr = gpuDynInst->addr[lane];
                        auto amo_op =
                            gpuDynInst->makeAtomicOpFunctor<T>(
                                &(reinterpret_cast<T*>(
                                    gpuDynInst->a_data))[lane],
                                &(reinterpret_cast<T*>(
                                    gpuDynInst->x_data))[lane]);

                        T tmp = wf->ldsChunk->read<T>(vaddr);
                        (*amo_op)(reinterpret_cast<uint8_t *>(&tmp));
                        wf->ldsChunk->write<T>(vaddr, tmp);
                        (reinterpret_cast<T*>(gpuDynInst->d_data))[lane] = tmp;
                    }
                }
            }
        }

        void
        calcAddr(GPUDynInstPtr gpuDynInst, ScalarRegU32 vaddr,
                 ScalarRegU32 saddr, ScalarRegI32 offset)
        {
            // Offset is a 13-bit field w/the following meanings:
            // In Flat instructions, offset is a 12-bit unsigned number
            // In Global/Scratch instructions, offset is a 13-bit signed number
            if (isFlat()) {
                offset = offset & 0xfff;
            } else {
                offset = (ScalarRegI32)sext<13>(offset);
            }
            // If saddr = 0x7f there is no scalar reg to read and address will
            // be a 64-bit address. Otherwise, saddr is the reg index for a
            // scalar reg used as the base address for a 32-bit address.
            if ((saddr == 0x7f && isFlatGlobal()) || isFlat()) {
                ConstVecOperandU64 vbase(gpuDynInst, vaddr);
                vbase.read();

                calcAddrVgpr(gpuDynInst, vbase, offset);
            } else if (isFlatGlobal()) {
                // Assume we are operating in 64-bit mode and read a pair of
                // SGPRs for the address base.
                ConstScalarOperandU64 sbase(gpuDynInst, saddr);
                sbase.read();

                ConstVecOperandU32 voffset(gpuDynInst, vaddr);
                voffset.read();

                calcAddrSgpr(gpuDynInst, voffset, sbase, offset);
            // For scratch, saddr = 0x7f there is no scalar reg to read and
            // a vgpr will be used for address offset. Otherwise, saddr is
            // the sgpr index holding the address offset. For scratch
            // instructions the offset GPR is always 32-bits.
            } else if (saddr != 0x7f) {
                assert(isFlatScratch());

                ConstScalarOperandU32 soffset(gpuDynInst, saddr);
                soffset.read();

                Addr flat_scratch_addr = readFlatScratch(gpuDynInst);

                int elemSize;
                auto staticInst = gpuDynInst->staticInstruction();
                if (gpuDynInst->isLoad()) {
                    elemSize = staticInst->getOperandSize(2);
                } else {
                    assert(gpuDynInst->isStore());
                    elemSize = staticInst->getOperandSize(1);
                }

                unsigned swizzleOffset = soffset.rawData() + offset;
                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        gpuDynInst->addr.at(lane) = flat_scratch_addr
                            + swizzle(swizzleOffset, lane, elemSize);
                    }
                }
            } else {
                assert(isFlatScratch());

                ConstVecOperandU32 voffset(gpuDynInst, vaddr);
                voffset.read();

                Addr flat_scratch_addr = readFlatScratch(gpuDynInst);

                int elemSize;
                auto staticInst = gpuDynInst->staticInstruction();
                if (gpuDynInst->isLoad()) {
                    elemSize = staticInst->getOperandSize(2);
                } else {
                    assert(gpuDynInst->isStore());
                    elemSize = staticInst->getOperandSize(1);
                }

                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        gpuDynInst->addr.at(lane) = flat_scratch_addr
                            + swizzle(voffset[lane] + offset, lane, elemSize);
                    }
                }
            }

            if (isFlat()) {
                gpuDynInst->resolveFlatSegment(gpuDynInst->exec_mask);
            } else if (isFlatGlobal()) {
                gpuDynInst->staticInstruction()->executed_as =
                    enums::SC_GLOBAL;
            } else {
                assert(isFlatScratch());
                gpuDynInst->staticInstruction()->executed_as =
                    enums::SC_PRIVATE;
                gpuDynInst->resolveFlatSegment(gpuDynInst->exec_mask);
            }
        }

        void
        issueRequestHelper(GPUDynInstPtr gpuDynInst)
        {
            if ((gpuDynInst->executedAs() == enums::SC_GLOBAL && isFlat())
                    || isFlatGlobal()) {
                gpuDynInst->computeUnit()->globalMemoryPipe
                    .issueRequest(gpuDynInst);
            } else if (gpuDynInst->executedAs() == enums::SC_GROUP) {
                assert(isFlat());
                gpuDynInst->computeUnit()->localMemoryPipe
                    .issueRequest(gpuDynInst);
            } else {
                assert(gpuDynInst->executedAs() == enums::SC_PRIVATE);
                gpuDynInst->computeUnit()->globalMemoryPipe
                    .issueRequest(gpuDynInst);
            }
        }

        // Execute for atomics is identical besides the flag set in the
        // constructor, except cmpswap. For cmpswap, the offset to the "cmp"
        // register is needed. For all other operations this offset is zero
        // and implies the atomic is not a cmpswap.
        // RegT defines the type of GPU register (e.g., ConstVecOperandU32)
        // LaneT defines the type of the register elements (e.g., VecElemU32)
        template<typename RegT, typename LaneT, int CmpRegOffset = 0>
        void
        atomicExecute(GPUDynInstPtr gpuDynInst)
        {
            Wavefront *wf = gpuDynInst->wavefront();

            if (gpuDynInst->exec_mask.none()) {
                wf->decVMemInstsIssued();
                if (isFlat()) {
                    wf->decLGKMInstsIssued();
                }
                return;
            }

            gpuDynInst->execUnitId = wf->execUnitId;
            gpuDynInst->latency.init(gpuDynInst->computeUnit());
            gpuDynInst->latency.set(gpuDynInst->computeUnit()->clockPeriod());

            RegT data(gpuDynInst, extData.DATA);
            RegT cmp(gpuDynInst, extData.DATA + CmpRegOffset);

            data.read();
            if constexpr (CmpRegOffset) {
                cmp.read();
            }

            calcAddr(gpuDynInst, extData.ADDR, extData.SADDR, instData.OFFSET);

            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    if constexpr (CmpRegOffset) {
                        (reinterpret_cast<VecElemU32*>(
                            gpuDynInst->x_data))[lane] = data[lane];
                        (reinterpret_cast<VecElemU32*>(
                            gpuDynInst->a_data))[lane] = cmp[lane];
                    } else {
                        (reinterpret_cast<LaneT*>(gpuDynInst->a_data))[lane]
                            = data[lane];
                    }
                }
            }

            issueRequestHelper(gpuDynInst);
        }

        // RegT defines the type of GPU register (e.g., ConstVecOperandU32)
        // LaneT defines the type of the register elements (e.g., VecElemU32)
        template<typename RegT, typename LaneT>
        void
        atomicComplete(GPUDynInstPtr gpuDynInst)
        {
            if (isAtomicRet()) {
                RegT vdst(gpuDynInst, extData.VDST);

                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (gpuDynInst->exec_mask[lane]) {
                        vdst[lane] = (reinterpret_cast<LaneT*>(
                            gpuDynInst->d_data))[lane];
                    }
                }

                vdst.write();
            }
        }

        bool
        vgprIsOffset()
        {
            return (extData.SADDR != 0x7f);
        }

        // first instruction DWORD
        InFmt_FLAT instData;
        // second instruction DWORD
        InFmt_FLAT_1 extData;

      private:
        void initFlatOperandInfo();
        void initGlobalScratchOperandInfo();

        void generateFlatDisassembly();
        void generateGlobalScratchDisassembly();

        void
        calcAddrSgpr(GPUDynInstPtr gpuDynInst, ConstVecOperandU32 &vaddr,
                     ConstScalarOperandU64 &saddr, ScalarRegI32 offset)
        {
            // Use SGPR pair as a base address and add VGPR-offset and
            // instruction offset. The VGPR-offset is always 32-bits so we
            // mask any upper bits from the vaddr.
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    ScalarRegI32 voffset = vaddr[lane];
                    gpuDynInst->addr.at(lane) =
                        saddr.rawData() + voffset + offset;
                }
            }
        }

        void
        calcAddrVgpr(GPUDynInstPtr gpuDynInst, ConstVecOperandU64 &addr,
                     ScalarRegI32 offset)
        {
            for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                if (gpuDynInst->exec_mask[lane]) {
                    gpuDynInst->addr.at(lane) = addr[lane] + offset;
                }
            }
        }

        VecElemU32
        swizzle(VecElemU32 offset, int lane, int elem_size)
        {
            // This is not described in the spec. We use the swizzle from
            // buffer memory instructions and fix the stride to 4. Multiply
            // the thread ID by the storage size to avoid threads clobbering
            // their data.
            return ((offset / 4) * 4 * 64)
                + (offset % 4) + (lane * elem_size);
        }

        Addr
        readFlatScratch(GPUDynInstPtr gpuDynInst)
        {
            return gpuDynInst->computeUnit()->shader->getScratchBase();
        }
    }; // Inst_FLAT
} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_VEGA_INSTS_OP_ENCODINGS_HH__
