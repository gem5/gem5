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

#ifndef __ARCH_HSAIL_INSTS_DECL_HH__
#define __ARCH_HSAIL_INSTS_DECL_HH__

#include <cmath>

#include "arch/hsail/insts/gpu_static_inst.hh"
#include "arch/hsail/operand.hh"
#include "debug/HSAIL.hh"
#include "gpu-compute/gpu_dyn_inst.hh"
#include "gpu-compute/shader.hh"

namespace HsailISA
{
    template<typename _DestOperand, typename _SrcOperand>
    class HsailOperandType
    {
      public:
        typedef _DestOperand DestOperand;
        typedef _SrcOperand SrcOperand;
    };

    typedef HsailOperandType<CRegOperand, CRegOrImmOperand> CRegOperandType;
    typedef HsailOperandType<SRegOperand, SRegOrImmOperand> SRegOperandType;
    typedef HsailOperandType<DRegOperand, DRegOrImmOperand> DRegOperandType;

    // The IsBits parameter serves only to disambiguate tbhe B* types from
    // the U* types, which otherwise would be identical (and
    // indistinguishable).
    template<typename _OperandType, typename _CType, Enums::MemType _memType,
             vgpr_type _vgprType, int IsBits=0>
    class HsailDataType
    {
      public:
        typedef _OperandType OperandType;
        typedef _CType CType;
        static const Enums::MemType memType = _memType;
        static const vgpr_type vgprType = _vgprType;
        static const char *label;
    };

    typedef HsailDataType<CRegOperandType, bool, Enums::M_U8, VT_32, 1> B1;
    typedef HsailDataType<SRegOperandType, uint8_t, Enums::M_U8, VT_32, 1> B8;

    typedef HsailDataType<SRegOperandType, uint16_t,
                          Enums::M_U16, VT_32, 1> B16;

    typedef HsailDataType<SRegOperandType, uint32_t,
                          Enums::M_U32, VT_32, 1> B32;

    typedef HsailDataType<DRegOperandType, uint64_t,
                          Enums::M_U64, VT_64, 1> B64;

    typedef HsailDataType<SRegOperandType, int8_t, Enums::M_S8, VT_32> S8;
    typedef HsailDataType<SRegOperandType, int16_t, Enums::M_S16, VT_32> S16;
    typedef HsailDataType<SRegOperandType, int32_t, Enums::M_S32, VT_32> S32;
    typedef HsailDataType<DRegOperandType, int64_t, Enums::M_S64, VT_64> S64;

    typedef HsailDataType<SRegOperandType, uint8_t, Enums::M_U8, VT_32> U8;
    typedef HsailDataType<SRegOperandType, uint16_t, Enums::M_U16, VT_32> U16;
    typedef HsailDataType<SRegOperandType, uint32_t, Enums::M_U32, VT_32> U32;
    typedef HsailDataType<DRegOperandType, uint64_t, Enums::M_U64, VT_64> U64;

    typedef HsailDataType<SRegOperandType, float, Enums::M_F32, VT_32> F32;
    typedef HsailDataType<DRegOperandType, double, Enums::M_F64, VT_64> F64;

    template<typename DestOperandType, typename SrcOperandType,
             int NumSrcOperands>
    class CommonInstBase : public HsailGPUStaticInst
    {
      protected:
        typename DestOperandType::DestOperand dest;
        typename SrcOperandType::SrcOperand src[NumSrcOperands];

        void
        generateDisassembly()
        {
            disassembly = csprintf("%s%s %s", opcode, opcode_suffix(),
                                   dest.disassemble());

            for (int i = 0; i < NumSrcOperands; ++i) {
                disassembly += ",";
                disassembly += src[i].disassemble();
            }
        }

        virtual std::string opcode_suffix() = 0;

      public:
        CommonInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj,
                       const char *opcode)
            : HsailGPUStaticInst(obj, opcode)
        {
            setFlag(ALU);

            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);

            dest.init(op_offs, obj);

            for (int i = 0; i < NumSrcOperands; ++i) {
                op_offs = obj->getOperandPtr(ib->operands, i + 1);
                src[i].init(op_offs, obj);
            }
        }

        bool isVectorRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return src[operandIndex].isVectorRegister();
            else
                return dest.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return src[operandIndex].isCondRegister();
            else
                return dest.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return src[operandIndex].isScalarRegister();
            else
                return dest.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return true;
            return false;
        }

        bool isDstOperand(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex >= NumSrcOperands)
                return true;
            return false;
        }
        int getOperandSize(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < NumSrcOperands)
                return src[operandIndex].opSize();
            else
                return dest.opSize();
        }
        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            assert(operandIndex >= 0 && operandIndex < getNumOperands());

            if (operandIndex < NumSrcOperands)
                return src[operandIndex].regIndex();
            else
                return dest.regIndex();
        }
        int numSrcRegOperands() {
            int operands = 0;
            for (int i = 0; i < NumSrcOperands; i++) {
                if (src[i].isVectorRegister()) {
                    operands++;
                }
            }
            return operands;
        }
        int numDstRegOperands() { return dest.isVectorRegister(); }
        int getNumOperands() { return NumSrcOperands + 1; }
    };

    template<typename DataType, int NumSrcOperands>
    class ArithInst : public CommonInstBase<typename DataType::OperandType,
                                            typename DataType::OperandType,
                                            NumSrcOperands>
    {
      public:
        std::string opcode_suffix() { return csprintf("_%s", DataType::label); }

        ArithInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                  const char *opcode)
            : CommonInstBase<typename DataType::OperandType,
                             typename DataType::OperandType,
                             NumSrcOperands>(ib, obj, opcode)
        {
        }
    };

    template<typename DestOperandType, typename Src0OperandType,
             typename Src1OperandType, typename Src2OperandType>
    class ThreeNonUniformSourceInstBase : public HsailGPUStaticInst
    {
      protected:
        typename DestOperandType::DestOperand dest;
        typename Src0OperandType::SrcOperand  src0;
        typename Src1OperandType::SrcOperand  src1;
        typename Src2OperandType::SrcOperand  src2;

        void
        generateDisassembly()
        {
            disassembly = csprintf("%s %s,%s,%s,%s", opcode, dest.disassemble(),
                                   src0.disassemble(), src1.disassemble(),
                                   src2.disassemble());
        }

      public:
        ThreeNonUniformSourceInstBase(const Brig::BrigInstBase *ib,
                                      const BrigObject *obj,
                                      const char *opcode)
            : HsailGPUStaticInst(obj, opcode)
        {
            setFlag(ALU);

            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            dest.init(op_offs, obj);

            op_offs = obj->getOperandPtr(ib->operands, 1);
            src0.init(op_offs, obj);

            op_offs = obj->getOperandPtr(ib->operands, 2);
            src1.init(op_offs, obj);

            op_offs = obj->getOperandPtr(ib->operands, 3);
            src2.init(op_offs, obj);
        }

        bool isVectorRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.isVectorRegister();
            else if (operandIndex == 1)
                return src1.isVectorRegister();
            else if (operandIndex == 2)
                return src2.isVectorRegister();
            else
                return dest.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.isCondRegister();
            else if (operandIndex == 1)
                return src1.isCondRegister();
            else if (operandIndex == 2)
                return src2.isCondRegister();
            else
                return dest.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.isScalarRegister();
            else if (operandIndex == 1)
                return src1.isScalarRegister();
            else if (operandIndex == 2)
                return src2.isScalarRegister();
            else
                return dest.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < 3)
                return true;
            else
                return false;
        }
        bool isDstOperand(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex >= 3)
                return true;
            else
                return false;
        }
        int getOperandSize(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.opSize();
            else if (operandIndex == 1)
                return src1.opSize();
            else if (operandIndex == 2)
                return src2.opSize();
            else
                return dest.opSize();
        }

        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.regIndex();
            else if (operandIndex == 1)
                return src1.regIndex();
            else if (operandIndex == 2)
                return src2.regIndex();
            else
                return dest.regIndex();
        }

        int numSrcRegOperands() {
            int operands = 0;
            if (src0.isVectorRegister()) {
                operands++;
            }
            if (src1.isVectorRegister()) {
                operands++;
            }
            if (src2.isVectorRegister()) {
                operands++;
            }
            return operands;
        }
        int numDstRegOperands() { return dest.isVectorRegister(); }
        int getNumOperands() { return 4; }
    };

    template<typename DestDataType, typename Src0DataType,
             typename Src1DataType, typename Src2DataType>
    class ThreeNonUniformSourceInst :
        public ThreeNonUniformSourceInstBase<typename DestDataType::OperandType,
                                             typename Src0DataType::OperandType,
                                             typename Src1DataType::OperandType,
                                             typename Src2DataType::OperandType>
    {
      public:
        typedef typename DestDataType::CType DestCType;
        typedef typename Src0DataType::CType Src0CType;
        typedef typename Src1DataType::CType Src1CType;
        typedef typename Src2DataType::CType Src2CType;

        ThreeNonUniformSourceInst(const Brig::BrigInstBase *ib,
                                  const BrigObject *obj, const char *opcode)
            : ThreeNonUniformSourceInstBase<typename DestDataType::OperandType,
                                         typename Src0DataType::OperandType,
                                         typename Src1DataType::OperandType,
                                         typename Src2DataType::OperandType>(ib,
                                                                    obj, opcode)
        {
        }
    };

    template<typename DataType>
    class CmovInst : public ThreeNonUniformSourceInst<DataType, B1,
                                                      DataType, DataType>
    {
      public:
        CmovInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                 const char *opcode)
            : ThreeNonUniformSourceInst<DataType, B1, DataType,
                                        DataType>(ib, obj, opcode)
        {
        }
    };

    template<typename DataType>
    class ExtractInsertInst : public ThreeNonUniformSourceInst<DataType,
                                                               DataType, U32,
                                                               U32>
    {
      public:
        ExtractInsertInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                          const char *opcode)
            : ThreeNonUniformSourceInst<DataType, DataType, U32,
                                        U32>(ib, obj, opcode)
        {
        }
    };

    template<typename DestOperandType, typename Src0OperandType,
             typename Src1OperandType>
    class TwoNonUniformSourceInstBase : public HsailGPUStaticInst
    {
      protected:
        typename DestOperandType::DestOperand dest;
        typename Src0OperandType::SrcOperand src0;
        typename Src1OperandType::SrcOperand src1;

        void
        generateDisassembly()
        {
            disassembly = csprintf("%s %s,%s,%s", opcode, dest.disassemble(),
                                   src0.disassemble(), src1.disassemble());
        }


      public:
        TwoNonUniformSourceInstBase(const Brig::BrigInstBase *ib,
                                    const BrigObject *obj, const char *opcode)
            : HsailGPUStaticInst(obj, opcode)
        {
            setFlag(ALU);

            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            dest.init(op_offs, obj);

            op_offs = obj->getOperandPtr(ib->operands, 1);
            src0.init(op_offs, obj);

            op_offs = obj->getOperandPtr(ib->operands, 2);
            src1.init(op_offs, obj);
        }
        bool isVectorRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.isVectorRegister();
            else if (operandIndex == 1)
                return src1.isVectorRegister();
            else
                return dest.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.isCondRegister();
            else if (operandIndex == 1)
                return src1.isCondRegister();
            else
                return dest.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.isScalarRegister();
            else if (operandIndex == 1)
                return src1.isScalarRegister();
            else
                return dest.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex < 2)
                return true;
            else
                return false;
        }
        bool isDstOperand(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (operandIndex >= 2)
                return true;
            else
                return false;
        }
        int getOperandSize(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.opSize();
            else if (operandIndex == 1)
                return src1.opSize();
            else
                return dest.opSize();
        }

        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            if (!operandIndex)
                return src0.regIndex();
            else if (operandIndex == 1)
                return src1.regIndex();
            else
                return dest.regIndex();
        }

        int numSrcRegOperands() {
            int operands = 0;
            if (src0.isVectorRegister()) {
                operands++;
            }
            if (src1.isVectorRegister()) {
                operands++;
            }
            return operands;
        }
        int numDstRegOperands() { return dest.isVectorRegister(); }
        int getNumOperands() { return 3; }
    };

    template<typename DestDataType, typename Src0DataType,
             typename Src1DataType>
    class TwoNonUniformSourceInst :
        public TwoNonUniformSourceInstBase<typename DestDataType::OperandType,
                                           typename Src0DataType::OperandType,
                                           typename Src1DataType::OperandType>
    {
      public:
        typedef typename DestDataType::CType DestCType;
        typedef typename Src0DataType::CType Src0CType;
        typedef typename Src1DataType::CType Src1CType;

        TwoNonUniformSourceInst(const Brig::BrigInstBase *ib,
                                const BrigObject *obj, const char *opcode)
            : TwoNonUniformSourceInstBase<typename DestDataType::OperandType,
                                         typename Src0DataType::OperandType,
                                         typename Src1DataType::OperandType>(ib,
                                                                    obj, opcode)
        {
        }
    };

    // helper function for ClassInst
    template<typename T>
    bool
    fpclassify(T src0, uint32_t src1)
    {
        int fpclass = std::fpclassify(src0);

        if ((src1 & 0x3) && (fpclass == FP_NAN)) {
            return true;
        }

        if (src0 <= -0.0) {
            if ((src1 & 0x4) && fpclass == FP_INFINITE)
                return true;
            if ((src1 & 0x8) && fpclass == FP_NORMAL)
                return true;
            if ((src1 & 0x10) && fpclass == FP_SUBNORMAL)
                return true;
            if ((src1 & 0x20) && fpclass == FP_ZERO)
                return true;
        } else {
            if ((src1 & 0x40) && fpclass == FP_ZERO)
                return true;
            if ((src1 & 0x80) && fpclass == FP_SUBNORMAL)
                return true;
            if ((src1 & 0x100) && fpclass == FP_NORMAL)
                return true;
            if ((src1 & 0x200) && fpclass == FP_INFINITE)
                return true;
        }
        return false;
    }

    template<typename DataType>
    class ClassInst : public TwoNonUniformSourceInst<B1, DataType, U32>
    {
      public:
        ClassInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                  const char *opcode)
            : TwoNonUniformSourceInst<B1, DataType, U32>(ib, obj, opcode)
        {
        }
    };

    template<typename DataType>
    class ShiftInst : public TwoNonUniformSourceInst<DataType, DataType, U32>
    {
      public:
        ShiftInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                  const char *opcode)
            : TwoNonUniformSourceInst<DataType, DataType, U32>(ib, obj, opcode)
        {
        }
    };

    // helper function for CmpInst
    template<typename T>
    bool
    compare(T src0, T src1, Brig::BrigCompareOperation cmpOp)
    {
        using namespace Brig;

        switch (cmpOp) {
          case BRIG_COMPARE_EQ:
          case BRIG_COMPARE_EQU:
          case BRIG_COMPARE_SEQ:
          case BRIG_COMPARE_SEQU:
            return (src0 == src1);

          case BRIG_COMPARE_NE:
          case BRIG_COMPARE_NEU:
          case BRIG_COMPARE_SNE:
          case BRIG_COMPARE_SNEU:
            return (src0 != src1);

          case BRIG_COMPARE_LT:
          case BRIG_COMPARE_LTU:
          case BRIG_COMPARE_SLT:
          case BRIG_COMPARE_SLTU:
            return (src0 < src1);

          case BRIG_COMPARE_LE:
          case BRIG_COMPARE_LEU:
          case BRIG_COMPARE_SLE:
          case BRIG_COMPARE_SLEU:
            return (src0 <= src1);

          case BRIG_COMPARE_GT:
          case BRIG_COMPARE_GTU:
          case BRIG_COMPARE_SGT:
          case BRIG_COMPARE_SGTU:
            return (src0 > src1);

          case BRIG_COMPARE_GE:
          case BRIG_COMPARE_GEU:
          case BRIG_COMPARE_SGE:
          case BRIG_COMPARE_SGEU:
            return (src0 >= src1);

          case BRIG_COMPARE_NUM:
          case BRIG_COMPARE_SNUM:
            return (src0 == src0) || (src1 == src1);

          case BRIG_COMPARE_NAN:
          case BRIG_COMPARE_SNAN:
            return (src0 != src0) || (src1 != src1);

          default:
            fatal("Bad cmpOp value %d\n", (int)cmpOp);
        }
    }

    template<typename T>
    int32_t
    firstbit(T src0)
    {
        if (!src0)
            return -1;

        //handle positive and negative numbers
        T tmp = ((int64_t)src0 < 0) ? (~src0) : (src0);

        //the starting pos is MSB
        int pos = 8 * sizeof(T) - 1;
        int cnt = 0;

        //search the first bit set to 1
        while (!(tmp & (1 << pos))) {
            ++cnt;
            --pos;
        }
        return cnt;
    }

    const char* cmpOpToString(Brig::BrigCompareOperation cmpOp);

    template<typename DestOperandType, typename SrcOperandType>
    class CmpInstBase : public CommonInstBase<DestOperandType, SrcOperandType,
                                              2>
    {
      protected:
        Brig::BrigCompareOperation cmpOp;

      public:
        CmpInstBase(const Brig::BrigInstBase *ib, const BrigObject *obj,
                    const char *_opcode)
            : CommonInstBase<DestOperandType, SrcOperandType, 2>(ib, obj,
                                                                 _opcode)
        {
            assert(ib->base.kind == Brig::BRIG_KIND_INST_CMP);
            Brig::BrigInstCmp *i = (Brig::BrigInstCmp*)ib;
            cmpOp = (Brig::BrigCompareOperation)i->compare;
        }
    };

    template<typename DestDataType, typename SrcDataType>
    class CmpInst : public CmpInstBase<typename DestDataType::OperandType,
                                       typename SrcDataType::OperandType>
    {
      public:
        std::string
        opcode_suffix()
        {
            return csprintf("_%s_%s_%s", cmpOpToString(this->cmpOp),
                            DestDataType::label, SrcDataType::label);
        }

        CmpInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                const char *_opcode)
            : CmpInstBase<typename DestDataType::OperandType,
                          typename SrcDataType::OperandType>(ib, obj, _opcode)
        {
        }
    };

    template<typename DestDataType, typename SrcDataType>
    class CvtInst : public CommonInstBase<typename DestDataType::OperandType,
                                          typename SrcDataType::OperandType, 1>
    {
      public:
        std::string opcode_suffix()
        {
            return csprintf("_%s_%s", DestDataType::label, SrcDataType::label);
        }

        CvtInst(const Brig::BrigInstBase *ib, const BrigObject *obj,
                const char *_opcode)
            : CommonInstBase<typename DestDataType::OperandType,
                             typename SrcDataType::OperandType,
                             1>(ib, obj, _opcode)
        {
        }
    };

    class SpecialInstNoSrcNoDest : public HsailGPUStaticInst
    {
      public:
        SpecialInstNoSrcNoDest(const Brig::BrigInstBase *ib,
                               const BrigObject *obj, const char *_opcode)
            : HsailGPUStaticInst(obj, _opcode)
        {
        }

        bool isVectorRegister(int operandIndex) { return false; }
        bool isCondRegister(int operandIndex) { return false; }
        bool isScalarRegister(int operandIndex) { return false; }
        bool isSrcOperand(int operandIndex) { return false; }
        bool isDstOperand(int operandIndex) { return false; }
        int getOperandSize(int operandIndex) { return 0; }

        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            return -1;
        }

        int numSrcRegOperands() { return 0; }
        int numDstRegOperands() { return 0; }
        int getNumOperands() { return 0; }
    };

    template<typename DestOperandType>
    class SpecialInstNoSrcBase : public HsailGPUStaticInst
    {
      protected:
        typename DestOperandType::DestOperand dest;

        void generateDisassembly()
        {
            disassembly = csprintf("%s %s", opcode, dest.disassemble());
        }

      public:
        SpecialInstNoSrcBase(const Brig::BrigInstBase *ib,
                             const BrigObject *obj, const char *_opcode)
            : HsailGPUStaticInst(obj, _opcode)
        {
            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            dest.init(op_offs, obj);
        }

        bool isVectorRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex) { return false; }
        bool isDstOperand(int operandIndex) { return true; }
        int getOperandSize(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.opSize();
        }

        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.regIndex();
        }

        int numSrcRegOperands() { return 0; }
        int numDstRegOperands() { return dest.isVectorRegister(); }
        int getNumOperands() { return 1; }
    };

    template<typename DestDataType>
    class SpecialInstNoSrc :
        public SpecialInstNoSrcBase<typename DestDataType::OperandType>
    {
      public:
        typedef typename DestDataType::CType DestCType;

        SpecialInstNoSrc(const Brig::BrigInstBase *ib, const BrigObject *obj,
                         const char *_opcode)
            : SpecialInstNoSrcBase<typename DestDataType::OperandType>(ib, obj,
                                                                       _opcode)
        {
        }
    };

    template<typename DestOperandType>
    class SpecialInst1SrcBase : public HsailGPUStaticInst
    {
      protected:
        typedef int SrcCType;  // used in execute() template

        typename DestOperandType::DestOperand dest;
        ImmOperand<SrcCType> src0;

        void
        generateDisassembly()
        {
            disassembly = csprintf("%s %s,%s", opcode, dest.disassemble(),
                                   src0.disassemble());
        }

      public:
        SpecialInst1SrcBase(const Brig::BrigInstBase *ib,
                            const BrigObject *obj, const char *_opcode)
            : HsailGPUStaticInst(obj, _opcode)
        {
            setFlag(ALU);

            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            dest.init(op_offs, obj);

            op_offs = obj->getOperandPtr(ib->operands, 1);
            src0.init(op_offs, obj);
        }
        bool isVectorRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.isVectorRegister();
        }
        bool isCondRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.isCondRegister();
        }
        bool isScalarRegister(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.isScalarRegister();
        }
        bool isSrcOperand(int operandIndex) { return false; }
        bool isDstOperand(int operandIndex) { return true; }
        int getOperandSize(int operandIndex) {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.opSize();
        }

        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            assert((operandIndex >= 0) && (operandIndex < getNumOperands()));
            return dest.regIndex();
        }

        int numSrcRegOperands() { return 0; }
        int numDstRegOperands() { return dest.isVectorRegister(); }
        int getNumOperands() { return 1; }
    };

    template<typename DestDataType>
    class SpecialInst1Src :
        public SpecialInst1SrcBase<typename DestDataType::OperandType>
    {
      public:
        typedef typename DestDataType::CType DestCType;

        SpecialInst1Src(const Brig::BrigInstBase *ib, const BrigObject *obj,
                        const char *_opcode)
            : SpecialInst1SrcBase<typename DestDataType::OperandType>(ib, obj,
                                                                      _opcode)
        {
        }
    };

    class Ret : public SpecialInstNoSrcNoDest
    {
      public:
        typedef SpecialInstNoSrcNoDest Base;

        Ret(const Brig::BrigInstBase *ib, const BrigObject *obj)
           : Base(ib, obj, "ret")
        {
            setFlag(GPUStaticInst::Return);
        }

        void execute(GPUDynInstPtr gpuDynInst);
    };

    class Barrier : public SpecialInstNoSrcNoDest
    {
      public:
        typedef SpecialInstNoSrcNoDest Base;
        uint8_t width;

        Barrier(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : Base(ib, obj, "barrier")
        {
            setFlag(GPUStaticInst::MemBarrier);
            assert(ib->base.kind == Brig::BRIG_KIND_INST_BR);
            width = (uint8_t)((Brig::BrigInstBr*)ib)->width;
        }

        void execute(GPUDynInstPtr gpuDynInst);
    };

    class MemFence : public SpecialInstNoSrcNoDest
    {
      public:
        typedef SpecialInstNoSrcNoDest Base;

        Brig::BrigMemoryOrder memFenceMemOrder;
        Brig::BrigMemoryScope memFenceScopeSegGroup;
        Brig::BrigMemoryScope memFenceScopeSegGlobal;
        Brig::BrigMemoryScope memFenceScopeSegImage;

        MemFence(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : Base(ib, obj, "memfence")
        {
            assert(ib->base.kind == Brig::BRIG_KIND_INST_MEM_FENCE);

            memFenceScopeSegGlobal = (Brig::BrigMemoryScope)
                ((Brig::BrigInstMemFence*)ib)->globalSegmentMemoryScope;

            memFenceScopeSegGroup = (Brig::BrigMemoryScope)
                ((Brig::BrigInstMemFence*)ib)->groupSegmentMemoryScope;

            memFenceScopeSegImage = (Brig::BrigMemoryScope)
                ((Brig::BrigInstMemFence*)ib)->imageSegmentMemoryScope;

            memFenceMemOrder = (Brig::BrigMemoryOrder)
                ((Brig::BrigInstMemFence*)ib)->memoryOrder;

            setFlag(MemoryRef);
            setFlag(GPUStaticInst::MemFence);

            switch (memFenceMemOrder) {
              case Brig::BRIG_MEMORY_ORDER_NONE:
                setFlag(NoOrder);
                break;
              case Brig::BRIG_MEMORY_ORDER_RELAXED:
                setFlag(RelaxedOrder);
                break;
              case Brig::BRIG_MEMORY_ORDER_SC_ACQUIRE:
                setFlag(Acquire);
                break;
              case Brig::BRIG_MEMORY_ORDER_SC_RELEASE:
                setFlag(Release);
                break;
              case Brig::BRIG_MEMORY_ORDER_SC_ACQUIRE_RELEASE:
                setFlag(AcquireRelease);
                break;
              default:
                fatal("MemInst has bad BrigMemoryOrder\n");
            }

            // set inst flags based on scopes
            if (memFenceScopeSegGlobal != Brig::BRIG_MEMORY_SCOPE_NONE &&
                memFenceScopeSegGroup != Brig::BRIG_MEMORY_SCOPE_NONE) {
                setFlag(GPUStaticInst::GlobalSegment);

                /**
                 * A memory fence that has scope for
                 * both segments will use the global
                 * segment, and be executed in the
                 * global memory pipeline, therefore,
                 * we set the segment to match the
                 * global scope only
                 */
                switch (memFenceScopeSegGlobal) {
                  case Brig::BRIG_MEMORY_SCOPE_NONE:
                    setFlag(NoScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_WORKITEM:
                    setFlag(WorkitemScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_WORKGROUP:
                    setFlag(WorkgroupScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_AGENT:
                    setFlag(DeviceScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_SYSTEM:
                    setFlag(SystemScope);
                    break;
                  default:
                    fatal("MemFence has bad global scope type\n");
                }
            } else if (memFenceScopeSegGlobal != Brig::BRIG_MEMORY_SCOPE_NONE) {
                setFlag(GPUStaticInst::GlobalSegment);

                switch (memFenceScopeSegGlobal) {
                  case Brig::BRIG_MEMORY_SCOPE_NONE:
                    setFlag(NoScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_WORKITEM:
                    setFlag(WorkitemScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_WORKGROUP:
                    setFlag(WorkgroupScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_AGENT:
                    setFlag(DeviceScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_SYSTEM:
                    setFlag(SystemScope);
                    break;
                  default:
                    fatal("MemFence has bad global scope type\n");
                }
            } else if (memFenceScopeSegGroup != Brig::BRIG_MEMORY_SCOPE_NONE) {
                setFlag(GPUStaticInst::GroupSegment);

                switch (memFenceScopeSegGroup) {
                  case Brig::BRIG_MEMORY_SCOPE_NONE:
                    setFlag(NoScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_WORKITEM:
                    setFlag(WorkitemScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_WORKGROUP:
                    setFlag(WorkgroupScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_AGENT:
                    setFlag(DeviceScope);
                    break;
                  case Brig::BRIG_MEMORY_SCOPE_SYSTEM:
                    setFlag(SystemScope);
                    break;
                  default:
                    fatal("MemFence has bad group scope type\n");
                }
            } else {
                fatal("MemFence constructor: bad scope specifiers\n");
            }
        }

        void
        initiateAcc(GPUDynInstPtr gpuDynInst)
        {
            Wavefront *wave = gpuDynInst->wavefront();
            wave->computeUnit->injectGlobalMemFence(gpuDynInst);
        }

        void
        execute(GPUDynInstPtr gpuDynInst)
        {
            Wavefront *w = gpuDynInst->wavefront();
            // 2 cases:
            //   * memfence to a sequentially consistent memory (e.g., LDS).
            //     These can be handled as no-ops.
            //   * memfence to a relaxed consistency cache (e.g., Hermes, Viper,
            //     etc.). We send a packet, tagged with the memory order and
            //     scope, and let the GPU coalescer handle it.

            if (isGlobalSeg()) {
                gpuDynInst->simdId = w->simdId;
                gpuDynInst->wfSlotId = w->wfSlotId;
                gpuDynInst->wfDynId = w->wfDynId;
                gpuDynInst->kern_id = w->kernId;
                gpuDynInst->cu_id = w->computeUnit->cu_id;

                gpuDynInst->useContinuation = false;
                GlobalMemPipeline* gmp = &(w->computeUnit->globalMemoryPipe);
                gmp->issueRequest(gpuDynInst);

                w->wrGmReqsInPipe--;
                w->rdGmReqsInPipe--;
                w->memReqsInPipe--;
                w->outstandingReqs++;
            } else if (isGroupSeg()) {
                // no-op
            } else {
                fatal("MemFence execute: bad op type\n");
            }
        }
    };

    class Call : public HsailGPUStaticInst
    {
      public:
        // private helper functions
        void calcAddr(Wavefront* w, GPUDynInstPtr m);

        void
        generateDisassembly()
        {
            if (dest.disassemble() == "") {
                disassembly = csprintf("%s %s (%s)", opcode, src0.disassemble(),
                                       src1.disassemble());
            } else {
                disassembly = csprintf("%s %s (%s) (%s)", opcode,
                                       src0.disassemble(), dest.disassemble(),
                                       src1.disassemble());
            }
        }

        bool
        isPseudoOp()
        {
            std::string func_name = src0.disassemble();
            if (func_name.find("__gem5_hsail_op") != std::string::npos) {
                return true;
            }
            return false;
        }

        // member variables
        ListOperand dest;
        FunctionRefOperand src0;
        ListOperand src1;
        HsailCode *func_ptr;

        // exec function for pseudo instructions mapped on top of call opcode
        void execPseudoInst(Wavefront *w, GPUDynInstPtr gpuDynInst);

        // user-defined pseudo instructions
        void MagicPrintLane(Wavefront *w);
        void MagicPrintLane64(Wavefront *w);
        void MagicPrintWF32(Wavefront *w);
        void MagicPrintWF64(Wavefront *w);
        void MagicPrintWFFloat(Wavefront *w);
        void MagicSimBreak(Wavefront *w);
        void MagicPrefixSum(Wavefront *w);
        void MagicReduction(Wavefront *w);
        void MagicMaskLower(Wavefront *w);
        void MagicMaskUpper(Wavefront *w);
        void MagicJoinWFBar(Wavefront *w);
        void MagicWaitWFBar(Wavefront *w);
        void MagicPanic(Wavefront *w);

        void MagicAtomicNRAddGlobalU32Reg(Wavefront *w,
                                          GPUDynInstPtr gpuDynInst);

        void MagicAtomicNRAddGroupU32Reg(Wavefront *w,
                                         GPUDynInstPtr gpuDynInst);

        void MagicLoadGlobalU32Reg(Wavefront *w, GPUDynInstPtr gpuDynInst);

        void MagicXactCasLd(Wavefront *w);
        void MagicMostSigThread(Wavefront *w);
        void MagicMostSigBroadcast(Wavefront *w);

        void MagicPrintWF32ID(Wavefront *w);
        void MagicPrintWFID64(Wavefront *w);

        Call(const Brig::BrigInstBase *ib, const BrigObject *obj)
            : HsailGPUStaticInst(obj, "call")
        {
            setFlag(ALU);
            unsigned op_offs = obj->getOperandPtr(ib->operands, 0);
            dest.init(op_offs, obj);
            op_offs = obj->getOperandPtr(ib->operands, 1);
            src0.init(op_offs, obj);

            func_ptr = nullptr;
            std::string func_name = src0.disassemble();
            if (!isPseudoOp()) {
                func_ptr = dynamic_cast<HsailCode*>(obj->
                                                    getFunction(func_name));

                if (!func_ptr)
                    fatal("call::exec cannot find function: %s\n", func_name);
            }

            op_offs = obj->getOperandPtr(ib->operands, 2);
            src1.init(op_offs, obj);
        }

        bool isVectorRegister(int operandIndex) { return false; }
        bool isCondRegister(int operandIndex) { return false; }
        bool isScalarRegister(int operandIndex) { return false; }
        bool isSrcOperand(int operandIndex) { return false; }
        bool isDstOperand(int operandIndex) { return false; }
        int getOperandSize(int operandIndex) { return 0; }

        int
        getRegisterIndex(int operandIndex, GPUDynInstPtr gpuDynInst)
        {
            return -1;
        }

        void
        execute(GPUDynInstPtr gpuDynInst)
        {
            Wavefront *w = gpuDynInst->wavefront();

            std::string func_name = src0.disassemble();
            if (isPseudoOp()) {
                execPseudoInst(w, gpuDynInst);
            } else {
                fatal("Native HSAIL functions are not yet implemented: %s\n",
                      func_name);
            }
        }
        int numSrcRegOperands() { return 0; }
        int numDstRegOperands() { return 0; }
        int getNumOperands() { return 2; }
    };

    template<typename T> T heynot(T arg) { return ~arg; }
    template<> inline bool heynot<bool>(bool arg) { return !arg; }
} // namespace HsailISA

#endif // __ARCH_HSAIL_INSTS_DECL_HH__
