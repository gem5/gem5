/*
 * Copyright (c) 2017-2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_GCN3_OPERAND_HH__
#define __ARCH_GCN3_OPERAND_HH__

#include <array>

#include "arch/amdgpu/gcn3/gpu_registers.hh"
#include "arch/generic/vec_reg.hh"
#include "gpu-compute/scalar_register_file.hh"
#include "gpu-compute/vector_register_file.hh"
#include "gpu-compute/wavefront.hh"

namespace gem5
{

/**
 * classes that represnt vector/scalar operands in GCN3 ISA. these classes
 * wrap the generic vector register type (i.e., src/arch/generic/vec_reg.hh)
 * and allow them to be manipulated in ways that are unique to GCN3 insts.
 */

namespace Gcn3ISA
{
    /**
     * convenience traits so we can automatically infer the correct FP type
     * without looking at the number of dwords (i.e., to determine if we
     * need a float or a double when creating FP constants).
     */
    template<typename T> struct OpTraits { typedef float FloatT; };
    template<> struct OpTraits<ScalarRegF64> { typedef double FloatT; };
    template<> struct OpTraits<ScalarRegU64> { typedef double FloatT; };

    class Operand
    {
      public:
        Operand() = delete;

        Operand(GPUDynInstPtr gpuDynInst, int opIdx)
            : _gpuDynInst(gpuDynInst), _opIdx(opIdx)
        {
            assert(_gpuDynInst);
            assert(_opIdx >= 0);
        }

        /**
         * read from and write to the underlying register(s) that
         * this operand is referring to.
         */
        virtual void read() = 0;
        virtual void write() = 0;

      protected:
        /**
         * instruction object that owns this operand
         */
        GPUDynInstPtr _gpuDynInst;
        /**
         * op selector value for this operand. note that this is not
         * the same as the register file index, be it scalar or vector.
         * this could refer to inline constants, system regs, or even
         * special values.
         */
        int _opIdx;
    };

    template<typename DataType, bool Const, size_t NumDwords>
    class ScalarOperand;

    template<typename DataType, bool Const,
        size_t NumDwords = sizeof(DataType) / sizeof(VecElemU32)>
    class VecOperand final : public Operand
    {
      static_assert(NumDwords >= 1 && NumDwords <= MaxOperandDwords,
            "Incorrect number of DWORDS for GCN3 operand.");

      public:
        VecOperand() = delete;

        VecOperand(GPUDynInstPtr gpuDynInst, int opIdx)
            : Operand(gpuDynInst, opIdx), scalar(false), absMod(false),
              negMod(false), scRegData(gpuDynInst, _opIdx),
              vrfData{{ nullptr }}
        {
            vecReg.zero();
        }

        ~VecOperand()
        {
        }

        /**
         * certain vector operands can read from the vrf/srf or constants.
         * we use this method to first determine the type of the operand,
         * then we read from the appropriate source. if vector we read
         * directly from the vrf. if scalar, we read in the data through
         * the scalar operand component. this should only be used for VSRC
         * operands.
         */
        void
        readSrc()
        {
            if (isVectorReg(_opIdx)) {
                _opIdx = opSelectorToRegIdx(_opIdx, _gpuDynInst->wavefront()
                    ->reservedScalarRegs);
                read();
            } else {
                readScalar();
            }
        }

        /**
         * read from the vrf. this should only be used by vector inst
         * source operands that are explicitly vector (i.e., VSRC).
         */
        void
        read() override
        {
            assert(_gpuDynInst);
            assert(_gpuDynInst->wavefront());
            assert(_gpuDynInst->computeUnit());
            Wavefront *wf = _gpuDynInst->wavefront();
            ComputeUnit *cu = _gpuDynInst->computeUnit();

            for (auto i = 0; i < NumDwords; ++i) {
                int vgprIdx = cu->registerManager->mapVgpr(wf, _opIdx + i);
                vrfData[i] = &cu->vrf[wf->simdId]->readWriteable(vgprIdx);

                DPRINTF(GPUVRF, "Read v[%d]\n", vgprIdx);
                cu->vrf[wf->simdId]->printReg(wf, vgprIdx);
            }

            if (NumDwords == 1) {
                assert(vrfData[0]);
                auto vgpr = vecReg.template as<DataType>();
                auto reg_file_vgpr = vrfData[0]->template as<VecElemU32>();
                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    std::memcpy((void*)&vgpr[lane],
                        (void*)&reg_file_vgpr[lane], sizeof(DataType));
                }
            } else if (NumDwords == 2) {
                assert(vrfData[0]);
                assert(vrfData[1]);
                auto vgpr = vecReg.template as<VecElemU64>();
                auto reg_file_vgpr0 = vrfData[0]->template as<VecElemU32>();
                auto reg_file_vgpr1 = vrfData[1]->template as<VecElemU32>();

                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    VecElemU64 tmp_val(0);
                    ((VecElemU32*)&tmp_val)[0] = reg_file_vgpr0[lane];
                    ((VecElemU32*)&tmp_val)[1] = reg_file_vgpr1[lane];
                    vgpr[lane] = tmp_val;
                }
            }
        }

        /**
         * write to the vrf. we maintain a copy of the underlying vector
         * reg(s) for this operand (i.e., vrfData/scRegData), as well as a
         * temporary vector register representation (i.e., vecReg) of the
         * vector register, which allows the execute() methods of instructions
         * to easily write their operand data using operator[] regardless of
         * their size. after the result is calculated we use write() to write
         * the data to the actual register file storage. this allows us to do
         * type conversion, etc., in a single call as opposed to doing it
         * in each execute() method.
         */
        void
        write() override
        {
            assert(_gpuDynInst);
            assert(_gpuDynInst->wavefront());
            assert(_gpuDynInst->computeUnit());
            Wavefront *wf = _gpuDynInst->wavefront();
            ComputeUnit *cu = _gpuDynInst->computeUnit();
            VectorMask &exec_mask = _gpuDynInst->isLoad()
                ? _gpuDynInst->exec_mask : wf->execMask();

            if (NumDwords == 1) {
                int vgprIdx = cu->registerManager->mapVgpr(wf, _opIdx);
                vrfData[0] = &cu->vrf[wf->simdId]->readWriteable(vgprIdx);
                assert(vrfData[0]);
                auto reg_file_vgpr = vrfData[0]->template as<VecElemU32>();
                auto vgpr = vecReg.template as<DataType>();

                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (exec_mask[lane] || _gpuDynInst->ignoreExec()) {
                        std::memcpy((void*)&reg_file_vgpr[lane],
                            (void*)&vgpr[lane], sizeof(DataType));
                    }
                }

                DPRINTF(GPUVRF, "Write v[%d]\n", vgprIdx);
                cu->vrf[wf->simdId]->printReg(wf, vgprIdx);
            } else if (NumDwords == 2) {
                int vgprIdx0 = cu->registerManager->mapVgpr(wf, _opIdx);
                int vgprIdx1 = cu->registerManager->mapVgpr(wf, _opIdx + 1);
                vrfData[0] = &cu->vrf[wf->simdId]->readWriteable(vgprIdx0);
                vrfData[1] = &cu->vrf[wf->simdId]->readWriteable(vgprIdx1);
                assert(vrfData[0]);
                assert(vrfData[1]);
                auto reg_file_vgpr0 = vrfData[0]->template as<VecElemU32>();
                auto reg_file_vgpr1 = vrfData[1]->template as<VecElemU32>();
                auto vgpr = vecReg.template as<VecElemU64>();

                for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
                    if (exec_mask[lane] || _gpuDynInst->ignoreExec()) {
                        reg_file_vgpr0[lane] = ((VecElemU32*)&vgpr[lane])[0];
                        reg_file_vgpr1[lane] = ((VecElemU32*)&vgpr[lane])[1];
                    }
                }

                DPRINTF(GPUVRF, "Write v[%d:%d]\n", vgprIdx0, vgprIdx1);
                cu->vrf[wf->simdId]->printReg(wf, vgprIdx0);
                cu->vrf[wf->simdId]->printReg(wf, vgprIdx1);
            }
        }

        void
        negModifier()
        {
            negMod = true;
        }

        void
        absModifier()
        {
            absMod = true;
        }

        /**
         * getter [] operator. only enable if this operand is constant
         * (i.e, a source operand) and if it can be represented using
         * primitive types (i.e., 8b to 64b primitives).
         */
        template<bool Condition = (NumDwords == 1 || NumDwords == 2) && Const>
        typename std::enable_if_t<Condition, const DataType>
        operator[](size_t idx) const
        {
            assert(idx < NumVecElemPerVecReg);

            if (scalar) {
                DataType ret_val = scRegData.rawData();

                if (absMod) {
                    assert(std::is_floating_point_v<DataType>);
                    ret_val = std::fabs(ret_val);
                }

                if (negMod) {
                    assert(std::is_floating_point_v<DataType>);
                    ret_val = -ret_val;
                }

                return ret_val;
            } else {
                auto vgpr = vecReg.template as<DataType>();
                DataType ret_val = vgpr[idx];

                if (absMod) {
                    assert(std::is_floating_point_v<DataType>);
                    ret_val = std::fabs(ret_val);
                }

                if (negMod) {
                    assert(std::is_floating_point_v<DataType>);
                    ret_val = -ret_val;
                }

                return ret_val;
            }
        }

        /**
         * setter [] operator. only enable if this operand is non-constant
         * (i.e, a destination operand) and if it can be represented using
         * primitive types (i.e., 8b to 64b primitives).
         */
        template<bool Condition = (NumDwords == 1 || NumDwords == 2) && !Const>
        typename std::enable_if_t<Condition, DataType&>
        operator[](size_t idx)
        {
            assert(!scalar);
            assert(idx < NumVecElemPerVecReg);

            return vecReg.template as<DataType>()[idx];
        }

        private:
          /**
           * if we determine that this operand is a scalar (reg or constant)
           * then we read the scalar data into the scalar operand data member.
           */
          void
          readScalar()
          {
              scalar = true;
              scRegData.read();
          }

          using VecRegCont =
              VecRegContainer<sizeof(DataType) * NumVecElemPerVecReg>;

          /**
           * whether this operand a scalar or not.
           */
          bool scalar;
          /**
           * absolute value and negative modifiers. VOP3 instructions
           * may indicate that their input/output operands must be
           * modified, either by taking the absolute value or negating
           * them. these bools indicate which modifier, if any, to use.
           */
          bool absMod;
          bool negMod;
          /**
           * this holds all the operand data in a single vector register
           * object (i.e., if an operand is 64b, this will hold the data
           * from both registers the operand is using).
           */
          VecRegCont vecReg;
          /**
           * for src operands that read scalars (i.e., scalar regs or
           * a scalar constant).
           */
          ScalarOperand<DataType, Const, NumDwords> scRegData;
          /**
           * pointers to the underlyding registers (i.e., the actual
           * registers in the register file).
           */
          std::array<VecRegContainerU32*, NumDwords> vrfData;
    };

    template<typename DataType, bool Const,
        size_t NumDwords = sizeof(DataType) / sizeof(ScalarRegU32)>
    class ScalarOperand final : public Operand
    {
      static_assert(NumDwords >= 1 && NumDwords <= MaxOperandDwords,
            "Incorrect number of DWORDS for GCN3 operand.");
      public:
        ScalarOperand() = delete;

        ScalarOperand(GPUDynInstPtr gpuDynInst, int opIdx)
            : Operand(gpuDynInst, opIdx)
        {
            std::memset(srfData.data(), 0, NumDwords * sizeof(ScalarRegU32));
        }

        ~ScalarOperand()
        {
        }

        /**
         * we store scalar data in a std::array, however if we need the
         * full operand data we use this method to copy all elements of
         * the scalar operand data to a single primitive container. only
         * useful for 8b to 64b primitive types, as they are the only types
         * that we need to perform computation on.
         */
        template<bool Condition = NumDwords == 1 || NumDwords == 2>
        typename std::enable_if_t<Condition, DataType>
        rawData() const
        {
            assert(sizeof(DataType) <= sizeof(srfData));
            DataType raw_data((DataType)0);
            std::memcpy((void*)&raw_data, (void*)srfData.data(),
                sizeof(DataType));

            return raw_data;
        }

        void*
        rawDataPtr()
        {
            return (void*)srfData.data();
        }

        void
        read() override
        {
            Wavefront *wf = _gpuDynInst->wavefront();
            ComputeUnit *cu = _gpuDynInst->computeUnit();

            if (!isScalarReg(_opIdx)) {
                readSpecialVal();
            } else {
                for (auto i = 0; i < NumDwords; ++i) {
                    int sgprIdx = regIdx(i);
                    srfData[i] = cu->srf[wf->simdId]->read(sgprIdx);
                    DPRINTF(GPUSRF, "Read s[%d]\n", sgprIdx);
                    cu->srf[wf->simdId]->printReg(wf, sgprIdx);
                }
            }
        }

        void
        write() override
        {
            Wavefront *wf = _gpuDynInst->wavefront();
            ComputeUnit *cu = _gpuDynInst->computeUnit();

            if (!isScalarReg(_opIdx)) {
                if (_opIdx == REG_EXEC_LO) {
                    ScalarRegU64 new_exec_mask_val
                        = wf->execMask().to_ullong();
                    if (NumDwords == 1) {
                        std::memcpy((void*)&new_exec_mask_val,
                            (void*)srfData.data(), sizeof(VecElemU32));
                    } else if (NumDwords == 2) {
                        std::memcpy((void*)&new_exec_mask_val,
                            (void*)srfData.data(), sizeof(VecElemU64));
                    } else {
                        panic("Trying to write more than 2 DWORDS to EXEC\n");
                    }
                    VectorMask new_exec_mask(new_exec_mask_val);
                    wf->execMask() = new_exec_mask;
                    DPRINTF(GPUSRF, "Write EXEC\n");
                    DPRINTF(GPUSRF, "EXEC = %#x\n", new_exec_mask_val);
                } else if (_opIdx == REG_EXEC_HI) {
                    /**
                     * If we're writing only the upper half of the EXEC mask
                     * this ought to be a single dword operand.
                     */
                    assert(NumDwords == 1);
                    ScalarRegU32 new_exec_mask_hi_val(0);
                    ScalarRegU64 new_exec_mask_val
                        = wf->execMask().to_ullong();
                    std::memcpy((void*)&new_exec_mask_hi_val,
                        (void*)srfData.data(), sizeof(new_exec_mask_hi_val));
                    replaceBits(new_exec_mask_val, 63, 32,
                                new_exec_mask_hi_val);
                    VectorMask new_exec_mask(new_exec_mask_val);
                    wf->execMask() = new_exec_mask;
                    DPRINTF(GPUSRF, "Write EXEC\n");
                    DPRINTF(GPUSRF, "EXEC = %#x\n", new_exec_mask_val);
                } else {
                    _gpuDynInst->writeMiscReg(_opIdx, srfData[0]);
                }
            } else {
                for (auto i = 0; i < NumDwords; ++i) {
                    int sgprIdx = regIdx(i);
                    auto &sgpr = cu->srf[wf->simdId]->readWriteable(sgprIdx);
                    if (_gpuDynInst->isLoad()) {
                        assert(sizeof(DataType) <= sizeof(ScalarRegU64));
                        sgpr = reinterpret_cast<ScalarRegU32*>(
                            _gpuDynInst->scalar_data)[i];
                    } else {
                        sgpr = srfData[i];
                    }
                    DPRINTF(GPUSRF, "Write s[%d]\n", sgprIdx);
                    cu->srf[wf->simdId]->printReg(wf, sgprIdx);
                }
            }
        }

        /**
         * bit access to scalar data. primarily used for setting vcc bits.
         */
        template<bool Condition = NumDwords == 1 || NumDwords == 2>
        typename std::enable_if_t<Condition, void>
        setBit(int bit, int bit_val)
        {
            DataType &sgpr = *((DataType*)srfData.data());
            replaceBits(sgpr, bit, bit_val);
        }

        template<bool Condition = (NumDwords == 1 || NumDwords == 2) && !Const>
        typename std::enable_if_t<Condition, ScalarOperand&>
        operator=(DataType rhs)
        {
            std::memcpy((void*)srfData.data(), (void*)&rhs, sizeof(DataType));
            return *this;
        }

      private:
        /**
         * we have determined that we are not reading our scalar operand data
         * from the register file, so here we figure out which special value
         * we are reading (i.e., float constant, int constant, inline
         * constant, or various other system registers (e.g., exec mask).
         */
        void
        readSpecialVal()
        {
            assert(NumDwords == 1 || NumDwords == 2);

            switch(_opIdx) {
              case REG_EXEC_LO:
                {
                    if (NumDwords == 1) {
                        ScalarRegU32 exec_mask = _gpuDynInst->wavefront()->
                            execMask().to_ulong();
                        std::memcpy((void*)srfData.data(), (void*)&exec_mask,
                            sizeof(exec_mask));
                        DPRINTF(GPUSRF, "Read EXEC\n");
                        DPRINTF(GPUSRF, "EXEC = %#x\n", exec_mask);
                    } else {
                        assert(NumDwords == 2);
                        ScalarRegU64 exec_mask = _gpuDynInst->wavefront()->
                            execMask().to_ullong();
                        std::memcpy((void*)srfData.data(), (void*)&exec_mask,
                            sizeof(exec_mask));
                        DPRINTF(GPUSRF, "Read EXEC\n");
                        DPRINTF(GPUSRF, "EXEC = %#x\n", exec_mask);
                    }
                }
                break;
              case REG_EXEC_HI:
                {
                    /**
                     * If we're reading only the upper half of the EXEC mask
                     * this ought to be a single dword operand.
                     */
                    assert(NumDwords == 1);
                    ScalarRegU64 exec_mask = _gpuDynInst->wavefront()
                        ->execMask().to_ullong();

                    ScalarRegU32 exec_mask_hi = bits(exec_mask, 63, 32);
                    std::memcpy((void*)srfData.data(), (void*)&exec_mask_hi,
                                sizeof(exec_mask_hi));
                    DPRINTF(GPUSRF, "Read EXEC_HI\n");
                    DPRINTF(GPUSRF, "EXEC_HI = %#x\n", exec_mask_hi);
                }
                break;
              case REG_SRC_SWDA:
              case REG_SRC_DPP:
              case REG_SRC_LITERAL:
                assert(NumDwords == 1);
                srfData[0] = _gpuDynInst->srcLiteral();
                break;
              case REG_POS_HALF:
                {
                    typename OpTraits<DataType>::FloatT pos_half = 0.5;
                    std::memcpy((void*)srfData.data(), (void*)&pos_half,
                        sizeof(pos_half));

                }
                break;
              case REG_NEG_HALF:
                {
                    typename OpTraits<DataType>::FloatT neg_half = -0.5;
                    std::memcpy((void*)srfData.data(), (void*)&neg_half,
                        sizeof(neg_half));
                }
                break;
              case REG_POS_ONE:
                {
                    typename OpTraits<DataType>::FloatT pos_one = 1.0;
                    std::memcpy(srfData.data(), &pos_one, sizeof(pos_one));
                }
                break;
              case REG_NEG_ONE:
                {
                    typename OpTraits<DataType>::FloatT neg_one = -1.0;
                    std::memcpy(srfData.data(), &neg_one, sizeof(neg_one));
                }
                break;
              case REG_POS_TWO:
                {
                    typename OpTraits<DataType>::FloatT pos_two = 2.0;
                    std::memcpy(srfData.data(), &pos_two, sizeof(pos_two));
                }
                break;
              case REG_NEG_TWO:
                {
                    typename OpTraits<DataType>::FloatT neg_two = -2.0;
                    std::memcpy(srfData.data(), &neg_two, sizeof(neg_two));
                }
                break;
              case REG_POS_FOUR:
                {
                    typename OpTraits<DataType>::FloatT pos_four = 4.0;
                    std::memcpy(srfData.data(), &pos_four, sizeof(pos_four));
                }
                break;
              case REG_NEG_FOUR:
                {
                    typename OpTraits<DataType>::FloatT neg_four = -4.0;
                    std::memcpy((void*)srfData.data(), (void*)&neg_four ,
                        sizeof(neg_four));
                }
                break;
                case REG_PI:
                {
                    assert(sizeof(DataType) == sizeof(ScalarRegF64)
                        || sizeof(DataType) == sizeof(ScalarRegF32));

                    const ScalarRegU32 pi_u32(0x3e22f983UL);
                    const ScalarRegU64 pi_u64(0x3fc45f306dc9c882ULL);

                    if (sizeof(DataType) == sizeof(ScalarRegF64)) {
                        std::memcpy((void*)srfData.data(),
                            (void*)&pi_u64, sizeof(pi_u64));
                    } else {
                        std::memcpy((void*)srfData.data(),
                            (void*)&pi_u32, sizeof(pi_u32));
                    }
                }
                break;
              default:
                {
                    assert(sizeof(DataType) <= sizeof(srfData));
                    DataType misc_val(0);
                    if (isConstVal(_opIdx)) {
                        misc_val = (DataType)_gpuDynInst
                            ->readConstVal<DataType>(_opIdx);
                    } else {
                        misc_val = (DataType)_gpuDynInst->readMiscReg(_opIdx);
                    }
                    std::memcpy((void*)srfData.data(), (void*)&misc_val,
                                sizeof(DataType));
                }
            }
        }

        /**
         * for scalars we need to do some extra work to figure out how to
         * map the op selector to the sgpr idx because some op selectors
         * do not map directly to the srf (i.e., vcc/flat_scratch).
         */
        int
        regIdx(int dword) const
        {
            Wavefront *wf = _gpuDynInst->wavefront();
            ComputeUnit *cu = _gpuDynInst->computeUnit();
            int sgprIdx(-1);

            if (_opIdx == REG_VCC_HI) {
                sgprIdx = cu->registerManager
                    ->mapSgpr(wf, wf->reservedScalarRegs - 1 + dword);
            } else if (_opIdx == REG_VCC_LO) {
                sgprIdx = cu->registerManager
                    ->mapSgpr(wf, wf->reservedScalarRegs - 2 + dword);
            } else if (_opIdx == REG_FLAT_SCRATCH_HI) {
                sgprIdx = cu->registerManager
                    ->mapSgpr(wf, wf->reservedScalarRegs - 3 + dword);
            } else if (_opIdx == REG_FLAT_SCRATCH_LO) {
                assert(NumDwords == 1);
                sgprIdx = cu->registerManager
                    ->mapSgpr(wf, wf->reservedScalarRegs - 4 + dword);
            } else {
                sgprIdx = cu->registerManager->mapSgpr(wf, _opIdx + dword);
            }

            assert(sgprIdx > -1);

            return sgprIdx;
        }

        /**
         * in GCN3 each register is represented as a 32b unsigned value,
         * however operands may require up to 16 registers, so we store
         * all the individual 32b components here. for sub-dword operand
         * we still consider them to be 1 dword because the minimum size
         * of a register is 1 dword. this class will take care to do the
         * proper packing/unpacking of sub-dword operands.
         */
        std::array<ScalarRegU32, NumDwords> srfData;
    };

    // typedefs for the various sizes/types of scalar operands
    using ScalarOperandU8 = ScalarOperand<ScalarRegU8, false, 1>;
    using ScalarOperandI8 = ScalarOperand<ScalarRegI8, false, 1>;
    using ScalarOperandU16 = ScalarOperand<ScalarRegU16, false, 1>;
    using ScalarOperandI16 = ScalarOperand<ScalarRegI16, false, 1>;
    using ScalarOperandU32 = ScalarOperand<ScalarRegU32, false>;
    using ScalarOperandI32 = ScalarOperand<ScalarRegI32, false>;
    using ScalarOperandF32 = ScalarOperand<ScalarRegF32, false>;
    using ScalarOperandU64 = ScalarOperand<ScalarRegU64, false>;
    using ScalarOperandI64 = ScalarOperand<ScalarRegI64, false>;
    using ScalarOperandF64 = ScalarOperand<ScalarRegF64, false>;
    using ScalarOperandU128 = ScalarOperand<ScalarRegU32, false, 4>;
    using ScalarOperandU256 = ScalarOperand<ScalarRegU32, false, 8>;
    using ScalarOperandU512 = ScalarOperand<ScalarRegU32, false, 16>;
    // non-writeable versions of scalar operands
    using ConstScalarOperandU8 = ScalarOperand<ScalarRegU8, true, 1>;
    using ConstScalarOperandI8 = ScalarOperand<ScalarRegI8, true, 1>;
    using ConstScalarOperandU16 = ScalarOperand<ScalarRegU16, true, 1>;
    using ConstScalarOperandI16 = ScalarOperand<ScalarRegI16, true, 1>;
    using ConstScalarOperandU32 = ScalarOperand<ScalarRegU32, true>;
    using ConstScalarOperandI32 = ScalarOperand<ScalarRegI32, true>;
    using ConstScalarOperandF32 = ScalarOperand<ScalarRegF32, true>;
    using ConstScalarOperandU64 = ScalarOperand<ScalarRegU64, true>;
    using ConstScalarOperandI64 = ScalarOperand<ScalarRegI64, true>;
    using ConstScalarOperandF64 = ScalarOperand<ScalarRegF64, true>;
    using ConstScalarOperandU128 = ScalarOperand<ScalarRegU32, true, 4>;
    using ConstScalarOperandU256 = ScalarOperand<ScalarRegU32, true, 8>;
    using ConstScalarOperandU512 = ScalarOperand<ScalarRegU32, true, 16>;
    // typedefs for the various sizes/types of vector operands
    using VecOperandU8 = VecOperand<VecElemU8, false, 1>;
    using VecOperandI8 = VecOperand<VecElemI8, false, 1>;
    using VecOperandU16 = VecOperand<VecElemU16, false, 1>;
    using VecOperandI16 = VecOperand<VecElemI16, false, 1>;
    using VecOperandU32 = VecOperand<VecElemU32, false>;
    using VecOperandI32 = VecOperand<VecElemI32, false>;
    using VecOperandF32 = VecOperand<VecElemF32, false>;
    using VecOperandU64 = VecOperand<VecElemU64, false>;
    using VecOperandF64 = VecOperand<VecElemF64, false>;
    using VecOperandI64 = VecOperand<VecElemI64, false>;
    using VecOperandU96 = VecOperand<VecElemU32, false, 3>;
    using VecOperandU128 = VecOperand<VecElemU32, false, 4>;
    using VecOperandU256 = VecOperand<VecElemU32, false, 8>;
    using VecOperandU512 = VecOperand<VecElemU32, false, 16>;
    // non-writeable versions of vector operands
    using ConstVecOperandU8 = VecOperand<VecElemU8, true, 1>;
    using ConstVecOperandI8 = VecOperand<VecElemI8, true, 1>;
    using ConstVecOperandU16 = VecOperand<VecElemU16, true, 1>;
    using ConstVecOperandI16 = VecOperand<VecElemI16, true, 1>;
    using ConstVecOperandU32 = VecOperand<VecElemU32, true>;
    using ConstVecOperandI32 = VecOperand<VecElemI32, true>;
    using ConstVecOperandF32 = VecOperand<VecElemF32, true>;
    using ConstVecOperandU64 = VecOperand<VecElemU64, true>;
    using ConstVecOperandI64 = VecOperand<VecElemI64, true>;
    using ConstVecOperandF64 = VecOperand<VecElemF64, true>;
    using ConstVecOperandU96 = VecOperand<VecElemU32, true, 3>;
    using ConstVecOperandU128 = VecOperand<VecElemU32, true, 4>;
    using ConstVecOperandU256 = VecOperand<VecElemU32, true, 8>;
    using ConstVecOperandU512 = VecOperand<VecElemU32, true, 16>;
}

} // namespace gem5

#endif // __ARCH_GCN3_OPERAND_HH__
