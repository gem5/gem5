/*
 * Copyright (c) 2015-2021 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_VEGA_INSTS_INST_UTIL_HH__
#define __ARCH_VEGA_INSTS_INST_UTIL_HH__

#include <cmath>

#include "arch/amdgpu/vega/gpu_registers.hh"
#include "arch/amdgpu/vega/insts/gpu_static_inst.hh"

namespace gem5
{

// values for SDWA select operations
enum SDWASelVals : int
{
    SDWA_BYTE_0 = 0, /* select data[7:0] */
    SDWA_BYTE_1 = 1, /* select data[15:8] */
    SDWA_BYTE_2 = 2, /* select data[23:16] */
    SDWA_BYTE_3 = 3, /* select data[31:24] */
    SDWA_WORD_0 = 4, /* select data[15:0] */
    SDWA_WORD_1 = 5, /* select data[31:16] */
    SDWA_DWORD  = 6  /* select data[31:0] */
};

// values for format of destination bits for SDWA operations
enum SDWADstVals : int
{
    SDWA_UNUSED_PAD      = 0, /* Pad all unused bits with 0 */
    SDWA_UNUSED_SEXT     = 1, /* Sign-extend upper bits; pad lower bits w/ 0 */
    SDWA_UNUSED_PRESERVE = 2  /* select data[31:0] */
};

// values for DPP operations
enum SqDPPVals : int
{
    SQ_DPP_QUAD_PERM_MAX   = 0xFF,
    SQ_DPP_RESERVED        = 0x100,
    SQ_DPP_ROW_SL1         = 0x101,
    SQ_DPP_ROW_SL15        = 0x10F,
    SQ_DPP_ROW_SR1         = 0x111,
    SQ_DPP_ROW_SR15        = 0x11F,
    SQ_DPP_ROW_RR1         = 0x121,
    SQ_DPP_ROW_RR15        = 0x12F,
    SQ_DPP_WF_SL1          = 0x130,
    SQ_DPP_WF_RL1          = 0x134,
    SQ_DPP_WF_SR1          = 0x138,
    SQ_DPP_WF_RR1          = 0x13C,
    SQ_DPP_ROW_MIRROR      = 0x140,
    SQ_DPP_ROW_HALF_MIRROR = 0x141,
    SQ_DPP_ROW_BCAST15     = 0x142,
    SQ_DPP_ROW_BCAST31     = 0x143
};
static const int ROW_SIZE = 16; /* 16 registers per row */
static const int NUM_BANKS = 4; /* 64 registers, 16/bank */

namespace VegaISA
{
    template<typename T>
    inline T
    wholeQuadMode(T val)
    {
        T wqm = 0;
        T mask = 0xF;

        for (T bits = val; mask != 0; mask <<= 4)
            if ((bits & mask) != 0)
                wqm |= mask;

        return wqm;
    }

    template<typename T>
    inline T
    quadMask(T val)
    {
        T qmsk = 0;
        T mask = 0xF;
        T qbit = 0x1;

        for (T bits = val; mask != 0; mask <<= 4, qbit <<= 1) {
            if (bits & mask) {
                qmsk |= qbit;
            }
        }

        return qmsk;
    }

    template<typename T>
    inline ScalarRegI32
    countZeroBits(T val)
    {
        ScalarRegI32 num_zeros
            = std::numeric_limits<T>::digits - popCount(val);

        return num_zeros;
    }

    template<typename T>
    inline ScalarRegI32
    findFirstZero(T val)
    {
        if (val == ~T(0)) {
            return -1;
        }

        return findLsbSet(~val);
    }

    template<typename T>
    inline ScalarRegI32
    findFirstOne(T val)
    {
        if (!val) {
            return -1;
        }

        return findLsbSet(val);
    }

    template<typename T>
    inline ScalarRegI32
    findFirstOneMsb(T val)
    {
        if (!val) {
            return -1;
        }

        return findMsbSet(val);
    }

    template<typename T>
    inline ScalarRegI32
    countZeroBitsMsb(T val)
    {
        if (!val) {
            return -1;
        }

        return std::numeric_limits<T>::digits - 1 - findMsbSet(val);
    }

    inline ScalarRegI32
    firstOppositeSignBit(ScalarRegI32 val)
    {
        bool found(false);
        bool sign_bit = (val & 0x80000000) != 0;
        ScalarRegU32 tmp_val(0);
        int count(0);

        if (!val || val == -1) {
            return -1;
        }

        for (int i = 0; i < std::numeric_limits<ScalarRegU32>::digits; ++i) {
            tmp_val = val & (0x80000000 >> i);

            if (!sign_bit) {
                if (tmp_val) {
                    found = true;
                    break;
                }
            } else {
                if (!tmp_val) {
                    found = true;
                    break;
                }
            }
            ++count;
        }

        if (found) {
            return count;
        } else {
            return -1;
        }
    }

    inline ScalarRegI32
    firstOppositeSignBit(ScalarRegI64 val)
    {
        bool found(false);
        bool sign_bit = (val & 0x8000000000000000ULL) != 0;
        ScalarRegU64 tmp_val(0);
        int count(0);

        if (!val || val == -1) {
            return -1;
        }

        for (int i = 0; i < std::numeric_limits<ScalarRegU64>::digits; ++i) {
            tmp_val = val & (0x8000000000000000ULL >> i);

            if (!sign_bit) {
                if (tmp_val) {
                    found = true;
                    break;
                }
            } else {
                if (!tmp_val) {
                    found = true;
                    break;
                }
            }
            ++count;
        }

        if (found) {
            return count;
        } else {
            return -1;
        }
    }

    template<typename T>
    inline T
    median(T val_0, T val_1, T val_2)
    {
        if (std::is_floating_point_v<T>) {
            return std::fmax(std::fmin(val_0, val_1),
                std::fmin(std::fmax(val_0, val_1), val_2));
        } else {
            return std::max(std::min(val_0, val_1),
                std::min(std::max(val_0, val_1), val_2));
        }
    }

    template <typename T>
    inline T roundNearestEven(T val)
    {
        T int_part = 0;
        T nearest_round = std::floor(val + 0.5);
        if ((int)std::floor(val) % 2 == 0
            && std::modf(std::abs(val), &int_part) == 0.5) {
          nearest_round = nearest_round - 1;
        }

        return nearest_round;
    }

    inline VecElemU32
    muladd(VecElemU64 &dst, VecElemU32 val_0, VecElemU32 val_1,
        VecElemU64 val_2)
    {
        __uint128_t u0 = (__uint128_t)val_0;
        __uint128_t u1 = (__uint128_t)val_1;
        __uint128_t u2 = (__uint128_t)val_2;
        __uint128_t result = u0 * u1 + u2;

        dst = (VecElemU64)result;

        return (VecElemU32)(result >> 64) ? 1 : 0;
    }

    inline VecElemU32
    muladd(VecElemI64 &dst, VecElemI32 val_0, VecElemI32 val_1,
        VecElemI64 val_2)
    {
        __int128_t u0 = (__int128_t)val_0;
        __int128_t u1 = (__int128_t)val_1;
        __int128_t u2 = (__int128_t)val_2;
        __int128_t result = u0 * u1 + u2;

        dst = (VecElemI64)result;

        return (VecElemU32)(result >> 64) ? 1 : 0;
    }

    /**
     * dppInstImpl is a helper function that performs the inputted operation
     * on the inputted vector register lane.  The returned output lane
     * represents the input lane given the destination lane and DPP_CTRL word.
     *
     * Currently the values are:
     * 0x0 - 0xFF: full permute of four threads
     * 0x100: reserved
     * 0x101 - 0x10F: row shift left by 1-15 threads
     * 0x111 - 0x11F: row shift right by 1-15 threads
     * 0x121 - 0x12F: row rotate right by 1-15 threads
     * 0x130: wavefront left shift by 1 thread
     * 0x134: wavefront left rotate by 1 thread
     * 0x138: wavefront right shift by 1 thread
     * 0x13C: wavefront right rotate by 1 thread
     * 0x140: mirror threads within row
     * 0x141: mirror threads within 1/2 row (8 threads)
     * 0x142: broadcast 15th thread of each row to next row
     * 0x143: broadcast thread 31 to rows 2 and 3
     */
    inline int
    dppInstImpl(SqDPPVals dppCtrl, int currLane, int rowNum,
                    int rowOffset, bool & outOfBounds)
    {
        // local variables
        // newLane will be the same as the input lane unless swizzling happens
        int newLane = currLane;
        // for shift/rotate permutations; positive values are LEFT rotates
        // shift/rotate left means lane n -> lane n-1 (e.g., lane 1 -> lane 0)
        int count = 0;
        int localRowOffset = rowOffset;
        int localRowNum = rowNum;

        if (dppCtrl <= SQ_DPP_QUAD_PERM_MAX) { // DPP_QUAD_PERM{00:FF}
            int quadBase = (currLane & ~(3));
            int quadPix = (currLane & 3);
            quadPix = ((dppCtrl >> (2 * quadPix)) & 3);
            newLane = (quadBase | quadPix);
        } else if (dppCtrl == SQ_DPP_RESERVED) {
            panic("ERROR: instruction using reserved DPP_CTRL value\n");
        } else if ((dppCtrl >= SQ_DPP_ROW_SL1) &&
                   (dppCtrl <= SQ_DPP_ROW_SL15)) { // DPP_ROW_SL{1:15}
            count = (dppCtrl - SQ_DPP_ROW_SL1 + 1);
            if ((localRowOffset + count >= 0) &&
                (localRowOffset + count < ROW_SIZE)) {
                localRowOffset += count;
                newLane = ((rowNum * ROW_SIZE) | localRowOffset);
            } else {
                outOfBounds = true;
            }
        } else if ((dppCtrl >= SQ_DPP_ROW_SR1) &&
                   (dppCtrl <= SQ_DPP_ROW_SR15)) { // DPP_ROW_SR{1:15}
            count = -(dppCtrl - SQ_DPP_ROW_SR1 + 1);
            if ((localRowOffset + count >= 0) &&
                (localRowOffset + count < ROW_SIZE)) {
                localRowOffset += count;
                newLane = ((rowNum * ROW_SIZE) | localRowOffset);
            } else {
                outOfBounds = true;
            }
        } else if ((dppCtrl >= SQ_DPP_ROW_RR1) &&
                   (dppCtrl <= SQ_DPP_ROW_RR15)) { // DPP_ROW_RR{1:15}
            count = -(dppCtrl - SQ_DPP_ROW_RR1 + 1);
            localRowOffset = (localRowOffset + count + ROW_SIZE) % ROW_SIZE;
            newLane = ((rowNum * ROW_SIZE) | localRowOffset);
        } else if (dppCtrl == SQ_DPP_WF_SL1) { // DPP_WF_SL1
            if ((currLane >= 0) && (currLane < NumVecElemPerVecReg)) {
                newLane += 1;
            } else {
                outOfBounds = true;
            }
        } else if (dppCtrl == SQ_DPP_WF_RL1) { // DPP_WF_RL1
            newLane = (currLane - 1 + NumVecElemPerVecReg) %
                      NumVecElemPerVecReg;
        } else if (dppCtrl == SQ_DPP_WF_SR1) { // DPP_WF_SR1
            int currVal = (currLane - 1);
            if ((currVal >= 0) && (currVal < NumVecElemPerVecReg)) {
                newLane -= 1;
            } else {
                outOfBounds = true;
            }
        } else if (dppCtrl == SQ_DPP_WF_RR1) { // DPP_WF_RR1
            newLane = (currLane - 1 + NumVecElemPerVecReg) %
                      NumVecElemPerVecReg;
        } else if (dppCtrl == SQ_DPP_ROW_MIRROR) { // DPP_ROW_MIRROR
            localRowOffset = (15 - localRowOffset);
            newLane = (rowNum | localRowOffset);
        } else if (dppCtrl == SQ_DPP_ROW_HALF_MIRROR) { // DPP_ROW_HALF_MIRROR
            localRowNum = (currLane & -0x7);
            localRowOffset = (currLane & 0x7);
            localRowOffset = (7 - localRowNum);
            newLane = (localRowNum | localRowOffset);
        } else if (dppCtrl == SQ_DPP_ROW_BCAST15) { // DPP_ROW_BCAST15
            count = 15;
            if (currLane > count) {
                // 0x30 selects which set of 16 lanes to use. We broadcast the
                // last lane of one set to all lanes of the next set (e.g.,
                // lane 15 is written to 16-31, 31 to 32-47, 47 to 48-63).
                newLane = (currLane & 0x30) - 1;
            } else {
                outOfBounds = true;
            }
        } else if (dppCtrl == SQ_DPP_ROW_BCAST31) { // DPP_ROW_BCAST31
            count = 31;
            if (currLane > count) {
                // 0x20 selects either the upper 32 or lower 32 lanes and
                // broadcasts the last lane of one set to all lanes of the
                // next set (e.g., lane 31 is written to 32-63).
                newLane = (currLane & 0x20) - 1;
            } else {
                outOfBounds = true;
            }
        } else {
            panic("Unimplemented DPP control operation: %d\n", dppCtrl);
        }

        return newLane;
    }

    /**
     * processDPP is a helper function for implementing Data Parallel Primitive
     * instructions.  This function may be called by many different VOP1
     * instructions to do operations within a register.
     */
    template<typename T>
    void processDPP(GPUDynInstPtr gpuDynInst, InFmt_VOP_DPP dppInst,
                    T & src0)
    {
        // local variables
        SqDPPVals dppCtrl = (SqDPPVals)dppInst.DPP_CTRL;
        int boundCtrl = dppInst.BC;
        int bankMask = dppInst.BANK_MASK;
        int rowMask = dppInst.ROW_MASK;
        // row, bank info to be calculated per lane
        int rowNum = 0, bankNum = 0, rowOffset = 0;
        // outLane will be the same as the input lane unless swizzling happens
        int outLane = 0;
        bool laneDisabled = false;
        // flags used for determining if a lane should be written to/reset/etc.
        bool outOfBounds = false, zeroSrc = false;
        long long threadValid = 0;

        /**
         * STEP 1a: check if the absolute value (ABS) or negation (NEG) tags
         * are set.  If so, do the appropriate action(s) on src0 and/or src1.
         *
         * NOTE: ABS takes priority over NEG.
         */
        if (dppInst.SRC0_NEG) {
            src0.negModifier();
        }

        if (dppInst.SRC0_ABS) {
            src0.absModifier();
        }

        // Need a copy of the original data since we update one lane at a time
        T src0_copy = src0;

        // iterate over all register lanes, performing steps 2-4
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            threadValid = (0x1LL << lane);
            /**
             * STEP 2: check the row and bank mask values.  These determine
             * which threads are enabled for the subsequent DPP_CTRL
             * operations.
             */
            rowNum = (lane / ROW_SIZE);
            rowOffset = (lane % ROW_SIZE);
            bankNum = (rowOffset / NUM_BANKS);

            if (((rowMask & (0x1 << rowNum)) == 0)   /* row mask */   ||
                ((bankMask & (0x1 << bankNum)) == 0) /* bank mask */) {
                laneDisabled = true;
            }

            /**
             * STEP 4: Handle the potential values of DPP_CTRL:
             * 0x0 - 0xFF: full permute of four threads
             * 0x100: reserved
             * 0x101 - 0x10F: row shift right by 1-15 threads
             * 0x111 - 0x11F: row shift right by 1-15 threads
             * 0x121 - 0x12F: row shift right by 1-15 threads
             * 0x130: wavefront left shift by 1 thread
             * 0x134: wavefront left rotate by 1 thread
             * 0x138: wavefront right shift by 1 thread
             * 0x13C: wavefront right rotate by 1 thread
             * 0x140: mirror threads within row
             * 0x141: mirror threads within 1/2 row (8 threads)
             * 0x142: broadcast 15th thread of each row to next row
             * 0x143: broadcast thread 31 to rows 2 and 3
             */
            if (!laneDisabled) {
                outLane = dppInstImpl(dppCtrl, lane, rowNum, rowOffset,
                                      outOfBounds);
            }

            /**
             * STEP 4: Implement bound control for disabled threads.  If thread
             * is disabled but boundCtrl is set, then we need to set the source
             * data to 0 (i.e., set this lane to 0).
             */
            if (laneDisabled) {
                threadValid = 0;
            } else if (outOfBounds) {
                if (boundCtrl == 1) {
                    zeroSrc = true;
                } else {
                    threadValid = 0;
                }
            } else if (!gpuDynInst->wavefront()->execMask(lane)) {
                if (boundCtrl == 1) {
                    zeroSrc = true;
                } else {
                    threadValid = 0;
                }
            }

            if (threadValid != 0 && !outOfBounds && !zeroSrc) {
                assert(!laneDisabled);
                src0[lane] = src0_copy[outLane];
            } else if (zeroSrc) {
                src0[lane] = 0;
            }

            // reset for next iteration
            laneDisabled = false;
            outOfBounds = false;
            zeroSrc = false;
        }
    }

    /**
     * processDPP is a helper function for implementing Data Parallel Primitive
     * instructions.  This function may be called by many different
     * VOP2/VOPC instructions to do operations within a register.
     */
    template<typename T>
    void processDPP(GPUDynInstPtr gpuDynInst, InFmt_VOP_DPP dppInst,
                    T & src0, T & src1)
    {
        /**
         * STEP 1b: check if the absolute value (ABS) or negation (NEG) tags
         * are set.  If so, do the appropriate action(s) on src0 and/or src1.
         *
         * NOTE: ABS takes priority over NEG.
         */
        if (dppInst.SRC1_NEG) {
            src1.negModifier();
        }

        if (dppInst.SRC1_ABS) {
            src1.absModifier();
        }

        // Since only difference for VOP1 and VOP2/VOPC instructions is SRC1,
        // which is only used for negation/absolute value, call other version
        // to do everything else.
        processDPP(gpuDynInst, dppInst, src0);
    }

    /**
     * sdwaInstSrcImpl_helper contains the per-lane code for selecting the
     * appropriate bytes/words of the lane and doing the appropriate
     * masking/padding/sign extending.  It returns the value after these
     * operations are done on it.
     */
    template<typename T>
    T sdwaInstSrcImpl_helper(T currOperVal, const T origOperVal,
                             const SDWASelVals sel, const bool signExt)
    {
        // local variables
        int low_bit = 0, high_bit = 0;
        bool signExt_local = signExt;
        T retVal = 0;

        // if we're preserving all of the bits, then we can immediately return
        if (sel == SDWA_DWORD) {
            return currOperVal;
        }

        if (sel < SDWA_WORD_0) { // we are selecting 1 byte
            /*
              Process byte 0 first.  This code eiter selects the original bits
              of byte 0, or makes the bits of the selected byte be byte 0 (and
              next either sign extends or zero's out upper bits).
            */
            low_bit = (sel * VegaISA::BITS_PER_BYTE);
            high_bit = low_bit + VegaISA::MSB_PER_BYTE;
            retVal = bits(currOperVal, high_bit, low_bit);

            // make sure update propagated, since used next
            panic_if(bits(retVal, VegaISA::MSB_PER_BYTE) !=
                     bits(origOperVal, high_bit),
                     "ERROR: SDWA byte update not propagated: retVal: %d, "
                     "orig: %d\n", bits(retVal, VegaISA::MSB_PER_BYTE),
                     bits(origOperVal, high_bit));
            // sign extended value depends on upper-most bit of the new byte 0
            signExt_local = (signExt &&
                             (bits(retVal, VegaISA::MSB_PER_BYTE, 0) & 0x80));

            // process all other bytes -- if sign extending, make them 1, else
            // all 0's so leave as is
            if (signExt_local) {
                retVal = (uint32_t)sext<VegaISA::MSB_PER_BYTE>(retVal);
            }
        } else if (sel < SDWA_DWORD) { // we are selecting 1 word
            /*
              Process word 0 first.  This code eiter selects the original bits
              of word 0, or makes the bits of the selected word be word 0 (and
              next either sign extends or zero's out upper bits).
            */
            low_bit = (sel & 1) * VegaISA::BITS_PER_WORD;
            high_bit = low_bit + VegaISA::MSB_PER_WORD;
            retVal = bits(currOperVal, high_bit, low_bit);

            // make sure update propagated, since used next
            panic_if(bits(retVal, VegaISA::MSB_PER_WORD) !=
                     bits(origOperVal, high_bit),
                     "ERROR: SDWA word update not propagated: retVal: %d, "
                     "orig: %d\n",
                     bits(retVal, VegaISA::MSB_PER_WORD),
                     bits(origOperVal, high_bit));
            // sign extended value depends on upper-most bit of the new word 0
            signExt_local = (signExt &&
                             (bits(retVal, VegaISA::MSB_PER_WORD, 0) &
                              0x8000));

            // process other word -- if sign extending, make them 1, else all
            // 0's so leave as is
            if (signExt_local) {
                retVal = (uint32_t)sext<VegaISA::MSB_PER_WORD>(retVal);
            }
        } else {
            assert(sel != SDWA_DWORD); // should have returned earlier
            panic("Unimplemented SDWA select operation: %d\n", sel);
        }

        return retVal;
    }


    /**
     * sdwaInstSrcImpl is a helper function that selects the appropriate
     * bits/bytes for each lane of the inputted source operand of an SDWA
     * instruction, does the appropriate masking/padding/sign extending for the
     * non-selected bits/bytes, and updates the operands values with the
     * resultant value.
     *
     * The desired behavior is:
     *   1.  Select the appropriate bits/bytes based on sel:
     *       0 (SDWA_BYTE_0): select data[7:0]
     *       1 (SDWA_BYTE_1): select data[15:8]
     *       2 (SDWA_BYTE_2): select data[23:16]
     *       3 (SDWA_BYTE_3): select data[31:24]
     *       4 (SDWA_WORD_0): select data[15:0]
     *       5 (SDWA_WORD_1): select data[31:16]
     *       6 (SDWA_DWORD): select data[31:0]
     *   2.  if sign extend is set, then sign extend the value
     */
    template<typename T>
    void sdwaInstSrcImpl(T & currOper, T & origCurrOper,
                         const SDWASelVals sel, const bool signExt)
    {
        // iterate over all lanes, setting appropriate, selected value
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            currOper[lane] = sdwaInstSrcImpl_helper(currOper[lane],
                                                    origCurrOper[lane], sel,
                                                    signExt);
        }
    }


    /**
     * sdwaInstDstImpl_helper contains the per-lane code for selecting the
     * appropriate bytes/words of the lane and doing the appropriate
     * masking/padding/sign extending.  It returns the value after these
     * operations are done on it.
     */
    template<typename T>
    T sdwaInstDstImpl_helper(T currDstVal, const T origDstVal,
                             const bool clamp, const SDWASelVals sel,
                             const SDWADstVals unusedBits_format)
    {
        // local variables
        int low_bit = 0, high_bit = 0;
        bool signExt = (unusedBits_format == SDWA_UNUSED_SEXT);
        //bool pad = (unusedBits_format == SDWA_UNUSED_PAD);
        bool preserve = (unusedBits_format == SDWA_UNUSED_PRESERVE);
        T retVal = 0, origBits_thisByte = 0, currBits_thisByte = 0,
          origBits_thisWord = 0, currBits_thisWord = 0, newBits = 0;

        // if we're preserving all of the bits, then we can immediately return
        if (unusedBits_format == SDWA_UNUSED_PRESERVE) {
            assert(sel == SDWA_DWORD);
            return currDstVal;
        } else if (sel == SDWA_DWORD) {
            // NOTE: users may set the unused bits variable to anything in this
            // scenario, because it will be ignored
            return currDstVal;
        }

        if (sel < SDWA_WORD_0) { // we are selecting 1 byte
            // if we sign extended depends on upper-most bit of byte 0
            signExt = (signExt &&
                       (bits(currDstVal, VegaISA::MSB_PER_BYTE, 0) & 0x80));

            for (int byte = 0; byte < 4; ++byte) {
                low_bit = byte * VegaISA::BITS_PER_BYTE;
                high_bit = low_bit + VegaISA::MSB_PER_BYTE;
                /*
                  Options:
                    1.  byte == sel: we are keeping all bits in this byte
                    2.  preserve is set: keep this byte as is because the
                    output preserve flag is set
                    3.  byte > sel && signExt: we're sign extending and
                    this byte is one of the bytes we need to sign extend
                */
                origBits_thisByte = bits(origDstVal, VegaISA::MSB_PER_BYTE, 0);
                currBits_thisByte = bits(currDstVal, high_bit, low_bit);
                newBits = ((byte == sel) ? origBits_thisByte :
                           ((preserve) ? currBits_thisByte :
                            (((byte > sel) && signExt) ? 0xff : 0)));
                retVal = insertBits(retVal, high_bit, low_bit, newBits);
            }
        } else if (sel < SDWA_DWORD) { // we are selecting 1 word
            low_bit = 0;
            high_bit = low_bit + VegaISA::MSB_PER_WORD;
            // if we sign extended depends on upper-most bit of word 0
            signExt = (signExt &&
                       (bits(currDstVal, high_bit, low_bit) & 0x8000));

            for (int word = 0; word < 2; ++word) {
                low_bit = word * VegaISA::BITS_PER_WORD;
                high_bit = low_bit + VegaISA::MSB_PER_WORD;
                /*
                  Options:
                    1.  word == sel & 1: we are keeping all bits in this word
                    2.  preserve is set: keep this word as is because the
                    output preserve flag is set
                    3.  word > (sel & 1) && signExt: we're sign extending and
                    this word is one of the words we need to sign extend
                */
                origBits_thisWord = bits(origDstVal, VegaISA::MSB_PER_WORD, 0);
                currBits_thisWord = bits(currDstVal, high_bit, low_bit);
                newBits = ((word == (sel & 0x1)) ? origBits_thisWord :
                           ((preserve) ? currBits_thisWord :
                            (((word > (sel & 0x1)) && signExt) ? 0xffff : 0)));
                retVal = insertBits(retVal, high_bit, low_bit, newBits);
            }
        } else {
            assert(sel != SDWA_DWORD); // should have returned earlier
            panic("Unimplemented SDWA select operation: %d\n", sel);
        }

        return retVal;
    }


    /**
     * sdwaInstDestImpl is a helper function that selects the appropriate
     * bits/bytes for the inputted dest operand of an SDWA instruction, does
     * the appropriate masking/padding/sign extending for the non-selected
     * bits/bytes, and updates the operands values with the resultant value.
     *
     * The desired behavior is:
     *   1.  Select the appropriate bits/bytes based on sel:
     *       0 (SDWA_BYTE_0): select data[7:0]
     *       1 (SDWA_BYTE_1): select data[15:8]
     *       2 (SDWA_BYTE_2): select data[23:16]
     *       3 (SDWA_BYTE_3): select data[31:24]
     *       4 (SDWA_WORD_0): select data[15:0]
     *       5 (SDWA_WORD_1): select data[31:16]
     *       6 (SDWA_DWORD): select data[31:0]
     *   2.  either pad, sign extend, or select all bits based on the value of
     *   unusedBits_format:
     *       0 (SDWA_UNUSED_PAD): pad all unused bits with 0
     *       1 (SDWA_UNUSED_SEXT): sign-extend upper bits; pad lower bits w/ 0
     *       2 (SDWA_UNUSED_PRESERVE): select data[31:0]
     */
    template<typename T>
    void sdwaInstDstImpl(T & dstOper, T & origDstOper, const bool clamp,
                         const SDWASelVals sel,
                         const SDWADstVals unusedBits_format)
    {
        // iterate over all lanes, setting appropriate, selected value
        for (int lane = 0; lane < NumVecElemPerVecReg; ++lane) {
            dstOper[lane] = sdwaInstDstImpl_helper(dstOper[lane],
                                                   origDstOper[lane], clamp,
                                                   sel, unusedBits_format);
        }
    }


    /**
     * processSDWA_srcHelper is a helper function for implementing sub d-word
     * addressing instructions for the src operands.  This function may be
     * called by many different VOP1/VOP2/VOPC instructions to do operations
     * within a register.  This function is also agnostic of which operand it
     * is operating on, so that it can be called for any src operand.
     */
    template<typename T>
    void processSDWA_src_helper(T & currSrc, T & origCurrSrc,
                                const SDWASelVals src_sel,
                                const bool src_signExt, const bool src_abs,
                                const bool src_neg)
    {
        /**
         * STEP 1: check if the absolute value (ABS) or negation (NEG) tags
         * are set.  If so, do the appropriate action(s) on the src operand.
         *
         * NOTE: According to the CSim implementation, ABS takes priority over
         * NEG.
         */
        if (src_neg) {
            currSrc.negModifier();
        }

        if (src_abs) {
            currSrc.absModifier();
        }

        /**
         * STEP 2: select the appropriate bits for each lane of source operand.
         */
        sdwaInstSrcImpl(currSrc, origCurrSrc, src_sel, src_signExt);
    }


    /**
     * processSDWA_src is a helper function for implementing sub d-word
     * addressing instructions for the src operands.  This function may be
     * called by many different VOP1 instructions to do operations within a
     * register.  processSDWA_dst is called after the math, while
     * processSDWA_src is called before the math.
     */
    template<typename T>
    void processSDWA_src(InFmt_VOP_SDWA sdwaInst, T & src0, T & origSrc0)
    {
        // local variables
        const SDWASelVals src0_sel = (SDWASelVals)sdwaInst.SRC0_SEL;
        const bool src0_signExt = sdwaInst.SRC0_SEXT;
        const bool src0_neg = sdwaInst.SRC0_NEG;
        const bool src0_abs = sdwaInst.SRC0_ABS;

        // NOTE: difference between VOP1 and VOP2/VOPC is that there is no src1
        // operand.  So ensure that SRC1 fields are not set, then call helper
        // function only on src0.
        assert(!sdwaInst.SRC1_SEXT);
        assert(!sdwaInst.SRC1_NEG);
        assert(!sdwaInst.SRC1_ABS);

        processSDWA_src_helper(src0, origSrc0, src0_sel, src0_signExt,
                               src0_abs, src0_neg);
    }


    /**
     * processSDWA_src is a helper function for implementing sub d-word
     * addressing instructions.  This function may be called by many different
     * VOP2/VOPC instructions to do operations within a register.
     * processSDWA_dst is called after the math, while processSDWA_src is
     * called before the math.
     */
    template<typename T>
    void processSDWA_src(InFmt_VOP_SDWA sdwaInst, T & src0, T & origSrc0,
                         T & src1, T & origSrc1)
    {
        // local variables
        const SDWASelVals src0_sel = (SDWASelVals)sdwaInst.SRC0_SEL;
        const bool src0_signExt = sdwaInst.SRC0_SEXT;
        const bool src0_neg = sdwaInst.SRC0_NEG;
        const bool src0_abs = sdwaInst.SRC0_ABS;
        const SDWASelVals src1_sel = (SDWASelVals)sdwaInst.SRC1_SEL;
        const bool src1_signExt = sdwaInst.SRC1_SEXT;
        const bool src1_neg = sdwaInst.SRC1_NEG;
        const bool src1_abs = sdwaInst.SRC1_ABS;

        processSDWA_src_helper(src0, origSrc0, src0_sel, src0_signExt,
                               src0_abs, src0_neg);
        processSDWA_src_helper(src1, origSrc1, src1_sel, src1_signExt,
                               src1_abs, src1_neg);
    }


    /**
     * processSDWA_dst is a helper function for implementing sub d-word
     * addressing instructions for the dst operand.  This function may be
     * called by many different VOP1/VOP2/VOPC instructions to do operations
     * within a register.  processSDWA_dst is called after the math, while
     * processSDWA_src is called before the math.
     */
    template<typename T>
    void processSDWA_dst(InFmt_VOP_SDWA sdwaInst, T & dst, T & origDst)
    {
        // local variables
        const SDWADstVals dst_unusedBits_format =
            (SDWADstVals)sdwaInst.DST_U;
        const SDWASelVals dst_sel = (SDWASelVals)sdwaInst.DST_SEL;
        const bool clamp = sdwaInst.CLMP;

        /**
         * STEP 1: select the appropriate bits for dst and pad/sign-extend as
         * appropriate.
         */
        sdwaInstDstImpl(dst, origDst, clamp, dst_sel, dst_unusedBits_format);
    }
} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_VEGA_INSTS_INST_UTIL_HH__
