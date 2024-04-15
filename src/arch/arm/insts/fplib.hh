/*
 * Copyright (c) 2012-2013, 2017-2018 ARM Limited
 * Copyright (c) 2020 Metempsy Technology Consulting
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file
 * Floating-point library code, which will gradually replace vfp.hh. For
 * portability, this library does not use floating-point data types. Currently,
 * C's standard integer types are used in the API, though this could be changed
 * to something like class Fp32 { uint32_t x; }, etc.
 */

#ifndef __ARCH_ARM_INSTS_FPLIB_HH__
#define __ARCH_ARM_INSTS_FPLIB_HH__

#include <stdint.h>

#include "arch/arm/regs/misc.hh"

namespace gem5
{

namespace ArmISA
{

enum FPRounding
{
    FPRounding_TIEEVEN = 0,
    FPRounding_POSINF = 1,
    FPRounding_NEGINF = 2,
    FPRounding_ZERO = 3,
    FPRounding_TIEAWAY = 4,
    FPRounding_ODD = 5
};

static inline FPRounding
FPCRRounding(FPSCR &fpscr)
{
    return (FPRounding)((uint32_t)fpscr >> 22 & 3);
}

/** Floating-point absolute value. */
template <class T>
T fplibAbs(T op);
/** Floating-point add. */
template <class T>
T fplibAdd(T op1, T op2, FPSCR &fpscr);
/** Floating-point compare (quiet and signaling). */
template <class T>
int fplibCompare(T op1, T op2, bool signal_nans, FPSCR &fpscr);
/** Floating-point compare equal. */
template <class T>
bool fplibCompareEQ(T op1, T op2, FPSCR &fpscr);
/** Floating-point compare greater than or equal. */
template <class T>
bool fplibCompareGE(T op1, T op2, FPSCR &fpscr);
/** Floating-point compare greater than. */
template <class T>
bool fplibCompareGT(T op1, T op2, FPSCR &fpscr);
/** Floating-point compare unordered. */
template <class T>
bool fplibCompareUN(T op1, T op2, FPSCR &fpscr);
/** Floating-point convert precision. */
template <class T1, class T2>
T2 fplibConvert(T1 op, FPRounding rounding, FPSCR &fpscr);
/** Floating-point division. */
template <class T>
T fplibDiv(T op1, T op2, FPSCR &fpscr);
/** Floating-point exponential accelerator. */
template <class T>
T fplibExpA(T op);
/** Floating-point maximum. */
template <class T>
T fplibMax(T op1, T op2, FPSCR &fpscr);
/** Floating-point maximum number. */
template <class T>
T fplibMaxNum(T op1, T op2, FPSCR &fpscr);
/** Floating-point minimum. */
template <class T>
T fplibMin(T op1, T op2, FPSCR &fpscr);
/** Floating-point minimum number. */
template <class T>
T fplibMinNum(T op1, T op2, FPSCR &fpscr);
/** Floating-point multiply. */
template <class T>
T fplibMul(T op1, T op2, FPSCR &fpscr);
/** Floating-point multiply-add. */
template <class T>
T fplibMulAdd(T addend, T op1, T op2, FPSCR &fpscr);
/** Floating-point multiply extended. */
template <class T>
T fplibMulX(T op1, T op2, FPSCR &fpscr);
/** Floating-point negate. */
template <class T>
T fplibNeg(T op);
/** Floating-point reciprocal square root estimate. */
template <class T>
T fplibRSqrtEstimate(T op, FPSCR &fpscr);
/** Floating-point reciprocal square root step. */
template <class T>
T fplibRSqrtStepFused(T op1, T op2, FPSCR &fpscr);
/** Floating-point reciprocal estimate. */
template <class T>
T fplibRecipEstimate(T op, FPSCR &fpscr);
/** Floating-point reciprocal step. */
template <class T>
T fplibRecipStepFused(T op1, T op2, FPSCR &fpscr);
/** Floating-point reciprocal exponent. */
template <class T>
T fplibRecpX(T op, FPSCR &fpscr);
/**  Floating-point convert to integer. */
template <class T>
T fplibRoundInt(T op, FPRounding rounding, bool exact, FPSCR &fpscr);
/** Floating-point adjust exponent. */
template <class T>
T fplibScale(T op1, T op2, FPSCR &fpscr);
/** Floating-point square root. */
template <class T>
T fplibSqrt(T op, FPSCR &fpscr);
/** Floating-point subtract. */
template <class T>
T fplibSub(T op1, T op2, FPSCR &fpscr);
/** Floating-point trigonometric multiply-add coefficient. */
template <class T>
T fplibTrigMulAdd(uint8_t coeff_index, T op1, T op2, FPSCR &fpscr);
/** Floating-point trigonometric starting value. */
template <class T>
T fplibTrigSMul(T op1, T op2, FPSCR &fpscr);
/** Floating-point trigonometric select coefficient. */
template <class T>
T fplibTrigSSel(T op1, T op2, FPSCR &fpscr);
/** Floating-point convert to fixed-point. */
template <class T1, class T2>
T2 fplibFPToFixed(T1 op, int fbits, bool u, FPRounding rounding, FPSCR &fpscr);
/** Floating-point convert from fixed-point. */
template <class T>
T fplibFixedToFP(uint64_t op, int fbits, bool u, FPRounding rounding,
                 FPSCR &fpscr);
/** Floating-point value for +/- infinity. */
template <class T>
T fplibInfinity(int sgn);
/** Foating-point value for default NaN. */
template <class T>
T fplibDefaultNaN();
/** Floating-point  JS convert to a signed integer, with rounding to zero. */
uint32_t fplibFPToFixedJS(uint64_t op, FPSCR &fpscr, bool Is64, uint8_t &nz);

/* Function specializations... */
template <>
uint16_t fplibAbs(uint16_t op);
template <>
uint32_t fplibAbs(uint32_t op);
template <>
uint64_t fplibAbs(uint64_t op);
template <>
uint16_t fplibAdd(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibAdd(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibAdd(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
int fplibCompare(uint16_t op1, uint16_t op2, bool signal_nans, FPSCR &fpscr);
template <>
int fplibCompare(uint32_t op1, uint32_t op2, bool signal_nans, FPSCR &fpscr);
template <>
int fplibCompare(uint64_t op1, uint64_t op2, bool signal_nans, FPSCR &fpscr);
template <>
bool fplibCompareEQ(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
bool fplibCompareEQ(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
bool fplibCompareEQ(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
bool fplibCompareGE(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
bool fplibCompareGE(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
bool fplibCompareGE(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
bool fplibCompareGT(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
bool fplibCompareGT(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
bool fplibCompareGT(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
bool fplibCompareUN(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
bool fplibCompareUN(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
bool fplibCompareUN(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibConvert(uint32_t op, FPRounding rounding, FPSCR &fpscr);
template <>
uint16_t fplibConvert(uint64_t op, FPRounding rounding, FPSCR &fpscr);
template <>
uint32_t fplibConvert(uint16_t op, FPRounding rounding, FPSCR &fpscr);
template <>
uint32_t fplibConvert(uint64_t op, FPRounding rounding, FPSCR &fpscr);
template <>
uint64_t fplibConvert(uint16_t op, FPRounding rounding, FPSCR &fpscr);
template <>
uint64_t fplibConvert(uint32_t op, FPRounding rounding, FPSCR &fpscr);
template <>
uint16_t fplibDiv(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibDiv(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibDiv(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibExpA(uint16_t op);
template <>
uint32_t fplibExpA(uint32_t op);
template <>
uint64_t fplibExpA(uint64_t op);
template <>
uint16_t fplibMax(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibMax(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibMax(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibMaxNum(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibMaxNum(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibMaxNum(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibMin(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibMin(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibMin(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibMinNum(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibMinNum(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibMinNum(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibMul(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibMul(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibMul(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibMulAdd(uint16_t addend, uint16_t op1, uint16_t op2,
                     FPSCR &fpscr);
template <>
uint32_t fplibMulAdd(uint32_t addend, uint32_t op1, uint32_t op2,
                     FPSCR &fpscr);
template <>
uint64_t fplibMulAdd(uint64_t addend, uint64_t op1, uint64_t op2,
                     FPSCR &fpscr);
template <>
uint16_t fplibMulX(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibMulX(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibMulX(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibNeg(uint16_t op);
template <>
uint32_t fplibNeg(uint32_t op);
template <>
uint64_t fplibNeg(uint64_t op);
template <>
uint16_t fplibRSqrtEstimate(uint16_t op, FPSCR &fpscr);
template <>
uint32_t fplibRSqrtEstimate(uint32_t op, FPSCR &fpscr);
template <>
uint64_t fplibRSqrtEstimate(uint64_t op, FPSCR &fpscr);
template <>
uint16_t fplibRSqrtStepFused(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibRSqrtStepFused(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibRSqrtStepFused(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibRecipEstimate(uint16_t op, FPSCR &fpscr);
template <>
uint32_t fplibRecipEstimate(uint32_t op, FPSCR &fpscr);
template <>
uint64_t fplibRecipEstimate(uint64_t op, FPSCR &fpscr);
template <>
uint16_t fplibRecipStepFused(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibRecipStepFused(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibRecipStepFused(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibRecpX(uint16_t op, FPSCR &fpscr);
template <>
uint32_t fplibRecpX(uint32_t op, FPSCR &fpscr);
template <>
uint64_t fplibRecpX(uint64_t op, FPSCR &fpscr);
template <>
uint16_t fplibRoundInt(uint16_t op, FPRounding rounding, bool exact,
                       FPSCR &fpscr);
template <>
uint32_t fplibRoundInt(uint32_t op, FPRounding rounding, bool exact,
                       FPSCR &fpscr);
template <>
uint64_t fplibRoundInt(uint64_t op, FPRounding rounding, bool exact,
                       FPSCR &fpscr);
template <>
uint16_t fplibScale(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibScale(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibScale(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibSqrt(uint16_t op, FPSCR &fpscr);
template <>
uint32_t fplibSqrt(uint32_t op, FPSCR &fpscr);
template <>
uint64_t fplibSqrt(uint64_t op, FPSCR &fpscr);
template <>
uint16_t fplibSub(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibSub(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibSub(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibTrigMulAdd(uint8_t coeff_index, uint16_t op1, uint16_t op2,
                         FPSCR &fpscr);
template <>
uint32_t fplibTrigMulAdd(uint8_t coeff_index, uint32_t op1, uint32_t op2,
                         FPSCR &fpscr);
template <>
uint64_t fplibTrigMulAdd(uint8_t coeff_index, uint64_t op1, uint64_t op2,
                         FPSCR &fpscr);
template <>
uint16_t fplibTrigSMul(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibTrigSMul(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibTrigSMul(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibTrigSSel(uint16_t op1, uint16_t op2, FPSCR &fpscr);
template <>
uint32_t fplibTrigSSel(uint32_t op1, uint32_t op2, FPSCR &fpscr);
template <>
uint64_t fplibTrigSSel(uint64_t op1, uint64_t op2, FPSCR &fpscr);
template <>
uint16_t fplibFPToFixed(uint16_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint32_t fplibFPToFixed(uint16_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint32_t fplibFPToFixed(uint32_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint32_t fplibFPToFixed(uint64_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint64_t fplibFPToFixed(uint16_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint64_t fplibFPToFixed(uint32_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint64_t fplibFPToFixed(uint64_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint16_t fplibFixedToFP(uint64_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint32_t fplibFixedToFP(uint64_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint64_t fplibFixedToFP(uint64_t op, int fbits, bool u, FPRounding rounding,
                        FPSCR &fpscr);
template <>
uint16_t fplibInfinity(int sgn);
template <>
uint32_t fplibInfinity(int sgn);
template <>
uint64_t fplibInfinity(int sgn);
template <>
uint16_t fplibDefaultNaN();
template <>
uint32_t fplibDefaultNaN();
template <>
uint64_t fplibDefaultNaN();

} // namespace ArmISA
} // namespace gem5

#endif
