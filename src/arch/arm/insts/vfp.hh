/*
 * Copyright (c) 2010 ARM Limited
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
 *
 * Authors: Gabe Black
 */

#ifndef __ARCH_ARM_INSTS_VFP_HH__
#define __ARCH_ARM_INSTS_VFP_HH__

#include "arch/arm/insts/misc.hh"
#include "arch/arm/miscregs.hh"
#include <fenv.h>
#include <cmath>

namespace ArmISA
{

enum VfpMicroMode {
    VfpNotAMicroop,
    VfpMicroop,
    VfpFirstMicroop,
    VfpLastMicroop
};

template<class T>
static inline void
setVfpMicroFlags(VfpMicroMode mode, T &flags)
{
    switch (mode) {
      case VfpMicroop:
        flags[StaticInst::IsMicroop] = true;
        break;
      case VfpFirstMicroop:
        flags[StaticInst::IsMicroop] =
            flags[StaticInst::IsFirstMicroop] = true;
        break;
      case VfpLastMicroop:
        flags[StaticInst::IsMicroop] =
            flags[StaticInst::IsLastMicroop] = true;
        break;
      case VfpNotAMicroop:
        break;
    }
    if (mode == VfpMicroop || mode == VfpFirstMicroop) {
        flags[StaticInst::IsDelayedCommit] = true;
    }
}

enum FeExceptionBit
{
    FeDivByZero = FE_DIVBYZERO,
    FeInexact = FE_INEXACT,
    FeInvalid = FE_INVALID,
    FeOverflow = FE_OVERFLOW,
    FeUnderflow = FE_UNDERFLOW,
    FeAllExceptions = FE_ALL_EXCEPT
};

enum FeRoundingMode
{
    FeRoundDown = FE_DOWNWARD,
    FeRoundNearest = FE_TONEAREST,
    FeRoundZero = FE_TOWARDZERO,
    FeRoundUpward = FE_UPWARD
};

enum VfpRoundingMode
{
    VfpRoundNearest = 0,
    VfpRoundUpward = 1,
    VfpRoundDown = 2,
    VfpRoundZero = 3
};

template <class fpType>
static inline bool
flushToZero(fpType &op)
{
    fpType junk = 0.0;
    if (std::fpclassify(op) == FP_SUBNORMAL) {
        uint64_t bitMask = ULL(0x1) << (sizeof(fpType) * 8 - 1);
        op = bitsToFp(fpToBits(op) & bitMask, junk);
        return true;
    }
    return false;
}

template <class fpType>
static inline bool
flushToZero(fpType &op1, fpType &op2)
{
    bool flush1 = flushToZero(op1);
    bool flush2 = flushToZero(op2);
    return flush1 || flush2;
}

template <class fpType>
static inline void
vfpFlushToZero(FPSCR &fpscr, fpType &op)
{
    if (fpscr.fz == 1 && flushToZero(op)) {
        fpscr.idc = 1;
    }
}

template <class fpType>
static inline void
vfpFlushToZero(FPSCR &fpscr, fpType &op1, fpType &op2)
{
    vfpFlushToZero(fpscr, op1);
    vfpFlushToZero(fpscr, op2);
}

static inline uint32_t
fpToBits(float fp)
{
    union
    {
        float fp;
        uint32_t bits;
    } val;
    val.fp = fp;
    return val.bits;
}

static inline uint64_t
fpToBits(double fp)
{
    union
    {
        double fp;
        uint64_t bits;
    } val;
    val.fp = fp;
    return val.bits;
}

static inline float
bitsToFp(uint64_t bits, float junk)
{
    union
    {
        float fp;
        uint32_t bits;
    } val;
    val.bits = bits;
    return val.fp;
}

static inline double
bitsToFp(uint64_t bits, double junk)
{
    union
    {
        double fp;
        uint64_t bits;
    } val;
    val.bits = bits;
    return val.fp;
}

typedef int VfpSavedState;

static inline VfpSavedState
prepFpState(uint32_t rMode)
{
    int roundingMode = fegetround();
    feclearexcept(FeAllExceptions);
    switch (rMode) {
      case VfpRoundNearest:
        fesetround(FeRoundNearest);
        break;
      case VfpRoundUpward:
        fesetround(FeRoundUpward);
        break;
      case VfpRoundDown:
        fesetround(FeRoundDown);
        break;
      case VfpRoundZero:
        fesetround(FeRoundZero);
        break;
    }
    return roundingMode;
}

static inline void
finishVfp(FPSCR &fpscr, VfpSavedState state)
{
    int exceptions = fetestexcept(FeAllExceptions);
    bool underflow = false;
    if (exceptions & FeInvalid) {
        fpscr.ioc = 1;
    }
    if (exceptions & FeDivByZero) {
        fpscr.dzc = 1;
    }
    if (exceptions & FeOverflow) {
        fpscr.ofc = 1;
    }
    if (exceptions & FeUnderflow) {
        underflow = true;
        fpscr.ufc = 1;
    }
    if ((exceptions & FeInexact) && !(underflow && fpscr.fz)) {
        fpscr.ixc = 1;
    }
    fesetround(state);
}

template <class fpType>
static inline fpType
fixDest(FPSCR fpscr, fpType val, fpType op1)
{
    int fpClass = std::fpclassify(val);
    fpType junk = 0.0;
    if (fpClass == FP_NAN) {
        const bool single = (sizeof(val) == sizeof(float));
        const uint64_t qnan = single ? 0x7fc00000 : ULL(0x7ff8000000000000);
        const bool nan = std::isnan(op1);
        if (!nan || (fpscr.dn == 1)) {
            val = bitsToFp(qnan, junk);
        } else if (nan) {
            val = bitsToFp(fpToBits(op1) | qnan, junk);
        }
    } else if (fpClass == FP_SUBNORMAL && fpscr.fz == 1) {
        // Turn val into a zero with the correct sign;
        uint64_t bitMask = ULL(0x1) << (sizeof(fpType) * 8 - 1);
        val = bitsToFp(fpToBits(val) & bitMask, junk);
        feclearexcept(FeInexact);
        feraiseexcept(FeUnderflow);
    }
    return val;
}

template <class fpType>
static inline fpType
fixDest(FPSCR fpscr, fpType val, fpType op1, fpType op2)
{
    int fpClass = std::fpclassify(val);
    fpType junk = 0.0;
    if (fpClass == FP_NAN) {
        const bool single = (sizeof(val) == sizeof(float));
        const uint64_t qnan = single ? 0x7fc00000 : ULL(0x7ff8000000000000);
        const bool nan1 = std::isnan(op1);
        const bool nan2 = std::isnan(op2);
        const bool signal1 = nan1 && ((fpToBits(op1) & qnan) != qnan);
        const bool signal2 = nan2 && ((fpToBits(op2) & qnan) != qnan);
        if ((!nan1 && !nan2) || (fpscr.dn == 1)) {
            val = bitsToFp(qnan, junk);
        } else if (signal1) {
            val = bitsToFp(fpToBits(op1) | qnan, junk);
        } else if (signal2) {
            val = bitsToFp(fpToBits(op2) | qnan, junk);
        } else if (nan1) {
            val = op1;
        } else if (nan2) {
            val = op2;
        }
    } else if (fpClass == FP_SUBNORMAL && fpscr.fz == 1) {
        // Turn val into a zero with the correct sign;
        uint64_t bitMask = ULL(0x1) << (sizeof(fpType) * 8 - 1);
        val = bitsToFp(fpToBits(val) & bitMask, junk);
        feclearexcept(FeInexact);
        feraiseexcept(FeUnderflow);
    }
    return val;
}

template <class fpType>
static inline fpType
fixDivDest(FPSCR fpscr, fpType val, fpType op1, fpType op2)
{
    fpType mid = fixDest(fpscr, val, op1, op2);
    const bool single = (sizeof(fpType) == sizeof(float));
    const fpType junk = 0.0;
    if ((single && (val == bitsToFp(0x00800000, junk) ||
                    val == bitsToFp(0x80800000, junk))) ||
        (!single && (val == bitsToFp(ULL(0x0010000000000000), junk) ||
                     val == bitsToFp(ULL(0x8010000000000000), junk)))
        ) {
        __asm__ __volatile__("" : "=m" (op1) : "m" (op1));
        fesetround(FeRoundZero);
        fpType temp = 0.0;
        __asm__ __volatile__("" : "=m" (temp) : "m" (temp));
        temp = op1 / op2;
        if (flushToZero(temp)) {
            feraiseexcept(FeUnderflow);
            if (fpscr.fz) {
                feclearexcept(FeInexact);
                mid = temp;
            }
        }
        __asm__ __volatile__("" :: "m" (temp));
    }
    return mid;
}

static inline float
fixFpDFpSDest(FPSCR fpscr, double val)
{
    const float junk = 0.0;
    float op1 = 0.0;
    if (std::isnan(val)) {
        uint64_t valBits = fpToBits(val);
        uint32_t op1Bits = bits(valBits, 50, 29) |
                           (mask(9) << 22) |
                           (bits(valBits, 63) << 31);
        op1 = bitsToFp(op1Bits, junk);
    }
    float mid = fixDest(fpscr, (float)val, op1);
    if (fpscr.fz && fetestexcept(FeUnderflow | FeInexact) ==
                    (FeUnderflow | FeInexact)) {
        feclearexcept(FeInexact);
    }
    if (mid == bitsToFp(0x00800000, junk) ||
        mid == bitsToFp(0x80800000, junk)) {
        __asm__ __volatile__("" : "=m" (val) : "m" (val));
        fesetround(FeRoundZero);
        float temp = 0.0;
        __asm__ __volatile__("" : "=m" (temp) : "m" (temp));
        temp = val;
        if (flushToZero(temp)) {
            feraiseexcept(FeUnderflow);
            if (fpscr.fz) {
                feclearexcept(FeInexact);
                mid = temp;
            }
        }
        __asm__ __volatile__("" :: "m" (temp));
    }
    return mid;
}

static inline double
fixFpSFpDDest(FPSCR fpscr, float val)
{
    const double junk = 0.0;
    double op1 = 0.0;
    if (std::isnan(val)) {
        uint32_t valBits = fpToBits(val);
        uint64_t op1Bits = ((uint64_t)bits(valBits, 21, 0) << 29) |
                           (mask(12) << 51) |
                           ((uint64_t)bits(valBits, 31) << 63);
        op1 = bitsToFp(op1Bits, junk);
    }
    double mid = fixDest(fpscr, (double)val, op1);
    if (mid == bitsToFp(ULL(0x0010000000000000), junk) ||
        mid == bitsToFp(ULL(0x8010000000000000), junk)) {
        __asm__ __volatile__("" : "=m" (val) : "m" (val));
        fesetround(FeRoundZero);
        double temp = 0.0;
        __asm__ __volatile__("" : "=m" (temp) : "m" (temp));
        temp = val;
        if (flushToZero(temp)) {
            feraiseexcept(FeUnderflow);
            if (fpscr.fz) {
                feclearexcept(FeInexact);
                mid = temp;
            }
        }
        __asm__ __volatile__("" :: "m" (temp));
    }
    return mid;
}

static inline double
makeDouble(uint32_t low, uint32_t high)
{
    double junk = 0.0;
    return bitsToFp((uint64_t)low | ((uint64_t)high << 32), junk);
}

static inline uint32_t
lowFromDouble(double val)
{
    return fpToBits(val);
}

static inline uint32_t
highFromDouble(double val)
{
    return fpToBits(val) >> 32;
}

static inline uint64_t
vfpFpSToFixed(float val, bool isSigned, bool half,
              uint8_t imm, bool rzero = true)
{
    int rmode = rzero ? FeRoundZero : fegetround();
    __asm__ __volatile__("" : "=m" (rmode) : "m" (rmode));
    fesetround(FeRoundNearest);
    val = val * powf(2.0, imm);
    __asm__ __volatile__("" : "=m" (val) : "m" (val));
    fesetround(rmode);
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (val) : "m" (val));
    float origVal = val;
    val = rintf(val);
    int fpType = std::fpclassify(val);
    if (fpType == FP_SUBNORMAL || fpType == FP_NAN) {
        if (fpType == FP_NAN) {
            feraiseexcept(FeInvalid);
        }
        val = 0.0;
    } else if (origVal != val) {
        switch (rmode) {
          case FeRoundNearest:
            if (origVal - val > 0.5)
                val += 1.0;
            else if (val - origVal > 0.5)
                val -= 1.0;
            break;
          case FeRoundDown:
            if (origVal < val)
                val -= 1.0;
            break;
          case FeRoundUpward:
            if (origVal > val)
                val += 1.0;
            break;
        }
        feraiseexcept(FeInexact);
    }

    if (isSigned) {
        if (half) {
            if ((double)val < (int16_t)(1 << 15)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int16_t)(1 << 15);
            }
            if ((double)val > (int16_t)mask(15)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int16_t)mask(15);
            }
            return (int16_t)val;
        } else {
            if ((double)val < (int32_t)(1 << 31)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int32_t)(1 << 31);
            }
            if ((double)val > (int32_t)mask(31)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int32_t)mask(31);
            }
            return (int32_t)val;
        }
    } else {
        if (half) {
            if ((double)val < 0) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return 0;
            }
            if ((double)val > (mask(16))) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return mask(16);
            }
            return (uint16_t)val;
        } else {
            if ((double)val < 0) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return 0;
            }
            if ((double)val > (mask(32))) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return mask(32);
            }
            return (uint32_t)val;
        }
    }
}

static inline float
vfpUFixedToFpS(FPSCR fpscr, uint32_t val, bool half, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (half)
        val = (uint16_t)val;
    float scale = powf(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(fpscr, val / scale, (float)val, scale);
}

static inline float
vfpSFixedToFpS(FPSCR fpscr, int32_t val, bool half, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (half)
        val = sext<16>(val & mask(16));
    float scale = powf(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(fpscr, val / scale, (float)val, scale);
}

static inline uint64_t
vfpFpDToFixed(double val, bool isSigned, bool half,
              uint8_t imm, bool rzero = true)
{
    int rmode = rzero ? FeRoundZero : fegetround();
    fesetround(FeRoundNearest);
    val = val * pow(2.0, imm);
    __asm__ __volatile__("" : "=m" (val) : "m" (val));
    fesetround(rmode);
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (val) : "m" (val));
    double origVal = val;
    val = rint(val);
    int fpType = std::fpclassify(val);
    if (fpType == FP_SUBNORMAL || fpType == FP_NAN) {
        if (fpType == FP_NAN) {
            feraiseexcept(FeInvalid);
        }
        val = 0.0;
    } else if (origVal != val) {
        switch (rmode) {
          case FeRoundNearest:
            if (origVal - val > 0.5)
                val += 1.0;
            else if (val - origVal > 0.5)
                val -= 1.0;
            break;
          case FeRoundDown:
            if (origVal < val)
                val -= 1.0;
            break;
          case FeRoundUpward:
            if (origVal > val)
                val += 1.0;
            break;
        }
        feraiseexcept(FeInexact);
    }
    if (isSigned) {
        if (half) {
            if (val < (int16_t)(1 << 15)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int16_t)(1 << 15);
            }
            if (val > (int16_t)mask(15)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int16_t)mask(15);
            }
            return (int16_t)val;
        } else {
            if (val < (int32_t)(1 << 31)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int32_t)(1 << 31);
            }
            if (val > (int32_t)mask(31)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return (int32_t)mask(31);
            }
            return (int32_t)val;
        }
    } else {
        if (half) {
            if (val < 0) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return 0;
            }
            if (val > mask(16)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return mask(16);
            }
            return (uint16_t)val;
        } else {
            if (val < 0) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return 0;
            }
            if (val > mask(32)) {
                feraiseexcept(FeInvalid);
                feclearexcept(FeInexact);
                return mask(32);
            }
            return (uint32_t)val;
        }
    }
}

static inline double
vfpUFixedToFpD(FPSCR fpscr, uint32_t val, bool half, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (half)
        val = (uint16_t)val;
    double scale = pow(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(fpscr, val / scale, (double)val, scale);
}

static inline double
vfpSFixedToFpD(FPSCR fpscr, int32_t val, bool half, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (half)
        val = sext<16>(val & mask(16));
    double scale = pow(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(fpscr, val / scale, (double)val, scale);
}

class VfpMacroOp : public PredMacroOp
{
  public:
    static bool
    inScalarBank(IntRegIndex idx)
    {
        return (idx % 32) < 8;
    }

  protected:
    bool wide;

    VfpMacroOp(const char *mnem, ExtMachInst _machInst,
            OpClass __opClass, bool _wide) :
        PredMacroOp(mnem, _machInst, __opClass), wide(_wide)
    {}

    IntRegIndex
    addStride(IntRegIndex idx, unsigned stride)
    {
        if (wide) {
            stride *= 2;
        }
        unsigned offset = idx % 8;
        idx = (IntRegIndex)(idx - offset);
        offset += stride;
        idx = (IntRegIndex)(idx + (offset % 8));
        return idx;
    }

    void
    nextIdxs(IntRegIndex &dest, IntRegIndex &op1, IntRegIndex &op2)
    {
        unsigned stride = (machInst.fpscrStride == 0) ? 1 : 2;
        assert(!inScalarBank(dest));
        dest = addStride(dest, stride);
        op1 = addStride(op1, stride);
        if (!inScalarBank(op2)) {
            op2 = addStride(op2, stride);
        }
    }

    void
    nextIdxs(IntRegIndex &dest, IntRegIndex &op1)
    {
        unsigned stride = (machInst.fpscrStride == 0) ? 1 : 2;
        assert(!inScalarBank(dest));
        dest = addStride(dest, stride);
        if (!inScalarBank(op1)) {
            op1 = addStride(op1, stride);
        }
    }

    void
    nextIdxs(IntRegIndex &dest)
    {
        unsigned stride = (machInst.fpscrStride == 0) ? 1 : 2;
        assert(!inScalarBank(dest));
        dest = addStride(dest, stride);
    }
};

static inline float
fpAddS(float a, float b)
{
    return a + b;
}

static inline double
fpAddD(double a, double b)
{
    return a + b;
}

static inline float
fpSubS(float a, float b)
{
    return a - b;
}

static inline double
fpSubD(double a, double b)
{
    return a - b;
}

static inline float
fpDivS(float a, float b)
{
    return a / b;
}

static inline double
fpDivD(double a, double b)
{
    return a / b;
}

static inline float
fpMulS(float a, float b)
{
    return a * b;
}

static inline double
fpMulD(double a, double b)
{
    return a * b;
}

class FpOp : public PredOp
{
  protected:
    FpOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass) :
        PredOp(mnem, _machInst, __opClass)
    {}

    virtual float
    doOp(float op1, float op2) const
    {
        panic("Unimplemented version of doOp called.\n");
    }

    virtual float
    doOp(float op1) const
    {
        panic("Unimplemented version of doOp called.\n");
    }

    virtual double
    doOp(double op1, double op2) const
    {
        panic("Unimplemented version of doOp called.\n");
    }

    virtual double
    doOp(double op1) const
    {
        panic("Unimplemented version of doOp called.\n");
    }

    double
    dbl(uint32_t low, uint32_t high) const
    {
        double junk = 0.0;
        return bitsToFp((uint64_t)low | ((uint64_t)high << 32), junk);
    }

    uint32_t
    dblLow(double val) const
    {
        return fpToBits(val);
    }

    uint32_t
    dblHi(double val) const
    {
        return fpToBits(val) >> 32;
    }

    template <class fpType>
    fpType
    binaryOp(FPSCR &fpscr, fpType op1, fpType op2,
            fpType (*func)(fpType, fpType),
            bool flush, uint32_t rMode) const
    {
        const bool single = (sizeof(fpType) == sizeof(float));
        fpType junk = 0.0;

        if (flush && flushToZero(op1, op2))
            fpscr.idc = 1;
        VfpSavedState state = prepFpState(rMode);
        __asm__ __volatile__ ("" : "=m" (op1), "=m" (op2), "=m" (state)
                                 : "m" (op1), "m" (op2), "m" (state));
        fpType dest = func(op1, op2);
        __asm__ __volatile__ ("" : "=m" (dest) : "m" (dest));

        int fpClass = std::fpclassify(dest);
        // Get NAN behavior right. This varies between x86 and ARM.
        if (fpClass == FP_NAN) {
            const bool single = (sizeof(fpType) == sizeof(float));
            const uint64_t qnan =
                single ? 0x7fc00000 : ULL(0x7ff8000000000000);
            const bool nan1 = std::isnan(op1);
            const bool nan2 = std::isnan(op2);
            const bool signal1 = nan1 && ((fpToBits(op1) & qnan) != qnan);
            const bool signal2 = nan2 && ((fpToBits(op2) & qnan) != qnan);
            if ((!nan1 && !nan2) || (fpscr.dn == 1)) {
                dest = bitsToFp(qnan, junk);
            } else if (signal1) {
                dest = bitsToFp(fpToBits(op1) | qnan, junk);
            } else if (signal2) {
                dest = bitsToFp(fpToBits(op2) | qnan, junk);
            } else if (nan1) {
                dest = op1;
            } else if (nan2) {
                dest = op2;
            }
        } else if (flush && flushToZero(dest)) {
            feraiseexcept(FeUnderflow);
        } else if ((
                    (single && (dest == bitsToFp(0x00800000, junk) ||
                         dest == bitsToFp(0x80800000, junk))) ||
                    (!single &&
                        (dest == bitsToFp(ULL(0x0010000000000000), junk) ||
                         dest == bitsToFp(ULL(0x8010000000000000), junk)))
                   ) && rMode != VfpRoundZero) {
            /*
             * Correct for the fact that underflow is detected -before- rounding
             * in ARM and -after- rounding in x86.
             */
            fesetround(FeRoundZero);
            __asm__ __volatile__ ("" : "=m" (op1), "=m" (op2)
                                     : "m" (op1), "m" (op2));
            fpType temp = func(op1, op2);
            __asm__ __volatile__ ("" : "=m" (temp) : "m" (temp));
            if (flush && flushToZero(temp)) {
                dest = temp;
            }
        }
        finishVfp(fpscr, state);
        return dest;
    }

    template <class fpType>
    fpType
    unaryOp(FPSCR &fpscr, fpType op1,
            fpType (*func)(fpType),
            bool flush, uint32_t rMode) const
    {
        const bool single = (sizeof(fpType) == sizeof(float));
        fpType junk = 0.0;

        if (flush && flushToZero(op1))
            fpscr.idc = 1;
        VfpSavedState state = prepFpState(rMode);
        __asm__ __volatile__ ("" : "=m" (op1), "=m" (state)
                                 : "m" (op1), "m" (state));
        fpType dest = func(op1);
        __asm__ __volatile__ ("" : "=m" (dest) : "m" (dest));

        int fpClass = std::fpclassify(dest);
        // Get NAN behavior right. This varies between x86 and ARM.
        if (fpClass == FP_NAN) {
            const bool single = (sizeof(fpType) == sizeof(float));
            const uint64_t qnan =
                single ? 0x7fc00000 : ULL(0x7ff8000000000000);
            const bool nan = std::isnan(op1);
            if (!nan || fpscr.dn == 1) {
                dest = bitsToFp(qnan, junk);
            } else if (nan) {
                dest = bitsToFp(fpToBits(op1) | qnan, junk);
            }
        } else if (flush && flushToZero(dest)) {
            feraiseexcept(FeUnderflow);
        } else if ((
                    (single && (dest == bitsToFp(0x00800000, junk) ||
                         dest == bitsToFp(0x80800000, junk))) ||
                    (!single &&
                        (dest == bitsToFp(ULL(0x0010000000000000), junk) ||
                         dest == bitsToFp(ULL(0x8010000000000000), junk)))
                   ) && rMode != VfpRoundZero) {
            /*
             * Correct for the fact that underflow is detected -before- rounding
             * in ARM and -after- rounding in x86.
             */
            fesetround(FeRoundZero);
            __asm__ __volatile__ ("" : "=m" (op1) : "m" (op1));
            fpType temp = func(op1);
            __asm__ __volatile__ ("" : "=m" (temp) : "m" (temp));
            if (flush && flushToZero(temp)) {
                dest = temp;
            }
        }
        finishVfp(fpscr, state);
        return dest;
    }
};

class FpRegRegOp : public FpOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;

    FpRegRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
               IntRegIndex _dest, IntRegIndex _op1,
               VfpMicroMode mode = VfpNotAMicroop) :
        FpOp(mnem, _machInst, __opClass), dest(_dest), op1(_op1)
    {
        setVfpMicroFlags(mode, flags);
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class FpRegImmOp : public FpOp
{
  protected:
    IntRegIndex dest;
    uint64_t imm;

    FpRegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
               IntRegIndex _dest, uint64_t _imm,
               VfpMicroMode mode = VfpNotAMicroop) :
        FpOp(mnem, _machInst, __opClass), dest(_dest), imm(_imm)
    {
        setVfpMicroFlags(mode, flags);
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class FpRegRegImmOp : public FpOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    uint64_t imm;

    FpRegRegImmOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  IntRegIndex _dest, IntRegIndex _op1,
                  uint64_t _imm, VfpMicroMode mode = VfpNotAMicroop) :
        FpOp(mnem, _machInst, __opClass), dest(_dest), op1(_op1), imm(_imm)
    {
        setVfpMicroFlags(mode, flags);
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class FpRegRegRegOp : public FpOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex op2;

    FpRegRegRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                  IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                  VfpMicroMode mode = VfpNotAMicroop) :
        FpOp(mnem, _machInst, __opClass), dest(_dest), op1(_op1), op2(_op2)
    {
        setVfpMicroFlags(mode, flags);
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

}

#endif //__ARCH_ARM_INSTS_VFP_HH__
