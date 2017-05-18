/*
 * Copyright (c) 2010-2013 ARM Limited
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

#include <fenv.h>

#include <cmath>

#include "arch/arm/insts/misc.hh"
#include "arch/arm/miscregs.hh"

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
    VfpRoundZero = 3,
    VfpRoundAway = 4
};

static inline float bitsToFp(uint64_t, float);
static inline double bitsToFp(uint64_t, double);
static inline uint32_t fpToBits(float);
static inline uint64_t fpToBits(double);

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

template <class fpType>
static inline bool
isSnan(fpType val)
{
    const bool single = (sizeof(fpType) == sizeof(float));
    const uint64_t qnan =
        single ? 0x7fc00000 : ULL(0x7ff8000000000000);
    return std::isnan(val) && ((fpToBits(val) & qnan) != qnan);
}

typedef int VfpSavedState;

VfpSavedState prepFpState(uint32_t rMode);
void finishVfp(FPSCR &fpscr, VfpSavedState state, bool flush, FPSCR mask = FpscrExcMask);

template <class fpType>
fpType fixDest(FPSCR fpscr, fpType val, fpType op1);

template <class fpType>
fpType fixDest(FPSCR fpscr, fpType val, fpType op1, fpType op2);

template <class fpType>
fpType fixDivDest(FPSCR fpscr, fpType val, fpType op1, fpType op2);

float fixFpDFpSDest(FPSCR fpscr, double val);
double fixFpSFpDDest(FPSCR fpscr, float val);

uint16_t vcvtFpSFpH(FPSCR &fpscr, bool flush, bool defaultNan,
                    uint32_t rMode, bool ahp, float op);
uint16_t vcvtFpDFpH(FPSCR &fpscr, bool flush, bool defaultNan,
                    uint32_t rMode, bool ahp, double op);

float  vcvtFpHFpS(FPSCR &fpscr, bool defaultNan, bool ahp, uint16_t op);
double vcvtFpHFpD(FPSCR &fpscr, bool defaultNan, bool ahp, uint16_t op);

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

static inline void
setFPExceptions(int exceptions) {
    feclearexcept(FeAllExceptions);
    feraiseexcept(exceptions);
}

template <typename T>
uint64_t
vfpFpToFixed(T val, bool isSigned, uint8_t width, uint8_t imm, bool
             useRmode = true, VfpRoundingMode roundMode = VfpRoundZero,
             bool aarch64 = false)
{
    int  rmode;
    bool roundAwayFix = false;

    if (!useRmode) {
        rmode = fegetround();
    } else {
        switch (roundMode)
        {
          case VfpRoundNearest:
            rmode = FeRoundNearest;
            break;
          case VfpRoundUpward:
            rmode = FeRoundUpward;
            break;
          case VfpRoundDown:
            rmode = FeRoundDown;
            break;
          case VfpRoundZero:
            rmode = FeRoundZero;
            break;
          case VfpRoundAway:
            // There is no equivalent rounding mode, use round down and we'll
            // fix it later
            rmode        = FeRoundDown;
            roundAwayFix = true;
            break;
          default:
            panic("Unsupported roundMode %d\n", roundMode);
        }
    }
    __asm__ __volatile__("" : "=m" (rmode) : "m" (rmode));
    fesetround(FeRoundNearest);
    val = val * pow(2.0, imm);
    __asm__ __volatile__("" : "=m" (val) : "m" (val));
    fesetround(rmode);
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (val) : "m" (val));
    T origVal = val;
    val = rint(val);
    __asm__ __volatile__("" : "=m" (val) : "m" (val));

    int exceptions = fetestexcept(FeAllExceptions);

    int fpType = std::fpclassify(val);
    if (fpType == FP_SUBNORMAL || fpType == FP_NAN) {
        if (fpType == FP_NAN) {
            exceptions |= FeInvalid;
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
            if (roundAwayFix) {
                // The ordering on the subtraction looks a bit odd in that we
                // don't do the obvious origVal - val, instead we do
                // -(val - origVal). This is required to get the corruct bit
                // exact behaviour when very close to the 0.5 threshold.
                volatile T error = val;
                error -= origVal;
                error = -error;
                if ( (error >  0.5) ||
                    ((error == 0.5) && (val >= 0)) )
                    val += 1.0;
            } else {
                if (origVal < val)
                    val -= 1.0;
            }
            break;
          case FeRoundUpward:
            if (origVal > val)
                val += 1.0;
            break;
        }
        exceptions |= FeInexact;
    }

    __asm__ __volatile__("" : "=m" (val) : "m" (val));

    if (isSigned) {
        bool     outOfRange = false;
        int64_t  result     = (int64_t) val;
        uint64_t finalVal;

        if (!aarch64) {
            if (width == 16) {
                finalVal = (int16_t)val;
            } else if (width == 32) {
                finalVal =(int32_t)val;
            } else if (width == 64) {
                finalVal = result;
            } else {
                panic("Unsupported width %d\n", width);
            }

            // check if value is in range
            int64_t minVal = ~mask(width-1);
            if ((double)val < minVal) {
                outOfRange = true;
                finalVal = minVal;
            }
            int64_t maxVal = mask(width-1);
            if ((double)val > maxVal) {
                outOfRange = true;
                finalVal = maxVal;
            }
        } else {
            bool isNeg = val < 0;
            finalVal = result & mask(width);
            // If the result is supposed to be less than 64 bits check that the
            // upper bits that got thrown away are just sign extension bits
            if (width != 64) {
                outOfRange = ((uint64_t) result >> (width - 1)) !=
                             (isNeg ? mask(64-width+1) : 0);
            }
            // Check if the original floating point value doesn't matches the
            // integer version we are also out of range. So create a saturated
            // result.
            if (isNeg) {
                outOfRange |= val < result;
                if (outOfRange) {
                    finalVal = 1LL << (width-1);
                }
            } else {
                outOfRange |= val > result;
                if (outOfRange) {
                    finalVal = mask(width-1);
                }
            }
        }

        // Raise an exception if the value was out of range
        if (outOfRange) {
            exceptions |= FeInvalid;
            exceptions &= ~FeInexact;
        }
        setFPExceptions(exceptions);
        return finalVal;
    } else {
        if ((double)val < 0) {
            exceptions |= FeInvalid;
            exceptions &= ~FeInexact;
            setFPExceptions(exceptions);
            return 0;
        }

        uint64_t result = ((uint64_t) val) & mask(width);
        if (val > result) {
            exceptions |= FeInvalid;
            exceptions &= ~FeInexact;
            setFPExceptions(exceptions);
            return mask(width);
        }

        setFPExceptions(exceptions);
        return result;
    }
};


float vfpUFixedToFpS(bool flush, bool defaultNan,
        uint64_t val, uint8_t width, uint8_t imm);
float vfpSFixedToFpS(bool flush, bool defaultNan,
        int64_t val, uint8_t width, uint8_t imm);

double vfpUFixedToFpD(bool flush, bool defaultNan,
        uint64_t val, uint8_t width, uint8_t imm);
double vfpSFixedToFpD(bool flush, bool defaultNan,
        int64_t val, uint8_t width, uint8_t imm);

float fprSqrtEstimate(FPSCR &fpscr, float op);
uint32_t unsignedRSqrtEstimate(uint32_t op);

float fpRecipEstimate(FPSCR &fpscr, float op);
uint32_t unsignedRecipEstimate(uint32_t op);

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

    IntRegIndex addStride(IntRegIndex idx, unsigned stride);
    void nextIdxs(IntRegIndex &dest, IntRegIndex &op1, IntRegIndex &op2);
    void nextIdxs(IntRegIndex &dest, IntRegIndex &op1);
    void nextIdxs(IntRegIndex &dest);
};

template <typename T>
static inline T
fpAdd(T a, T b)
{
    return a + b;
};

template <typename T>
static inline T
fpSub(T a, T b)
{
    return a - b;
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

template <typename T>
static inline T
fpDiv(T a, T b)
{
    return a / b;
};

template <typename T>
static inline T
fpMulX(T a, T b)
{
    uint64_t opData;
    uint32_t sign1;
    uint32_t sign2;
    const bool single = (sizeof(T) == sizeof(float));
    if (single) {
        opData = (fpToBits(a));
        sign1 = opData>>31;
        opData = (fpToBits(b));
        sign2 = opData>>31;
    } else {
        opData = (fpToBits(a));
        sign1 = opData>>63;
        opData = (fpToBits(b));
        sign2 = opData>>63;
    }
    bool inf1 = (std::fpclassify(a) == FP_INFINITE);
    bool inf2 = (std::fpclassify(b) == FP_INFINITE);
    bool zero1 = (std::fpclassify(a) == FP_ZERO);
    bool zero2 = (std::fpclassify(b) == FP_ZERO);
    if ((inf1 && zero2) || (zero1 && inf2)) {
        if (sign1 ^ sign2)
            return (T)(-2.0);
        else
            return (T)(2.0);
    } else {
        return (a * b);
    }
};


template <typename T>
static inline T
fpMul(T a, T b)
{
    return a * b;
};

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

template <typename T>
static inline T
// @todo remove this when all calls to it have been replaced with the new fplib implementation
fpMulAdd(T op1, T op2, T addend)
{
    T result;

    if (sizeof(T) == sizeof(float))
        result = fmaf(op1, op2, addend);
    else
        result = fma(op1, op2, addend);

    // ARM doesn't generate signed nan's from this opperation, so fix up the result
    if (std::isnan(result) && !std::isnan(op1) &&
        !std::isnan(op2) && !std::isnan(addend))
    {
        uint64_t bitMask = ULL(0x1) << ((sizeof(T) * 8) - 1);
        result = bitsToFp(fpToBits(result) & ~bitMask, op1);
    }
    return result;
}

template <typename T>
static inline T
fpRIntX(T a, FPSCR &fpscr)
{
    T rVal;

    rVal = rint(a);
    if (rVal != a && !std::isnan(a))
        fpscr.ixc = 1;
    return (rVal);
};

template <typename T>
static inline T
fpMaxNum(T a, T b)
{
    const bool     single = (sizeof(T) == sizeof(float));
    const uint64_t qnan   = single ? 0x7fc00000 : ULL(0x7ff8000000000000);

    if (std::isnan(a))
        return ((fpToBits(a) & qnan) == qnan) ? b : a;
    if (std::isnan(b))
        return ((fpToBits(b) & qnan) == qnan) ? a : b;
    // Handle comparisons of +0 and -0.
    if (!std::signbit(a) && std::signbit(b))
        return a;
    return fmax(a, b);
};

template <typename T>
static inline T
fpMax(T a, T b)
{
    if (std::isnan(a))
        return a;
    if (std::isnan(b))
        return b;
    return fpMaxNum<T>(a, b);
};

template <typename T>
static inline T
fpMinNum(T a, T b)
{
    const bool     single = (sizeof(T) == sizeof(float));
    const uint64_t qnan   = single ? 0x7fc00000 : ULL(0x7ff8000000000000);

    if (std::isnan(a))
        return ((fpToBits(a) & qnan) == qnan) ? b : a;
    if (std::isnan(b))
        return ((fpToBits(b) & qnan) == qnan) ? a : b;
    // Handle comparisons of +0 and -0.
    if (std::signbit(a) && !std::signbit(b))
        return a;
    return fmin(a, b);
};

template <typename T>
static inline T
fpMin(T a, T b)
{
    if (std::isnan(a))
        return a;
    if (std::isnan(b))
        return b;
    return fpMinNum<T>(a, b);
};

template <typename T>
static inline T
fpRSqrts(T a, T b)
{
    int fpClassA = std::fpclassify(a);
    int fpClassB = std::fpclassify(b);
    T aXb;
    int fpClassAxB;

    if ((fpClassA == FP_ZERO && fpClassB == FP_INFINITE) ||
        (fpClassA == FP_INFINITE && fpClassB == FP_ZERO)) {
        return 1.5;
    }
    aXb = a*b;
    fpClassAxB = std::fpclassify(aXb);
    if (fpClassAxB == FP_SUBNORMAL) {
       feraiseexcept(FeUnderflow);
       return 1.5;
    }
    return (3.0 - (a * b)) / 2.0;
};

template <typename T>
static inline T
fpRecps(T a, T b)
{
    int fpClassA = std::fpclassify(a);
    int fpClassB = std::fpclassify(b);
    T aXb;
    int fpClassAxB;

    if ((fpClassA == FP_ZERO && fpClassB == FP_INFINITE) ||
        (fpClassA == FP_INFINITE && fpClassB == FP_ZERO)) {
        return 2.0;
    }
    aXb = a*b;
    fpClassAxB = std::fpclassify(aXb);
    if (fpClassAxB == FP_SUBNORMAL) {
       feraiseexcept(FeUnderflow);
       return 2.0;
    }
    return 2.0 - (a * b);
};


static inline float
fpRSqrtsS(float a, float b)
{
    int fpClassA = std::fpclassify(a);
    int fpClassB = std::fpclassify(b);
    float aXb;
    int fpClassAxB;

    if ((fpClassA == FP_ZERO && fpClassB == FP_INFINITE) ||
        (fpClassA == FP_INFINITE && fpClassB == FP_ZERO)) {
        return 1.5;
    }
    aXb = a*b;
    fpClassAxB = std::fpclassify(aXb);
    if (fpClassAxB == FP_SUBNORMAL) {
       feraiseexcept(FeUnderflow);
       return 1.5;
    }
    return (3.0 - (a * b)) / 2.0;
}

static inline float
fpRecpsS(float a, float b)
{
    int fpClassA = std::fpclassify(a);
    int fpClassB = std::fpclassify(b);
    float aXb;
    int fpClassAxB;

    if ((fpClassA == FP_ZERO && fpClassB == FP_INFINITE) ||
        (fpClassA == FP_INFINITE && fpClassB == FP_ZERO)) {
        return 2.0;
    }
    aXb = a*b;
    fpClassAxB = std::fpclassify(aXb);
    if (fpClassAxB == FP_SUBNORMAL) {
       feraiseexcept(FeUnderflow);
       return 2.0;
    }
    return 2.0 - (a * b);
}

template <typename T>
static inline T
roundNEven(T a) {
    T val;

    val = round(a);
    if (a - val == 0.5) {
        if ( (((int) a) & 1) == 0 ) val += 1.0;
    }
    else if (a - val == -0.5) {
        if ( (((int) a) & 1) == 0 ) val -= 1.0;
    }
    return val;
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
    processNans(FPSCR &fpscr, bool &done, bool defaultNan,
                fpType op1, fpType op2) const;

    template <class fpType>
    fpType
    ternaryOp(FPSCR &fpscr, fpType op1, fpType op2, fpType op3,
              fpType (*func)(fpType, fpType, fpType),
              bool flush, bool defaultNan, uint32_t rMode) const;

    template <class fpType>
    fpType
    binaryOp(FPSCR &fpscr, fpType op1, fpType op2,
            fpType (*func)(fpType, fpType),
            bool flush, bool defaultNan, uint32_t rMode) const;

    template <class fpType>
    fpType
    unaryOp(FPSCR &fpscr, fpType op1,
            fpType (*func)(fpType),
            bool flush, uint32_t rMode) const;

    void
    advancePC(PCState &pcState) const
    {
        if (flags[IsLastMicroop]) {
            pcState.uEnd();
        } else if (flags[IsMicroop]) {
            pcState.uAdvance();
        } else {
            pcState.advance();
        }
    }

    float
    fpSqrt (FPSCR fpscr,float x) const
    {

        return unaryOp(fpscr,x,sqrtf,fpscr.fz,fpscr.rMode);

    }

    double
    fpSqrt (FPSCR fpscr,double x) const
    {

        return unaryOp(fpscr,x,sqrt,fpscr.fz,fpscr.rMode);

    }
};

class FpCondCompRegOp : public FpOp
{
  protected:
    IntRegIndex op1, op2;
    ConditionCode condCode;
    uint8_t defCc;

    FpCondCompRegOp(const char *mnem, ExtMachInst _machInst,
                       OpClass __opClass, IntRegIndex _op1, IntRegIndex _op2,
                       ConditionCode _condCode, uint8_t _defCc) :
        FpOp(mnem, _machInst, __opClass),
        op1(_op1), op2(_op2), condCode(_condCode), defCc(_defCc)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class FpCondSelOp : public FpOp
{
  protected:
    IntRegIndex dest, op1, op2;
    ConditionCode condCode;

    FpCondSelOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                ConditionCode _condCode) :
        FpOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), condCode(_condCode)
    {}

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
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

class FpRegRegRegCondOp : public FpOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex op2;
    ConditionCode cond;

    FpRegRegRegCondOp(const char *mnem, ExtMachInst _machInst,
                      OpClass __opClass, IntRegIndex _dest, IntRegIndex _op1,
                      IntRegIndex _op2, ConditionCode _cond,
                      VfpMicroMode mode = VfpNotAMicroop) :
        FpOp(mnem, _machInst, __opClass), dest(_dest), op1(_op1), op2(_op2),
        cond(_cond)
    {
        setVfpMicroFlags(mode, flags);
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class FpRegRegRegRegOp : public FpOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex op2;
    IntRegIndex op3;

    FpRegRegRegRegOp(const char *mnem, ExtMachInst _machInst, OpClass __opClass,
                     IntRegIndex _dest, IntRegIndex _op1, IntRegIndex _op2,
                     IntRegIndex _op3, VfpMicroMode mode = VfpNotAMicroop) :
        FpOp(mnem, _machInst, __opClass), dest(_dest), op1(_op1), op2(_op2),
        op3(_op3)
    {
        setVfpMicroFlags(mode, flags);
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

class FpRegRegRegImmOp : public FpOp
{
  protected:
    IntRegIndex dest;
    IntRegIndex op1;
    IntRegIndex op2;
    uint64_t imm;

    FpRegRegRegImmOp(const char *mnem, ExtMachInst _machInst,
                     OpClass __opClass, IntRegIndex _dest,
                     IntRegIndex _op1, IntRegIndex _op2,
                     uint64_t _imm, VfpMicroMode mode = VfpNotAMicroop) :
        FpOp(mnem, _machInst, __opClass),
        dest(_dest), op1(_op1), op2(_op2), imm(_imm)
    {
        setVfpMicroFlags(mode, flags);
    }

    std::string generateDisassembly(Addr pc, const SymbolTable *symtab) const;
};

}

#endif //__ARCH_ARM_INSTS_VFP_HH__
