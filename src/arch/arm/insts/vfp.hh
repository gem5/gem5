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
    VfpRoundZero = 3
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
static bool
isSnan(fpType val)
{
    const bool single = (sizeof(fpType) == sizeof(float));
    const uint64_t qnan =
        single ? 0x7fc00000 : ULL(0x7ff8000000000000);
    return std::isnan(val) && ((fpToBits(val) & qnan) != qnan);
}

typedef int VfpSavedState;

VfpSavedState prepFpState(uint32_t rMode);
void finishVfp(FPSCR &fpscr, VfpSavedState state, bool flush);

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
float vcvtFpHFpS(FPSCR &fpscr, bool defaultNan, bool ahp, uint16_t op);

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

uint64_t vfpFpSToFixed(float val, bool isSigned, bool half,
                       uint8_t imm, bool rzero = true);
float vfpUFixedToFpS(bool flush, bool defaultNan,
        uint32_t val, bool half, uint8_t imm);
float vfpSFixedToFpS(bool flush, bool defaultNan,
        int32_t val, bool half, uint8_t imm);

uint64_t vfpFpDToFixed(double val, bool isSigned, bool half,
                       uint8_t imm, bool rzero = true);
double vfpUFixedToFpD(bool flush, bool defaultNan,
        uint32_t val, bool half, uint8_t imm);
double vfpSFixedToFpD(bool flush, bool defaultNan,
        int32_t val, bool half, uint8_t imm);

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

static inline float
fpMaxS(float a, float b)
{
    // Handle comparisons of +0 and -0.
    if (!std::signbit(a) && std::signbit(b))
        return a;
    return fmaxf(a, b);
}

static inline float
fpMinS(float a, float b)
{
    // Handle comparisons of +0 and -0.
    if (std::signbit(a) && !std::signbit(b))
        return a;
    return fminf(a, b);
}

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
    if(fpClassAxB == FP_SUBNORMAL) {
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
    if(fpClassAxB == FP_SUBNORMAL) {
       feraiseexcept(FeUnderflow);
       return 2.0;
    }
    return 2.0 - (a * b);
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
