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

#include "arch/arm/insts/vfp.hh"

/*
 * The asm statements below are to keep gcc from reordering code. Otherwise
 * the rounding mode might be set after the operation it was intended for, the
 * exception bits read before it, etc.
 */

std::string
FpCondCompRegOp::generateDisassembly(
        Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printReg(ss, op1);
    ccprintf(ss, ", ");
    printReg(ss, op2);
    ccprintf(ss, ", #%d", defCc);
    ccprintf(ss, ", ");
    printCondition(ss, condCode, true);
    return ss.str();
}

std::string
FpCondSelOp::generateDisassembly(
        Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss, "", false);
    printReg(ss, dest);
    ccprintf(ss, ", ");
    printReg(ss, op1);
    ccprintf(ss, ", ");
    printReg(ss, op2);
    ccprintf(ss, ", ");
    printCondition(ss, condCode, true);
    return ss.str();
}

std::string
FpRegRegOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op1 + FP_Reg_Base);
    return ss.str();
}

std::string
FpRegImmOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Reg_Base);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
FpRegRegImmOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op1 + FP_Reg_Base);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
FpRegRegRegOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op1 + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op2 + FP_Reg_Base);
    return ss.str();
}

std::string
FpRegRegRegRegOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op1 + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op2 + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op3 + FP_Reg_Base);
    return ss.str();
}

std::string
FpRegRegRegImmOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op1 + FP_Reg_Base);
    ss << ", ";
    printReg(ss, op2 + FP_Reg_Base);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

namespace ArmISA
{

VfpSavedState
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

void
finishVfp(FPSCR &fpscr, VfpSavedState state, bool flush, FPSCR mask)
{
    int exceptions = fetestexcept(FeAllExceptions);
    bool underflow = false;
    if ((exceptions & FeInvalid) && mask.ioc) {
        fpscr.ioc = 1;
    }
    if ((exceptions & FeDivByZero) && mask.dzc) {
        fpscr.dzc = 1;
    }
    if ((exceptions & FeOverflow) && mask.ofc) {
        fpscr.ofc = 1;
    }
    if (exceptions & FeUnderflow) {
        underflow = true;
        if (mask.ufc)
            fpscr.ufc = 1;
    }
    if ((exceptions & FeInexact) && !(underflow && flush) && mask.ixc) {
        fpscr.ixc = 1;
    }
    fesetround(state);
}

template <class fpType>
fpType
fixDest(bool flush, bool defaultNan, fpType val, fpType op1)
{
    int fpClass = std::fpclassify(val);
    fpType junk = 0.0;
    if (fpClass == FP_NAN) {
        const bool single = (sizeof(val) == sizeof(float));
        const uint64_t qnan = single ? 0x7fc00000 : ULL(0x7ff8000000000000);
        const bool nan = std::isnan(op1);
        if (!nan || defaultNan) {
            val = bitsToFp(qnan, junk);
        } else if (nan) {
            val = bitsToFp(fpToBits(op1) | qnan, junk);
        }
    } else if (fpClass == FP_SUBNORMAL && flush == 1) {
        // Turn val into a zero with the correct sign;
        uint64_t bitMask = ULL(0x1) << (sizeof(fpType) * 8 - 1);
        val = bitsToFp(fpToBits(val) & bitMask, junk);
        feclearexcept(FeInexact);
        feraiseexcept(FeUnderflow);
    }
    return val;
}

template
float fixDest<float>(bool flush, bool defaultNan, float val, float op1);
template
double fixDest<double>(bool flush, bool defaultNan, double val, double op1);

template <class fpType>
fpType
fixDest(bool flush, bool defaultNan, fpType val, fpType op1, fpType op2)
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
        if ((!nan1 && !nan2) || defaultNan) {
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
    } else if (fpClass == FP_SUBNORMAL && flush) {
        // Turn val into a zero with the correct sign;
        uint64_t bitMask = ULL(0x1) << (sizeof(fpType) * 8 - 1);
        val = bitsToFp(fpToBits(val) & bitMask, junk);
        feclearexcept(FeInexact);
        feraiseexcept(FeUnderflow);
    }
    return val;
}

template
float fixDest<float>(bool flush, bool defaultNan,
                     float val, float op1, float op2);
template
double fixDest<double>(bool flush, bool defaultNan,
                       double val, double op1, double op2);

template <class fpType>
fpType
fixDivDest(bool flush, bool defaultNan, fpType val, fpType op1, fpType op2)
{
    fpType mid = fixDest(flush, defaultNan, val, op1, op2);
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
            if (flush) {
                feclearexcept(FeInexact);
                mid = temp;
            }
        }
        __asm__ __volatile__("" :: "m" (temp));
    }
    return mid;
}

template
float fixDivDest<float>(bool flush, bool defaultNan,
                        float val, float op1, float op2);
template
double fixDivDest<double>(bool flush, bool defaultNan,
                          double val, double op1, double op2);

float
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
    float mid = fixDest(fpscr.fz, fpscr.dn, (float)val, op1);
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

double
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
    double mid = fixDest(fpscr.fz, fpscr.dn, (double)val, op1);
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

static inline uint16_t
vcvtFpFpH(FPSCR &fpscr, bool flush, bool defaultNan,
          uint32_t rMode, bool ahp, uint64_t opBits, bool isDouble)
{
    uint32_t mWidth;
    uint32_t eWidth;
    uint32_t eHalfRange;
    uint32_t sBitPos;

    if (isDouble) {
        mWidth = 52;
        eWidth = 11;
    } else {
        mWidth = 23;
        eWidth = 8;
    }
    sBitPos    = eWidth + mWidth;
    eHalfRange = (1 << (eWidth-1)) - 1;

    // Extract the operand.
    bool neg = bits(opBits, sBitPos);
    uint32_t exponent = bits(opBits, sBitPos-1, mWidth);
    uint64_t oldMantissa = bits(opBits, mWidth-1, 0);
    uint32_t mantissa = oldMantissa >> (mWidth - 10);
    // Do the conversion.
    uint64_t extra = oldMantissa & mask(mWidth - 10);
    if (exponent == mask(eWidth)) {
        if (oldMantissa != 0) {
            // Nans.
            if (bits(mantissa, 9) == 0) {
                // Signalling nan.
                fpscr.ioc = 1;
            }
            if (ahp) {
                mantissa = 0;
                exponent = 0;
                fpscr.ioc = 1;
            } else if (defaultNan) {
                mantissa = (1 << 9);
                exponent = 0x1f;
                neg = false;
            } else {
                exponent = 0x1f;
                mantissa |= (1 << 9);
            }
        } else {
            // Infinities.
            exponent = 0x1F;
            if (ahp) {
                fpscr.ioc = 1;
                mantissa = 0x3ff;
            } else {
                mantissa = 0;
            }
        }
    } else if (exponent == 0 && oldMantissa == 0) {
        // Zero, don't need to do anything.
    } else {
        // Normalized or denormalized numbers.

        bool inexact = (extra != 0);

        if (exponent == 0) {
            // Denormalized.
            // If flush to zero is on, this shouldn't happen.
            assert(!flush);

            // Check for underflow
            if (inexact || fpscr.ufe)
                fpscr.ufc = 1;

            // Handle rounding.
            unsigned mode = rMode;
            if ((mode == VfpRoundUpward && !neg && extra) ||
                (mode == VfpRoundDown && neg && extra) ||
                (mode == VfpRoundNearest &&
                 (extra > (1 << 9) ||
                  (extra == (1 << 9) && bits(mantissa, 0))))) {
                mantissa++;
            }

            // See if the number became normalized after rounding.
            if (mantissa == (1 << 10)) {
                mantissa = 0;
                exponent = 1;
            }
        } else {
            // Normalized.

            // We need to track the dropped bits differently since
            // more can be dropped by denormalizing.
            bool topOne = bits(extra, mWidth - 10 - 1);
            bool restZeros = bits(extra, mWidth - 10 - 2, 0) == 0;

            if (exponent <= (eHalfRange - 15)) {
                // The result is too small. Denormalize.
                mantissa |= (1 << 10);
                while (mantissa && exponent <= (eHalfRange - 15)) {
                    restZeros = restZeros && !topOne;
                    topOne = bits(mantissa, 0);
                    mantissa = mantissa >> 1;
                    exponent++;
                }
                if (topOne || !restZeros)
                    inexact = true;
                exponent = 0;
            } else {
                // Change bias.
                exponent -= (eHalfRange - 15);
            }

            if (exponent == 0 && (inexact || fpscr.ufe)) {
                // Underflow
                fpscr.ufc = 1;
            }

            // Handle rounding.
            unsigned mode = rMode;
            bool nonZero = topOne || !restZeros;
            if ((mode == VfpRoundUpward && !neg && nonZero) ||
                (mode == VfpRoundDown && neg && nonZero) ||
                (mode == VfpRoundNearest && topOne &&
                 (!restZeros || bits(mantissa, 0)))) {
                mantissa++;
            }

            // See if we rounded up and need to bump the exponent.
            if (mantissa == (1 << 10)) {
                mantissa = 0;
                exponent++;
            }

            // Deal with overflow
            if (ahp) {
                if (exponent >= 0x20) {
                    exponent = 0x1f;
                    mantissa = 0x3ff;
                    fpscr.ioc = 1;
                    // Supress inexact exception.
                    inexact = false;
                }
            } else {
                if (exponent >= 0x1f) {
                    if ((mode == VfpRoundNearest) ||
                        (mode == VfpRoundUpward && !neg) ||
                        (mode == VfpRoundDown && neg)) {
                        // Overflow to infinity.
                        exponent = 0x1f;
                        mantissa = 0;
                    } else {
                        // Overflow to max normal.
                        exponent = 0x1e;
                        mantissa = 0x3ff;
                    }
                    fpscr.ofc = 1;
                    inexact = true;
                }
            }
        }

        if (inexact) {
            fpscr.ixc = 1;
        }
    }
    // Reassemble and install the result.
    uint32_t result = bits(mantissa, 9, 0);
    replaceBits(result, 14, 10, exponent);
    if (neg)
        result |= (1 << 15);
    return result;
}

uint16_t
vcvtFpSFpH(FPSCR &fpscr, bool flush, bool defaultNan,
           uint32_t rMode, bool ahp, float op)
{
    uint64_t opBits = fpToBits(op);
    return vcvtFpFpH(fpscr, flush, defaultNan, rMode, ahp, opBits, false);
}

uint16_t
vcvtFpDFpH(FPSCR &fpscr, bool flush, bool defaultNan,
           uint32_t rMode, bool ahp, double op)
{
    uint64_t opBits = fpToBits(op);
    return vcvtFpFpH(fpscr, flush, defaultNan, rMode, ahp, opBits, true);
}

static inline uint64_t
vcvtFpHFp(FPSCR &fpscr, bool defaultNan, bool ahp, uint16_t op, bool isDouble)
{
    uint32_t mWidth;
    uint32_t eWidth;
    uint32_t eHalfRange;
    uint32_t sBitPos;

    if (isDouble) {
        mWidth = 52;
        eWidth = 11;
    } else {
        mWidth = 23;
        eWidth = 8;
    }
    sBitPos    = eWidth + mWidth;
    eHalfRange = (1 << (eWidth-1)) - 1;

    // Extract the bitfields.
    bool neg = bits(op, 15);
    uint32_t exponent = bits(op, 14, 10);
    uint64_t mantissa = bits(op, 9, 0);
    // Do the conversion.
    if (exponent == 0) {
        if (mantissa != 0) {
            // Normalize the value.
            exponent = exponent + (eHalfRange - 15) + 1;
            while (mantissa < (1 << 10)) {
                mantissa = mantissa << 1;
                exponent--;
            }
        }
        mantissa = mantissa << (mWidth - 10);
    } else if (exponent == 0x1f && !ahp) {
        // Infinities and nans.
        exponent = mask(eWidth);
        if (mantissa != 0) {
            // Nans.
            mantissa = mantissa << (mWidth - 10);
            if (bits(mantissa, mWidth-1) == 0) {
                // Signalling nan.
                fpscr.ioc = 1;
                mantissa |= (((uint64_t) 1) << (mWidth-1));
            }
            if (defaultNan) {
                mantissa &= ~mask(mWidth-1);
                neg = false;
            }
        }
    } else {
        exponent = exponent + (eHalfRange - 15);
        mantissa = mantissa << (mWidth - 10);
    }
    // Reassemble the result.
    uint64_t result = bits(mantissa, mWidth-1, 0);
    replaceBits(result, sBitPos-1, mWidth, exponent);
    if (neg) {
        result |= (((uint64_t) 1) << sBitPos);
    }
    return result;
}

double
vcvtFpHFpD(FPSCR &fpscr, bool defaultNan, bool ahp, uint16_t op)
{
    double junk = 0.0;
    uint64_t result;

    result = vcvtFpHFp(fpscr, defaultNan, ahp, op, true);
    return bitsToFp(result, junk);
}

float
vcvtFpHFpS(FPSCR &fpscr, bool defaultNan, bool ahp, uint16_t op)
{
    float junk = 0.0;
    uint64_t result;

    result = vcvtFpHFp(fpscr, defaultNan, ahp, op, false);
    return bitsToFp(result, junk);
}

float
vfpUFixedToFpS(bool flush, bool defaultNan,
        uint64_t val, uint8_t width, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (width == 16)
        val = (uint16_t)val;
    else if (width == 32)
        val = (uint32_t)val;
    else if (width != 64)
        panic("Unsupported width %d", width);
    float scale = powf(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(flush, defaultNan, val / scale, (float)val, scale);
}

float
vfpSFixedToFpS(bool flush, bool defaultNan,
        int64_t val, uint8_t width, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (width == 16)
        val = sext<16>(val & mask(16));
    else if (width == 32)
        val = sext<32>(val & mask(32));
    else if (width != 64)
        panic("Unsupported width %d", width);

    float scale = powf(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(flush, defaultNan, val / scale, (float)val, scale);
}


double
vfpUFixedToFpD(bool flush, bool defaultNan,
        uint64_t val, uint8_t width, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (width == 16)
        val = (uint16_t)val;
    else if (width == 32)
        val = (uint32_t)val;
    else if (width != 64)
        panic("Unsupported width %d", width);

    double scale = pow(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(flush, defaultNan, val / scale, (double)val, scale);
}

double
vfpSFixedToFpD(bool flush, bool defaultNan,
        int64_t val, uint8_t width, uint8_t imm)
{
    fesetround(FeRoundNearest);
    if (width == 16)
        val = sext<16>(val & mask(16));
    else if (width == 32)
        val = sext<32>(val & mask(32));
    else if (width != 64)
        panic("Unsupported width %d", width);

    double scale = pow(2.0, imm);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    feclearexcept(FeAllExceptions);
    __asm__ __volatile__("" : "=m" (scale) : "m" (scale));
    return fixDivDest(flush, defaultNan, val / scale, (double)val, scale);
}

// This function implements a magic formula taken from the architecture
// reference manual. It was originally called recip_sqrt_estimate.
static double
recipSqrtEstimate(double a)
{
    int64_t q0, q1, s;
    double r;
    if (a < 0.5) {
        q0 = (int64_t)(a * 512.0);
        r = 1.0 / sqrt(((double)q0 + 0.5) / 512.0);
    } else {
        q1 = (int64_t)(a * 256.0);
        r = 1.0 / sqrt(((double)q1 + 0.5) / 256.0);
    }
    s = (int64_t)(256.0 * r + 0.5);
    return (double)s / 256.0;
}

// This function is only intended for use in Neon instructions because
// it ignores certain bits in the FPSCR.
float
fprSqrtEstimate(FPSCR &fpscr, float op)
{
    const uint32_t qnan = 0x7fc00000;
    float junk = 0.0;
    int fpClass = std::fpclassify(op);
    if (fpClass == FP_NAN) {
        if ((fpToBits(op) & qnan) != qnan)
            fpscr.ioc = 1;
        return bitsToFp(qnan, junk);
    } else if (fpClass == FP_ZERO) {
        fpscr.dzc = 1;
        // Return infinity with the same sign as the operand.
        return bitsToFp((std::signbit(op) << 31) |
                       (0xFF << 23) | (0 << 0), junk);
    } else if (std::signbit(op)) {
        // Set invalid op bit.
        fpscr.ioc = 1;
        return bitsToFp(qnan, junk);
    } else if (fpClass == FP_INFINITE) {
        return 0.0;
    } else {
        uint64_t opBits = fpToBits(op);
        double scaled;
        if (bits(opBits, 23)) {
            scaled = bitsToFp((0 << 0) | (bits(opBits, 22, 0) << 29) |
                              (ULL(0x3fd) << 52) | (bits(opBits, 31) << 63),
                              (double)0.0);
        } else {
            scaled = bitsToFp((0 << 0) | (bits(opBits, 22, 0) << 29) |
                              (ULL(0x3fe) << 52) | (bits(opBits, 31) << 63),
                              (double)0.0);
        }
        uint64_t resultExp = (380 - bits(opBits, 30, 23)) / 2;

        uint64_t estimate = fpToBits(recipSqrtEstimate(scaled));

        return bitsToFp((bits(estimate, 63) << 31) |
                        (bits(resultExp, 7, 0) << 23) |
                        (bits(estimate, 51, 29) << 0), junk);
    }
}

uint32_t
unsignedRSqrtEstimate(uint32_t op)
{
    if (bits(op, 31, 30) == 0) {
        return -1;
    } else {
        double dpOp;
        if (bits(op, 31)) {
            dpOp = bitsToFp((ULL(0) << 63) |
                            (ULL(0x3fe) << 52) |
                            (bits((uint64_t)op, 30, 0) << 21) |
                            (0 << 0), (double)0.0);
        } else {
            dpOp = bitsToFp((ULL(0) << 63) |
                            (ULL(0x3fd) << 52) |
                            (bits((uint64_t)op, 29, 0) << 22) |
                            (0 << 0), (double)0.0);
        }
        uint64_t estimate = fpToBits(recipSqrtEstimate(dpOp));
        return (1 << 31) | bits(estimate, 51, 21);
    }
}

// This function implements a magic formula taken from the architecture
// reference manual. It was originally called recip_estimate.

static double
recipEstimate(double a)
{
    int64_t q, s;
    double r;
    q = (int64_t)(a * 512.0);
    r = 1.0 / (((double)q + 0.5) / 512.0);
    s = (int64_t)(256.0 * r + 0.5);
    return (double)s / 256.0;
}

// This function is only intended for use in Neon instructions because
// it ignores certain bits in the FPSCR.
float
fpRecipEstimate(FPSCR &fpscr, float op)
{
    const uint32_t qnan = 0x7fc00000;
    float junk = 0.0;
    int fpClass = std::fpclassify(op);
    if (fpClass == FP_NAN) {
        if ((fpToBits(op) & qnan) != qnan)
            fpscr.ioc = 1;
        return bitsToFp(qnan, junk);
    } else if (fpClass == FP_INFINITE) {
        return bitsToFp(std::signbit(op) << 31, junk);
    } else if (fpClass == FP_ZERO) {
        fpscr.dzc = 1;
        // Return infinity with the same sign as the operand.
        return bitsToFp((std::signbit(op) << 31) |
                       (0xFF << 23) | (0 << 0), junk);
    } else if (fabs(op) >= pow(2.0, 126)) {
        fpscr.ufc = 1;
        return bitsToFp(std::signbit(op) << 31, junk);
    } else {
        uint64_t opBits = fpToBits(op);
        double scaled;
        scaled = bitsToFp((0 << 0) | (bits(opBits, 22, 0) << 29) |
                          (ULL(0x3fe) << 52) | (ULL(0) << 63),
                          (double)0.0);
        uint64_t resultExp = 253 - bits(opBits, 30, 23);

        uint64_t estimate = fpToBits(recipEstimate(scaled));

        return bitsToFp((bits(opBits, 31) << 31) |
                        (bits(resultExp, 7, 0) << 23) |
                        (bits(estimate, 51, 29) << 0), junk);
    }
}

uint32_t
unsignedRecipEstimate(uint32_t op)
{
    if (bits(op, 31) == 0) {
        return -1;
    } else {
        double dpOp;
        dpOp = bitsToFp((ULL(0) << 63) |
                        (ULL(0x3fe) << 52) |
                        (bits((uint64_t)op, 30, 0) << 21) |
                        (0 << 0), (double)0.0);
        uint64_t estimate = fpToBits(recipEstimate(dpOp));
        return (1 << 31) | bits(estimate, 51, 21);
    }
}

template <class fpType>
fpType
FpOp::processNans(FPSCR &fpscr, bool &done, bool defaultNan,
                  fpType op1, fpType op2) const
{
    done = true;
    fpType junk = 0.0;
    fpType dest = 0.0;
    const bool single = (sizeof(fpType) == sizeof(float));
    const uint64_t qnan =
        single ? 0x7fc00000 : ULL(0x7ff8000000000000);
    const bool nan1 = std::isnan(op1);
    const bool nan2 = std::isnan(op2);
    const bool signal1 = nan1 && ((fpToBits(op1) & qnan) != qnan);
    const bool signal2 = nan2 && ((fpToBits(op2) & qnan) != qnan);
    if (nan1 || nan2) {
        if (defaultNan) {
            dest = bitsToFp(qnan, junk);
        }  else if (signal1) {
            dest = bitsToFp(fpToBits(op1) | qnan, junk);
        } else if (signal2) {
            dest = bitsToFp(fpToBits(op2) | qnan, junk);
        } else if (nan1) {
            dest = op1;
        } else if (nan2) {
            dest = op2;
        }
        if (signal1 || signal2) {
            fpscr.ioc = 1;
        }
    } else {
        done = false;
    }
    return dest;
}

template
float FpOp::processNans(FPSCR &fpscr, bool &done, bool defaultNan,
                        float op1, float op2) const;
template
double FpOp::processNans(FPSCR &fpscr, bool &done, bool defaultNan,
                         double op1, double op2) const;

// @TODO remove this function when we've finished switching all FMA code to use the new FPLIB
template <class fpType>
fpType
FpOp::ternaryOp(FPSCR &fpscr, fpType op1, fpType op2, fpType op3,
                fpType (*func)(fpType, fpType, fpType),
                bool flush, bool defaultNan, uint32_t rMode) const
{
    const bool single = (sizeof(fpType) == sizeof(float));
    fpType junk = 0.0;

    if (flush && (flushToZero(op1, op2) || flushToZero(op3)))
        fpscr.idc = 1;
    VfpSavedState state = prepFpState(rMode);
    __asm__ __volatile__ ("" : "=m" (op1), "=m" (op2), "=m" (op3), "=m" (state)
                             :  "m" (op1),  "m" (op2),  "m" (op3),  "m" (state));
    fpType dest = func(op1, op2, op3);
    __asm__ __volatile__ ("" : "=m" (dest) : "m" (dest));

    int fpClass = std::fpclassify(dest);
    // Get NAN behavior right. This varies between x86 and ARM.
    if (fpClass == FP_NAN) {
        const uint64_t qnan =
            single ? 0x7fc00000 : ULL(0x7ff8000000000000);
        const bool nan1 = std::isnan(op1);
        const bool nan2 = std::isnan(op2);
        const bool nan3 = std::isnan(op3);
        const bool signal1 = nan1 && ((fpToBits(op1) & qnan) != qnan);
        const bool signal2 = nan2 && ((fpToBits(op2) & qnan) != qnan);
        const bool signal3 = nan3 && ((fpToBits(op3) & qnan) != qnan);
        if ((!nan1 && !nan2 && !nan3) || (defaultNan == 1)) {
            dest = bitsToFp(qnan, junk);
        } else if (signal1) {
            dest = bitsToFp(fpToBits(op1) | qnan, junk);
        } else if (signal2) {
            dest = bitsToFp(fpToBits(op2) | qnan, junk);
        } else if (signal3) {
            dest = bitsToFp(fpToBits(op3) | qnan, junk);
        } else if (nan1) {
            dest = op1;
        } else if (nan2) {
            dest = op2;
        } else if (nan3) {
            dest = op3;
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
        __asm__ __volatile__ ("" : "=m" (op1), "=m" (op2), "=m" (op3)
                                 :  "m" (op1),  "m" (op2),  "m" (op3));
        fpType temp = func(op1, op2, op2);
        __asm__ __volatile__ ("" : "=m" (temp) : "m" (temp));
        if (flush && flushToZero(temp)) {
            dest = temp;
        }
    }
    finishVfp(fpscr, state, flush);
    return dest;
}

template
float FpOp::ternaryOp(FPSCR &fpscr, float op1, float op2, float op3,
                      float (*func)(float, float, float),
                      bool flush, bool defaultNan, uint32_t rMode) const;
template
double FpOp::ternaryOp(FPSCR &fpscr, double op1, double op2, double op3,
                       double (*func)(double, double, double),
                       bool flush, bool defaultNan, uint32_t rMode) const;

template <class fpType>
fpType
FpOp::binaryOp(FPSCR &fpscr, fpType op1, fpType op2,
               fpType (*func)(fpType, fpType),
               bool flush, bool defaultNan, uint32_t rMode) const
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

    // Get NAN behavior right. This varies between x86 and ARM.
    if (std::isnan(dest)) {
        const uint64_t qnan =
            single ? 0x7fc00000 : ULL(0x7ff8000000000000);
        const bool nan1 = std::isnan(op1);
        const bool nan2 = std::isnan(op2);
        const bool signal1 = nan1 && ((fpToBits(op1) & qnan) != qnan);
        const bool signal2 = nan2 && ((fpToBits(op2) & qnan) != qnan);
        if ((!nan1 && !nan2) || (defaultNan == 1)) {
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
    finishVfp(fpscr, state, flush);
    return dest;
}

template
float FpOp::binaryOp(FPSCR &fpscr, float op1, float op2,
                     float (*func)(float, float),
                     bool flush, bool defaultNan, uint32_t rMode) const;
template
double FpOp::binaryOp(FPSCR &fpscr, double op1, double op2,
                      double (*func)(double, double),
                      bool flush, bool defaultNan, uint32_t rMode) const;

template <class fpType>
fpType
FpOp::unaryOp(FPSCR &fpscr, fpType op1, fpType (*func)(fpType),
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

    // Get NAN behavior right. This varies between x86 and ARM.
    if (std::isnan(dest)) {
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
    finishVfp(fpscr, state, flush);
    return dest;
}

template
float FpOp::unaryOp(FPSCR &fpscr, float op1, float (*func)(float),
                    bool flush, uint32_t rMode) const;
template
double FpOp::unaryOp(FPSCR &fpscr, double op1, double (*func)(double),
                     bool flush, uint32_t rMode) const;

IntRegIndex
VfpMacroOp::addStride(IntRegIndex idx, unsigned stride)
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
VfpMacroOp::nextIdxs(IntRegIndex &dest, IntRegIndex &op1, IntRegIndex &op2)
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
VfpMacroOp::nextIdxs(IntRegIndex &dest, IntRegIndex &op1)
{
    unsigned stride = (machInst.fpscrStride == 0) ? 1 : 2;
    assert(!inScalarBank(dest));
    dest = addStride(dest, stride);
    if (!inScalarBank(op1)) {
        op1 = addStride(op1, stride);
    }
}

void
VfpMacroOp::nextIdxs(IntRegIndex &dest)
{
    unsigned stride = (machInst.fpscrStride == 0) ? 1 : 2;
    assert(!inScalarBank(dest));
    dest = addStride(dest, stride);
}

}
