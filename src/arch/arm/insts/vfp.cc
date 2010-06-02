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

#include "arch/arm/insts/vfp.hh"

std::string
FpRegRegOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Base_DepTag);
    ss << ", ";
    printReg(ss, op1 + FP_Base_DepTag);
    return ss.str();
}

std::string
FpRegImmOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Base_DepTag);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
FpRegRegImmOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Base_DepTag);
    ss << ", ";
    printReg(ss, op1 + FP_Base_DepTag);
    ccprintf(ss, ", #%d", imm);
    return ss.str();
}

std::string
FpRegRegRegOp::generateDisassembly(Addr pc, const SymbolTable *symtab) const
{
    std::stringstream ss;
    printMnemonic(ss);
    printReg(ss, dest + FP_Base_DepTag);
    ss << ", ";
    printReg(ss, op1 + FP_Base_DepTag);
    ss << ", ";
    printReg(ss, op2 + FP_Base_DepTag);
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
fpType
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

template
float fixDest<float>(FPSCR fpscr, float val, float op1);
template
double fixDest<double>(FPSCR fpscr, double val, double op1);

template <class fpType>
fpType
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

template
float fixDest<float>(FPSCR fpscr, float val, float op1, float op2);
template
double fixDest<double>(FPSCR fpscr, double val, double op1, double op2);

template <class fpType>
fpType
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

template
float fixDivDest<float>(FPSCR fpscr, float val, float op1, float op2);
template
double fixDivDest<double>(FPSCR fpscr, double val, double op1, double op2);

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

float
vcvtFpSFpH(FPSCR &fpscr, float op, float dest, bool top)
{
    float junk = 0.0;
    uint32_t destBits = fpToBits(dest);
    uint32_t opBits = fpToBits(op);
    // Extract the operand.
    bool neg = bits(opBits, 31);
    uint32_t exponent = bits(opBits, 30, 23);
    uint32_t oldMantissa = bits(opBits, 22, 0);
    uint32_t mantissa = oldMantissa >> (23 - 10);
    // Do the conversion.
    uint32_t extra = oldMantissa & mask(23 - 10);
    if (exponent == 0xff) {
        if (oldMantissa != 0) {
            // Nans.
            if (bits(mantissa, 9) == 0) {
                // Signalling nan.
                fpscr.ioc = 1;
            }
            if (fpscr.ahp) {
                mantissa = 0;
                exponent = 0;
                fpscr.ioc = 1;
            } else if (fpscr.dn) {
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
            if (fpscr.ahp) {
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
            assert(fpscr.fz == 0);

            // Check for underflow
            if (inexact || fpscr.ufe)
                fpscr.ufc = 1;

            // Handle rounding.
            unsigned mode = fpscr.rMode;
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
            bool topOne = bits(extra, 12);
            bool restZeros = bits(extra, 11, 0) == 0;

            if (exponent <= (127 - 15)) {
                // The result is too small. Denormalize.
                mantissa |= (1 << 10);
                while (mantissa && exponent <= (127 - 15)) {
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
                exponent -= (127 - 15);
            }

            if (exponent == 0 && (inexact || fpscr.ufe)) {
                // Underflow
                fpscr.ufc = 1;
            }

            // Handle rounding.
            unsigned mode = fpscr.rMode;
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
            if (fpscr.ahp) {
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
    if (top)
        replaceBits(destBits, 31, 16, result);
    else
        replaceBits(destBits, 15, 0, result);
    return bitsToFp(destBits, junk);
}

float
vcvtFpHFpS(FPSCR &fpscr, float op, bool top)
{
    float junk = 0.0;
    uint32_t opBits = fpToBits(op);
    // Extract the operand.
    if (top)
        opBits = bits(opBits, 31, 16);
    else
        opBits = bits(opBits, 15, 0);
    // Extract the bitfields.
    bool neg = bits(opBits, 15);
    uint32_t exponent = bits(opBits, 14, 10);
    uint32_t mantissa = bits(opBits, 9, 0);
    // Do the conversion.
    if (exponent == 0) {
        if (mantissa != 0) {
            // Normalize the value.
            exponent = exponent + (127 - 15) + 1;
            while (mantissa < (1 << 10)) {
                mantissa = mantissa << 1;
                exponent--;
            }
        }
        mantissa = mantissa << (23 - 10);
    } else if (exponent == 0x1f && !fpscr.ahp) {
        // Infinities and nans.
        exponent = 0xff;
        if (mantissa != 0) {
            // Nans.
            mantissa = mantissa << (23 - 10);
            if (bits(mantissa, 22) == 0) {
                // Signalling nan.
                fpscr.ioc = 1;
                mantissa |= (1 << 22);
            }
            if (fpscr.dn) {
                mantissa &= ~mask(22);
                neg = false;
            }
        }
    } else {
        exponent = exponent + (127 - 15);
        mantissa = mantissa << (23 - 10);
    }
    // Reassemble the result.
    uint32_t result = bits(mantissa, 22, 0);
    replaceBits(result, 30, 23, exponent);
    if (neg)
        result |= (1 << 31);
    return bitsToFp(result, junk);
}

uint64_t
vfpFpSToFixed(float val, bool isSigned, bool half,
              uint8_t imm, bool rzero)
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

float
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

float
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

uint64_t
vfpFpDToFixed(double val, bool isSigned, bool half,
              uint8_t imm, bool rzero)
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

double
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

double
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

template <class fpType>
fpType
FpOp::binaryOp(FPSCR &fpscr, fpType op1, fpType op2,
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

template
float FpOp::binaryOp(FPSCR &fpscr, float op1, float op2,
                     float (*func)(float, float),
                     bool flush, uint32_t rMode) const;
template
double FpOp::binaryOp(FPSCR &fpscr, double op1, double op2,
                      double (*func)(double, double),
                      bool flush, uint32_t rMode) const;

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
