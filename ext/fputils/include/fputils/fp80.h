/*
 * Copyright (c) 2013, Andreas Sandberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _FP80_H
#define _FP80_H 1

#include <math.h> /* FP_NAN et al. */
#include <stdio.h>

#include <fputils/fptypes.h>


#ifdef  __cplusplus
extern "C" {
#endif

/**
 * @defgroup fp80 80-bit Floats
 * Functions handling 80-bit floats.
 *
 * @{
 */

/** Constant representing +inf */
extern const fp80_t fp80_pinf;
/** Constant representing -inf */
extern const fp80_t fp80_ninf;
/** Constant representing a quiet NaN */
extern const fp80_t fp80_qnan;
/** Constant representing a quiet indefinite NaN */
extern const fp80_t fp80_qnani;
/** Constant representing a signaling NaN */
extern const fp80_t fp80_snan;
/** Alias for fp80_qnan */
extern const fp80_t fp80_nan;

/**
 * Is the value a special floating point value?
 *
 * Determine if a floating point value is one of the special values
 * (i.e., one of the infinities or NaNs). In practice, this function
 * only checks if the exponent is set to the maximum value supported
 * by the binary representation, which is a reserved value used for
 * such special numbers.
 *
 * @param fp80 value to analyze.
 * @return 1 if the value is special, 0 otherwise.
 */
int fp80_isspecial(fp80_t fp80);
/**
 * Is the value a quiet NaN?
 *
 * @param fp80 value to analyze.
 * @return 1 if true, 0 otherwise.
 */
int fp80_isqnan(fp80_t fp80);
/**
 * Is the value an indefinite quiet NaN?
 *
 * @param fp80 value to analyze.
 * @return 1 if true, 0 otherwise.
 */
int fp80_isqnani(fp80_t fp80);
/**
 * Is the value a signaling NaN?
 *
 * @param fp80 value to analyze.
 * @return 1 if true, 0 otherwise.
 */
int fp80_issnan(fp80_t fp80);

/**
 * Classify a floating point number.
 *
 * This function implements the same classification as the standard
 * fpclassify() function. It returns one of the following floating
 * point classes:
 * <ul>
 *   <li>FP_NAN - The value is NaN.
 *   <li>FP_INFINITE - The value is either +inf or -inf.
 *   <li>FP_ZERO - The value is either +0 or -0.
 *   <li>FP_SUBNORMAL - The value is to small to be represented as a
 *                      normalized float. See fp80_issubnormal().
 *   <li>FP_NORMAL - The value is neither of above.
 * </ul>
 *
 * @param fp80 value to analyze.
 * @return Floating point classification.
 */
int fp80_classify(fp80_t fp80);

/**
 * Is a value finite?
 *
 * Check if a value is a finite value. That is, not one of the
 * infinities or NaNs.
 *
 * @param fp80 value to analyze.
 * @return -1 if negative finite, +1 if positive finite, 0 otherwise.
 */
int fp80_isfinite(fp80_t fp80);
/**
 * Is the value a non-zero normal?
 *
 * This function checks if a floating point value is a normal (having
 * an exponent larger or equal to 1) or not. See fp80_issubnormal()
 * for a description of what a denormal value is.
 *
 * @see fp80_issubnormal()
 *
 * @param fp80 value to analyze.
 * @return -1 if negative normal, +1 if positive normal, 0 otherwise.
 */
int fp80_isnormal(fp80_t fp80);
/**
 * Is the value a NaN of any kind?
 *
 * @param fp80 value to analyze.
 * @return -1 if negative NaN, +1 if positive NaN, 0 otherwise.
 */
int fp80_isnan(fp80_t fp80);
/**
 * Is the value one of the infinities?
 *
 * @param fp80 value to analyze.
 * @return -1 if -inf, +1 if +inf, 0 otherwise.
 */
int fp80_isinf(fp80_t fp80);
/**
 * Determine value of the sign-bit of a floating point number.
 *
 * @note Floats can represent both positive and negative zeros.
 *
 * @param fp80 value to analyze.
 * @return -1 if negative, +1 if positive.
 */
int fp80_sgn(fp80_t fp80);
/**
 * Is the value zero?
 *
 * @param fp80 value to analyze.
 * @return -1 if negative zero, +1 if positive zero, 0 otherwise.
 */
int fp80_iszero(fp80_t fp80);
/**
 * Is the value a denormal?
 *
 * Numbers that are close to the minimum of what can be stored in a
 * floating point number start loosing precision because bits in the
 * fraction get used (implicitly) to store parts of the negative
 * exponent (i.e., the exponent is saturated and the fraction is less
 * than 1). Such numbers are known as denormals. This function checks
 * whether a float is a denormal or not.
 *
 * @param fp80 value to analyze.
 * @return -1 if negative denormal, +1 if positive denormal, 0 otherwise.
 */
int fp80_issubnormal(fp80_t fp80);


/**
 * Convert an 80-bit float to a 64-bit double.
 *
 * Convenience wrapper around fp80_cvtfp64() that returns a double
 * instead of the internal fp64_t representation.
 *
 * Note that this conversion is lossy, see fp80_cvtfp64() for details
 * of the conversion.
 *
 * @param fp80 Source value to convert.
 * @return value represented as double.
 */
double fp80_cvtd(fp80_t fp80);

/**
 * Convert an 80-bit float to a 64-bit double.
 *
 * This function converts an 80-bit float into a standard 64-bit
 * double. This conversion is inherently lossy since a double can only
 * represent a subset of what an 80-bit float can represent. The
 * fraction of the source value will always be truncated to fit the
 * lower precision. If a value falls outside of the range that can be
 * accurately represented by double by truncating the fraction, one of
 * the following happens:
 * <ul>
 *   <li>A denormal will be generated if that can approximate the
 *       value.
 *   <li>[-]0 will be generated if the magnitude of the value is too
 *       small to be represented at all.
 *   <li>+-Inf will be generated if the magnitude of the value is too
 *       large to be represented.
 * </ul>
 *
 * NaN values will be preserved across the conversion.
 *
 * @param fp80 Source value to convert.
 * @return 64-bit version of the float.
 */
fp64_t fp80_cvtfp64(fp80_t fp80);

/**
 * Convert a double to an 80-bit float.
 *
 * This is a convenience wrapper around fp80_cvffp64() and provides a
 * convenient way of using the native double type instead of the
 * internal fp64_t representation.
 *
 * @param fpd Source value to convert.
 * @return 80-bit version of the float.
 */
fp80_t fp80_cvfd(double fpd);

/**
 * Convert a 64-bit float to an 80-bit float.
 *
 * This function converts the internal representation of a 64-bit
 * float into an 80-bit float. This conversion is completely lossless
 * since the 80-bit float represents a superset of what a 64-bit
 * float can represent.
 *
 * @note Denormals will be converted to normalized values.
 *
 * @param fp64 64-bit float to convert.
 * @return 80-bit version of the float.
 */
fp80_t fp80_cvffp64(fp64_t fp64);

/**
 * Dump the components of an 80-bit float to a file.
 *
 * @warning This function is intended for debugging and the format of
 * the output is not guaranteed to be stable.
 *
 * @param fout Output stream (e.g., stdout)
 * @param fp80 value to dump.
 */
void fp80_debug_dump(FILE *fout, fp80_t fp80);

/** @} */

#ifdef  __cplusplus
} /* extern "C" */
#endif

#endif
