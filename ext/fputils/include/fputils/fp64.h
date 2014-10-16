/*
 * Copyright (c) 2014, Andreas Sandberg
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

#ifndef _FP64_H
#define _FP64_H 1

#include <fputils/fptypes.h>

#ifdef  __cplusplus
extern "C" {
#endif


/**
 * @defgroup fp64 64-bit Floats
 * Functions handling 64-bit floats.
 *
 * @{
 */


/** Constant representing +inf */
extern const fp64_t fp64_pinf;
/** Constant representing -inf */
extern const fp64_t fp64_ninf;

/** Constant representing a quiet NaN */
extern const fp64_t fp64_qnan;
/** Constant representing a negative quiet NaN */
extern const fp64_t fp64_nqnan;
/** Constant representing a quiet indefinite NaN */
extern const fp64_t fp64_qnani;
/** Constant representing a signaling NaN */
extern const fp64_t fp64_snan;
/** Constant representing a negative signaling NaN */
extern const fp64_t fp64_nsnan;

/** Alias for fp64_qnan */
extern const fp64_t fp64_nan;

/** @} */

#ifdef  __cplusplus
} /* extern "C" */
#endif

#endif
