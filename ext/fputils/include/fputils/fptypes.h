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

#ifndef _FPTYPES_H
#define _FPTYPES_H 1

#include <stdint.h>

#ifdef  __cplusplus
extern "C" {
#endif

/**
 * @addtogroup fp64
 * @{
 */

/** Internal representation of a 64-bit float */
typedef union {
    /**
     * Raw value exposed as an unsigned integer. Mainly used for bit
     * manipulation.
     */
    uint64_t bits;
    /** Representation using the built-in double type */
    double value;
} fp64_t;

/** @} */


/**
 * @addtogroup fp80
 * @{
 */

/** Internal representation of an 80-bit float. */
typedef union  {
    struct {
        /** Raw representation of the integer part bit and the
         * fraction. Note that unlike 64-bit floating point
         * representations the integer bit is explicit. */
        uint64_t fi;
        /** Raw representation of sign bit and exponent */
        uint16_t se;
        /** Add explicit padding to ensure this data structure
         * is properly aligned.
         */
        uint16_t pad[3];
    } repr;
    /**
     * Represented as a char array, mainly intended for debug dumping
     * and serialization.
     */
    char bits[16];
} fp80_t;

/** @} */

#ifdef  __cplusplus
} /* extern "C" */
#endif

#endif
