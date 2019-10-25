/*
 * Copyright (c) 2014 Andreas Sandberg
 * All rights reserved
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
 * Authors: Andreas Sandberg
 */

#include <fputils/fp64.h>
#include "fpbits.h"

#include <assert.h>
#include <stdint.h>

#include <stdio.h>

const fp64_t fp64_pinf  = BUILD_FP64(0, 0,               FP64_EXP_SPECIAL);
const fp64_t fp64_ninf  = BUILD_FP64(1, 0,               FP64_EXP_SPECIAL);
const fp64_t fp64_qnan  = BUILD_FP64(0, FP64_FRAC_QNAN,  FP64_EXP_SPECIAL);
const fp64_t fp64_nqnan = BUILD_FP64(1, FP64_FRAC_QNAN,  FP64_EXP_SPECIAL);
const fp64_t fp64_qnani = BUILD_FP64(1, FP64_FRAC_QNANI, FP64_EXP_SPECIAL);
const fp64_t fp64_snan  = BUILD_FP64(0, FP64_FRAC_SNAN,  FP64_EXP_SPECIAL);
const fp64_t fp64_nsnan = BUILD_FP64(1, FP64_FRAC_SNAN,  FP64_EXP_SPECIAL);
const fp64_t fp64_nan   = BUILD_FP64(0, FP64_FRAC_QNAN,  FP64_EXP_SPECIAL);
