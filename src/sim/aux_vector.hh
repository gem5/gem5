/*
 * Copyright (c) 2016 Advanced Micro Devices, Inc.
 * All rights reserved.
 *
 * For use for simulation and test purposes only
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Brandon Potter
 */

#ifndef __AUX_VECTOR_HH__
#define __AUX_VECTOR_HH__

template<class IntType>
struct AuxVector
{
    IntType a_type;
    IntType a_val;

    AuxVector()
    {}

    AuxVector(IntType type, IntType val);
};

enum AuxiliaryVectorType {
    M5_AT_NULL = 0,
    M5_AT_IGNORE = 1,
    M5_AT_EXECFD = 2,
    M5_AT_PHDR = 3,
    M5_AT_PHENT = 4,
    M5_AT_PHNUM = 5,
    M5_AT_PAGESZ = 6,
    M5_AT_BASE = 7,
    M5_AT_FLAGS = 8,
    M5_AT_ENTRY = 9,
    M5_AT_NOTELF = 10,
    M5_AT_UID = 11,
    M5_AT_EUID = 12,
    M5_AT_GID = 13,
    M5_AT_EGID = 14,
    M5_AT_PLATFORM = 15,
    M5_AT_HWCAP = 16,
    M5_AT_CLKTCK = 17,
    M5_AT_SECURE = 23,
    M5_BASE_PLATFORM = 24,
    M5_AT_RANDOM = 25,
    M5_AT_EXECFN = 31,
    M5_AT_VECTOR_SIZE = 44
};

#endif // __AUX_VECTOR_HH__
