/*
 * Copyright (c) 2012-2015 Advanced Micro Devices, Inc.
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
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
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
 * Author: Steve Reinhardt
 */

#ifndef __HSA_KERNEL_INFO_HH__
#define __HSA_KERNEL_INFO_HH__

// This file defines the public interface between the HSA emulated
// driver and application programs.

#include <cstdint>

static const int HSA_GET_SIZES = 0x4801;
static const int HSA_GET_KINFO = 0x4802;
static const int HSA_GET_STRINGS = 0x4803;
static const int HSA_GET_CODE = 0x4804;
static const int HSA_GET_READONLY_DATA = 0x4805;
static const int HSA_GET_CU_CNT = 0x4806;
static const int HSA_GET_VSZ = 0x4807;

// Return value (via buffer ptr) for HSA_GET_SIZES
struct HsaDriverSizes
{
    uint32_t num_kernels;
    uint32_t string_table_size;
    uint32_t code_size;
    uint32_t readonly_size;
};

// HSA_GET_KINFO returns an array of num_kernels of these structs
struct HsaKernelInfo
{
    // byte offset into string table
    uint32_t name_offs;
    // byte offset into code array
    uint32_t code_offs;
    uint32_t static_lds_size;
    uint32_t private_mem_size;
    uint32_t spill_mem_size;
    // Number of s registers
    uint32_t sRegCount;
    // Number of d registers
    uint32_t dRegCount;
    // Number of c registers
    uint32_t cRegCount;
};

#endif // __HSA_KERNEL_INFO_HH__
