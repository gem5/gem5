/*
 * Copyright (c) 2015-2021 Advanced Micro Devices, Inc.
 * All rights reserved.
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
 */

#ifndef __ARCH_VEGA_GPU_TYPES_HH__
#define __ARCH_VEGA_GPU_TYPES_HH__

#include <cstdint>

namespace gem5
{

namespace VegaISA
{
union InstFormat;

/**
 * used to represnt a GPU inst in its raw format. VEGA
 * instructions may be 32b or 64b, therefore we represent
 * a raw inst with 64b to ensure that all of its inst data,
 * including potential immediate values, may be represented
 * in the worst case.
 */
typedef uint64_t RawMachInst;

/**
 * used to represent the encoding of a VEGA inst. each portion
 * of a VEGA inst must be 1 DWORD (32b), so we use a pointer
 * to InstFormat type (which is 32b). for the case in which we
 * need multiple DWORDS to represnt a single inst, this pointer
 * essentialy acts as an array of the DWORDs needed to represent
 * the entire inst encoding.
 */
typedef InstFormat *MachInst;

} // namespace VegaISA
} // namespace gem5

#endif // __ARCH_VEGA_GPU_TYPES_HH__
