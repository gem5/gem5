/*
 * Copyright (c) 2024 Advanced Micro Devices, Inc.
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

#ifndef __ARCH_AMDGPU_COMMON_DTYPE_MXFP_HH__
#define __ARCH_AMDGPU_COMMON_DTYPE_MXFP_HH__

#include <cmath>
#include <cstdint>
#include <iostream>

#include "arch/amdgpu/common/dtype/mxfp_convert.hh"

namespace gem5
{

namespace AMDGPU
{

// Base class for all microscaling types. The sizes of everything are
// determined by the enum fields in the FMT struct. All of these share the
// same operator overloads which convert to float before arithmetic and
// convert back if assigned to a microscaling type.
template<typename FMT>
class mxfp
{
  public:
    mxfp() = default;
    mxfp(float f) : mode(roundTiesToEven)
    {
        data = float_to_mxfp(f);
    }

    // Set raw bits, used by gem5 to set a raw value read from VGPRs.
    mxfp(const uint32_t& raw)
    {
        // The info unions end up being "left" aligned. For example, in FP4
        // only the bits 31:28 are used. Shift the input by the storage size
        // of 32 by the type size (sign + exponent + mantissa bits).
        data = raw;
        data <<= (32 - int(FMT::sbits) - int(FMT::ebits) - int(FMT::mbits));
    }

    mxfp(const mxfp& f)
    {
        FMT conv_out;
        conv_out = convertMXFP<FMT, decltype(f.getFmt())>(f.getFmt());
        data = conv_out.storage;
    }

    mxfp&
    operator=(const float& f)
    {
       data = float_to_mxfp(f);
       return *this;
    }

    mxfp&
    operator=(const mxfp& f)
    {
        FMT conv_out;
        conv_out = convertMXFP<FMT, decltype(f.getFmt())>(f.getFmt());
        data = conv_out.storage;
        return *this;
    }

    operator float() const
    {
        binary32 out;
        FMT in;
        in.storage = data;
        out = convertMXFP<binary32, FMT>(in, mode);

        return out.fp32;
    }

    constexpr static int
    size()
    {
        return int(FMT::mbits) + int(FMT::ebits) + int(FMT::sbits);
    }

    // Intentionally use storage > size() so that a storage type is not needed
    // as a template parameter.
    uint32_t data = 0;

    FMT
    getFmt() const
    {
        FMT out;
        out.storage = data;
        return out;
    }

    void
    setFmt(FMT in)
    {
        data = in.storage;
    }

    void
    scale(const float& f)
    {
        binary32 bfp;
        bfp.fp32 = f;
        int scale_val = bfp.exp - bfp.bias;

        // Scale value of 0xFF is NaN. Scaling by NaN returns NaN.
        // In this implementation, types without NaN define it as zero.
        if (scale_val == 0xFF) {
            data = FMT::nan;
            return;
        }

        FMT in = getFmt();
        int exp = in.exp;

        if (exp + scale_val > max_exp<FMT>()) {
            in.exp = max_exp<FMT>();
        } else if (exp + scale_val < min_exp<FMT>()) {
            in.exp = min_exp<FMT>();
        } else {
            in.exp = exp + scale_val;
        }

        data = in.storage;
    }

  private:
    mxfpRoundingMode mode = roundTiesToEven;

    uint32_t
    float_to_mxfp(float f)
    {
        if (std::isinf(f)) {
            assert(std::numeric_limits<FMT>::has_infinity);
            return FMT::inf;
        }

        if (std::isnan(f)) {
            assert(std::numeric_limits<FMT>::has_quiet_NaN);
            return FMT::nan;
        }

        return float_to_mxfp_nocheck(f);
    }

    uint32_t
    float_to_mxfp_nocheck(float f)
    {
        binary32 in;
        in.fp32 = f;

        FMT out;
        out.storage = 0;

        out = convertMXFP<FMT, binary32>(in, mode);

        return out.storage;
    }
};

// Unary operators
template<typename T>
inline T operator+(T a)
{
    return a;
}

template<typename T>
inline T operator-(T a)
{
    // Flip sign bit
    a.data ^= 0x80000000;
    return a;
}

template<typename T>
inline T operator++(T a)
{
    a = a + T(1.0f);
    return a;
}

template<typename T>
inline T operator--(T a)
{
    a = a - T(1.0f);
    return a;
}

template<typename T>
inline T operator++(T a, int)
{
    T original = a;
    ++a;
    return original;
}

template<typename T>
inline T operator--(T a, int)
{
    T original = a;
    --a;
    return original;
}

// Math operators
template<typename T>
inline T operator+(T a, T b)
{
    return T(float(a) + float(b));
}

template<typename T>
inline T operator-(T a, T b)
{
    return T(float(a) - float(b));
}

template<typename T>
inline T operator*(T a, T b)
{
    return T(float(a) * float(b));
}

template<typename T>
inline T operator/(T a, T b)
{
    return T(float(a) / float(b));
}

template<typename T>
inline T operator+=(T &a, T b)
{
    a = a + b;
    return a;
}

template<typename T>
inline T operator-=(T &a, T b)
{
    a = a - b;
    return a;
}

template<typename T>
inline T operator*=(T &a, T b)
{
    a = a * b;
    return a;
}

template<typename T>
inline T operator/=(T &a, T b)
{
    a = a / b;
    return a;
}

// Comparison operators
template<typename T>
inline bool operator<(T a, T b)
{
    return float(a) < float(b);
}

template<typename T>
inline bool operator>(T a, T b)
{
    return float(a) > float(b);
}

template<typename T>
inline bool operator<=(T a, T b)
{
    return float(a) <= float(b);
}

template<typename T>
inline bool operator>=(T a, T b)
{
    return float(a) >= float(b);
}

template<typename T>
inline bool operator==(T a, T b)
{
    return float(a) == float(b);
}

template<typename T>
inline bool operator!=(T a, T b)
{
    return float(a) != float(b);
}

} // namespace AMDGPU

} // namespace gem5

#endif // __ARCH_AMDGPU_COMMON_DTYPE_MXFP_HH__
