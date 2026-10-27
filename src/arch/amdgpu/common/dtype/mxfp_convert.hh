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

#ifndef __ARCH_AMDGPU_COMMON_DTYPE_MXFP_CONVERT_HH__
#define __ARCH_AMDGPU_COMMON_DTYPE_MXFP_CONVERT_HH__

#include <algorithm>
#include <cassert>

#include "arch/amdgpu/common/dtype/mxfp_type_info.hh"
#include "base/bitfield.hh"

namespace gem5
{

namespace AMDGPU
{

// The various rounding modes for microscaling formats. roundTiesToEven must
// be supported. Other rounding modes may be supported.
enum mxfpRoundingMode
{
    roundTiesToEven,
    roundStochastic
};

// Conversion functions - For instructions that convert from one microscaling
// format to another. We only need the conversion functions as there do not
// appear to be any instructions yet which operate directly on the MX formats.
//
// in - An MXFP info struct type
// mode - rounding mode
// seed - input value for stochastic rounding function
template <typename dFMT, typename sFMT>
dFMT
convertMXFP(sFMT in, mxfpRoundingMode mode = roundTiesToEven,
            uint32_t seed = 0)
{
    // We assume that *both* exponent and mantissa bits are both >= or <=
    // the target type. Checkable at compile time.
    //
    // This is not necessarily a limitation, others just are not implemented.
    // Figuring this out would be interesting for converting FP8 <-> BF8 for
    // example. So far all GPU conversion instructions convert explicitly to
    // a larger type from a smaller type or smaller to larger.
    static_assert(((int(sFMT::mbits) >= int(dFMT::mbits)) &&
                   (int(sFMT::ebits) >= int(dFMT::ebits))) ||
                  ((int(sFMT::mbits) <= int(dFMT::mbits)) &&
                   (int(sFMT::ebits) <= int(dFMT::ebits))));

    dFMT out;
    out.storage = 0;

    // Optimize self-conversions which may happen due to copy constructors.
    if (std::is_same_v<sFMT, dFMT>) {
        out.storage = in.storage;
        return out;
    }

    if constexpr (int(sFMT::mbits) >= int(dFMT::mbits) &&
                  int(sFMT::ebits) >= int(dFMT::ebits)) {
        // Input format is larger, truncate and round mantissa. MX formats
        // are subnormal if exp == 0. Zero out exp in that case.

        if (std::isnan(in)) {
            // For types with no NaN return max value.
            if (std::numeric_limits<dFMT>::has_quiet_NaN) {
                out = std::numeric_limits<dFMT>::quiet_NaN();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            } else {
                out = std::numeric_limits<dFMT>::max();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            }
        } else if (std::isinf(in)) {
            // For types with no Inf return max value.
            if (std::numeric_limits<dFMT>::has_infinity) {
                out = std::numeric_limits<dFMT>::infinity();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            } else {
                out = std::numeric_limits<dFMT>::max();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            }
        } else if (in.mant == 0 && in.exp == 0) {
            // All MX formats FP32, and FP64 encode 0 as all zeros. Keep sign.
            out.mant = 0;
            out.exp = 0;
            out.sign = in.sign;
        } else {
            // Extra bits are needed for the mantissa conversion.
            uint32_t mant = in.mant & mask(sFMT::mbits);
            int32_t exp = in.exp - sFMT::bias + dFMT::bias;
            out.sign = in.sign;

            // Input is not subnormal, add the implicit 1 bit. A subnormal
            // input has the exponent of the smallest normal value.
            if (in.exp) {
                mant |= (1 << sFMT::mbits);
            } else {
                exp++;
            }

            // Shift out the extra mantissa bits. If the output becomes
            // subnormal, shift out 1 - exp more bits and keep the smallest
            // normal exponent. Shifting out more than mbits + 2 bits leaves
            // zero and less than half either way, so limit the shift there.
            int shift = sFMT::mbits - dFMT::mbits;
            if (exp < 1) {
                shift = std::min(shift + 1 - exp, int(sFMT::mbits) + 2);
                exp = 1;
            }
            uint32_t shifted_out = mant & mask(shift);
            mant >>= shift;

            if (int(sFMT::mbits) > int(dFMT::mbits) &&
                mode == roundTiesToEven) {
                // roundTiesToEven is the only required rounding mode for MXFP
                // types. Round up if the bits shifted out are more than half
                // of the last kept bit, or exactly half and the kept mantissa
                // is odd.
                uint32_t half = 1 << (shift - 1);
                if (shifted_out > half ||
                    (shifted_out == half && (mant & 1))) {
                    mant++;
                }
            } else if (int(sFMT::mbits) > int(dFMT::mbits) &&
                       mode == roundStochastic) {
                // Use the discarded mantissa divided by the max mantissa of
                // the source format to determine the probability of rounding
                // up. An alternate implementation of this would be to get a
                // random number and add that to the input mantissa. Then
                // follow the normal rounding path above.
                uint32_t discarded = in.mant & mask(sFMT::mbits - dFMT::mbits);
                uint32_t max_mant = mask(sFMT::mbits);

                float round_prob = float(discarded) / float(max_mant);

                // Use a stochastic rounding function with the seed value to
                // determine compare probability. This is implemented as a
                // "Galois LFSR."
                auto srFunc = [](uint32_t in) {
                    uint32_t bit = (in ^ (in >> 1) ^ (in >> 3) ^ (in >> 12));
                    return (in >> 1) | (bit << 15);
                };

                // Assume stochastic rounding returns up to max uint32_t.
                // This will return an FP value between 0.0f and 1.0f.
                float draw_prob = float(srFunc(seed)) /
                                  float(std::numeric_limits<uint32_t>::max());

                // Round up if the number we drew is less than the rounding
                // probability. E.g., if round_prob is 90% (0.9) we choose
                // values 0.0f - 0.90f to round up.
                if (round_prob <= draw_prob) {
                    mant++;
                }
            }

            // The exponent field sits right above the mantissa, so adding the
            // mantissa with its implicit bit to (exp - 1) << mbits gives the
            // encoding. A carry from rounding moves to the next exponent and
            // a subnormal result, which has no implicit bit, gets exponent 0.
            uint32_t bits = ((exp - 1) << dFMT::mbits) + mant;

            // Largest finite encoding. Formats with infinity reserve the top
            // exponent and E4M3 reserves only its NaN encoding.
            uint32_t max_bits = mask(dFMT::ebits + dFMT::mbits);
            if (std::numeric_limits<dFMT>::has_infinity) {
                max_bits -= 1 << dFMT::mbits;
            } else if (std::numeric_limits<dFMT>::has_quiet_NaN) {
                max_bits -= 1;
            }

            // Values too large for the format convert like an infinity.
            if (bits > max_bits) {
                if (std::numeric_limits<dFMT>::has_infinity) {
                    out = std::numeric_limits<dFMT>::infinity();
                } else {
                    out = std::numeric_limits<dFMT>::max();
                }
                out.sign = in.sign;
            } else {
                out.exp = bits >> dFMT::mbits;
                out.mant = bits & mask(dFMT::mbits);
            }
        }
    } else if constexpr (int(sFMT::mbits) <= int(dFMT::mbits) &&
                         int(sFMT::ebits) <= int(dFMT::ebits)) {
        // Input format is smaller. Extend mantissa / exponent and pad with 0.
        // Should be the same for all non-stochastic rounding modes.

        if (std::isnan(in)) {
            // For types with no NaN return max value.
            if (std::numeric_limits<dFMT>::has_quiet_NaN) {
                out = std::numeric_limits<dFMT>::quiet_NaN();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            } else {
                out = std::numeric_limits<dFMT>::max();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            }
        } else if (std::isinf(in)) {
            // For types with no Inf return max value.
            if (std::numeric_limits<dFMT>::has_infinity) {
                out = std::numeric_limits<dFMT>::infinity();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            } else {
                out = std::numeric_limits<dFMT>::max();
                // Preserve sign bit
                if (in.storage & 0x80000000) {
                    out.storage |= 0x80000000;
                }
            }
        } else if (in.mant == 0 && in.exp == 0) {
            // All MX formats FP32, and FP64 encode 0 as all zeros. Keep sign.
            out.mant = 0;
            out.exp = 0;
            out.sign = in.sign;
        } else {
            out.mant = in.mant << (dFMT::mbits - sFMT::mbits);
            out.exp = in.exp + dFMT::bias - sFMT::bias;
            out.sign = in.sign;

            // Normalize input denormals. This only applies when exponents are
            // different. Otherwise, the operation is simply a zero extend of
            // mantissa (e.g., bf16 -> f32).
            if (!in.exp && int(sFMT::ebits) != int(dFMT::ebits)) {
                uint32_t m = out.mant;
                if (m != 0) {
                    out.exp++;
                    while (!(m >> dFMT::mbits)) {
                        m <<= 1;
                        out.exp--;
                    }
                    out.mant = m & mask(dFMT::mbits);
                }
            }
        }
    } else {
        assert(false);
    }

    return out;
}

template <typename FMT>
int
min_exp()
{
    return 0;
}

template <typename FMT>
int
max_exp()
{
    return (1 << FMT::ebits) - 1;
}

} // namespace AMDGPU

} // namespace gem5

#endif // __ARCH_AMDGPU_COMMON_DTYPE_MXFP_CONVERT_HH__
