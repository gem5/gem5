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
template<typename dFMT, typename sFMT>
dFMT convertMXFP(sFMT in, mxfpRoundingMode mode = roundTiesToEven,
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
                   (int(sFMT::ebits) >= int(dFMT::ebits)))
               || ((int(sFMT::mbits) <= int(dFMT::mbits)) &&
                   (int(sFMT::ebits) <= int(dFMT::ebits))));

    dFMT out;
    out.storage = 0;

    if (int(sFMT::mbits) >= int(dFMT::mbits) &&
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
            out.exp  = 0;
            out.sign = in.sign;
        } else {
            // Extra bits are needed for the mantissa conversion.
            uint32_t mant = in.mant & mask(sFMT::mbits);
            int32_t exp   = in.exp - sFMT::bias + dFMT::bias;
            out.sign = in.sign;

            // Input is not subnormal, add the implicit 1 bit.
            if (in.exp) {
                mant |= (1 << sFMT::mbits);
            }

            mant >>= (sFMT::mbits - dFMT::mbits);

            // Output became subnormal
            if (exp < 1) {
                int shift = 1 - exp;
                mant >>= shift;
                out.exp = 0;
            } else {
                out.exp = exp;
            }

            mant &= mask(dFMT::mbits);
            out.mant = mant;

            // roundTiesToEven is the only required rounding mode for MXFP
            // types. Here we take the original mantissa and check the final
            // bit which is shifted out when converting the mantissa. If that
            // value is one, then we should round up to the next representable
            // number. If the value is one and all other discarded mantissa
            // bits are zero, round towards the number which has an even (0)
            // bit value in the least significant mantissa bit.
            //
            // For denormals, the process is similar however we check the nth
            // bit of the converted mantissa, where n is the absolute value of
            // the converted exponent. If the value of |exp| is larger than
            // the max exponent, round to zero. If it is exactly equal, always
            // round up.
            //
            // If the number of destination and source format mantissa bits are
            // the same, the mantissa is unchanged.
            if (int(sFMT::mbits) > int(dFMT::mbits)
                    && mode == roundTiesToEven) {
                bool round_up = false;

                int check_shift = sFMT::mbits - dFMT::mbits - 1;
                uint32_t check_mant = in.mant & mask(sFMT::mbits);

                check_mant >>= check_shift;

                // out.exp == 0 means subnormal
                if (out.exp == 0) {
                    check_mant = in.mant >> (sFMT::mbits - dFMT::mbits);

                    uint32_t max_exp = mask(dFMT::ebits);
                    if (-exp > max_exp) {
                        // if exp < -(1 << dFMT::ebits), result should be 0
                        round_up = false;
                    } else if (-exp == max_exp) {
                        // if exp == -(1 << dFMT::ebits), round up
                        round_up = true;
                    } else {
                        // Use the |exp|'th bit to determine rounding
                        int check_bit = 1 << -exp;
                        round_up = (check_mant & check_bit);
                    }
                } else {
                    round_up = (check_mant & 0x1);
                }

                // For roundTiesToEven, if we are exactly between two
                // representable numbers, pick the one with an even least
                // significant mantissa bit. We are exactly between when
                // all of the discarded mantissa bits are 0 (i.e., !sticky).
                int sticky = in.mant & mask(sFMT::mbits - dFMT::mbits);
                if (round_up && !sticky) {
                    if (!(out.mant & 1)) {
                        round_up = false;
                    }
                }

                if (round_up) {
                    if (out.mant == mask(dFMT::mbits)) {
                        // mantissa at max value, increment exponent if not inf
                        if (out.exp != mask(dFMT::ebits)) {
                            out.exp++;
                        }
                        out.mant = 0;
                    } else {
                        out.mant++;
                    }
                }
            } else if (int(sFMT::mbits) > int(dFMT::mbits)
                    && mode == roundStochastic) {
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
                float draw_prob = float(srFunc(seed))
                    / float(std::numeric_limits<uint32_t>::max());

                // Round up if the number we drew is less than the rounding
                // probability. E.g., if round_prob is 90% (0.9) we choose
                // values 0.0f - 0.90f to round up.
                if (round_prob >= draw_prob) {
                    if (out.mant == mask(dFMT::mbits)) {
                        // mantissa at max value, increment exponent if not inf
                        if (out.exp != mask(dFMT::ebits)) {
                            out.exp++;
                        }
                        out.mant = 0;
                    } else {
                        out.mant++;
                    }
                }
            }
        }
    } else if (int(sFMT::mbits) <= int(dFMT::mbits) &&
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
            out.exp  = 0;
            out.sign = in.sign;
        } else {
            out.mant = in.mant << (dFMT::mbits - sFMT::mbits);
            out.exp  = in.exp + dFMT::bias - sFMT::bias;
            out.sign = in.sign;

            // Normalize input denormals
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
            } else if (!in.exp) {
                // Exponent is the same, but output is not denorm, so add
                // implicit 1. This is specific mainly to bf16 -> f32.
                uint32_t m = out.mant;
                m <<= 1;
                out.mant = m & mask(dFMT::mbits);
            }
        }
    } else {
        assert(false);
    }

    return out;
}

template<typename FMT>
int min_exp()
{
    return 1;
}

template<typename FMT>
int max_exp()
{
    return (1 << FMT::ebits) - 1;
}


} // namespace AMDGPU

} // namespace gem5

#endif // __ARCH_AMDGPU_COMMON_DTYPE_MXFP_CONVERT_HH__
