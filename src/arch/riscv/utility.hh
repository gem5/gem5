/*
 * Copyright (c) 2013 ARM Limited
 * Copyright (c) 2014-2015 Sven Karlsson
 * Copyright (c) 2018 TU Dresden
 * Copyright (c) 2020 Barkhausen Institut
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2016-2017 The University of Virginia
 * All rights reserved.
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
 */

#ifndef __ARCH_RISCV_UTILITY_HH__
#define __ARCH_RISCV_UTILITY_HH__

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

#include "arch/riscv/regs/float.hh"
#include "arch/riscv/regs/int.hh"
#include "arch/riscv/regs/vector.hh"
#include "base/types.hh"
#include "cpu/reg_class.hh"
#include "cpu/static_inst.hh"
#include "cpu/thread_context.hh"
#include "enums/RiscvType.hh"
#include "rvk.hh"

namespace gem5
{

namespace RiscvISA
{

template<typename Type> struct double_width;
template<> struct double_width<uint8_t>     { using type = uint16_t;};
template<> struct double_width<uint16_t>    { using type = uint32_t;};
template<> struct double_width<uint32_t>    { using type = uint64_t;};
template<> struct double_width<uint64_t>    { using type = __uint128_t;};
template<> struct double_width<int8_t>      { using type = int16_t; };
template<> struct double_width<int16_t>     { using type = int32_t; };
template<> struct double_width<int32_t>     { using type = int64_t; };
template<> struct double_width<int64_t>     { using type = __int128_t; };
template<> struct double_width<float32_t>   { using type = float64_t;};

template<typename Type> struct double_widthf;
template<> struct double_widthf<uint32_t>    { using type = float64_t;};
template<> struct double_widthf<int32_t>     { using type = float64_t;};

template<typename T> inline bool
isquietnan(T val)
{
    return false;
}

template<> inline bool
isquietnan<float>(float val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint32_t&>(val)&0x00400000);
}

template<> inline bool
isquietnan<double>(double val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint64_t&>(val)&0x0008000000000000ULL);
}

template<typename T> inline bool
issignalingnan(T val)
{
    return false;
}

template<> inline bool
issignalingnan<float>(float val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint32_t&>(val)&0x00200000);
}

template<> inline bool
issignalingnan<double>(double val)
{
    return std::isnan(val)
        && (reinterpret_cast<uint64_t&>(val)&0x0004000000000000ULL);
}

inline std::string
registerName(RegId reg)
{
    if (reg.is(IntRegClass)) {
        if (reg.index() >= int_reg::NumArchRegs) {
            /*
             * This should only happen if a instruction is being speculatively
             * executed along a not-taken branch, and if that instruction's
             * width was incorrectly predecoded (i.e., it was predecoded as a
             * full instruction rather than a compressed one or vice versa).
             * It also should only happen if a debug flag is on that prints
             * disassembly information, so rather than panic the incorrect
             * value is printed for debugging help.
             */
            std::stringstream str;
            str << "?? (x" << reg.index() << ')';
            return str.str();
        }
        return int_reg::RegNames[reg.index()];
    } else if (reg.is(FloatRegClass)) {
        if (reg.index() >= float_reg::NumRegs) {
            std::stringstream str;
            str << "?? (f" << reg.index() << ')';
            return str.str();
        }
        return float_reg::RegNames[reg.index()];
    } else if (reg.is(VecRegClass)) {
        if (reg.index() >= NumVecRegs) {
            std::stringstream str;
            str << "?? (v" << reg.index() << ')';
            return str.str();
        }
        return VecRegNames[reg.index()];
    } else  {
        /* It must be an InvalidRegClass, in RISC-V we should treat it as a
         * zero register for the disassembler to work correctly.
         */
        return int_reg::RegNames[reg.index()];
    }
}

template <typename T> inline std::make_unsigned_t<T>
mulhu(std::make_unsigned_t<T> rs1, std::make_unsigned_t<T> rs2)
{
    using WideT = typename double_width<std::make_unsigned_t<T>>::type;
    return ((WideT)rs1 * rs2) >> (sizeof(T) * 8);
}

template <typename T> inline std::make_signed_t<T>
mulh(std::make_signed_t<T> rs1, std::make_signed_t<T> rs2)
{
    using WideT = typename double_width<std::make_signed_t<T>>::type;
    return ((WideT)rs1 * rs2) >> (sizeof(T) * 8);
}

template <typename T> inline std::make_signed_t<T>
mulhsu(std::make_signed_t<T> rs1, std::make_unsigned_t<T> rs2)
{
    using WideT = typename double_width<std::make_signed_t<T>>::type;
    return ((WideT)rs1 * rs2) >> (sizeof(T) * 8);
}

template<typename T> inline T
div(T rs1, T rs2)
{
    constexpr T kRsMin = std::numeric_limits<T>::min();
    if (rs2 == 0) {
        return -1;
    } else if (rs1 == kRsMin && rs2 == -1) {
        return kRsMin;
    } else {
        return rs1 / rs2;
    }
}

template<typename T> inline T
divu(T rs1, T rs2)
{
    if (rs2 == 0) {
        return std::numeric_limits<T>::max();
    } else {
        return rs1 / rs2;
    }
}

template<typename T> inline T
rem(T rs1, T rs2)
{
    constexpr T kRsMin = std::numeric_limits<T>::min();
    if (rs2 == 0) {
        return rs1;
    } else if (rs1 == kRsMin && rs2 == -1) {
        return 0;
    } else {
        return rs1 % rs2;
    }
}

template<typename T> inline T
remu(T rs1, T rs2)
{
    return (rs2 == 0) ? rs1 : rs1 % rs2;
}

// Vector extension functions
inline uint64_t
vtype_SEW(const uint64_t vtype)
{
    return 8 << bits(vtype, 5, 3);
}

/*
* Encode LMUL to lmul as follows:
*     LMUL    vlmul    lmul
*      1       000       0
*      2       001       1
*      4       010       2
*      8       011       3
*      -       100       -
*     1/8      101      -3
*     1/4      110      -2
*     1/2      111      -1
*
* then, we can calculate VLMAX = vlen >> (vsew + 3 - lmul)
* e.g. vlen = 256 bits, SEW = 16, LMUL = 1/8
*      => VLMAX = vlen >> (1 + 3 - (-3))
*               = 256 >> 7
*               = 2
* Ref: https://github.com/qemu/qemu/blob/5e9d14f2/target/riscv/cpu.h
*/
inline uint64_t
vtype_VLMAX(const uint64_t vtype, const uint64_t vlen,
    const bool per_reg = false)
{
    int64_t lmul = (int64_t)sext<3>(bits(vtype, 2, 0));
    lmul = per_reg ? std::min<int64_t>(0, lmul) : lmul;
    int64_t vsew = bits(vtype, 5, 3);
    return vlen >> (vsew + 3 - lmul);
}

inline int64_t
vtype_vlmul(const uint64_t vtype)
{
    return (int64_t)sext<3>(bits(vtype, 2, 0));
}

inline uint64_t
vtype_regs_per_group(const uint64_t vtype)
{
    int64_t lmul = (int64_t)sext<3>(bits(vtype, 2, 0));
    return 1 << std::max<int64_t>(0, lmul);
}

inline void
vtype_set_vill(uint64_t& vtype)
{
    vtype = (uint64_t)0 ^ (1UL << (sizeof(RegVal) * 8 - 1));
}

inline uint64_t
width_EEW(uint64_t width)
{
    switch (width) {
    case 0b000: return 8;
    case 0b101: return 16;
    case 0b110: return 32;
    case 0b111: return 64;
    default: GEM5_UNREACHABLE;
    }
}

/*
  *  Spec Section 4.5
  *  Ref:
  *  https://github.com/qemu/qemu/blob/c7d773ae/target/riscv/vector_helper.c
*/
template<typename T>
inline int
elem_mask(const T* vs, const int index)
{
    static_assert(std::is_integral_v<T>);
    int idx = index / (sizeof(T)*8);
    int pos = index % (sizeof(T)*8);
    return (vs[idx] >> pos) & 1;
}

template<typename T>
inline int
elem_mask_vlseg(const T* vs, const int elem, const int num_fields)
{
    int index = floor(elem / num_fields);
    static_assert(std::is_integral_v<T>);
    int idx = index / (sizeof(T)*8);
    int pos = index % (sizeof(T)*8);
    return (vs[idx] >> pos) & 1;
}

template<typename FloatType, typename IntType = decltype(FloatType::v)> auto
ftype(IntType a) -> FloatType
{
    if constexpr(std::is_same_v<uint32_t, IntType>)
        return f32(a);
    else if constexpr(std::is_same_v<uint64_t, IntType>)
        return f64(a);
    GEM5_UNREACHABLE;
}

// TODO: Consolidate ftype_freg(freg_t a) and ftype(IntType a) into a
// single function
template<typename FloatType, typename IntType = decltype(FloatType::v)> auto
ftype_freg(freg_t a) -> FloatType
{
    if constexpr(std::is_same_v<uint32_t, IntType>)
        return f32(a);
    else if constexpr(std::is_same_v<uint64_t, IntType>)
        return f64(a);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fadd(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_add(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_add(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fsub(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_sub(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_sub(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fmin(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_min(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_min(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fmax(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_max(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_max(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fdiv(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_div(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_div(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fmul(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_mul(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_mul(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fsqrt(FloatType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_sqrt(a);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_sqrt(a);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
frsqrte7(FloatType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_rsqrte7(a);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_rsqrte7(a);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
frecip7(FloatType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_recip7(a);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_recip7(a);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fclassify(FloatType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32(f32_classify(a));
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64(f64_classify(a));
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fsgnj(FloatType a, FloatType b, bool n, bool x)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return fsgnj32(a, b, n, x);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return fsgnj64(a, b, n, x);
    GEM5_UNREACHABLE;
}

template<typename FloatType> bool
fle(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_le(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_le(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> bool
feq(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_eq(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_eq(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> bool
flt(FloatType a, FloatType b)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_lt(a, b);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_lt(a, b);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fmadd(FloatType a, FloatType b, FloatType c)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_mulAdd(a, b, c);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_mulAdd(a, b, c);
    GEM5_UNREACHABLE;
}

template<typename FloatType> FloatType
fneg(FloatType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32(a.v ^ uint32_t(mask(31, 31)));
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64(a.v ^ mask(63, 63));
    GEM5_UNREACHABLE;
}

template<typename FT, typename WFT = typename double_width<FT>::type> WFT
fwiden(FT a)
{
    if constexpr(std::is_same_v<float32_t, FT>)
        return f32_to_f64(a);
    GEM5_UNREACHABLE;
}

template<typename FloatType, typename IntType = decltype(FloatType::v)> IntType
f_to_ui(FloatType a, uint_fast8_t mode)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_to_ui32(a, mode, true);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_to_ui64(a, mode, true);
    GEM5_UNREACHABLE;
}

template<
    typename FloatType,
    typename IntType = decltype(double_width<FloatType>::type::v)
> IntType
f_to_wui(FloatType a, uint_fast8_t mode)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_to_ui64(a, mode, true);
    GEM5_UNREACHABLE;
}

template<
    typename IntType,
    typename FloatType = typename double_widthf<IntType>::type
> IntType
f_to_nui(FloatType a, uint_fast8_t mode)
{
    if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_to_ui32(a, mode, true);
    GEM5_UNREACHABLE;
}

template<typename FloatType, typename IntType = decltype(FloatType::v)> IntType
f_to_i(FloatType a, uint_fast8_t mode)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return (uint32_t)f32_to_i32(a, mode, true);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return (uint64_t)f64_to_i64(a, mode, true);
    GEM5_UNREACHABLE;
}

template<
    typename FloatType,
    typename IntType = decltype(double_width<FloatType>::type::v)
> IntType
f_to_wi(FloatType a, uint_fast8_t mode)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return (uint64_t)f32_to_i64(a, mode, true);
    GEM5_UNREACHABLE;
}

template<
    typename IntType,
    typename FloatType = typename double_widthf<IntType>::type
> IntType
f_to_ni(FloatType a, uint_fast8_t mode)
{
    if constexpr(std::is_same_v<float64_t, FloatType>)
        return (uint32_t)f64_to_i32(a, mode, true);
    GEM5_UNREACHABLE;
}

template<typename FloatType, typename IntType = decltype(FloatType::v)>
FloatType
ui_to_f(IntType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return ui32_to_f32(a);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return ui64_to_f64(a);
    GEM5_UNREACHABLE;
}

template<
    typename IntType,
    typename FloatType = typename double_widthf<IntType>::type
> FloatType
ui_to_wf(IntType a)
{
    if constexpr(std::is_same_v<float64_t, FloatType>)
        return ui32_to_f64(a);
    GEM5_UNREACHABLE;
}

template<
    typename FloatType,
    typename IntType = decltype(double_width<FloatType>::type::v)
> FloatType
ui_to_nf(IntType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return ui64_to_f32(a);
    GEM5_UNREACHABLE;
}

template<typename FloatType, typename IntType = decltype(FloatType::v)>
FloatType
i_to_f(IntType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return i32_to_f32((int32_t)a);
    else if constexpr(std::is_same_v<float64_t, FloatType>)
        return i64_to_f64((int64_t)a);
    GEM5_UNREACHABLE;
}

template<
    typename IntType,
    typename FloatType = typename double_widthf<IntType>::type
> FloatType
i_to_wf(IntType a)
{
    if constexpr(std::is_same_v<float64_t, FloatType>)
        return i32_to_f64((int32_t)a);
    GEM5_UNREACHABLE;
}

template<
    typename FloatType,
    typename IntType = std::make_signed_t<
        decltype(double_width<FloatType>::type::v)
    >
> FloatType
i_to_nf(IntType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return i64_to_f32(a);
    GEM5_UNREACHABLE;
}

template<
    typename FloatType,
    typename FloatWType = typename double_width<FloatType>::type
> FloatWType
f_to_wf(FloatType a)
{
    if constexpr(std::is_same_v<float32_t, FloatType>)
        return f32_to_f64(a);
    GEM5_UNREACHABLE;
}

template<
    typename FloatNType,
    typename FloatType = typename double_width<FloatNType>::type
> FloatNType
f_to_nf(FloatType a)
{
    if constexpr(std::is_same_v<float64_t, FloatType>)
        return f64_to_f32(a);
    GEM5_UNREACHABLE;
}

//ref:  https://locklessinc.com/articles/sat_arithmetic/
template<typename T> T
sat_add(T x, T y, bool* sat)
{
    using UT = std::make_unsigned_t<T>;
    UT ux = x;
    UT uy = y;
    UT res = ux + uy;

    int sh = sizeof(T) * 8 - 1;

    ux = (ux >> sh) + (((UT)0x1 << sh) - 1);

    if ((T) ((ux ^ uy) | ~(uy ^ res)) >= 0) {
    res = ux;
    *sat = true;
    }
    return res;
}

template<typename T> T
sat_sub(T x, T y, bool* sat)
{
    using UT = std::make_unsigned_t<T>;
    UT ux = x;
    UT uy = y;
    UT res = ux - uy;

    int sh = sizeof(T) * 8 - 1;

    ux = (ux >> sh) + (((UT)0x1 << sh) - 1);

    if ((T) ((ux ^ uy) & (ux ^ res)) < 0) {
    res = ux;
    *sat = true;
    }
    return res;
}

template<typename T> T
sat_addu(T x, T y, bool* sat)
{
    T res = x + y;

    bool t = res < x;
    if (false == *sat){
    *sat = t;
    }
    res |= -(res < x);

    return res;
}

template<typename T> T
sat_subu(T x, T y, bool* sat)
{
    T res = x - y;

    bool t = !(res <= x);
    if (false == *sat){
    *sat = t;
    }

    res &= -(res <= x);

    return res;
}

/**
 * Ref:
 * https://github.com/riscv-software-src/riscv-isa-sim
 */
template<typename T> T
int_rounding(T result, uint8_t xrm, unsigned gb) {
    const uint64_t lsb = 1UL << gb;
    const uint64_t lsb_half = lsb >> 1;
    switch (xrm) {
    case 0 /* RNU */:
        result += lsb_half;
        break;
    case 1 /* RNE */:
        if ((result & lsb_half) &&
            ((result & (lsb_half - 1)) || (result & lsb)))
            result += lsb;
        break;
    case 2 /* RDN */:
        break;
    case 3 /* ROD */:
        if (result & (lsb - 1))
            result |= lsb;
        break;
    default:
        panic("Invalid xrm value %d", (int)xrm);
    }

    return result;
}

} // namespace RiscvISA
} // namespace gem5

#endif // __ARCH_RISCV_UTILITY_HH__
