/*
 * Copyright (c) 2016 The University of Virginia
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
 *
 * Authors: Alec Roelke
 */

#pragma once

#include <cstdint>
#include <limits>

#include "insttest.h"

namespace F
{

constexpr inline uint32_t
bits(float f)
{
    return reinterpret_cast<uint32_t&>(f);
}

constexpr inline float
number(uint32_t b)
{
    return reinterpret_cast<float&>(b);
}

inline bool
isquietnan(float f)
{
    return std::isnan(f) && (bits(f)&0x00400000) != 0;
}

inline bool
issignalingnan(float f)
{
    return std::isnan(f) && (bits(f)&0x00400000) == 0;
}

inline float
load(float mem)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    asm volatile("flw %0,%1"
        : "=f" (fd)
        : "m" (mem));
    return fd;
}

inline float
store(float fs)
{
    float mem = std::numeric_limits<float>::signaling_NaN();
    asm volatile("fsw %1,%0" : "=m" (mem) : "f" (fs));
    return mem;
}

inline uint64_t
frflags()
{
    uint64_t rd = -1;
    asm volatile("frflags %0" : "=r" (rd));
    return rd;
}

inline uint64_t
fsflags(uint64_t rs1)
{
    uint64_t rd = -1;
    asm volatile("fsflags %0,%1" : "=r" (rd) : "r" (rs1));
    return rd;
}

inline float
fmadd_s(float fs1, float fs2, float fs3)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FR4OP("fmadd.s", fd, fs1, fs2, fs3);
    return fd;
}

inline float
fmsub_s(float fs1, float fs2, float fs3)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FR4OP("fmsub.s", fd, fs1, fs2, fs3);
    return fd;
}

inline float
fnmsub_s(float fs1, float fs2, float fs3)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FR4OP("fnmsub.s", fd, fs1, fs2, fs3);
    return fd;
}

inline float
fnmadd_s(float fs1, float fs2, float fs3)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FR4OP("fnmadd.s", fd, fs1, fs2, fs3);
    return fd;
}

inline float
fadd_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fadd.s", fd, fs1, fs2);
    return fd;
}

inline float
fsub_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fsub.s", fd, fs1, fs2);
    return fd;
}

inline float
fmul_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fmul.s", fd, fs1, fs2);
    return fd;
}

inline float
fdiv_s(float fs1, float fs2)
{

    float fd = 0.0;
    FROP("fdiv.s", fd, fs1, fs2);
    return fd;
}

inline float
fsqrt_s(float fs1)
{
    float fd = std::numeric_limits<float>::infinity();
    asm volatile("fsqrt.s %0,%1" : "=f" (fd) : "f" (fs1));
    return fd;
}

inline float
fsgnj_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fsgnj.s", fd, fs1, fs2);
    return fd;
}

inline float
fsgnjn_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fsgnjn.s", fd, fs1, fs2);
    return fd;
}

inline float
fsgnjx_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fsgnjx.s", fd, fs1, fs2);
    return fd;
}

inline float
fmin_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fmin.s", fd, fs1, fs2);
    return fd;
}

inline float
fmax_s(float fs1, float fs2)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    FROP("fmax.s", fd, fs1, fs2);
    return fd;
}

inline int64_t
fcvt_w_s(float fs1)
{
    int64_t rd = 0;
    asm volatile("fcvt.w.s %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline uint64_t
fcvt_wu_s(float fs1)
{
    uint64_t rd = 0;
    asm volatile("fcvt.wu.s %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline uint64_t
fmv_x_s(float fs1)
{
    uint64_t rd = 0;
    asm volatile("fmv.x.s %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline bool
feq_s(float fs1, float fs2)
{
    bool rd = false;
    asm volatile("feq.s %0,%1,%2" : "=r" (rd) : "f" (fs1), "f" (fs2));
    return rd;
}

inline bool
flt_s(float fs1, float fs2)
{
    bool rd = false;
    asm volatile("flt.s %0,%1,%2" : "=r" (rd) : "f" (fs1), "f" (fs2));
    return rd;
}

inline bool
fle_s(float fs1, float fs2)
{
    bool rd = false;
    asm volatile("fle.s %0,%1,%2" : "=r" (rd) : "f" (fs1), "f" (fs2));
    return rd;
}

inline uint64_t
fclass_s(float fs1)
{
    uint64_t rd = -1;
    asm volatile("fclass.s %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline float
fcvt_s_w(int64_t rs1)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    asm volatile("fcvt.s.w %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline float
fcvt_s_wu(uint64_t rs1)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    asm volatile("fcvt.s.wu %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline float
fmv_s_x(uint64_t rs1)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    asm volatile("fmv.s.x %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline uint64_t
frcsr()
{
    uint64_t rd = -1;
    asm volatile("frcsr %0" : "=r" (rd));
    return rd;
}

inline uint64_t
frrm()
{
    uint64_t rd = -1;
    asm volatile("frrm %0" : "=r" (rd));
    return rd;
}

inline uint64_t
fscsr(uint64_t rs1)
{
    uint64_t rd = -1;
    asm volatile("fscsr %0,%1" : "=r" (rd) : "r" (rs1));
    return rd;
}

inline uint64_t
fsrm(uint64_t rs1)
{
    uint64_t rd = -1;
    asm volatile("fsrm %0,%1" : "=r" (rd) : "r" (rs1));
    return rd;
}

inline int64_t
fcvt_l_s(float fs1)
{
    int64_t rd = 0;
    asm volatile("fcvt.l.s %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline uint64_t
fcvt_lu_s(float fs1)
{

    int64_t rd = 0;
    asm volatile("fcvt.lu.s %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline float
fcvt_s_l(int64_t rs1)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    asm volatile("fcvt.s.l %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline float
fcvt_s_lu(uint64_t rs1)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    asm volatile("fcvt.s.lu %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

} // namespace F
