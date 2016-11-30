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

namespace D
{

constexpr inline uint64_t
bits(double d)
{
    return reinterpret_cast<uint64_t&>(d);
}

constexpr inline double
number(uint64_t b)
{
    return reinterpret_cast<double&>(b);
}

inline bool
isquietnan(double f)
{
    return std::isnan(f) && (bits(f)&0x0008000000000000ULL) != 0;
}

inline bool
issignalingnan(double f)
{
    return std::isnan(f) && (bits(f)&0x0008000000000000ULL) == 0;
}

inline double
load(double mem)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fld %0,%1"
        : "=f" (fd)
        : "m" (mem));
    return fd;
}

inline double
store(double fs)
{
    double mem = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fsd %1,%0" : "=m" (mem) : "f" (fs));
    return mem;
}

inline double
fmadd_d(double fs1, double fs2, double fs3)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FR4OP("fmadd.d", fd, fs1, fs2, fs3);
    return fd;
}

inline double
fmsub_d(double fs1, double fs2, double fs3)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FR4OP("fmsub.d", fd, fs1, fs2, fs3);
    return fd;
}

inline double
fnmsub_d(double fs1, double fs2, double fs3)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FR4OP("fnmsub.d", fd, fs1, fs2, fs3);
    return fd;
}

inline double
fnmadd_d(double fs1, double fs2, double fs3)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FR4OP("fnmadd.d", fd, fs1, fs2, fs3);
    return fd;
}

inline double
fadd_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fadd.d", fd, fs1, fs2);
    return fd;
}

inline double
fsub_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fsub.d", fd, fs1, fs2);
    return fd;
}

inline double
fmul_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fmul.d", fd, fs1, fs2);
    return fd;
}

inline double
fdiv_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fdiv.d", fd, fs1, fs2);
    return fd;
}

inline double
fsqrt_d(double fs1)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fsqrt.d %0,%1" : "=f" (fd) : "f" (fs1));
    return fd;
}

inline double
fsgnj_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fsgnj.d", fd, fs1, fs2);
    return fd;
}

inline double
fsgnjn_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fsgnjn.d", fd, fs1, fs2);
    return fd;
}

inline double
fsgnjx_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fsgnjx.d", fd, fs1, fs2);
    return fd;
}

inline double
fmin_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fmin.d", fd, fs1, fs2);
    return fd;
}

inline double
fmax_d(double fs1, double fs2)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    FROP("fmax.d", fd, fs1, fs2);
    return fd;
}

inline float
fcvt_s_d(double fs1)
{
    float fd = std::numeric_limits<float>::signaling_NaN();
    asm volatile("fcvt.s.d %0,%1" : "=f" (fd) : "f" (fs1));
    return fd;
}

inline double
fcvt_d_s(float fs1)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fcvt.d.s %0,%1" : "=f" (fd) : "f" (fs1));
    return fd;
}

inline bool
feq_d(double fs1, double fs2)
{
    bool rd = false;
    asm volatile("feq.d %0,%1,%2" : "=r" (rd) : "f" (fs1), "f" (fs2));
    return rd;
}

inline bool
flt_d(double fs1, double fs2)
{
    bool rd = false;
    asm volatile("flt.d %0,%1,%2" : "=r" (rd) : "f" (fs1), "f" (fs2));
    return rd;
}

inline bool
fle_d(double fs1, double fs2)
{
    bool rd = false;
    asm volatile("fle.d %0,%1,%2" : "=r" (rd) : "f" (fs1), "f" (fs2));
    return rd;
}

inline uint64_t
fclass_d(double fs1)
{
    uint64_t rd = -1;
    asm volatile("fclass.d %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline int64_t
fcvt_w_d(double fs1)
{
    int64_t rd = 0;
    asm volatile("fcvt.w.d %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline uint64_t
fcvt_wu_d(double fs1)
{
    uint64_t rd = 0;
    asm volatile("fcvt.wu.d %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline float
fcvt_d_w(int64_t rs1)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fcvt.d.w %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline double
fcvt_d_wu(uint64_t rs1)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fcvt.d.wu %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline int64_t
fcvt_l_d(double fs1)
{
    int64_t rd = 0;
    asm volatile("fcvt.l.d %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline uint64_t
fcvt_lu_d(double fs1)
{
    uint64_t rd = 0;
    asm volatile("fcvt.lu.d %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline uint64_t
fmv_x_d(double fs1)
{
    uint64_t rd = 0;
    asm volatile("fmv.x.d %0,%1" : "=r" (rd) : "f" (fs1));
    return rd;
}

inline double
fcvt_d_l(int64_t rs1)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fcvt.d.l %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline double
fcvt_d_lu(uint64_t rs1)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fcvt.d.lu %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

inline double
fmv_d_x(uint64_t rs1)
{
    double fd = std::numeric_limits<double>::signaling_NaN();
    asm volatile("fmv.d.x %0,%1" : "=f" (fd) : "r" (rs1));
    return fd;
}

} // namespace D
