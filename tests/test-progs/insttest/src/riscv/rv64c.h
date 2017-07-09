/*
 * Copyright (c) 2017 The University of Virginia
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
#include <type_traits>

#include "insttest.h"

#define CIOP(op, r, imm) asm volatile(op " %0,%1" : "+r" (r) : "i" (imm));
#define CROP(op, rd, rs) asm volatile(op " %0,%1" : "+r" (rd) : "r" (rs))

namespace C
{

inline int64_t
c_li(const int8_t imm)
{
    int64_t rd = 0;
    CIOP("c.li", rd, imm);
    return rd;
}

inline int64_t
c_lui(const int8_t imm)
{
    int64_t rd = 0;
    CIOP("c.lui", rd, imm);
    return rd;
}

inline int64_t
c_addi(int64_t r, const int8_t imm)
{
    CIOP("c.addi", r, imm);
    return r;
}

inline int64_t
c_addiw(int64_t r, const int8_t imm)
{
    CIOP("c.addiw", r, imm);
    return r;
}

inline uint64_t
c_addi4spn(const int16_t imm)
{
    uint64_t rd = 0;
    asm volatile("c.addi4spn %0,sp,%1" : "=r" (rd) : "i" (imm));
    return rd;
}

inline uint64_t
c_slli(uint64_t r, uint8_t shamt)
{
    CIOP("c.slli", r, shamt);
    return r;
}

inline uint64_t
c_srli(uint64_t r, uint8_t shamt)
{
    CIOP("c.srli", r, shamt);
    return r;
}

inline int64_t
c_srai(int64_t r, uint8_t shamt)
{
    CIOP("c.srai", r, shamt);
    return r;
}

inline uint64_t
c_andi(uint64_t r, uint8_t imm)
{
    CIOP("c.andi", r, imm);
    return r;
}

inline int64_t
c_mv(int64_t rs)
{
    int64_t rd = 0;
    CROP("c.mv", rd, rs);
    return rd;
}

inline int64_t
c_add(int64_t rd, int64_t rs)
{
    CROP("c.add", rd, rs);
    return rd;
}

inline uint64_t
c_and(int64_t rd, int64_t rs)
{
    CROP("c.and", rd, rs);
    return rd;
}

inline uint64_t
c_or(int64_t rd, int64_t rs)
{
    CROP("c.or", rd, rs);
    return rd;
}

inline uint64_t
c_xor(int64_t rd, int64_t rs)
{
    CROP("c.xor", rd, rs);
    return rd;
}

inline int64_t
c_sub(int64_t rd, int64_t rs)
{
    CROP("c.sub", rd, rs);
    return rd;
}

inline int64_t
c_addw(int64_t rd, int64_t rs)
{
    CROP("c.addw", rd, rs);
    return rd;
}

inline int64_t
c_subw(int64_t rd, int64_t rs)
{
    CROP("c.subw", rd, rs);
    return rd;
}

template<typename M, typename R> inline R
c_load(M m)
{
    R r = 0;
    switch (sizeof(M))
    {
      case 4:
        asm volatile("c.lw %0,0(%1)" : "=r" (r) : "r" (&m) : "memory");
        break;
      case 8:
        if (std::is_floating_point<M>::value)
            asm volatile("c.fld %0,0(%1)" : "=f" (r) : "r" (&m) : "memory");
        else
            asm volatile("c.ld %0,0(%1)" : "=r" (r) : "r" (&m) : "memory");
        break;
    }
    return r;
}

template<typename M> inline M
c_store(const M& rs)
{
    M mem = 0;
    switch (sizeof(M))
    {
      case 4:
        asm volatile("c.sw %0,0(%1)" : : "r" (rs), "r" (&mem) : "memory");
        break;
      case 8:
        if (std::is_floating_point<M>::value)
            asm volatile("c.fsd %0,0(%1)" : : "f" (rs), "r" (&mem) : "memory");
        else
            asm volatile("c.sd %0,0(%1)" : : "r" (rs), "r" (&mem) : "memory");
        break;
    }
    return mem;
}

inline bool
c_j()
{
    asm volatile goto("c.j %l[jallabel]" : : : : jallabel);
    return false;
  jallabel:
    return true;
}

inline bool
c_jr()
{
    uint64_t a = 0;
    asm volatile("auipc %0,0;"
                 "c.addi %0,12;"
                 "c.jr %0;"
                 "addi %0,zero,0;"
                 "addi %0,%0,0;"
                 : "+r" (a));
    return a > 0;
}

inline bool
c_jalr()
{
    int64_t a = 0;
    asm volatile("auipc %0,0;"
                 "c.addi %0,12;"
                 "c.jalr %0;"
                 "addi %0,zero,0;"
                 "sub %0,ra,%0;"
                 : "+r" (a)
                 :
                 : "ra");
    return a == -4;
}

inline bool
c_beqz(int64_t a)
{
    asm volatile goto("c.beqz %0,%l[beqlabel]"
            :
            : "r" (a)
            :
            : beqlabel);
    return false;
  beqlabel:
    return true;
}

inline bool
c_bnez(int64_t a)
{
    asm volatile goto("c.bnez %0,%l[beqlabel]"
            :
            : "r" (a)
            :
            : beqlabel);
    return false;
  beqlabel:
    return true;
}

} // namespace C