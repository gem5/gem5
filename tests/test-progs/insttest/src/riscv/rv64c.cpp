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

#include <limits>

#include "insttest.h"
#include "rv64c.h"
#include "rv64d.h"

int main()
{
    using namespace insttest;
    using namespace std;

    // C.LWSP
    expect<bool>(true, []{
        uint64_t lw = 0, lwsp = -1;
        int64_t i = 16;
        asm volatile("lw %0,%2(sp);"
                     "c.lwsp %1,%2(sp);"
                     : "=r" (lw), "=r" (lwsp)
                     : "i" (i));
        return lw == lwsp;
    }, "c.lwsp");

    // C.LDSP
    expect<bool>(true, []{
        uint64_t ld = 0, ldsp = -1;
        int64_t i = 8;
        asm volatile("ld %0,%2(sp);"
                     "c.ldsp %1,%2(sp);"
                     : "=r" (ld), "=r" (ldsp)
                     : "i" (i));
        return ld == ldsp;
    }, "c.ldsp");

    // C.FLDSP
    expect<bool>(true, []{
        double fld = 0.0, fldsp = -1.0;
        int64_t i = 32;
        asm volatile("fld %0,%2(sp);"
                     "c.fldsp %1,%2(sp);"
                     : "=f" (fld), "=f" (fldsp)
                     : "i" (i));
        return D::bits(fld) == D::bits(fldsp);
    }, "c.fldsp");

    // C.SWSP
    expect<bool>(true, []{
        int64_t value = -1, result = 0;
        asm volatile("addi sp,sp,-8;"
                     "c.swsp %1,8(sp);"
                     "lw %0,8(sp);"
                     "addi sp,sp,8;"
                     : "=r" (result)
                     : "r" (value)
                     : "memory");
        return value == result;
    }, "c.swsp");

    // C.SDSP
    expect<bool>(true, []{
        int64_t value = -1, result = 0;
        asm volatile("addi sp,sp,-8;"
                     "c.sdsp %1,8(sp);"
                     "ld %0,8(sp);"
                     "addi sp,sp,8;"
                     : "=r" (result)
                     : "r" (value)
                     : "memory");
        return value == result;
    }, "c.sdsp");

    // C.FSDSP
    expect<bool>(true, []{
        double value = 0.1, result = numeric_limits<double>::signaling_NaN();
        asm volatile("addi sp,sp,-8;"
                     "c.fsdsp %1,8(sp);"
                     "fld %0,8(sp);"
                     "addi sp,sp,8;"
                     : "=f" (result)
                     : "f" (value)
                     : "memory");
        return value == result;
    }, "c.fsdsp");

    // C.LW, C.LD, C.FLD
    expect<int64_t>(458752,
            []{return C::c_load<int32_t, int64_t>(0x00070000);},
            "c.lw, positive");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return C::c_load<int32_t, int64_t>(0x80000000);},
            "c.lw, negative");
    expect<int64_t>(30064771072,
            []{return C::c_load<int64_t, int64_t>(30064771072);}, "c.ld");
    expect<double>(3.1415926, []{return C::c_load<double, double>(3.1415926);},
        "c.fld");

    // C.SW, C.SD, C.FSD
    expect<uint32_t>(0xFFFFFFFF, []{return C::c_store<int32_t>(-1);}, "c.sw");
    expect<uint64_t>(-1, []{return C::c_store<int64_t>(-1);}, "c.sd");
    expect<double>(1.61803398875,
            []{return C::c_store<double>(1.61803398875);}, "c.fsd");

    // C.J, C.JR, C.JALR
    expect<bool>(true, []{return C::c_j();}, "c.j");
    expect<bool>(true, []{return C::c_jr();}, "c.jr");
    expect<bool>(true, []{return C::c_jalr();}, "c.jalr");

    // C.BEQZ
    expect<bool>(true, []{return C::c_beqz(0);}, "c.beqz, zero");
    expect<bool>(false, []{return C::c_beqz(7);}, "c.beqz, not zero");

    // C.BNEZ
    expect<bool>(true, []{return C::c_bnez(15);}, "c.bnez, not zero");
    expect<bool>(false, []{return C::c_bnez(0);}, "c.bnez, zero");

    // C.LI
    expect<int64_t>(1, []{return C::c_li(1);}, "c.li");
    expect<int64_t>(-1, []{return C::c_li(-1);}, "c.li, sign extend");

    // C.LUI
    expect<int64_t>(4096, []{return C::c_lui(1);}, "c.lui");
    // Note that sign extension can't be tested here because apparently the
    // compiler doesn't allow the 6th (sign) bit of the immediate to be 1

    // C.ADDI
    expect<int64_t>(15, []{return C::c_addi(7, 8);}, "c.addi");

    // C.ADDIW
    expect<int64_t>(15, []{return C::c_addiw(8, 7);}, "c.addiw");
    expect<int64_t>(1, []{return C::c_addiw(0xFFFFFFFF, 2);},
            "c.addiw, overflow");
    expect<int64_t>(1, []{return C::c_addiw(0x100000001, 0);},
            "c.addiw, truncate");

    // C.ADDI16SP
    expect<bool>(true, []{
        uint64_t sp = 0, rd = 0;
        const int16_t i = 4;
        asm volatile("mv %0,sp;"
                     "c.addi16sp sp,%2;"
                     "mv %1,sp;"
                     "mv sp,%0;"
                     : "+r" (sp), "=r" (rd)
                     : "i" (i*16));
        return rd == sp + i*16;
    }, "c.addi16sp");

    // C.ADDI4SPN
    expect<bool>(true, []{
        uint64_t sp = 0, rd = 0;
        const int16_t i = 3;
        asm volatile("mv %0,sp;"
                     "c.addi4spn %1,sp,%2;"
                     : "=r" (sp), "=r" (rd)
                     : "i" (i*4));
        return rd == sp + i*4;
    }, "c.addi4spn");

    // C.SLLI
    expect<uint64_t>(16, []{return C::c_slli(1, 4);}, "c.slli");
    expect<uint64_t>(0, []{return C::c_slli(8, 61);}, "c.slli, overflow");

    // C.SRLI
    expect<uint64_t>(4, []{return C::c_srli(128, 5);}, "c.srli");
    expect<uint64_t>(0, []{return C::c_srli(128, 8);}, "c.srli, overflow");
    expect<uint64_t>(1, []{return C::c_srli(-1, 63);}, "c.srli, -1");

    // C.SRAI
    expect<uint64_t>(4, []{return C::c_srai(128, 5);}, "c.srai");
    expect<uint64_t>(0, []{return C::c_srai(128, 8);}, "c.srai, overflow");
    expect<uint64_t>(-1, []{return C::c_srai(-2, 63);}, "c.srai, -1");

    // C.ANDI
    expect<uint64_t>(0, []{return C::c_andi(-1, 0);}, "c.andi (0)");
    expect<uint64_t>(0x1234567812345678ULL,
            []{return C::c_andi(0x1234567812345678ULL, -1);}, "c.andi (1)");

    // C.MV
    expect<int64_t>(1024, []{return C::c_mv(1024);}, "c.mv");

    // C.ADD
    expect<int64_t>(15, []{return C::c_add(10, 5);}, "c.add");

    // C.AND
    expect<uint64_t>(0, []{return C::c_and(-1, 0);}, "c.and (0)");
    expect<uint64_t>(0x1234567812345678ULL,
            []{return C::c_and(0x1234567812345678ULL, -1);}, "c.and (-1)");

    // C.OR
    expect<uint64_t>(-1,
            []{return C::c_or(0xAAAAAAAAAAAAAAAAULL,
                    0x5555555555555555ULL);},
            "c.or (1)");
    expect<uint64_t>(0xAAAAAAAAAAAAAAAAULL,
            []{return C::c_or(0xAAAAAAAAAAAAAAAAULL,
                    0xAAAAAAAAAAAAAAAAULL);},
            "c.or (A)");

    // C.XOR
    expect<uint64_t>(-1,
            []{return C::c_xor(0xAAAAAAAAAAAAAAAAULL,
                    0x5555555555555555ULL);},
            "c.xor (1)");
    expect<uint64_t>(0,
            []{return C::c_xor(0xAAAAAAAAAAAAAAAAULL,
                    0xAAAAAAAAAAAAAAAAULL);},
            "c.xor (0)");

    // C.SUB
    expect<int64_t>(65535, []{return C::c_sub(65536, 1);}, "c.sub");

    // C.ADDW
    expect<int64_t>(1073742078, []{return C::c_addw(0x3FFFFFFF, 255);},
            "c.addw");
    expect<int64_t>(-1, []{return C::c_addw(0x7FFFFFFF, 0x80000000);},
            "c.addw, overflow");
    expect<int64_t>(65536, []{return C::c_addw(0xFFFFFFFF0000FFFFLL, 1);},
            "c.addw, truncate");

    // C.SUBW
    expect<int64_t>(65535, []{return C::c_subw(65536, 1);}, "c.subw");
    expect<int64_t>(-1, []{return C::c_subw(0x7FFFFFFF, 0x80000000);},
            "c.subw, \"overflow\"");
    expect<int64_t>(0,
            []{return C::c_subw(0xAAAAAAAAFFFFFFFFULL,0x55555555FFFFFFFFULL);},
            "c.subw, truncate");
}