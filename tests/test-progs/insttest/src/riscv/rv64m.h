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
 */

#pragma once

#include <cstdint>

#include "insttest.h"

namespace M
{

inline int64_t
mul(int64_t rs1, int64_t rs2)
{
    int64_t rd = 0;
    ROP("mul", rd, rs1, rs2);
    return rd;
}

inline int64_t
mulh(int64_t rs1, int64_t rs2)
{
    int64_t rd = 0;
    ROP("mulh", rd, rs1, rs2);
    return rd;
}

inline int64_t
mulhsu(int64_t rs1, uint64_t rs2)
{
    int64_t rd = 0;
    ROP("mulhsu", rd, rs1, rs2);
    return rd;
}

inline uint64_t
mulhu(uint64_t rs1, uint64_t rs2)
{
    uint64_t rd = 0;
    ROP("mulhu", rd, rs1, rs2);
    return rd;
}

inline int64_t
div(int64_t rs1, int64_t rs2)
{
    int64_t rd = 0;
    ROP("div", rd, rs1, rs2);
    return rd;
}

inline uint64_t
divu(uint64_t rs1, uint64_t rs2)
{
    uint64_t rd = 0;
    ROP("divu", rd, rs1, rs2);
    return rd;
}

inline int64_t
rem(int64_t rs1, int64_t rs2)
{
    int64_t rd = 0;
    ROP("rem", rd, rs1, rs2);
    return rd;
}

inline uint64_t
remu(uint64_t rs1, uint64_t rs2)
{
    uint64_t rd = 0;
    ROP("remu", rd, rs1, rs2);
    return rd;
}

inline int64_t
mulw(int64_t rs1, int64_t rs2)
{
    int64_t rd = 0;
    ROP("mulw", rd, rs1, rs2);
    return rd;
}

inline int64_t
divw(int64_t rs1, int64_t rs2)
{
    int64_t rd = 0;
    ROP("divw", rd, rs1, rs2);
    return rd;
}

inline uint64_t
divuw(uint64_t rs1, uint64_t rs2)
{
    uint64_t rd = 0;
    ROP("divuw", rd, rs1, rs2);
    return rd;
}

inline int64_t
remw(int64_t rs1, int64_t rs2)
{
    int64_t rd = 0;
    ROP("remw", rd, rs1, rs2);
    return rd;
}

inline uint64_t
remuw(uint64_t rs1, uint64_t rs2)
{
    uint64_t rd = 0;
    ROP("remuw", rd, rs1, rs2);
    return rd;
}

} // namespace M
