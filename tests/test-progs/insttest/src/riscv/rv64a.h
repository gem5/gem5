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
#include <tuple>

#include "insttest.h"

namespace A
{

inline int64_t
lr_w(int32_t& mem)
{
    int64_t r = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("lr.w %0,(%1)" : "=r" (r) : "r" (addr) : "memory");
    return r;
}

inline std::pair<int64_t, uint64_t>
sc_w(int64_t rs2, int32_t& mem)
{
    uint64_t addr = (uint64_t)&mem;
    uint64_t rd = -1;
    asm volatile("sc.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amoswap_w(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoswap.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amoadd_w(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoadd.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amoxor_w(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoxor.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amoand_w(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoand.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amoor_w(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoor.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amomin_w(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amomin.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amomax_w(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amomax.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amominu_w(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amominu.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amomaxu_w(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amomaxu.w %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline int64_t
lr_d(int64_t& mem)
{
    int64_t r = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("lr.d %0,(%1)" : "=r" (r) : "r" (addr) : "memory");
    return r;
}

inline std::pair<int64_t, uint64_t>
sc_d(int64_t rs2, int64_t& mem)
{
    uint64_t addr = (uint64_t)&mem;
    uint64_t rd = -1;
    asm volatile("sc.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amoswap_d(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoswap.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amoadd_d(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoadd.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amoxor_d(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoxor.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amoand_d(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoand.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amoor_d(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amoor.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amomin_d(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amomin.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<int64_t, int64_t>
amomax_d(int64_t mem, int64_t rs2)
{
    int64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amomax.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amominu_d(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amominu.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

inline std::pair<uint64_t, uint64_t>
amomaxu_d(uint64_t mem, uint64_t rs2)
{
    uint64_t rd = 0;
    uint64_t addr = (uint64_t)&mem;
    asm volatile("amomaxu.d %0,%2,(%1)"
            : "=r" (rd)
            : "r" (addr), "r" (rs2)
            : "memory");
    return {mem, rd};
}

} // namespace A
