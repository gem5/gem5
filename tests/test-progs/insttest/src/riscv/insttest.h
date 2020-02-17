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

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>

#define IOP(inst, rd, rs1, imm) \
    asm volatile(inst " %0,%1,%2" : "=r" (rd) : "r" (rs1), "i" (imm))

#define ROP(inst, rd, rs1, rs2) \
    asm volatile(inst " %0,%1,%2" : "=r" (rd) : "r" (rs1), "r" (rs2))

#define FROP(inst, fd, fs1, fs2) \
    asm volatile(inst " %0,%1,%2" : "=f" (fd) : "f" (fs1), "f" (fs2))

#define FR4OP(inst, fd, fs1, fs2, fs3) \
    asm volatile(inst " %0,%1,%2,%3" \
            : "=f" (fd) \
            : "f" (fs1), "f" (fs2), "f" (fs3))

template<typename A, typename B> std::ostream&
operator<<(std::ostream& os, const std::pair<A, B>& p)
{
    return os << '(' << p.first << ", " << p.second << ')';
}

namespace insttest
{

template<typename T> void
expect(const T& expected, std::function<T()> func,
        const std::string& test)
{
    using namespace std;

    T result = func();
    cout << test << ": ";
    if (result == expected) {
        cout << "PASS" << endl;
    } else {
        cout << "\033[1;31mFAIL\033[0m (expected " << expected << "; found " <<
            result << ")" << endl;
    }
}

} // namespace insttest
