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

#include <cstdint>
#include <limits>

#include "insttest.h"
#include "rv64m.h"

int main()
{
    using namespace std;
    using namespace insttest;

    // MUL
    expect<int64_t>(39285, []{return M::mul(873, 45);}, "mul");
    expect<int64_t>(0, []{return M::mul(0x4000000000000000LL, 4);},
            "mul, overflow");

    // MULH
    expect<int64_t>(1, []{return M::mulh(0x4000000000000000LL, 4);}, "mulh");
    expect<int64_t>(-1, []{return M::mulh(numeric_limits<int64_t>::min(), 2);},
            "mulh, negative");
    expect<int64_t>(0, []{return M::mulh(-1, -1);}, "mulh, all bits set");

    // MULHSU
    expect<int64_t>(-1, []{return M::mulhsu(-1, -1);}, "mulhsu, all bits set");
    expect<int64_t>(-1,
            []{return M::mulhsu(numeric_limits<int64_t>::min(), 2);},\
            "mulhsu");

    // MULHU
    expect<uint64_t>(1, []{return M::mulhu(0x8000000000000000ULL, 2);},
            "mulhu");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFEULL, []{return M::mulhu(-1, -1);},
            "mulhu, all bits set");

    // DIV
    expect<int64_t>(-7, []{return M::div(-59, 8);}, "div");
    expect<int64_t>(-1, []{return M::div(255, 0);}, "div/0");
    expect<int64_t>(numeric_limits<int64_t>::min(),
            []{return M::div(numeric_limits<int64_t>::min(), -1);},
            "div, overflow");

    // DIVU
    expect<uint64_t>(2305843009213693944LL, []{return M::divu(-59, 8);},
            "divu");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return M::divu(255, 0);}, "divu/0");
    expect<uint64_t>(0,
            []{return M::divu(numeric_limits<uint64_t>::min(), -1);},
            "divu, \"overflow\"");

    // REM
    expect<int64_t>(-3, []{return M::rem(-59, 8);}, "rem");
    expect<int64_t>(255, []{return M::rem(255, 0);}, "rem/0");
    expect<int64_t>(0, []{return M::rem(numeric_limits<int64_t>::min(), -1);},
            "rem, overflow");

    // REMU
    expect<uint64_t>(5, []{return M::remu(-59, 8);}, "remu");
    expect<uint64_t>(255, []{return M::remu(255, 0);}, "remu/0");
    expect<uint64_t>(0x8000000000000000ULL,
            []{return M::remu(0x8000000000000000ULL, -1);},
            "remu, \"overflow\"");

    // MULW
    expect<int64_t>(-100,
            []{return M::mulw(0x7FFFFFFF00000005LL, 0x80000000FFFFFFECLL);},
            "mulw, truncate");
    expect<int64_t>(0, []{return M::mulw(0x40000000, 4);}, "mulw, overflow");

    // DIVW
    expect<int64_t>(-7,
            []{return M::divw(0x7FFFFFFFFFFFFFC5LL, 0xFFFFFFFF00000008LL);},
            "divw, truncate");
    expect<int64_t>(-1, []{return M::divw(65535, 0);}, "divw/0");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return M::divw(numeric_limits<int32_t>::min(), -1);},
            "divw, overflow");

    // DIVUW
    expect<int64_t>(536870904,
            []{return M::divuw(0x7FFFFFFFFFFFFFC5LL, 0xFFFFFFFF00000008LL);},
            "divuw, truncate");
    expect<int64_t>(numeric_limits<uint64_t>::max(),
            []{return M::divuw(65535, 0);}, "divuw/0");
    expect<int64_t>(0,
            []{return M::divuw(numeric_limits<int32_t>::min(), -1);},
            "divuw, \"overflow\"");
    expect<int64_t>(-1,
            []{return M::divuw(numeric_limits<uint32_t>::max(), 1);},
            "divuw, sign extend");

    // REMW
    expect<int64_t>(-3,
            []{return M::remw(0x7FFFFFFFFFFFFFC5LL, 0xFFFFFFFF00000008LL);},
            "remw, truncate");
    expect<int64_t>(65535, []{return M::remw(65535, 0);}, "remw/0");
    expect<int64_t>(0, []{return M::remw(numeric_limits<int32_t>::min(), -1);},
            "remw, overflow");

    // REMUW
    expect<int64_t>(5,
            []{return M::remuw(0x7FFFFFFFFFFFFFC5LL, 0xFFFFFFFF00000008LL);},
            "remuw, truncate");
    expect<int64_t>(65535, []{return M::remuw(65535, 0);}, "remuw/0");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return M::remuw(numeric_limits<int32_t>::min(), -1);},
            "remuw, \"overflow\"");
    expect<int64_t>(0xFFFFFFFF80000000,
            []{return M::remuw(0x80000000, 0xFFFFFFFF);},
            "remuw, sign extend");

    return 0;
}
