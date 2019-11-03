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

#include <cstdint>
#include <limits>

#include "insttest.h"
#include "rv64a.h"

int main()
{
    using namespace std;
    using namespace insttest;

    // Memory (LR.W, SC.W)
    expect<pair<int64_t, int64_t>>({-1, 256}, []{
            int32_t mem = -1;
            int64_t rs2 = 256;
            int64_t rd;
            pair<int64_t, uint64_t> result;
            do {
                rd = A::lr_w(mem);
                result = A::sc_w(rs2, mem);
            } while (result.second == 1);
            return pair<int64_t, uint64_t>(rd, result.first);
        }, "lr.w/sc.w");
    expect<pair<bool, int64_t>>({true, 200}, []{
            int32_t mem = 200;
            pair<int64_t, uint64_t> result = A::sc_w(50, mem);
            return pair<bool, int64_t>(result.second == 1, mem);
        }, "sc.w, no preceding lr.d");

    // AMOSWAP.W
    expect<pair<int64_t, int64_t>>({65535, 255},
            []{return A::amoswap_w(255, 65535);}, "amoswap.w");
    expect<pair<int64_t, int64_t>>({0xFFFFFFFF, -1},
            []{return A::amoswap_w(0xFFFFFFFF, 0xFFFFFFFF);},
            "amoswap.w, sign extend");
    expect<pair<int64_t, int64_t>>({0x0000000180000000LL, -1},
            []{return A::amoswap_w(0x00000001FFFFFFFFLL,
                    0x7FFFFFFF80000000LL);},
            "amoswap.w, truncate");

    // AMOADD.W
    expect<pair<int64_t, int64_t>>({256, 255},
            []{return A::amoadd_w(255, 1);}, "amoadd.w");
    expect<pair<int64_t, int64_t>>({0, -1},
            []{return A::amoadd_w(0xFFFFFFFF, 1);},
            "amoadd.w, truncate/overflow");
    expect<pair<int64_t, int64_t>>({0xFFFFFFFF, 0x7FFFFFFF},
            []{return A::amoadd_w(0x7FFFFFFF, 0x80000000);},
            "amoadd.w, sign extend");

    // AMOXOR.W
    expect<pair<uint64_t, uint64_t>>({0xFFFFFFFFAAAAAAAALL, -1},
            []{return A::amoxor_w(-1, 0x5555555555555555LL);},
            "amoxor.w, truncate");
    expect<pair<uint64_t, uint64_t>>({0x80000000, -1},
            []{return A::amoxor_w(0xFFFFFFFF, 0x7FFFFFFF);},
            "amoxor.w, sign extend");

    // AMOAND.W
    expect<pair<uint64_t, uint64_t>>({0xFFFFFFFF00000000LL, -1},
            []{return A::amoand_w(-1, 0);}, "amoand.w, truncate");
    expect<pair<uint64_t, uint64_t>>({0x0000000080000000LL, -1},
            []{return A::amoand_w(0xFFFFFFFF,numeric_limits<int32_t>::min());},
            "amoand.w, sign extend");

    // AMOOR.W
    expect<pair<uint64_t, uint64_t>>({0x00000000FFFFFFFFLL, 0},
            []{return A::amoor_w(0, -1);}, "amoor.w, truncate");
    expect<pair<uint64_t, uint64_t>>({0x0000000080000000LL, 0},
            []{return A::amoor_w(0, numeric_limits<int32_t>::min());},
            "amoor.w, sign extend");

    // AMOMIN.W
    expect<pair<int64_t, int64_t>>({0x7FFFFFFF00000001LL, 1},
            []{return A::amomin_w(0x7FFFFFFF00000001LL, 0xFFFFFFFF000000FF);},
            "amomin.w, truncate");
    expect<pair<int64_t, int64_t>>({0x00000000FFFFFFFELL, -1},
            []{return A::amomin_w(0xFFFFFFFF, -2);}, "amomin.w, sign extend");

    // AMOMAX.W
    expect<pair<int64_t, int64_t>>({0x70000000000000FFLL, 1},
            []{return A::amomax_w(0x7000000000000001LL,0x7FFFFFFF000000FFLL);},
            "amomax.w, truncate");
    expect<pair<int64_t, int64_t>>({-1, numeric_limits<int32_t>::min()},
            []{return A::amomax_w(numeric_limits<int32_t>::min(), -1);},
            "amomax.w, sign extend");

    // AMOMINU.W
    expect<pair<uint64_t, uint64_t>>({0x0FFFFFFF000000FFLL, -1},
            []{return A::amominu_w(0x0FFFFFFFFFFFFFFFLL, 0xFFFFFFFF000000FF);},
            "amominu.w, truncate");
    expect<pair<uint64_t, uint64_t>>({0x0000000080000000LL, -1},
            []{return A::amominu_w(0x00000000FFFFFFFFLL, 0x80000000);},
            "amominu.w, sign extend");

    // AMOMAXU.W
    expect<pair<uint64_t, uint64_t>>({-1, 0},
            []{return A::amomaxu_w(0xFFFFFFFF00000000LL,
                    0x00000000FFFFFFFFLL);},
            "amomaxu.w, truncate");
    expect<pair<uint64_t, uint64_t>>(
            {0xFFFFFFFF, numeric_limits<int32_t>::min()},
            []{return A::amomaxu_w(0x80000000, 0xFFFFFFFF);},
            "amomaxu.w, sign extend");

    // Memory (LR.D, SC.D)
    expect<pair<int64_t, int64_t>>({-1, 256}, []{
            int64_t mem = -1;
            int64_t rs2 = 256;
            int64_t rd;
            pair<int64_t, uint64_t> result;
            do {
                rd = A::lr_d(mem);
                result = A::sc_d(rs2, mem);
            } while (result.second == 1);
            return pair<int64_t, uint64_t>(rd, result.first);
        }, "lr.d/sc.d");
    expect<pair<bool, int64_t>>({true, 200}, []{
            int64_t mem = 200;
            pair<int64_t, uint64_t> result = A::sc_d(50, mem);
            return pair<bool, int64_t>(result.second == 1, mem);
        }, "sc.d, no preceding lr.d");

    // AMOSWAP.D
    expect<pair<int64_t, int64_t>>({1, -1}, []{return A::amoswap_d(-1, 1);},
            "amoswap.d");

    // AMOADD.D
    expect<pair<int64_t, int64_t>>({0x7000000000000000LL,0x0FFFFFFFFFFFFFFFLL},
            []{return A::amoadd_d(0x0FFFFFFFFFFFFFFFLL,0x6000000000000001LL);},
            "amoadd.d");
    expect<pair<int64_t, int64_t>>({0, 0x7FFFFFFFFFFFFFFFLL},
            []{return A::amoadd_d(0x7FFFFFFFFFFFFFFFLL,0x8000000000000001LL);},
            "amoadd.d, overflow");

    // AMOXOR.D
    expect<pair<int64_t, int64_t>>({-1, 0xAAAAAAAAAAAAAAAALL},
            []{return A::amoxor_d(0xAAAAAAAAAAAAAAAALL,0x5555555555555555LL);},
            "amoxor.d (1)");
    expect<pair<int64_t, int64_t>>({0, 0xAAAAAAAAAAAAAAAALL},
            []{return A::amoxor_d(0xAAAAAAAAAAAAAAAALL,0xAAAAAAAAAAAAAAAALL);},
            "amoxor.d (0)");

    // AMOAND.D
    expect<pair<int64_t, int64_t>>({0xAAAAAAAAAAAAAAAALL, -1},
            []{return A::amoand_d(-1, 0xAAAAAAAAAAAAAAAALL);}, "amoand.d");

    // AMOOR.D
    expect<pair<int64_t, int64_t>>({-1, 0xAAAAAAAAAAAAAAAALL},
            []{return A::amoor_d(0xAAAAAAAAAAAAAAAALL, 0x5555555555555555LL);},
            "amoor.d");

    // AMOMIN.D
    expect<pair<int64_t, int64_t>>({-1, -1},
            []{return A::amomin_d(-1, 0);}, "amomin.d");

    // AMOMAX.D
    expect<pair<int64_t, int64_t>>({0, -1}, []{return A::amomax_d(-1, 0);},
            "amomax.d");

    // AMOMINU.D
    expect<pair<uint64_t, uint64_t>>({0, -1},
            []{return A::amominu_d(-1, 0);}, "amominu.d");

    // AMOMAXU.D
    expect<pair<uint64_t, uint64_t>>({-1, -1}, []{return A::amomaxu_d(-1, 0);},
            "amomaxu.d");

    return 0;
}
