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
#include "rv64f.h"

int main()
{
    using namespace std;
    using namespace insttest;

    // FLAGS
    expect<uint64_t>(0, []{
            F::fsflags(0);
            return F::frflags();
        }, "clear fsflags");

    // Memory
    expect<float>(3.14, []{return F::load(3.14);}, "flw");
    expect<float>(1.816, []{return F::store(1.816);}, "fsw");

    // FMADD.S
    expect<float>(7.11624, []{return F::fmadd_s(3.14, 1.816, 1.414);},
            "fmadd.s");
    expect<bool>(true, []{
            float fd = F::fmadd_s(numeric_limits<float>::quiet_NaN(), 3.14,
                    1.816);
            return F::isquietnan(fd);
        }, "fmadd.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fmadd_s(3.14, numeric_limits<float>::signaling_NaN(),
                    1.816);
            return F::isquietnan(fd);
        }, "fmadd.s, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fmadd_s(3.14, numeric_limits<float>::infinity(),
                    1.414);},
            "fmadd.s, infinity");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fmadd_s(3.14, -numeric_limits<float>::infinity(),
                    1.414);},
            "fmadd.s, -infinity");

    // FMSUB.S
    expect<float>(4.28824, []{return F::fmsub_s(3.14, 1.816, 1.414);},
            "fmsub.s");
    expect<bool>(true, []{
            float fd = F::fmsub_s(3.14, numeric_limits<float>::quiet_NaN(),
                    1.816);
            return F::isquietnan(fd);
        }, "fmsub.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fmsub_s(3.14, 1.816,
                    numeric_limits<float>::signaling_NaN());
            return F::isquietnan(fd);
        }, "fmsub.s, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fmsub_s(numeric_limits<float>::infinity(), 1.816,
                    1.414);},
            "fmsub.s, infinity");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fmsub_s(3.14, -numeric_limits<float>::infinity(),
                    1.414);},
            "fmsub.s, -infinity");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fmsub_s(3.14, 1.816,
                    numeric_limits<float>::infinity());},
            "fmsub.s, subtract infinity");

    // FNMSUB.S
    expect<float>(-4.28824, []{return F::fnmsub_s(3.14, 1.816, 1.414);},
            "fnmsub.s");
    expect<bool>(true, []{
            float fd = F::fnmsub_s(3.14, 1.816,
                    numeric_limits<float>::quiet_NaN());
            return F::isquietnan(fd);
        }, "fnmsub.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fnmsub_s(numeric_limits<float>::signaling_NaN(),
                    1.816, 1.414);
            return F::isquietnan(fd);
        }, "fnmsub.s, signaling NaN");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fnmsub_s(numeric_limits<float>::infinity(),
                    1.816, 1.414);},
            "fnmsub.s, infinity");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fnmsub_s(3.14, -numeric_limits<float>::infinity(),
                    1.414);},
            "fnmsub.s, -infinity");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fnmsub_s(3.14, 1.816,
                    numeric_limits<float>::infinity());},
            "fnmsub.s, subtract infinity");

    // FNMADD.S
    expect<float>(-7.11624, []{return F::fnmadd_s(3.14, 1.816, 1.414);},
            "fnmadd.s");
    expect<bool>(true, []{
            float fd = F::fnmadd_s(numeric_limits<float>::quiet_NaN(), 3.14,
                    1.816);
            return F::isquietnan(fd);
        }, "fnmadd.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fnmadd_s(3.14,numeric_limits<float>::signaling_NaN(),
                    1.816);
            return F::isquietnan(fd);
        }, "fnmadd.s, signaling NaN");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fnmadd_s(3.14, numeric_limits<float>::infinity(),
                    1.414);},
            "fnmadd.s, infinity");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fnmadd_s(3.14, -numeric_limits<float>::infinity(),
                    1.414);},
            "fnmadd.s, -infinity");

    // FADD.S
    expect<float>(4.554, []{return F::fadd_s(3.14, 1.414);}, "fadd.s");
    expect<bool>(true, []{
            float fd = F::fadd_s(numeric_limits<float>::quiet_NaN(), 1.414);
            return F::isquietnan(fd);
        }, "fadd.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fadd_s(3.14, numeric_limits<float>::signaling_NaN());
            return F::isquietnan(fd);
        }, "fadd.s, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fadd_s(3.14, numeric_limits<float>::infinity());},
            "fadd.s, infinity");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fadd_s(-numeric_limits<float>::infinity(), 1.816);},
            "fadd.s, -infinity");

    // FSUB.S
    expect<float>(F::number(0xbfdced92), []{return F::fsub_s(1.414, 3.14);},
            "fsub.s");
    expect<bool>(true, []{
            float fd = F::fsub_s(numeric_limits<float>::quiet_NaN(), 1.414);
            return F::isquietnan(fd);
        }, "fsub.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fsub_s(3.14, numeric_limits<float>::signaling_NaN());
            return F::isquietnan(fd);
        }, "fsub.s, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fsub_s(numeric_limits<float>::infinity(), 3.14);},
            "fsub.s, infinity");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fsub_s(-numeric_limits<float>::infinity(), 3.14);},
            "fsub.s, -infinity");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fsub_s(1.414, numeric_limits<float>::infinity());},
            "fsub.s, subtract infinity");

    // FMUL.S
    expect<float>(F::number(0x4024573b), []{return F::fmul_s(1.816, 1.414);},
            "fmul.s");
    expect<bool>(true, []{
            float fd = F::fmul_s(numeric_limits<float>::quiet_NaN(), 1.414);
            return F::isquietnan(fd);
        }, "fmul.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fmul_s(1.816,
                    numeric_limits<float>::signaling_NaN());
            return F::isquietnan(fd);
        }, "fmul.s, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fmul_s(numeric_limits<float>::infinity(), 2.718);},
            "fmul.s, infinity");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fmul_s(2.5966, -numeric_limits<float>::infinity());},
            "fmul.s, -infinity");
    expect<bool>(true, []{
            float fd = F::fmul_s(0.0, numeric_limits<float>::infinity());
            return F::isquietnan(fd);
        }, "fmul.s, 0*infinity");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fmul_s(numeric_limits<float>::max(), 2.0);},
            "fmul.s, overflow");
    expect<float>(0.0,
            []{return F::fmul_s(numeric_limits<float>::min(),
                    numeric_limits<float>::min());},
            "fmul.s, underflow");

    // FDIV.S
    expect<float>(2.5, []{return F::fdiv_s(10.0, 4.0);}, "fdiv.s");
    expect<bool>(true, []{
            float fd = F::fdiv_s(numeric_limits<float>::quiet_NaN(), 4.0);
            return F::isquietnan(fd);
        }, "fdiv.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fdiv_s(10.0, numeric_limits<float>::signaling_NaN());
            return F::isquietnan(fd);
        }, "fdiv.s, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
        []{return F::fdiv_s(10.0, 0.0);}, "fdiv.s/0");
    expect<float>(0.0,
        []{return F::fdiv_s(10.0, numeric_limits<float>::infinity());},
        "fdiv.s/infinity");
    expect<bool>(true, []{
            float fd = F::fdiv_s(numeric_limits<float>::infinity(),
                    numeric_limits<float>::infinity());
            return F::isquietnan(fd);
            }, "fdiv.s, infinity/infinity");
    expect<bool>(true, []{
            float fd = F::fdiv_s(0.0, 0.0);
            return F::isquietnan(fd);
        }, "fdiv.s, 0/0");
    expect<float>(numeric_limits<float>::infinity(),
        []{return F::fdiv_s(numeric_limits<float>::infinity(), 0.0);},
        "fdiv.s, infinity/0");
    expect<float>(0.0,
        []{return F::fdiv_s(0.0, numeric_limits<float>::infinity());},
        "fdiv.s, 0/infinity");
    expect<float>(0.0,
            []{return F::fdiv_s(numeric_limits<float>::min(),
                    numeric_limits<float>::max());},
            "fdiv.s, underflow");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fdiv_s(numeric_limits<float>::max(),
                    numeric_limits<float>::min());},
            "fdiv.s, overflow");

    // FSQRT.S
    expect<float>(0.3, []{return F::fsqrt_s(0.09);}, "fsqrt.s");
    expect<bool>(true, []{
            float fd = F::fsqrt_s(-1.0);
            return F::isquietnan(fd);
        }, "fsqrt.s, NaN");
    expect<bool>(true, []{
            float fd = F::fsqrt_s(numeric_limits<float>::quiet_NaN());
            return F::isquietnan(fd);
        }, "fsqrt.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fsqrt_s(numeric_limits<float>::signaling_NaN());
            return F::isquietnan(fd);
        }, "fsqrt.s, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fsqrt_s(numeric_limits<float>::infinity());},
            "fsqrt.s, infinity");

    // FSGNJ.S
    expect<float>(1.0, []{return F::fsgnj_s(1.0, 25.0);}, "fsgnj.s, ++");
    expect<float>(-1.0, []{return F::fsgnj_s(1.0, -25.0);}, "fsgnj.s, +-");
    expect<float>(1.0, []{return F::fsgnj_s(-1.0, 25.0);}, "fsgnj.s, -+");
    expect<float>(-1.0, []{return F::fsgnj_s(-1.0, -25.0);}, "fsgnj.s, --");
    expect<bool>(true, []{
            float fd = F::fsgnj_s(numeric_limits<float>::quiet_NaN(), -4.0);
            return F::isquietnan(fd);
        }, "fsgnj.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fsgnj_s(numeric_limits<float>::signaling_NaN(),
                    -4.0);
            return F::issignalingnan(fd);
        }, "fsgnj.s, signaling NaN");
    expect<float>(4.0, []{return F::fsgnj_s(4.0,
                numeric_limits<float>::quiet_NaN());}, "fsgnj.s, inject NaN");
    expect<float>(-4.0,
            []{return F::fsgnj_s(4.0, -numeric_limits<float>::quiet_NaN());},
            "fsgnj.s, inject -NaN");

    // FSGNJN.S
    expect<float>(-1.0, []{return F::fsgnjn_s(1.0, 25.0);}, "fsgnjn.s, ++");
    expect<float>(1.0, []{return F::fsgnjn_s(1.0, -25.0);}, "fsgnjn.s, +-");
    expect<float>(-1.0, []{return F::fsgnjn_s(-1.0, 25.0);}, "fsgnjn.s, -+");
    expect<float>(1.0, []{return F::fsgnjn_s(-1.0, -25.0);}, "fsgnjn.s, --");
    expect<bool>(true, []{
            float fd = F::fsgnjn_s(numeric_limits<float>::quiet_NaN(), -4.0);
            return F::isquietnan(fd);
        }, "fsgnjn.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fsgnjn_s(numeric_limits<float>::signaling_NaN(),
                    -4.0);
            return F::issignalingnan(fd);
        }, "fsgnjn.s, signaling NaN");
    expect<float>(-4.0,
            []{return F::fsgnjn_s(4.0, numeric_limits<float>::quiet_NaN());},
            "fsgnjn.s, inject NaN");
    expect<float>(4.0,
            []{return F::fsgnjn_s(4.0, -numeric_limits<float>::quiet_NaN());},
            "fsgnjn.s, inject NaN");

    // FSGNJX.S
    expect<float>(1.0, []{return F::fsgnjx_s(1.0, 25.0);}, "fsgnjx.s, ++");
    expect<float>(-1.0, []{return F::fsgnjx_s(1.0, -25.0);}, "fsgnjx.s, +-");
    expect<float>(-1.0, []{return F::fsgnjx_s(-1.0, 25.0);}, "fsgnjx.s, -+");
    expect<float>(1.0, []{return F::fsgnjx_s(-1.0, -25.0);}, "fsgnjx.s, --");
    expect<bool>(true, []{
            float fd = F::fsgnjx_s(numeric_limits<float>::quiet_NaN(), -4.0);
            return F::isquietnan(fd);
        }, "fsgnjx.s, quiet NaN");
    expect<bool>(true, []{
            float fd = F::fsgnjx_s(numeric_limits<float>::signaling_NaN(),
                    -4.0);
            return F::issignalingnan(fd);
        }, "fsgnjx.s, signaling NaN");
    expect<float>(4.0,
            []{return F::fsgnjx_s(4.0, numeric_limits<float>::quiet_NaN());},
            "fsgnjx.s, inject NaN");
    expect<float>(-4.0,
            []{return F::fsgnjx_s(4.0, -numeric_limits<float>::quiet_NaN());},
            "fsgnjx.s, inject -NaN");

    // FMIN.S
    expect<float>(2.718, []{return F::fmin_s(3.14, 2.718);}, "fmin.s");
    expect<float>(-numeric_limits<float>::infinity(),
            []{return F::fmin_s(-numeric_limits<float>::infinity(),
                    numeric_limits<float>::min());},
            "fmin.s, -infinity");
    expect<float>(numeric_limits<float>::max(),
            []{return F::fmin_s(numeric_limits<float>::infinity(),
                    numeric_limits<float>::max());},
            "fmin.s, infinity");
    expect<float>(-1.414,
            []{return F::fmin_s(numeric_limits<float>::quiet_NaN(), -1.414);},
            "fmin.s, quiet NaN first");
    expect<float>(2.718,
            []{return F::fmin_s(2.718, numeric_limits<float>::quiet_NaN());},
            "fmin.s, quiet NaN second");
    expect<bool>(true, []{
            float fd = F::fmin_s(numeric_limits<float>::quiet_NaN(),
                    numeric_limits<float>::quiet_NaN());
            return F::isquietnan(fd);
        }, "fmin.s, quiet NaN both");
    expect<float>(3.14,
            []{return F::fmin_s(numeric_limits<float>::signaling_NaN(),
                    3.14);},
            "fmin.s, signaling NaN first");
    expect<float>(1.816,
            []{return F::fmin_s(1.816,
                    numeric_limits<float>::signaling_NaN());},
            "fmin.s, signaling NaN second");
    expect<bool>(true, []{
            float fd = F::fmin_s(numeric_limits<float>::signaling_NaN(),
                    numeric_limits<float>::signaling_NaN());
            return F::issignalingnan(fd);
        }, "fmin.s, signaling NaN both");

    // FMAX.S
    expect<float>(3.14, []{return F::fmax_s(3.14, 2.718);}, "fmax.s");
    expect<float>(numeric_limits<float>::min(),
            []{return F::fmax_s(-numeric_limits<float>::infinity(),
                    numeric_limits<float>::min());},
            "fmax.s, -infinity");
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fmax_s(numeric_limits<float>::infinity(),
                    numeric_limits<float>::max());},
            "fmax.s, infinity");
    expect<float>(-1.414,
            []{return F::fmax_s(numeric_limits<float>::quiet_NaN(), -1.414);},
            "fmax.s, quiet NaN first");
    expect<float>(2.718,
            []{return F::fmax_s(2.718, numeric_limits<float>::quiet_NaN());},
            "fmax.s, quiet NaN second");
    expect<bool>(true, []{
            float fd = F::fmax_s(numeric_limits<float>::quiet_NaN(),
                    numeric_limits<float>::quiet_NaN());
            return F::isquietnan(fd);
        }, "fmax.s, quiet NaN both");
    expect<float>(3.14,
            []{return F::fmax_s(numeric_limits<float>::signaling_NaN(),
                    3.14);},
            "fmax.s, signaling NaN first");
    expect<float>(1.816, []{return F::fmax_s(1.816,
                numeric_limits<float>::signaling_NaN());},
            "fmax.s, signaling NaN second");
    expect<bool>(true, []{
            float fd = F::fmax_s(numeric_limits<float>::signaling_NaN(),
                    numeric_limits<float>::signaling_NaN());
            return F::issignalingnan(fd);
        }, "fmax.s, signaling NaN both");

    // FCVT.W.S
    expect<int64_t>(256, []{return F::fcvt_w_s(256.3);},
            "fcvt.w.s, truncate positive");
    expect<int64_t>(-256, []{return F::fcvt_w_s(-256.2);},
            "fcvt.w.s, truncate negative");
    expect<int64_t>(0, []{return F::fcvt_w_s(0.0);}, "fcvt.w.s, 0.0");
    expect<int64_t>(0, []{return F::fcvt_w_s(-0.0);}, "fcvt.w.s, -0.0");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return F::fcvt_w_s(numeric_limits<float>::max());},
            "fcvt.w.s, overflow");
    expect<int64_t>(0, []{return F::fcvt_w_s(numeric_limits<float>::min());},
            "fcvt.w.s, underflow");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return F::fcvt_w_s(numeric_limits<float>::infinity());},
            "fcvt.w.s, infinity");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return F::fcvt_w_s(-numeric_limits<float>::infinity());},
            "fcvt.w.s, -infinity");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return F::fcvt_w_s(numeric_limits<float>::quiet_NaN());},
            "fcvt.w.s, quiet NaN");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return F::fcvt_w_s(-numeric_limits<float>::quiet_NaN());},
            "fcvt.w.s, quiet -NaN");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return F::fcvt_w_s(numeric_limits<float>::signaling_NaN());},
            "fcvt.w.s, signaling NaN");

    // FCVT.WU.S
    expect<uint64_t>(256, []{return F::fcvt_wu_s(256.3);},
            "fcvt.wu.s, truncate positive");
    expect<uint64_t>(0, []{return F::fcvt_wu_s(-256.2);},
            "fcvt.wu.s, truncate negative");
    expect<uint64_t>(0, []{return F::fcvt_wu_s(0.0);}, "fcvt.wu.s, 0.0");
    expect<uint64_t>(0, []{return F::fcvt_wu_s(-0.0);}, "fcvt.wu.s, -0.0");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return F::fcvt_wu_s(numeric_limits<float>::max());},
            "fcvt.wu.s, overflow");
    expect<uint64_t>(0, []{return F::fcvt_wu_s(numeric_limits<float>::min());},
            "fcvt.wu.s, underflow");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return F::fcvt_wu_s(numeric_limits<float>::infinity());},
            "fcvt.wu.s, infinity");
    expect<uint64_t>(0,
            []{return F::fcvt_wu_s(-numeric_limits<float>::infinity());},
            "fcvt.wu.s, -infinity");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return F::fcvt_wu_s(numeric_limits<float>::quiet_NaN());},
            "fcvt.wu.s, quiet NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return F::fcvt_wu_s(-numeric_limits<float>::quiet_NaN());},
            "fcvt.wu.s, quiet -NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return F::fcvt_wu_s(numeric_limits<float>::signaling_NaN());},
            "fcvt.wu.s, signaling NaN");

    // FMV.X.S
    expect<uint64_t>(0x000000004048F5C3ULL, []{return F::fmv_x_s(3.14);},
            "fmv.x.s, positive");
    expect<uint64_t>(0xFFFFFFFFC048F5C3ULL, []{return F::fmv_x_s(-3.14);},
            "fmv.x.s, negative");
    expect<uint64_t>(0x0000000000000000ULL, []{return F::fmv_x_s(0.0);},
            "fmv.x.s, 0.0");
    expect<uint64_t>(0xFFFFFFFF80000000ULL, []{return F::fmv_x_s(-0.0);},
            "fmv.x.s, -0.0");

    // FEQ.S
    expect<bool>(true, []{return F::feq_s(1.414, 1.414);}, "feq.s, equal");
    expect<bool>(false, []{return F::feq_s(2.718, 1.816);},
            "feq.s, not equal");
    expect<bool>(true, []{return F::feq_s(0.0, -0.0);}, "feq.s, 0 == -0");
    expect<bool>(false,
            []{return F::feq_s(numeric_limits<float>::quiet_NaN(), -1.0);},
            "feq.s, quiet NaN first");
    expect<bool>(false,
            []{return F::feq_s(2.0, numeric_limits<float>::quiet_NaN());},
            "feq.s, quiet NaN second");
    expect<bool>(false,
            []{return F::feq_s(numeric_limits<float>::quiet_NaN(),
                    numeric_limits<float>::quiet_NaN());},
            "feq.s, quiet NaN both");
    expect<bool>(false,
            []{return F::feq_s(numeric_limits<float>::signaling_NaN(), -1.0);},
            "feq.s, signaling NaN first");
    expect<bool>(false,
            []{return F::feq_s(2.0, numeric_limits<float>::signaling_NaN());},
            "feq.s, signaling NaN second");
    expect<bool>(false,
            []{return F::feq_s(numeric_limits<float>::signaling_NaN(),
                    numeric_limits<float>::signaling_NaN());},
            "feq.s, signaling NaN both");

    // FLT.S
    expect<bool>(false, []{return F::flt_s(1.414, 1.414);}, "flt.s, equal");
    expect<bool>(true, []{return F::flt_s(1.816, 2.718);}, "flt.s, less");
    expect<bool>(false, []{return F::flt_s(2.718, 1.816);}, "flt.s, greater");
    expect<bool>(false,
            []{return F::flt_s(numeric_limits<float>::quiet_NaN(), -1.0);},
            "flt.s, quiet NaN first");
    expect<bool>(false,
            []{return F::flt_s(2.0, numeric_limits<float>::quiet_NaN());},
            "flt.s, quiet NaN second");
    expect<bool>(false,
            []{return F::flt_s(numeric_limits<float>::quiet_NaN(),
                    numeric_limits<float>::quiet_NaN());},
            "flt.s, quiet NaN both");
    expect<bool>(false,
            []{return F::flt_s(numeric_limits<float>::signaling_NaN(), -1.0);},
            "flt.s, signaling NaN first");
    expect<bool>(false,
            []{return F::flt_s(2.0, numeric_limits<float>::signaling_NaN());},
            "flt.s, signaling NaN second");
    expect<bool>(false,
            []{return F::flt_s(numeric_limits<float>::signaling_NaN(),
                    numeric_limits<float>::signaling_NaN());},
            "flt.s, signaling NaN both");

    // FLE.S
    expect<bool>(true, []{return F::fle_s(1.414, 1.414);}, "fle.s, equal");
    expect<bool>(true, []{return F::fle_s(1.816, 2.718);}, "fle.s, less");
    expect<bool>(false, []{return F::fle_s(2.718, 1.816);}, "fle.s, greater");
    expect<bool>(true, []{return F::fle_s(0.0, -0.0);}, "fle.s, 0 == -0");
    expect<bool>(false,
            []{return F::fle_s(numeric_limits<float>::quiet_NaN(), -1.0);},
            "fle.s, quiet NaN first");
    expect<bool>(false,
            []{return F::fle_s(2.0, numeric_limits<float>::quiet_NaN());},
            "fle.s, quiet NaN second");
    expect<bool>(false,
            []{return F::fle_s(numeric_limits<float>::quiet_NaN(),
                    numeric_limits<float>::quiet_NaN());},
            "fle.s, quiet NaN both");
    expect<bool>(false,
            []{return F::fle_s(numeric_limits<float>::signaling_NaN(), -1.0);},
            "fle.s, signaling NaN first");
    expect<bool>(false,
            []{return F::fle_s(2.0, numeric_limits<float>::signaling_NaN());},
            "fle.s, signaling NaN second");
    expect<bool>(false,
            []{return F::fle_s(numeric_limits<float>::signaling_NaN(),
                    numeric_limits<float>::signaling_NaN());},
            "fle.s, signaling NaN both");

    // FCLASS.S
    expect<uint64_t>(0x1,
            []{return F::fclass_s(-numeric_limits<float>::infinity());},
            "fclass.s, -infinity");
    expect<uint64_t>(0x2, []{return F::fclass_s(-3.14);}, "fclass.s, -normal");
    expect<uint64_t>(0x4, []{return F::fclass_s(F::number(0x807FFFFF));},
            "fclass.s, -subnormal");
    expect<uint64_t>(0x8, []{return F::fclass_s(-0.0);}, "fclass.s, -0.0");
    expect<uint64_t>(0x10, []{return F::fclass_s(0.0);}, "fclass.s, 0.0");
    expect<uint64_t>(0x20, []{return F::fclass_s(F::number(0x007FFFFF));},
            "fclass.s, subnormal");
    expect<uint64_t>(0x40, []{return F::fclass_s(1.816);}, "fclass.s, normal");
    expect<uint64_t>(0x80,
            []{return F::fclass_s(numeric_limits<float>::infinity());},
            "fclass.s, infinity");
    expect<uint64_t>(0x100,
            []{return F::fclass_s(numeric_limits<float>::signaling_NaN());},
            "fclass.s, signaling NaN");
    expect<uint64_t>(0x200,
            []{return F::fclass_s(numeric_limits<float>::quiet_NaN());},
            "fclass.s, quiet NaN");

    // FCVT.S.W
    expect<float>(0.0, []{return F::fcvt_s_w(0);}, "fcvt.s.w, 0");
    expect<float>(-2147483648.0,
            []{return F::fcvt_s_w(numeric_limits<int32_t>::min());},
            "fcvt.s.w, negative");
    expect<float>(255.0, []{return F::fcvt_s_w(0xFFFFFFFF000000FFLL);},
            "fcvt.s.w, truncate");

    // FCVT.S.WU
    expect<float>(0.0, []{return F::fcvt_s_wu(0);}, "fcvt.s.wu, 0");
    expect<float>(2147483648.0,
            []{return F::fcvt_s_wu(numeric_limits<int32_t>::min());},
            "fcvt.s.wu");
    expect<float>(255.0, []{return F::fcvt_s_wu(0xFFFFFFFF000000FFLL);},
            "fcvt.s.wu, truncate");

    // FMV.S.X
    expect<float>(numeric_limits<float>::infinity(),
            []{return F::fmv_s_x(0x7F800000);}, "fmv.s.x");
    expect<float>(-0.0, []{return F::fmv_s_x(0xFFFFFFFF80000000ULL);},
            "fmv.s.x, truncate");

    // FCSR functions
    int rm = F::frrm();
    expect<uint64_t>(0x7, []{ // FSRM
            F::fsrm(-1);
            return F::frrm();
        }, "fsrm");
    expect<uint64_t>(0x1F, []{ // FSFLAGS
            F::fsflags(0);
            F::fsflags(-1);
            return F::frflags();
        }, "fsflags");
    expect<uint64_t>(0xFF, []{ // FSCSR
            F::fsflags(0);
            F::fsrm(0);
            F::fscsr(-1);
            return F::frcsr();
        }, "fscsr");
    expect<int>(rm << 5, [=]{
            F::fscsr(0);
            F::fsrm(rm);
            return F::frcsr();
        }, "restore initial round mode");

    F::fsflags(0);

    // FCVT.L.S
    expect<int64_t>(256, []{return F::fcvt_l_s(256.3);},
            "fcvt.l.s, truncate positive");
    expect<int64_t>(-256, []{return F::fcvt_l_s(-256.2);},
            "fcvt.l.s, truncate negative");
    expect<int64_t>(0, []{return F::fcvt_l_s(0.0);}, "fcvt.l.s, 0.0");
    expect<int64_t>(0, []{return F::fcvt_l_s(-0.0);}, "fcvt.l.s, -0.0");
    expect<int64_t>(-8589934592LL, []{return F::fcvt_l_s(-8589934592.0);},
            "fcvt.l.s, 32-bit overflow");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return F::fcvt_l_s(numeric_limits<float>::max());},
            "fcvt.l.s, overflow");
    expect<int64_t>(0, []{return F::fcvt_l_s(numeric_limits<float>::min());},
            "fcvt.l.s, underflow");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return F::fcvt_l_s(numeric_limits<float>::infinity());},
            "fcvt.l.s, infinity");
    expect<int64_t>(numeric_limits<int64_t>::min(),
            []{return F::fcvt_l_s(-numeric_limits<float>::infinity());},
            "fcvt.l.s, -infinity");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return F::fcvt_l_s(numeric_limits<float>::quiet_NaN());},
            "fcvt.l.s, quiet NaN");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return F::fcvt_l_s(-numeric_limits<float>::quiet_NaN());},
            "fcvt.l.s, quiet -NaN");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return F::fcvt_l_s(numeric_limits<float>::signaling_NaN());},
            "fcvt.l.s, signaling NaN");

    // FCVT.LU.S
    expect<uint64_t>(256, []{return F::fcvt_lu_s(256.3);},
            "fcvt.lu.s, truncate positive");
    expect<uint64_t>(0, []{return F::fcvt_lu_s(-256.2);},
            "fcvt.lu.s, truncate negative");
    expect<uint64_t>(0, []{return F::fcvt_lu_s(0.0);}, "fcvt.lu.s, 0.0");
    expect<uint64_t>(0, []{return F::fcvt_lu_s(-0.0);}, "fcvt.lu.s, -0.0");
    expect<uint64_t>(8589934592LL,
            []{return F::fcvt_lu_s(8589934592.0);},
            "fcvt.lu.s, 32-bit overflow");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return F::fcvt_lu_s(numeric_limits<float>::max());},
            "fcvt.lu.s, overflow");
    expect<uint64_t>(0, []{return F::fcvt_lu_s(numeric_limits<float>::min());},
            "fcvt.lu.s, underflow");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return F::fcvt_lu_s(numeric_limits<float>::infinity());},
            "fcvt.lu.s, infinity");
    expect<uint64_t>(0,
            []{return F::fcvt_lu_s(-numeric_limits<float>::infinity());},
            "fcvt.lu.s, -infinity");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return F::fcvt_lu_s(numeric_limits<float>::quiet_NaN());},
            "fcvt.lu.s, quiet NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return F::fcvt_lu_s(-numeric_limits<float>::quiet_NaN());},
            "fcvt.lu.s, quiet -NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return F::fcvt_lu_s(numeric_limits<float>::signaling_NaN());},
            "fcvt.lu.s, signaling NaN");

    // FCVT.S.L
    expect<float>(0.0, []{return F::fcvt_s_l(0);}, "fcvt.s.l, 0");
    expect<float>(-9.223372e18,
            []{return F::fcvt_s_l(numeric_limits<int64_t>::min());},
            "fcvt.s.l, negative");
    expect<float>(-4.29496704e9, []{return F::fcvt_s_l(0xFFFFFFFF000000FFLL);},
            "fcvt.s.l, 32-bit truncate");

    // FCVT.S.LU
    expect<float>(0.0, []{return F::fcvt_s_lu(0);}, "fcvt.s.lu, 0");
    expect<float>(9.223372e18,
            []{return F::fcvt_s_lu(numeric_limits<int64_t>::min());},
            "fcvt.s.lu");
    expect<float>(1.8446744e19, []{return F::fcvt_s_lu(0xFFFFFFFF000000FFLL);},
            "fcvt.s.lu, 32-bit truncate");

    return 0;
}
