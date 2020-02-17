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
#include "rv64d.h"
#include "rv64f.h"

int main()
{
    using namespace std;
    using namespace insttest;

    // Memory (FLD, FSD)
    expect<double>(3.1415926, []{return D::load(3.1415926);}, "fld");
    expect<double>(1.61803398875, []{return D::store(1.61803398875);}, "fsd");

    // FMADD.D
    expect<double>(D::number(0x4019FD5AED13B1CEULL),
                []{return D::fmadd_d(3.1415926, 1.61803398875,1.41421356237);},
                "fmadd.d");
    expect<bool>(true, []{
            double fd = D::fmadd_d(numeric_limits<double>::quiet_NaN(), 3.14,
                    1.816);
            return D::isquietnan(fd);
        }, "fmadd.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fmadd_d(3.14,
                    numeric_limits<double>::signaling_NaN(), 1.816);
            return D::isquietnan(fd);
        }, "fmadd.d, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
        []{return D::fmadd_d(3.14, numeric_limits<double>::infinity(),1.414);},
        "fmadd.d, infinity");
    expect<double>(-numeric_limits<double>::infinity(),
        []{return D::fmadd_d(3.14,-numeric_limits<double>::infinity(),1.414);},
        "fmadd.d, -infinity");

    // FMSUB.D
    expect<double>(D::number(0x400d5A1773A85E43ULL),
        []{return D::fmsub_d(3.1415926, 1.61803398875, 1.41421356237);},
        "fmsub.d");
    expect<bool>(true, []{
            double fd = D::fmsub_d(3.14, numeric_limits<double>::quiet_NaN(),
                    1.414);
            return D::isquietnan(fd);
        }, "fmsub.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fmsub_d(3.14, 1.816,
                    numeric_limits<double>::signaling_NaN());
            return D::isquietnan(fd);
        }, "fmsub.d, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fmsub_d(numeric_limits<double>::infinity(), 1.816,
                    1.414);},
            "fmsub.d, infinity");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fmsub_d(3.14, -numeric_limits<double>::infinity(),
                    1.414);},
            "fmsub.d, -infinity");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fmsub_d(3.14, 1.816,
                    numeric_limits<double>::infinity());},
            "fmsub.d, subtract infinity");

    // FNMSUB.D
    expect<double>(D::number(0xC00D5A1773A85E43ULL),
            []{return D::fnmsub_d(3.1415926, 1.61803398875, 1.41421356237);},
            "fnmsub.d");
    expect<bool>(true, []{
            double fd = D::fnmsub_d(3.14, 1.816,
                    numeric_limits<double>::quiet_NaN());
            return D::isquietnan(fd);
        }, "fnmsub.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fnmsub_d(numeric_limits<double>::signaling_NaN(),
                    1.816, 1.414);
            return D::isquietnan(fd);
        }, "fnmsub.d, signaling NaN");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fnmsub_d(numeric_limits<double>::infinity(), 1.816,
                    1.414);},
            "fnmsub.d, infinity");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fnmsub_d(3.14, -numeric_limits<double>::infinity(),
                    1.414);},
            "fnmsub.d, -infinity");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fnmsub_d(3.14, 1.816,
                    numeric_limits<double>::infinity());},
            "fnmsub.d, subtract infinity");

    // FNMADD.D
    expect<double>(D::number(0xC019FD5AED13B1CEULL),
            []{return D::fnmadd_d(3.1415926, 1.61803398875, 1.41421356237);},
            "fnmadd.d");
    expect<bool>(true, []{
            double fd = D::fnmadd_d(numeric_limits<double>::quiet_NaN(), 3.14,
                    1.816);
            return D::isquietnan(fd);
        }, "fnmadd.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fnmadd_d(3.14,
                    numeric_limits<double>::signaling_NaN(), 1.816);
            return D::isquietnan(fd);
        }, "fnmadd.d, signaling NaN");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fnmadd_d(3.14, numeric_limits<double>::infinity(),
                    1.414);},
            "fnmadd.d, infinity");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fnmadd_d(3.14, -numeric_limits<double>::infinity(),
                    1.414);},
            "fnmadd.d, -infinity");

    // FADD.D
    expect<double>(D::number(0x4012392540292D7CULL),
            []{return D::fadd_d(3.1415926, 1.41421356237);}, "fadd.d");
    expect<bool>(true, []{
            double fd = D::fadd_d(numeric_limits<double>::quiet_NaN(), 1.414);
            return D::isquietnan(fd);
        }, "fadd.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fadd_d(3.14,
                    numeric_limits<double>::signaling_NaN());
            return D::isquietnan(fd);
        }, "fadd.d, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fadd_d(3.14, numeric_limits<double>::infinity());},
            "fadd.d, infinity");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fadd_d(-numeric_limits<double>::infinity(), 1.816);},
            "fadd.d, -infinity");

    // FSUB.D
    expect<double>(D::number(0xBFFBA35833AB7AAEULL),
            []{return D::fsub_d(1.4142135623, 3.1415926);}, "fsub.d");
    expect<bool>(true, []{
            double fd = D::fsub_d(numeric_limits<double>::quiet_NaN(), 1.414);
            return D::isquietnan(fd);
        }, "fsub.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fsub_d(3.14,
                    numeric_limits<double>::signaling_NaN());
            return D::isquietnan(fd);
        }, "fsub.d, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fsub_d(numeric_limits<double>::infinity(), 3.14);},
            "fsub.d, infinity");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fsub_d(-numeric_limits<double>::infinity(), 3.14);},
            "fsub.d, -infinity");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fsub_d(1.414, numeric_limits<double>::infinity());},
            "fsub.d, subtract infinity");

    // FMUL.D
    expect<double>(D::number(0x40024E53B708ED9AULL),
            []{return D::fmul_d(1.61803398875, 1.4142135623);}, "fmul.d");
    expect<bool>(true, []{
            double fd = D::fmul_d(numeric_limits<double>::quiet_NaN(), 1.414);
            return D::isquietnan(fd);
        }, "fmul.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fmul_d(1.816,
                    numeric_limits<double>::signaling_NaN());
            return D::isquietnan(fd);
        }, "fmul.d, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fmul_d(numeric_limits<double>::infinity(), 2.718);},
            "fmul.d, infinity");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fmul_d(2.5966, -numeric_limits<double>::infinity());},
            "fmul.d, -infinity");
    expect<bool>(true, []{
            double fd = D::fmul_d(0.0, numeric_limits<double>::infinity());
            return D::isquietnan(fd);
        }, "fmul.d, 0*infinity");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fmul_d(numeric_limits<double>::max(), 2.0);},
            "fmul.d, overflow");
    expect<double>(0.0,
            []{return D::fmul_d(numeric_limits<double>::min(),
                    numeric_limits<double>::min());},
            "fmul.d, underflow");

    // FDIV.D
    expect<double>(2.5, []{return D::fdiv_d(10.0, 4.0);}, "fdiv.d");
    expect<bool>(true, []{
            double fd = D::fdiv_d(numeric_limits<double>::quiet_NaN(), 4.0);
            return D::isquietnan(fd);
        }, "fdiv.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fdiv_d(10.0,
                    numeric_limits<double>::signaling_NaN());
            return D::isquietnan(fd);
        }, "fdiv.d, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fdiv_d(10.0, 0.0);}, "fdiv.d/0");
    expect<double>(0.0,
            []{return D::fdiv_d(10.0, numeric_limits<double>::infinity());},
            "fdiv.d/infinity");
    expect<bool>(true, []{
            double fd = D::fdiv_d(numeric_limits<double>::infinity(),
                    numeric_limits<double>::infinity());
            return D::isquietnan(fd);
        }, "fdiv.d, infinity/infinity");
    expect<bool>(true, []{
            double fd = D::fdiv_d(0.0, 0.0);
            return D::isquietnan(fd);
        }, "fdiv.d, 0/0");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fdiv_d(numeric_limits<double>::infinity(), 0.0);},
            "fdiv.d, infinity/0");
    expect<double>(0.0,
            []{return D::fdiv_d(0.0, numeric_limits<double>::infinity());},
            "fdiv.d, 0/infinity");
    expect<double>(0.0,
            []{return D::fdiv_d(numeric_limits<double>::min(),
                    numeric_limits<double>::max());},
            "fdiv.d, underflow");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fdiv_d(numeric_limits<double>::max(),
                    numeric_limits<double>::min());},
            "fdiv.d, overflow");

    // FSQRT.D
    expect<double>(1e154, []{return D::fsqrt_d(1e308);}, "fsqrt.d");
    expect<bool>(true, []{
            double fd = D::fsqrt_d(-1.0);
            return D::isquietnan(fd);
        }, "fsqrt.d, NaN");
    expect<bool>(true, []{
            double fd = D::fsqrt_d(numeric_limits<double>::quiet_NaN());
            return D::isquietnan(fd);
        }, "fsqrt.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fsqrt_d(numeric_limits<double>::signaling_NaN());
            return D::isquietnan(fd);
        }, "fsqrt.d, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fsqrt_d(numeric_limits<double>::infinity());},
            "fsqrt.d, infinity");

    // FSGNJ.D
    expect<double>(1.0, []{return D::fsgnj_d(1.0, 25.0);}, "fsgnj.d, ++");
    expect<double>(-1.0, []{return D::fsgnj_d(1.0, -25.0);}, "fsgnj.d, +-");
    expect<double>(1.0, []{return D::fsgnj_d(-1.0, 25.0);}, "fsgnj.d, -+");
    expect<double>(-1.0, []{return D::fsgnj_d(-1.0, -25.0);}, "fsgnj.d, --");
    expect<bool>(true, []{
            double fd = D::fsgnj_d(numeric_limits<double>::quiet_NaN(), -4.0);
            return D::isquietnan(fd);
        }, "fsgnj.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fsgnj_d(numeric_limits<double>::signaling_NaN(),
                    -4.0);
            return D::issignalingnan(fd);
        }, "fsgnj.d, signaling NaN");
    expect<double>(4.0,
            []{return D::fsgnj_d(4.0, numeric_limits<double>::quiet_NaN());},
            "fsgnj.d, inject NaN");
    expect<double>(-4.0,
            []{return D::fsgnj_d(4.0, -numeric_limits<double>::quiet_NaN());},
            "fsgnj.d, inject -NaN");

    // FSGNJN.D
    expect<double>(-1.0, []{return D::fsgnjn_d(1.0, 25.0);}, "fsgnjn.d, ++");
    expect<double>(1.0, []{return D::fsgnjn_d(1.0, -25.0);}, "fsgnjn.d, +-");
    expect<double>(-1.0, []{return D::fsgnjn_d(-1.0, 25.0);}, "fsgnjn.d, -+");
    expect<double>(1.0, []{return D::fsgnjn_d(-1.0, -25.0);}, "fsgnjn.d, --");
    expect<bool>(true, []{
            double fd = D::fsgnjn_d(numeric_limits<double>::quiet_NaN(), -4.0);
            return D::isquietnan(fd);
        }, "fsgnjn.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fsgnjn_d(numeric_limits<double>::signaling_NaN(),
                    -4.0);
            return D::issignalingnan(fd);
        }, "fsgnjn.d, signaling NaN");
    expect<double>(-4.0,
            []{return D::fsgnjn_d(4.0, numeric_limits<double>::quiet_NaN());},
            "fsgnjn.d, inject NaN");
    expect<double>(4.0,
            []{return D::fsgnjn_d(4.0, -numeric_limits<double>::quiet_NaN());},
            "fsgnjn.d, inject NaN");

    // FSGNJX.D
    expect<double>(1.0, []{return D::fsgnjx_d(1.0, 25.0);}, "fsgnjx.d, ++");
    expect<double>(-1.0, []{return D::fsgnjx_d(1.0, -25.0);}, "fsgnjx.d, +-");
    expect<double>(-1.0, []{return D::fsgnjx_d(-1.0, 25.0);}, "fsgnjx.d, -+");
    expect<double>(1.0, []{return D::fsgnjx_d(-1.0, -25.0);}, "fsgnjx.d, --");
    expect<bool>(true, []{
            double fd = D::fsgnjx_d(numeric_limits<double>::quiet_NaN(), -4.0);
            return D::isquietnan(fd);
        }, "fsgnjx.d, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fsgnjx_d(numeric_limits<double>::signaling_NaN(),
                    -4.0);
            return D::issignalingnan(fd);
        }, "fsgnjx.d, signaling NaN");
    expect<double>(4.0,
            []{return D::fsgnjx_d(4.0, numeric_limits<double>::quiet_NaN());},
            "fsgnjx.d, inject NaN");
    expect<double>(-4.0,
            []{return D::fsgnjx_d(4.0, -numeric_limits<double>::quiet_NaN());},
            "fsgnjx.d, inject NaN");

    // FMIN.D
    expect<double>(2.718, []{return D::fmin_d(3.14, 2.718);}, "fmin.d");
    expect<double>(-numeric_limits<double>::infinity(),
            []{return D::fmin_d(-numeric_limits<double>::infinity(),
                    numeric_limits<double>::min());},
            "fmin.d, -infinity");
    expect<double>(numeric_limits<double>::max(),
            []{return D::fmin_d(numeric_limits<double>::infinity(),
                    numeric_limits<double>::max());},
            "fmin.d, infinity");
    expect<double>(-1.414,
            []{return D::fmin_d(numeric_limits<double>::quiet_NaN(), -1.414);},
            "fmin.d, quiet NaN first");
    expect<double>(2.718,
            []{return D::fmin_d(2.718, numeric_limits<double>::quiet_NaN());},
            "fmin.d, quiet NaN second");
    expect<bool>(true, []{
            double fd = D::fmin_d(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN());
            return D::isquietnan(fd);
        }, "fmin.d, quiet NaN both");
    expect<double>(3.14,
            []{return D::fmin_d(numeric_limits<double>::signaling_NaN(),
                    3.14);},
            "fmin.d, signaling NaN first");
    expect<double>(1.816,
            []{return D::fmin_d(1.816,
                    numeric_limits<double>::signaling_NaN());},
            "fmin.d, signaling NaN second");
    expect<bool>(true, []{
            double fd = D::fmin_d(numeric_limits<double>::signaling_NaN(),
                    numeric_limits<double>::signaling_NaN());
            return D::issignalingnan(fd);
        }, "fmin.d, signaling NaN both");

    // FMAX.D
    expect<double>(3.14, []{return D::fmax_d(3.14, 2.718);}, "fmax.d");
    expect<double>(numeric_limits<double>::min(),
            []{return D::fmax_d(-numeric_limits<double>::infinity(),
                    numeric_limits<double>::min());},
            "fmax.d, -infinity");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fmax_d(numeric_limits<double>::infinity(),
                    numeric_limits<double>::max());},
            "fmax.d, infinity");
    expect<double>(-1.414,
            []{return D::fmax_d(numeric_limits<double>::quiet_NaN(), -1.414);},
            "fmax.d, quiet NaN first");
    expect<double>(2.718,
            []{return D::fmax_d(2.718, numeric_limits<double>::quiet_NaN());},
            "fmax.d, quiet NaN second");
    expect<bool>(true, []{
            double fd = D::fmax_d(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN());
            return D::isquietnan(fd);
        }, "fmax.d, quiet NaN both");
    expect<double>(3.14,
            []{return D::fmax_d(numeric_limits<double>::signaling_NaN(),
                    3.14);},
            "fmax.d, signaling NaN first");
    expect<double>(1.816,
            []{return D::fmax_d(1.816,
                    numeric_limits<double>::signaling_NaN());},
            "fmax.d, signaling NaN second");
    expect<bool>(true, []{
            double fd = D::fmax_d(numeric_limits<double>::signaling_NaN(),
                    numeric_limits<double>::signaling_NaN());
            return D::issignalingnan(fd);
        }, "fmax.d, signaling NaN both");

    // FCVT.S.D
    expect<float>(4.0, []{return D::fcvt_s_d(4.0);}, "fcvt.s.d");
    expect<bool>(true, []{
            float fd = D::fcvt_s_d(numeric_limits<double>::quiet_NaN());
            return F::isquietnan(fd);
        }, "fcvt.s.d, quiet NaN");
    expect<bool>(true, []{
            float fd = D::fcvt_s_d(numeric_limits<double>::signaling_NaN());
            return F::isquietnan(fd);
        }, "fcvt.s.d, signaling NaN");
    expect<float>(numeric_limits<float>::infinity(),
            []{return D::fcvt_s_d(numeric_limits<double>::infinity());},
            "fcvt.s.d, infinity");
    expect<float>(numeric_limits<float>::infinity(),
            []{return D::fcvt_s_d(numeric_limits<double>::max());},
            "fcvt.s.d, overflow");
    expect<float>(0.0, []{return D::fcvt_s_d(numeric_limits<double>::min());},
            "fcvt.s.d, underflow");

    // FCVT.D.S
    expect<double>(D::number(0x4005BE76C0000000),
            []{return D::fcvt_d_s(2.718);}, "fcvt.d.s");
    expect<bool>(true, []{
            double fd = D::fcvt_d_s(numeric_limits<float>::quiet_NaN());
            return D::isquietnan(fd);
        }, "fcvt.d.s, quiet NaN");
    expect<bool>(true, []{
            double fd = D::fcvt_d_s(numeric_limits<float>::signaling_NaN());
            return D::isquietnan(fd);
        }, "fcvt.d.s, signaling NaN");
    expect<double>(numeric_limits<double>::infinity(),
            []{return D::fcvt_d_s(numeric_limits<float>::infinity());},
            "fcvt.d.s, infinity");

    // FEQ.D
    expect<bool>(true, []{return D::feq_d(1.414, 1.414);}, "feq.d, equal");
    expect<bool>(false,[]{return D::feq_d(2.718, 1.816);}, "feq.d, not equal");
    expect<bool>(true, []{return D::feq_d(0.0, -0.0);}, "feq.d, 0 == -0");
    expect<bool>(false,
            []{return D::feq_d(numeric_limits<double>::quiet_NaN(), -1.0);},
            "feq.d, quiet NaN first");
    expect<bool>(false,
            []{return D::feq_d(2.0, numeric_limits<double>::quiet_NaN());},
            "feq.d, quiet NaN second");
    expect<bool>(false,
            []{return D::feq_d(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN());},
            "feq.d, quiet NaN both");
    expect<bool>(false,
            []{return D::feq_d(numeric_limits<double>::signaling_NaN(),-1.0);},
            "feq.d, signaling NaN first");
    expect<bool>(false,
            []{return D::feq_d(2.0, numeric_limits<double>::signaling_NaN());},
            "feq.d, signaling NaN second");
    expect<bool>(false,
            []{return D::feq_d(numeric_limits<double>::signaling_NaN(),
                    numeric_limits<double>::signaling_NaN());},
            "feq.d, signaling NaN both");

    // FLT.D
    expect<bool>(false, []{return D::flt_d(1.414, 1.414);}, "flt.d, equal");
    expect<bool>(true, []{return D::flt_d(1.816, 2.718);}, "flt.d, less");
    expect<bool>(false, []{return D::flt_d(2.718, 1.816);}, "flt.d, greater");
    expect<bool>(false,
            []{return D::flt_d(numeric_limits<double>::quiet_NaN(), -1.0);},
            "flt.d, quiet NaN first");
    expect<bool>(false,
            []{return D::flt_d(2.0, numeric_limits<double>::quiet_NaN());},
            "flt.d, quiet NaN second");
    expect<bool>(false,
            []{return D::flt_d(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN());},
            "flt.d, quiet NaN both");
    expect<bool>(false,
            []{return D::flt_d(numeric_limits<double>::signaling_NaN(),-1.0);},
            "flt.d, signaling NaN first");
    expect<bool>(false,
            []{return D::flt_d(2.0, numeric_limits<double>::signaling_NaN());},
            "flt.d, signaling NaN second");
    expect<bool>(false,
            []{return D::flt_d(numeric_limits<double>::signaling_NaN(),
                    numeric_limits<double>::signaling_NaN());},
            "flt.d, signaling NaN both");

    // FLE.D
    expect<bool>(true, []{return D::fle_d(1.414, 1.414);}, "fle.d, equal");
    expect<bool>(true, []{return D::fle_d(1.816, 2.718);}, "fle.d, less");
    expect<bool>(false, []{return D::fle_d(2.718, 1.816);}, "fle.d, greater");
    expect<bool>(true, []{return D::fle_d(0.0, -0.0);}, "fle.d, 0 == -0");
    expect<bool>(false,
            []{return D::fle_d(numeric_limits<double>::quiet_NaN(), -1.0);},
            "fle.d, quiet NaN first");
    expect<bool>(false,
            []{return D::fle_d(2.0, numeric_limits<double>::quiet_NaN());},
            "fle.d, quiet NaN second");
    expect<bool>(false,
            []{return D::fle_d(numeric_limits<double>::quiet_NaN(),
                    numeric_limits<double>::quiet_NaN());},
            "fle.d, quiet NaN both");
    expect<bool>(false,
            []{return D::fle_d(numeric_limits<double>::signaling_NaN(),-1.0);},
            "fle.d, signaling NaN first");
    expect<bool>(false,
            []{return D::fle_d(2.0, numeric_limits<double>::signaling_NaN());},
            "fle.d, signaling NaN second");
    expect<bool>(false,
            []{return D::fle_d(numeric_limits<double>::signaling_NaN(),
                    numeric_limits<double>::signaling_NaN());},
            "fle.d, signaling NaN both");

    // FCLASS.D
    expect<uint64_t>(0x1,
            []{return D::fclass_d(-numeric_limits<double>::infinity());},
            "fclass.d, -infinity");
    expect<uint64_t>(0x2,
            []{return D::fclass_d(-3.14);}, "fclass.d, -normal");
    expect<uint64_t>(0x4,
            []{return D::fclass_d(D::number(0x800FFFFFFFFFFFFFULL));},
            "fclass.d, -subnormal");
    expect<uint64_t>(0x8, []{return D::fclass_d(-0.0);}, "fclass.d, -0.0");
    expect<uint64_t>(0x10, []{return D::fclass_d(0.0);}, "fclass.d, 0.0");
    expect<uint64_t>(0x20,
            []{return D::fclass_d(D::number(0x000FFFFFFFFFFFFFULL));},
            "fclass.d, subnormal");
    expect<uint64_t>(0x40, []{return D::fclass_d(1.816);}, "fclass.d, normal");
    expect<uint64_t>(0x80,
            []{return D::fclass_d(numeric_limits<double>::infinity());},
            "fclass.d, infinity");
    expect<uint64_t>(0x100,
            []{return D::fclass_d(numeric_limits<double>::signaling_NaN());},
            "fclass.d, signaling NaN");
    expect<uint64_t>(0x200,
            []{return D::fclass_d(numeric_limits<double>::quiet_NaN());},
            "fclass.s, quiet NaN");

    // FCVT.W.D
    expect<int64_t>(256, []{return D::fcvt_w_d(256.3);},
            "fcvt.w.d, truncate positive");
    expect<int64_t>(-256, []{return D::fcvt_w_d(-256.2);},
            "fcvt.w.d, truncate negative");
    expect<int64_t>(0, []{return D::fcvt_w_d(0.0);}, "fcvt.w.d, 0.0");
    expect<int64_t>(0, []{return D::fcvt_w_d(-0.0);}, "fcvt.w.d, -0.0");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return D::fcvt_w_d(numeric_limits<double>::max());},
            "fcvt.w.d, overflow");
    expect<int64_t>(0, []{return D::fcvt_w_d(numeric_limits<double>::min());},
            "fcvt.w.d, underflow");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return D::fcvt_w_d(numeric_limits<double>::infinity());},
            "fcvt.w.d, infinity");
    expect<int64_t>(numeric_limits<int32_t>::min(),
            []{return D::fcvt_w_d(-numeric_limits<double>::infinity());},
            "fcvt.w.d, -infinity");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return D::fcvt_w_d(numeric_limits<double>::quiet_NaN());},
            "fcvt.w.d, quiet NaN");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return D::fcvt_w_d(-numeric_limits<double>::quiet_NaN());},
            "fcvt.w.d, quiet -NaN");
    expect<int64_t>(numeric_limits<int32_t>::max(),
            []{return D::fcvt_w_d(numeric_limits<double>::signaling_NaN());},
            "fcvt.w.d, signaling NaN");

    // FCVT.WU.D
    expect<uint64_t>(256, []{return D::fcvt_wu_d(256.3);},
            "fcvt.wu.d, truncate positive");
    expect<uint64_t>(0, []{return D::fcvt_wu_d(-256.2);},
            "fcvt.wu.d, truncate negative");
    expect<uint64_t>(0, []{return D::fcvt_wu_d(0.0);}, "fcvt.wu.d, 0.0");
    expect<uint64_t>(0, []{return D::fcvt_wu_d(-0.0);}, "fcvt.wu.d, -0.0");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return D::fcvt_wu_d(numeric_limits<double>::max());},
            "fcvt.wu.d, overflow");
    expect<uint64_t>(0,[]{return D::fcvt_wu_d(numeric_limits<double>::min());},
            "fcvt.wu.d, underflow");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return D::fcvt_wu_d(numeric_limits<double>::infinity());},
            "fcvt.wu.d, infinity");
    expect<uint64_t>(0,
            []{return D::fcvt_wu_d(-numeric_limits<double>::infinity());},
            "fcvt.wu.d, -infinity");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return D::fcvt_wu_d(numeric_limits<double>::quiet_NaN());},
            "fcvt.wu.d, quiet NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return D::fcvt_wu_d(-numeric_limits<double>::quiet_NaN());},
            "fcvt.wu.d, quiet -NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return D::fcvt_wu_d(numeric_limits<double>::signaling_NaN());},
            "fcvt.wu.d, signaling NaN");

    // FCVT.D.W
    expect<double>(0.0, []{return D::fcvt_d_w(0);}, "fcvt.d.w, 0");
    expect<double>(-2147483648.0,
            []{return D::fcvt_d_w(numeric_limits<int32_t>::min());},
            "fcvt.d.w, negative");
    expect<double>(255.0, []{return D::fcvt_d_w(0xFFFFFFFF000000FFLL);},
            "fcvt.d.w, truncate");

    // FCVT.D.WU
    expect<double>(0.0, []{return D::fcvt_d_wu(0);}, "fcvt.d.wu, 0");
    expect<double>(2147483648.0,
            []{return D::fcvt_d_wu(numeric_limits<int32_t>::min());},
            "fcvt.d.wu");
    expect<double>(255.0,
            []{return D::fcvt_d_wu(0xFFFFFFFF000000FFLL);},
            "fcvt.d.wu, truncate");

    // FCVT.L.D
    expect<int64_t>(256, []{return D::fcvt_l_d(256.3);},
            "fcvt.l.d, truncate positive");
    expect<int64_t>(-256, []{return D::fcvt_l_d(-256.2);},
            "fcvt.l.d, truncate negative");
    expect<int64_t>(0, []{return D::fcvt_l_d(0.0);}, "fcvt.l.d, 0.0");
    expect<int64_t>(0, []{return D::fcvt_l_d(-0.0);}, "fcvt.l.d, -0.0");
    expect<int64_t>(-8589934592LL, []{return D::fcvt_l_d(-8589934592.0);},
            "fcvt.l.d, 32-bit overflow");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return D::fcvt_l_d(numeric_limits<double>::max());},
            "fcvt.l.d, overflow");
    expect<int64_t>(0, []{return D::fcvt_l_d(numeric_limits<double>::min());},
            "fcvt.l.d, underflow");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return D::fcvt_l_d(numeric_limits<double>::infinity());},
            "fcvt.l.d, infinity");
    expect<int64_t>(numeric_limits<int64_t>::min(),
            []{return D::fcvt_l_d(-numeric_limits<double>::infinity());},
            "fcvt.l.d, -infinity");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return D::fcvt_l_d(numeric_limits<double>::quiet_NaN());},
            "fcvt.l.d, quiet NaN");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return D::fcvt_l_d(-numeric_limits<double>::quiet_NaN());},
            "fcvt.l.d, quiet -NaN");
    expect<int64_t>(numeric_limits<int64_t>::max(),
            []{return D::fcvt_l_d(numeric_limits<double>::signaling_NaN());},
            "fcvt.l.d, signaling NaN");

    // FCVT.LU.D
    expect<uint64_t>(256, []{return D::fcvt_lu_d(256.3);},
            "fcvt.lu.d, truncate positive");
    expect<uint64_t>(0, []{return D::fcvt_lu_d(-256.2);},
            "fcvt.lu.d, truncate negative");
    expect<uint64_t>(0, []{return D::fcvt_lu_d(0.0);}, "fcvt.lu.d, 0.0");
    expect<uint64_t>(0, []{return D::fcvt_lu_d(-0.0);}, "fcvt.lu.d, -0.0");
    expect<uint64_t>(8589934592LL, []{return D::fcvt_lu_d(8589934592.0);},
            "fcvt.lu.d, 32-bit overflow");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return D::fcvt_lu_d(numeric_limits<double>::max());},
            "fcvt.lu.d, overflow");
    expect<uint64_t>(0,[]{return D::fcvt_lu_d(numeric_limits<double>::min());},
            "fcvt.lu.d, underflow");
    expect<uint64_t>(numeric_limits<uint64_t>::max(),
            []{return D::fcvt_lu_d(numeric_limits<double>::infinity());},
            "fcvt.lu.d, infinity");
    expect<uint64_t>(0,
            []{return D::fcvt_lu_d(-numeric_limits<double>::infinity());},
            "fcvt.lu.d, -infinity");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return D::fcvt_lu_d(numeric_limits<double>::quiet_NaN());},
            "fcvt.lu.d, quiet NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return D::fcvt_lu_d(-numeric_limits<double>::quiet_NaN());},
            "fcvt.lu.d, quiet -NaN");
    expect<uint64_t>(0xFFFFFFFFFFFFFFFFULL,
            []{return D::fcvt_lu_d(numeric_limits<double>::signaling_NaN());},
            "fcvt.lu.d, signaling NaN");

    // FMV.X.D
    expect<uint64_t>(0x40091EB851EB851FULL, []{return D::fmv_x_d(3.14);},
            "fmv.x.d, positive");
    expect<uint64_t>(0xC0091EB851EB851FULL, []{return D::fmv_x_d(-3.14);},
            "fmv.x.d, negative");
    expect<uint64_t>(0x0000000000000000ULL, []{return D::fmv_x_d(0.0);},
            "fmv.x.d, 0.0");
    expect<uint64_t>(0x8000000000000000ULL, []{return D::fmv_x_d(-0.0);},
            "fmv.x.d, -0.0");

    // FCVT.D.L
    expect<double>(0.0, []{return D::fcvt_d_l(0);}, "fcvt.d.l, 0");
    expect<double>(D::number(0xC3E0000000000000),
            []{return D::fcvt_d_l(numeric_limits<int64_t>::min());},
            "fcvt.d.l, negative");
    expect<double>(D::number(0xC1EFFFFFE0200000),
            []{return D::fcvt_d_l(0xFFFFFFFF000000FFLL);},
            "fcvt.d.l, 32-bit truncate");

    // FCVT.D.LU
    expect<double>(0.0, []{return D::fcvt_d_lu(0);}, "fcvt.d.lu, 0");
    expect<double>(D::number(0x43E0000000000000),
            []{return D::fcvt_d_lu(numeric_limits<int64_t>::min());},
            "fcvt.d.lu");
    expect<double>(D::number(0x43EFFFFFFFE00000),
            []{return D::fcvt_d_lu(0xFFFFFFFF000000FFLL);},
            "fcvt.d.lu, 32-bit truncate");

    // FMV.D.X
    expect<double>(-numeric_limits<float>::infinity(),
            []{return D::fmv_d_x(0xFFF0000000000000ULL);}, "fmv.d.x");

    return 0;
}
