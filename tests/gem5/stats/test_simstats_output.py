# Copyright (c) 2024 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from testlib import *

gem5_verify_config(
    name="simstat-scaler-int-test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "simstat_output_check.py",
    ),
    config_args=[
        "42",
        "--name",
        "scalar_test",
        "--description",
        "A scalar statistic with a int value",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="simstat-scaler-int-zero-test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "simstat_output_check.py",
    ),
    config_args=[
        "0",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="simstat-scaler-int-negative-test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "simstat_output_check.py",
    ),
    config_args=[
        "-245",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="simstat-scaler-float-test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "simstat_output_check.py",
    ),
    config_args=[
        "42.869",
        "--name",
        "float_test",
        "--description",
        "A scalar statistic with a float value",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="pystat_vector_test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "pystat_vector_check.py",
    ),
    config_args=[
        "2.0,4,5.9,2.3,-8,0,0.0,-8.9",
        "--name",
        "vector_stat",
        "--description",
        "A vector statistic with a float value",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="pystat_vector_with_subnames_test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "pystat_vector_check.py",
    ),
    config_args=[
        "2.0,4,3",
        "--name",
        "vector_stat",
        "--description",
        "A vector statistic with a float value",
        "--subnames",
        "first,second,third",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="pystat_vector_with_subdescs_test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "pystat_vector_check.py",
    ),
    config_args=[
        "2.0,4,3",
        "--name",
        "vector_stat",
        "--description",
        "A vector statistic with a float value",
        "--subdescs",
        "first,second",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="pystat_vector2d_test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "pystat_vector2d_check.py",
    ),
    config_args=[
        "2.4,4.3,3.7,-1.4,-2,4,0,0",
        2,
        "--name",
        "vector2d_stat",
        "--description",
        "A 2d vector statistic with",
        "--subnames",
        "decimals,integers",
        "--subdescs",
        "A random collection of decimals,A random collection of integers",
        "--ysubnames",
        "first,second,third,fourth",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="pystat-sparsehist-test",
    fixtures=(),
    verifiers=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "stats",
        "configs",
        "pystat_sparse_dist_check.py",
    ),
    config_args=[
        "1.0,1,1.00,23,23,0.2,0.2,0.2,0.2,-1,-1.0,264",
        "--name",
        "sparsehist_stat",
        "--description",
        "A sparse histogram statistic.",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)
