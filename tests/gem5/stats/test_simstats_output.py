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
        "scalar",
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
        "scalar",
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
        "scalar",
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
        "scalar",
        "42.869",
        "--name",
        "float_test",
        "--description",
        "A scalar statistic with a float value",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)
