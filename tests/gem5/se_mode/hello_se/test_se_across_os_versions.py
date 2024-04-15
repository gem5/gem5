# Copyright (c) 2024 The Regents of the University of California
# All Rights Reserved.
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

"""This test will run, in SE mode, two 'matrix-multiply' binaries, one built on
Ubuntu 22.04 and the other on Ubuntu 24.04, and two 'print-this' binaries, one
built on Ubuntu 22.04 and the other on Ubuntu 24.04.

This purpose of this is to check gem5's SE mode's ability to interpret a very
simple suite of Syscalls across different versions of the same OS.
"""
from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "..", "resources")

gem5_verify_config(
    name="test-se-across-os-versions-x86-matrix-multiply-22-04",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "se_mode",
        "hello_se",
        "configs",
        "simple_binary_run.py",
    ),
    config_args=[
        "x86-matrix-multiply-22-04",
        "atomic",
        "x86",
        "--num-cores",
        "1",
        "--resource-directory",
        resource_path,
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-se-across-os-versions-x86-matrix-multiply-24-04",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "se_mode",
        "hello_se",
        "configs",
        "simple_binary_run.py",
    ),
    config_args=[
        "x86-matrix-multiply-24-04",
        "atomic",
        "x86",
        "--num-cores",
        "1",
        "--resource-directory",
        resource_path,
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-se-across-os-versions-x86-print-this-22-04",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "se_mode",
        "hello_se",
        "configs",
        "simple_binary_run.py",
    ),
    config_args=[
        "x86-print-this-22-04",
        "atomic",
        "x86",
        "--num-cores",
        "1",
        "--resource-directory",
        resource_path,
        "--arguments",
        "print this",
        "--arguments",
        "2000",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-se-across-os-versions-x86-print-this-24-04",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "se_mode",
        "hello_se",
        "configs",
        "simple_binary_run.py",
    ),
    config_args=[
        "x86-print-this-24-04",
        "atomic",
        "x86",
        "--num-cores",
        "1",
        "--resource-directory",
        resource_path,
        "--arguments",
        "print this",
        "--arguments",
        "2000",
    ],
    valid_isas=(constants.all_compiled_tag,),
    length=constants.quick_tag,
)
