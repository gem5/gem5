# Copyright (c) 2025 The Regents of the University of California
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

"""
This runs tests to verify that configuration output files are generated
correctly when using various dump-config options.
"""

from testlib import *
from testlib.helper import joinpath

# Test the --dump-config option. `arm-hello.py` checks for `custom_config.ini`
# in the `m5out` directory after the simulation finishes.
gem5_verify_config(
    name="arm_hello_dump_config",
    verifiers=(),
    fixtures=(),
    gem5_args=[
        "--dump-config=custom_config.ini",
    ],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "config_output_files",
        "configs",
        "arm-hello.py",
    ),
    config_args=["--dump-config"],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# Test the --json-config option. Checks for `custom_config.json`.
gem5_verify_config(
    name="arm_hello_json_config",
    verifiers=(),
    fixtures=(),
    gem5_args=[
        "--json-config=custom_config.json",
    ],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "config_output_files",
        "configs",
        "arm-hello.py",
    ),
    config_args=["--json-config"],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# Test the --dot-config option. Checks for `custom_config.dot`,
# `custom_config.dot.pdf` and `custom_config.dot.svg`.
gem5_verify_config(
    name="arm_hello_dot_config",
    verifiers=(),
    fixtures=(),
    gem5_args=[
        "--dot-config=custom_config.dot",
    ],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "config_output_files",
        "configs",
        "arm-hello.py",
    ),
    config_args=["--dot-config"],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# Test with all three options together.
gem5_verify_config(
    name="arm_hello_all_config_options",
    verifiers=(),
    fixtures=(),
    gem5_args=[
        "--dump-config=combined_config.ini",
        "--json-config=combined_config.json",
        "--dot-config=combined_config.dot",
    ],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "config_output_files",
        "configs",
        "arm-hello.py",
    ),
    config_args=["--dump-config", "--json-config", "--dot-config"],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# Test without passing any options in.
gem5_verify_config(
    name="arm_hello_default_config_names",
    verifiers=(),
    fixtures=(),
    gem5_args=[],
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "config_output_files",
        "configs",
        "arm-hello.py",
    ),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)
