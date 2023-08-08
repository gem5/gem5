# Copyright (c) 2021-2023 The Regents of the University of California
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
This runs simple tests to ensure the examples in `configs/example/gem5_library`
still function. They simply check the simulation completed.
"""
from testlib import *
import re
import os

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")

hello_verifier = verifier.MatchRegex(re.compile(r"Hello world!"))
save_checkpoint_verifier = verifier.MatchRegex(
    re.compile(r"Done taking checkpoint")
)


gem5_verify_config(
    name="test-checkpoint-arm-hello-save-checkpoint",
    fixtures=(),
    verifiers=(save_checkpoint_verifier,),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "arm-hello-save-checkpoint.py",
    ),
    config_args=[
        "--checkpoint-path",
        joinpath(resource_path, "arm-hello-test-checkpoint"),
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-checkpoint-arm-hello-restore-checkpoint",
    fixtures=(),
    verifiers=(hello_verifier,),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "arm-hello-restore-checkpoint.py",
    ),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-checkpoint-x86-hello-save-checkpoint",
    fixtures=(),
    verifiers=(save_checkpoint_verifier,),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "x86-hello-save-checkpoint.py",
    ),
    config_args=[
        "--checkpoint-path",
        joinpath(resource_path, "x86-hello-test-checkpoint"),
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-checkpoint-x86-hello-restore-checkpoint",
    fixtures=(),
    verifiers=(hello_verifier,),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "x86-hello-restore-checkpoint.py",
    ),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-checkpoint-x86-fs-save-checkpoint",
    fixtures=(),
    verifiers=(save_checkpoint_verifier,),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "x86-fs-save-checkpoint.py",
    ),
    config_args=[
        "--checkpoint-path",
        joinpath(resource_path, "x86-fs-test-checkpoint"),
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-checkpoint-x86-fs-restore-checkpoint",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "x86-fs-restore-checkpoint.py",
    ),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-checkpoint-power-hello-save-checkpoint",
    fixtures=(),
    verifiers=(save_checkpoint_verifier,),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "power-hello-save-checkpoint.py",
    ),
    config_args=[
        "--checkpoint-path",
        joinpath(resource_path, "power-hello-test-checkpoint"),
    ],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

gem5_verify_config(
    name="test-checkpoint-power-hello-restore-checkpoint",
    fixtures=(),
    verifiers=(hello_verifier,),
    config=joinpath(
        config.base_dir,
        "tests",
        "gem5",
        "checkpoint-tests",
        "power-hello-restore-checkpoint.py",
    ),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)

# gem5_verify_config(
#     name="test-checkpoint-mips-hello-save-checkpoint",
#     fixtures=(),
#     verifiers=(save_checkpoint_verifier,),
#     config=joinpath(
#         config.base_dir,
#         "tests",
#         "gem5",
#         "checkpoint-tests",
#         "mips-hello-save-checkpoint.py",
#     ),
#     config_args=[
#         # "--checkpoint-path",
#         # joinpath(resource_path, "mips-hello-test-checkpoint"),
#     ],
#     valid_isas=(constants.all_compiled_tag,),
#     valid_hosts=constants.supported_hosts,
#     length=constants.quick_tag,
# )

# gem5_verify_config(
#     name="test-checkpoint-mips-hello-restore-checkpoint",
#     fixtures=(),
#     verifiers=(hello_verifier,),
#     config=joinpath(
#         config.base_dir,
#         "tests",
#         "gem5",
#         "checkpoint-tests",
#         "mips-hello-restore-checkpoint.py",
#     ),
#     config_args=[],
#     valid_isas=(constants.all_compiled_tag,),
#     valid_hosts=constants.supported_hosts,
#     length=constants.quick_tag,
# )

# gem5_verify_config(
#     name="test-checkpoint-sparc-hello-save-checkpoint",
#     fixtures=(),
#     verifiers=(save_checkpoint_verifier,),
#     config=joinpath(
#         config.base_dir,
#         "tests",
#         "gem5",
#         "checkpoint-tests",
#         "sparc-hello-save-checkpoint.py",
#     ),
#     config_args=[
#         # "--checkpoint-path",
#         # joinpath(resource_path, "sparc-hello-test-checkpoint"),
#     ],
#     valid_isas=(constants.all_compiled_tag,),
#     valid_hosts=constants.supported_hosts,
#     length=constants.quick_tag,
# )

# gem5_verify_config(
#     name="test-checkpoint-sparc-hello-restore-checkpoint",
#     fixtures=(),
#     verifiers=(hello_verifier,),
#     config=joinpath(
#         config.base_dir,
#         "tests",
#         "gem5",
#         "checkpoint-tests",
#         "sparc-hello-restore-checkpoint.py",
#     ),
#     config_args=[],
#     valid_isas=(constants.all_compiled_tag,),
#     valid_hosts=constants.supported_hosts,
#     length=constants.quick_tag,
# )
