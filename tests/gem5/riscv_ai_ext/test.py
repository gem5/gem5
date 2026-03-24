# Copyright (c) 2026
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

import os
import re

from testlib import *
from testlib import verifier


bin_dir = joinpath(absdirpath(__file__), "bin")
config_path = joinpath(
    config.base_dir, "tests", "gem5", "riscv_ai_ext", "configs", "local_binary_run.py"
)

test_matrix = {
    "ai_ops_smoke": {
        "cpus": ("atomic", "o3"),
        "regex": re.compile(r"^ai_ops_smoke: PASS$"),
    },
    "p_lw_smoke": {
        "cpus": ("atomic", "o3"),
        "regex": re.compile(r"^p_lw_smoke: PASS$"),
    },
    "hwloop_smoke": {
        "cpus": ("atomic", "minor", "o3"),
        "regex": re.compile(r"^hwloop_smoke: PASS$"),
    },
}

for binary_name, test_data in test_matrix.items():
    binary_path = joinpath(bin_dir, binary_name)
    if not os.path.isfile(binary_path):
        continue

    out_verifier = verifier.MatchRegex(test_data["regex"], match_stderr=False)

    for cpu in test_data["cpus"]:
        gem5_verify_config(
            name=f"test-riscv-{binary_name}-{cpu}",
            fixtures=(),
            verifiers=(out_verifier,),
            config=config_path,
            config_args=[binary_path, cpu],
            valid_isas=(constants.riscv_tag,),
            length=constants.quick_tag,
        )
