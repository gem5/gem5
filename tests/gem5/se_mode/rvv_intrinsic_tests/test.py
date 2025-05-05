# Copyright (c) 2024 Barcelona Supercomputing Center
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import re
import sys

from testlib import *

resources = [
    "rvv-branch",
    "rvv-index",
    "rvv-matmul",
    "rvv-memcpy",
    "rvv-reduce",
    "rvv-saxpy",
    "rvv-sgemm",
    "rvv-strcmp",
    "rvv-strcpy",
    "rvv-strlen",
    "rvv-strlen-fault",
    "rvv-strncpy",
]

vlens = [2**x for x in range(7, 15)]

for resource in resources:
    out_verifier = verifier.MatchRegex(re.compile(f"^.*{resource}: pass$"))

    for vlen in vlens:
        gem5_verify_config(
            name=f"test-riscv-{resource}-vlen_{vlen}-O3-se-mode",
            fixtures=(),
            verifiers=(out_verifier,),
            config=f"{config.base_dir}/configs/example/gem5_library/riscv-rvv-example.py",
            config_args=[resource, f"--vlen={vlen}"],
            valid_isas=(constants.all_compiled_tag,),
            length=constants.quick_tag,
        )
