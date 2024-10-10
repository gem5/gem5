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

"""
These tests verify the that the CHI protocol works with the RISCV, X86, and ARM
ISAs with varying numbers of cores (1, 2, and 4).
"""

from testlib import (
    absdirpath,
    config,
    constants,
    joinpath,
    verifier,
)

from gem5.suite import gem5_verify_config

if config.bin_path:
    resource_directory = config.bin_path
else:
    resource_directory = joinpath(
        config.base_dir, "tests", "gem5", "resources"
    )

for isa in ("arm", "riscv", "x86"):
    for num_cores in (1, 2, 4):
        gem5_verify_config(
            name=f"test-chi with-{isa}-{num_cores}",
            fixtures=(),
            verifiers=(
                verifier.MatchStdoutNoPerf(
                    joinpath(
                        absdirpath(__file__),
                        "refs",
                        "matrix-multiply-stdout.txt",
                    )
                ),
            ),
            config=joinpath(
                absdirpath(__file__),
                "configs",
                "chi-with-isa.py",
            ),
            config_args=[
                isa,
                f"--num-cores={num_cores}",
                f"--resource-directory={resource_directory}",
            ],
            valid_isas=(constants.all_compiled_tag,),
            valid_hosts=constants.supported_hosts,
            protocol="CHI",
            length=constants.long_tag,
        )
