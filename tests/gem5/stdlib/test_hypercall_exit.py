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


"""Runs a series of tests to ensure the hypercall calls are being handled
correctly. Passed from a simulated system all the way to handling in the
Simulator module.
"""

from pathlib import Path

from testlib import (
    config,
    constants,
    gem5_verify_config,
)

resource_directory = (
    config.bin_path
    if config.bin_path
    else str(Path(Path(__file__).parent.parent, "resources"))
)

tests = [
    "1",  # Index 0
    "2",  # Index 1
    "1,2",  # Index 2
    "1,2,4,5,6,7,8,9",  # Index 3
    "10,16,56,24,98,57436",  # Index 4
]


for index, test in enumerate(tests):
    gem5_verify_config(
        name=f"hypercall-exit-handling-test-{index}",
        verifiers=[],
        fixtures=[],
        config=str(
            Path(
                Path(__file__).parent,
                "configs",
                "hypercall-exit-check.py",
            )
        ),
        config_args=[
            test,
            f"--resource-directory='{resource_directory}'",
        ],
        valid_isas=(constants.all_compiled_tag,),
        length=constants.quick_tag,
    )
