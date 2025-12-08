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

import os
import re

from testlib import *
from testlib.log import *


def test_processor_switch_systemboot(
    isa: str, num_cores: int, start_cores: str, switch_cores: str
):
    verifiers = []
    exit_regex = re.compile(
        f"Successful completion of {start_cores} to {switch_cores} {num_cores} core {isa} processor switch test"
    )
    verifiers.append(verifier.MatchRegex(exit_regex))

    gem5_verify_config(
        name=f"processor_switch_{start_cores}_to_{switch_cores}_afterboot_{isa}_{num_cores}core",
        fixtures=(),
        verifiers=verifiers,
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "processor_switch_tests",
            "configs",
            f"cross-product-switch-afterboot.py",
        ),
        config_args=[
            "--num_cores",
            num_cores,
            "--start_cores",
            start_cores,
            "--switch_cores",
            switch_cores,
            "--isa",
            isa,
        ],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=constants.supported_hosts,
        valid_variants=(constants.fast_tag,),
        length=constants.very_long_tag,
    )


def test_processor_switch_matmul(
    isa: str, start_cores: str, switch_cores: str
):
    verifiers = []
    exit_regex = re.compile(
        f"Successful completion of {start_cores} to {switch_cores} 1 core {isa} processor switch test"
    )
    verifiers.append(verifier.MatchRegex(exit_regex))

    gem5_verify_config(
        name=f"processor_switch_{start_cores}_to_{switch_cores}_matmul_{isa}_1core",
        fixtures=(),
        verifiers=verifiers,
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "processor_switch_tests",
            "configs",
            f"cross-product-switch-matmul.py",
        ),
        config_args=[
            "--start_cores",
            start_cores,
            "--switch_cores",
            switch_cores,
            "--isa",
            isa,
        ],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=constants.supported_hosts,
        valid_variants=(constants.fast_tag,),
        length=constants.long_tag,
    )


for isa in ["arm", "x86", "riscv"]:
    for num_cores in [4, 8, 16]:
        for start_cores in ["atomic", "timing", "o3", "minor"]:  # , "kvm",
            for switch_cores in ["atomic", "timing", "o3", "minor"]:  # "kvm",
                # Skip 16 core tests if the ISA is Arm because there were
                # issues with taking the checkpoint.
                # Skip tests that start with minor cores on Arm because there
                # were failures when running them outside of TestLib
                if isa == "arm" and (
                    num_cores == 16 or start_cores == "minor"
                ):
                    continue
                # Skip tests that start with o3 and minor cores on X86 because
                # there were failures when running them outside of Testlib
                if isa == "x86" and (
                    start_cores == "o3" or start_cores == "minor"
                ):
                    continue
                # There were issues with running these combinations in
                # TestLib but not outside of TestLib.
                if (
                    isa == "x86"
                    and (num_cores == 8 or num_cores == 16)
                    and (start_cores == "timing")
                ):
                    continue
                # These combinations take too long to run.
                if (
                    isa == "riscv"
                    and num_cores == 16
                    and (start_cores == "o3" or start_cores == "minor")
                ):
                    continue

                test_processor_switch_systemboot(
                    isa, num_cores, start_cores, switch_cores
                )

for isa in ["x86", "arm", "riscv"]:
    for start_cores in ["atomic", "timing", "o3", "minor"]:  # "kvm",
        for switch_cores in ["atomic", "timing", "o3", "minor"]:  # "kvm",
            test_processor_switch_matmul(isa, start_cores, switch_cores)
