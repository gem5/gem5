# Copyright (c) 2021 The Regents of the University of California
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

from typing import Optional
import re

from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")


def test_boot(
    cpu: str,
    num_cpus: int,
    mem_system: str,
    length: str,
    boot_type: str = "init",
    to_tick: Optional[int] = None,
):

    name = "{}-cpu_{}-cores_{}_{}_x86-boot-test".format(
        cpu, str(num_cpus), mem_system, boot_type
    )
    verifiers = []
    additional_config_args = []

    if to_tick != None:
        name += "_to-tick"
        exit_regex = re.compile(
            "Exiting @ tick {} because simulate\(\) limit reached".format(
                str(to_tick)
            )
        )
        verifiers.append(verifier.MatchRegex(exit_regex))
        additional_config_args.append("--tick-exit")
        additional_config_args.append(str(to_tick))

    if mem_system == "mesi_two_level":
        protocol_to_use="MESI_Two_Level"
        isa_to_use=constants.x86_tag
    elif mem_system == "mi_example":
        protocol_to_use=None
        isa_to_use=constants.x86_tag
    else:
        protocol_to_use=None
        isa_to_use=constants.gcn3_x86_tag

    gem5_verify_config(
        name=name,
        verifiers=verifiers,
        fixtures=(),
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "configs",
            "x86_boot_exit_run.py",
        ),
        config_args=[
            "--cpu",
            cpu,
            "--num-cpus",
            str(num_cpus),
            "--mem-system",
            mem_system,
            "--boot-type",
            boot_type,
            "--resource-directory",
            resource_path,
        ]
        + additional_config_args,
        valid_isas=(isa_to_use,),
        valid_hosts=constants.supported_hosts,
        protocol=protocol_to_use,
        length=length,
    )


#### The quick (pre-submit/Kokoro) tests ####

test_boot(
    cpu="atomic",
    num_cpus=1,
    mem_system="classic",
    to_tick=10000000000, #Simulates 1/100th of a second.
    length=constants.quick_tag,
)

test_boot(
    cpu="timing",
    num_cpus=1,
    mem_system="classic",
    to_tick=10000000000,
    length=constants.quick_tag,
)

test_boot(
    cpu="atomic",
    num_cpus=4,
    mem_system="classic",
    to_tick=10000000000,
    length=constants.quick_tag,
)

test_boot(
    cpu="o3",
    num_cpus=1,
    mem_system="classic",
    to_tick=10000000000,
    length=constants.quick_tag,
)

#### The long (Nightly) tests ####

test_boot(
    cpu="atomic",
    num_cpus=1,
    mem_system="classic",
    boot_type="init",
    length=constants.long_tag,
)

test_boot(
    cpu="timing",
    num_cpus=1,
    mem_system="mesi_two_level",
    boot_type="init",
    length=constants.long_tag,
)

test_boot(
    cpu="timing",
    num_cpus=1,
    mem_system="mi_example",
    boot_type="init",
    length=constants.long_tag,
)

test_boot(
    cpu="atomic",
    num_cpus=4,
    mem_system="classic",
    boot_type="systemd",
    length=constants.long_tag,
)

test_boot(
    cpu="o3",
    num_cpus=2,
    mem_system="mesi_two_level",
    boot_type="init",
    length=constants.long_tag,
)

#### The very-long (Weekly) tests ####

# This maps the cross product of the test to run. As 'init' is a subset
# of the 'systemd' boot-type, we only run 'systemd'. A test with a value of
# 'False' will not be run. This is either due to 'Timeout' (this test cannot
# complete within 12 hours), this setup is not supported at present, or a
# simulation crash.
run_map = {
    # The Memory System.
    "classic": {
        # The CPU Type.
        "atomic": {
            # The number of cores.
            1: True,
            2: True,
            4: False,  # We already run this in the long (Nightly) tests.
            8: True,
        },
        "timing": {
            1: True,
            2: False,  # Timeout
            4: False,  # Timeout
            8: False,  # Timeout
        },
        "o3": {
            1: False,  # Timeout
            2: False,  # Not Supported
            4: False,  # Not Supported
            8: False,  # Not Supported
        },
    },
    "mi_example": {
        "atomic": {
            1: False,  # Not Supported
            2: False,  # Not Supported
            4: False,  # Not Supported
            8: False,  # Not Supported
        },
        "timing": {
            1: True,
            2: True,
            4: True,
            8: True,
        },
        "o3": {
            1: False,  # Timeout
            2: False,  # Timeout
            4: False,  # Timeout
            8: False,  # Timeout
        },
    },
    "mesi_two_level": {
        "atomic": {
            1: False,  # Not Supported
            2: False,  # Not Supported
            4: False,  # Not Supported
            8: False,  # Not Supported
        },
        "timing": {
            1: True,
            2: True,
            4: True,
            8: True,
        },
        "o3": {
            1: False,  # Timeout
            2: False,  # Timeout
            4: False,  # Simulation Crash
            8: False,  # Simulation Crash
        },
    },
}

for mem_system in run_map:
    for cpu in run_map[mem_system]:
        for num_cpus in run_map[mem_system][cpu]:
            if run_map[mem_system][cpu][num_cpus]:

                test_boot(
                    cpu=cpu,
                    num_cpus=num_cpus,
                    mem_system=mem_system,
                    boot_type="systemd",
                    length=constants.very_long_tag,
                )
