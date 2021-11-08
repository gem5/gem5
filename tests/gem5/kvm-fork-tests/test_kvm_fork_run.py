# Copyright (c) 2021 The Regents of the University of California
# Copyright (c) 2021 The University of Texas at Austin
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
The purpose of this Suite of tests is to check that the simulator can be forked
with the KVM cpu then switch to a different cpu and run in the child.
"""

import os

from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")


def test_kvm_fork_run(cpu: str, num_cpus: int, mem_system: str, length: str):

    if not os.access("/dev/kvm", mode=os.R_OK | os.W_OK):
        # Don't run the tests if KVM is unavailable.
        return

    name = "{}-cpu_{}-cores_{}_kvm-fork-run-test".format(
        cpu, str(num_cpus), mem_system
    )
    verifiers = []

    if mem_system == "mesi_two_level":
        protocol_to_use = "MESI_Two_Level"
        isa_to_use = constants.x86_tag
    elif mem_system == "mi_example":
        protocol_to_use = None
        isa_to_use = constants.x86_tag
    else:
        protocol_to_use = None
        isa_to_use = constants.gcn3_x86_tag

    gem5_verify_config(
        name=name,
        verifiers=verifiers,
        fixtures=(),
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "configs",
            "boot_kvm_fork_run.py",
        ),
        config_args=[
            "--cpu",
            cpu,
            "--num-cpus",
            str(num_cpus),
            "--num-forks",
            "4",
            "--mem-system",
            mem_system,
            "--resource-directory",
            resource_path,
            "--kernel-args=''",
        ],
        valid_isas=(isa_to_use,),
        valid_hosts=constants.supported_hosts,
        protocol=protocol_to_use,
        length=length,
    )


#### The quick (pre-submit/Kokoro) tests ####

test_kvm_fork_run(
    cpu="atomic", num_cpus=1, mem_system="classic", length=constants.quick_tag
)
test_kvm_fork_run(
    cpu="timing", num_cpus=1, mem_system="classic", length=constants.quick_tag
)
test_kvm_fork_run(
    cpu="o3", num_cpus=1, mem_system="classic", length=constants.quick_tag
)
test_kvm_fork_run(
    cpu="timing", num_cpus=4, mem_system="classic", length=constants.quick_tag
)

### The long (nightly) tests ####

test_kvm_fork_run(
    cpu="timing",
    num_cpus=4,
    mem_system="mi_example",
    length=constants.long_tag,
)

test_kvm_fork_run(
    cpu="timing",
    num_cpus=1,
    mem_system="mesi_two_level",
    length=constants.long_tag,
)
test_kvm_fork_run(
    cpu="o3",
    num_cpus=8,
    mem_system="mesi_two_level",
    length=constants.long_tag,
)

test_kvm_fork_run(
    cpu="timing",
    num_cpus=2,
    mem_system="mesi_two_level",
    length=constants.long_tag,
)
