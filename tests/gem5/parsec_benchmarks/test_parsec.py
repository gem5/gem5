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
import os

from testlib import *

if config.bin_path:
    resource_path = config.bin_path
else:
    resource_path = joinpath(absdirpath(__file__), "..", "resources")


def test_parsec(
    boot_cpu: str,
    detailed_cpu: str,
    num_cpus: int,
    mem_system: str,
    benchmark: str,
    size: str,
    length: str,
):
    if (boot_cpu == "kvm" or detailed_cpu == "kvm") and not os.access(
        "/dev/kvm",
        mode=os.R_OK | os.W_OK,
    ):
        # Don't run the tests if KVM is unavailable.
        return

    print(
        "WARNING: PARSEC tests are disabled. This is due to our GitHub "
        "Actions self-hosted runners only having 60GB of disk space. The "
        "PARSEC Disk image is too big to use.",
    )
    return  # Remove this line to re-enable PARSEC tests.

    gem5_verify_config(
        name="{}-boot-cpu_{}-detailed-cpu_{}-cores_{}_{}_{}_parsec-test".format(
            boot_cpu,
            detailed_cpu,
            str(num_cpus),
            mem_system,
            benchmark,
            size,
        ),
        verifiers=(),
        fixtures=(),
        config=joinpath(
            config.base_dir,
            "tests",
            "gem5",
            "parsec_benchmarks",
            "configs",
            "parsec_disk_run.py",
        ),
        config_args=[
            "--cpu",
            detailed_cpu,
            "--boot-cpu",
            boot_cpu,
            "--num-cpus",
            str(num_cpus),
            "--mem-system",
            mem_system,
            "--benchmark",
            benchmark,
            "--size",
            size,
            "--resource-directory",
            resource_path,
        ],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=(constants.host_x86_64_tag,),
        length=length,
        uses_kvm=True,
    )


#### The very long (Weekly) tests ####

# Note: The cross product of all possible PARSEC tests is huge, and there is
# little value in doing all. Therefore a sensible selection covering all
# benchmarks have been chosen.
#
# Note: At present the MESI_Two_Level protocol does not appear to work
# correctly with the SwitchableProcessor. As such they are commented out. This
# issue is documented here: https://gem5.atlassian.net/browse/GEM5-1085.

test_parsec(
    boot_cpu="kvm",
    detailed_cpu="atomic",
    num_cpus=2,
    mem_system="classic",
    benchmark="blackscholes",
    size="simsmall",
    length=constants.very_long_tag,
)

# test_parsec(
#    boot_cpu="kvm",
#    detailed_cpu="timing",
#    num_cpus=1,
#    mem_system="mesi_two_level",
#    benchmark="bodytrack",
#    size="simsmall",
#    length=constants.very_long_tag,
# )

test_parsec(
    boot_cpu="kvm",
    detailed_cpu="o3",
    num_cpus=1,
    mem_system="classic",
    benchmark="canneal",
    size="simsmall",
    length=constants.very_long_tag,
)

# test_parsec(
#    boot_cpu="kvm",
#    detailed_cpu="kvm",
#    num_cpus=8,
#    mem_system="mesi_two_level",
#    benchmark="dedup",
#    size="simsmall",
#    length=constants.very_long_tag,
# )

test_parsec(
    boot_cpu="kvm",
    detailed_cpu="atomic",
    num_cpus=2,
    mem_system="classic",
    benchmark="facesim",
    size="simsmall",
    length=constants.very_long_tag,
)

# test_parsec(
#    boot_cpu="kvm",
#    detailed_cpu="timing",
#    num_cpus=1,
#    mem_system="mesi_two_level",
#    benchmark="ferret",
#    size="simsmall",
#    length=constants.very_long_tag,
# )

test_parsec(
    boot_cpu="kvm",
    detailed_cpu="o3",
    num_cpus=1,
    mem_system="classic",
    benchmark="fluidanimate",
    size="simsmall",
    length=constants.very_long_tag,
)

# test_parsec(
#    boot_cpu="kvm",
#    detailed_cpu="kvm",
#    num_cpus=8,
#    mem_system="mesi_two_level",
#    benchmark="freqmine",
#    size="simsmall",
#    length=constants.very_long_tag,
# )


test_parsec(
    boot_cpu="kvm",
    detailed_cpu="atomic",
    num_cpus=2,
    mem_system="classic",
    benchmark="raytrace",
    size="simsmall",
    length=constants.very_long_tag,
)

# test_parsec(
#    boot_cpu="kvm",
#    detailed_cpu="timing",
#    num_cpus=1,
#    mem_system="mesi_two_level",
#    benchmark="streamcluster",
#    size="simsmall",
#    length=constants.very_long_tag,
# )

test_parsec(
    boot_cpu="kvm",
    detailed_cpu="o3",
    num_cpus=1,
    mem_system="classic",
    benchmark="swaptions",
    size="simsmall",
    length=constants.very_long_tag,
)

# test_parsec(
#    boot_cpu="kvm",
#    detailed_cpu="kvm",
#    num_cpus=8,
#    mem_system="mesi_two_level",
#    benchmark="vips",
#    size="simsmall",
#    length=constants.very_long_tag,
# )

# test_parsec(
#    boot_cpu="kvm",
#    detailed_cpu="timing",
#    num_cpus=1,
#    mem_system="mesi_two_level",
#    benchmark="x264",
#    size="simsmall",
#    length=constants.very_long_tag,
# )
