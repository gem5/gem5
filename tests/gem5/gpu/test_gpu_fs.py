# Copyright (c) 2026 The Regents of The University of California
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

import re

from testlib import *

serial_output = "board.pc.com_1.device"
resource_directory = (
    config.bin_path
    if config.bin_path
    else joinpath(absdirpath(__file__), "resources")
)
gpu_config_directory = joinpath(
    config.base_dir, "configs", "example", "gem5_library"
)
gpu_check_script = joinpath(absdirpath(__file__), "gpu_fs_check.sh")

mi355x_checkpoint_resource = "x86-mi355x-gpu-fs-smoke-checkpoint"
mi355x_checkpoint_resource_version = "1.0.0"
gpu_fs_boot_services = (
    "multipathd.service",
    "snapd.service",
    "snapd.seeded.service",
    "systemd-networkd.service",
    "systemd-networkd-wait-online.service",
    "systemd-resolved.service",
    "systemd-timesyncd.service",
    "thermald.service",
    "unattended-upgrades.service",
)


def gpu_fs_boot_args():
    """Return example arguments that disable unrelated guest services."""

    args = []
    for service in gpu_fs_boot_services:
        args.extend(("--kernel-arg", f"systemd.mask={service}"))
    return tuple(args)


def gpu_fs_test(name, config_file, expected_gpu, length):
    """Register a GPUFS driver smoke test for one simulated GPU model."""

    gem5_verify_config(
        name=name,
        verifiers=(
            verifier.MatchFileRegex(
                re.compile(rf"^GPU full-system test passed: {expected_gpu}$"),
                (serial_output,),
            ),
        ),
        config=joinpath(gpu_config_directory, config_file),
        config_args=(
            "--resource-directory",
            resource_directory,
            "--cpu-type",
            "atomic",
            "--num-cus",
            "4",
            "--app",
            gpu_check_script,
            "--opts",
            expected_gpu,
        )
        + gpu_fs_boot_args(),
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=constants.supported_hosts,
        length=length,
    )


gpu_fs_test(
    name="gpu-fs-mi200-stdlib-driver",
    config_file="x86-mi200-gpu.py",
    expected_gpu="gfx90a",
    # Retain a full boot in Daily to cover the CDNA2 driver/device path.
    length=constants.long_tag,
)

gpu_fs_test(
    name="gpu-fs-mi355x-stdlib-driver",
    config_file="x86-mi355x-gpu.py",
    expected_gpu="gfx950",
    # Retain a full boot in Daily to cover the CDNA4 driver/device path.
    length=constants.long_tag,
)

gem5_verify_config(
    name="gpu-fs-mi355x-checkpoint-kernel",
    verifiers=(
        verifier.MatchFileRegex(
            re.compile(r"^GPU checkpoint restore test passed$"),
            (serial_output,),
        ),
    ),
    config=joinpath(gpu_config_directory, "x86-mi355x-gpu.py"),
    config_args=(
        "--resource-directory",
        resource_directory,
        "--cpu-type",
        "atomic",
        "--num-cus",
        "4",
        "--checkpoint-resource",
        mi355x_checkpoint_resource,
        "--checkpoint-resource-version",
        mi355x_checkpoint_resource_version,
        "--kernel-arg",
        "init=/home/gem5/run_gem5_app.sh",
    ),
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)
