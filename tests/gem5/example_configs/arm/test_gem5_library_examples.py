# Copyright (c) 2021-2026 The Regents of the University of California
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
import platform
import re
import shutil
import subprocess
import sys

from testlib import (
    config,
    constants,
    gem5_verify_config,
    joinpath,
    verifier,
)
from testlib.configuration import constants

hello_verifier = verifier.MatchRegex(re.compile(r"Hello world!"))
save_checkpoint_verifier = verifier.MatchRegex(
    re.compile(r"Done taking a checkpoint")
)


def supports_apple_virt():
    if (
        sys.platform != "darwin"
        or platform.machine().lower() not in ("arm64", "aarch64")
        or shutil.which("codesign") is None
    ):
        return False

    try:
        return (
            subprocess.check_output(
                ("sysctl", "-n", "kern.hv_support"), text=True
            ).strip()
            == "1"
        )
    except (OSError, subprocess.CalledProcessError):
        return False


def supports_arm_kvm():
    return (
        sys.platform.startswith("linux")
        and platform.machine().lower() in ("armv7l", "aarch64", "arm64")
        and os.access("/dev/kvm", mode=os.R_OK | os.W_OK)
    )


gem5_verify_config(
    name="test-gem5-library-example-arm-hello",
    fixtures=(),
    verifiers=(hello_verifier,),
    config=joinpath(
        config.base_dir, "configs", "example", "gem5_library", "arm-hello.py"
    ),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.quick_tag,
)


gem5_verify_config(
    name="test-gem5-library-example-arm-ubuntu-run-test",
    fixtures=(),
    verifiers=(),
    config=joinpath(
        config.base_dir,
        "configs",
        "example",
        "gem5_library",
        "arm-ubuntu-run.py",
    ),
    config_args=[],
    valid_isas=(constants.all_compiled_tag,),
    valid_hosts=constants.supported_hosts,
    length=constants.long_tag,
)


if supports_apple_virt():
    for example in ("apple_virt_simple", "apple_virt_kernel_disk"):
        gem5_verify_config(
            name=f"test-gem5-library-example-{example.replace('_', '-')}",
            fixtures=(),
            verifiers=(),
            config=joinpath(
                config.base_dir,
                "configs",
                "example",
                "gem5_library",
                f"{example}.py",
            ),
            config_args=[],
            valid_isas=(constants.all_compiled_tag,),
            valid_hosts=(constants.host_arm_tag,),
            length=constants.quick_tag,
        )

    gem5_verify_config(
        name="test-gem5-library-example-apple-virt-switch",
        fixtures=(),
        verifiers=(
            verifier.MatchRegex(
                re.compile("AppleVirt switching completed successfully")
            ),
        ),
        config=joinpath(
            config.base_dir,
            "configs",
            "example",
            "gem5_library",
            "apple_virt_switch.py",
        ),
        config_args=[],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=(constants.host_arm_tag,),
        length=constants.quick_tag,
    )


if supports_apple_virt() or supports_arm_kvm():
    gem5_verify_config(
        name="test-gem5-library-example-virtualized-processor-switch",
        fixtures=(),
        verifiers=(
            verifier.MatchRegex(
                re.compile(
                    "Host-resolved virtualized processor completed "
                    "successfully"
                )
            ),
        ),
        config=joinpath(
            config.base_dir,
            "configs",
            "example",
            "gem5_library",
            "virtualized_processor_switch.py",
        ),
        config_args=[],
        valid_isas=(constants.all_compiled_tag,),
        valid_hosts=(constants.host_arm_tag,),
        length=constants.quick_tag,
        uses_kvm=supports_arm_kvm(),
    )
