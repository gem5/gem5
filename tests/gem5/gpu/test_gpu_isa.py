# Copyright (c) 2026 Boyana Norris
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

"""MI300X instruction regressions using one GPUFS boot per program."""

import os
import re
import sys
import threading
from pathlib import Path

import testlib.log as log
from testlib import *
from testlib.fixture import Fixture
from testlib.helper import log_call


class GpuIsaApplications(Fixture):
    def __init__(self, directory, rocm_path):
        super().__init__(name="gfx942-isa-applications")
        self.directory = directory
        self.rocm_path = rocm_path
        self.lock = threading.Lock()
        self.ready = False

    def setup(self, testitem):
        with self.lock:
            if self.ready:
                return
            log_call(
                log.test_log,
                [
                    sys.executable,
                    joinpath(absdirpath(__file__), "isa", "build.py"),
                    "--rocm-path",
                    self.rocm_path,
                    "--output-dir",
                    self.directory,
                ],
                time=None,
            )
            self.ready = True


application_directory = joinpath(config.build_dir, "gpu-isa-regressions")
rocm_path = Path(os.environ.get("ROCM_PATH", "/opt/rocm"))
applications = GpuIsaApplications(application_directory, rocm_path)
resource_directory = (
    config.bin_path
    if config.bin_path
    else joinpath(absdirpath(__file__), "resources")
)

# Local resources avoid repeated downloads when running the same tests by
# hand; otherwise the example obtains its pinned GPUFS resource versions.
local_resource_args = []
for option, variable in (
    ("--image", "GEM5_GPU_TEST_IMAGE"),
    ("--kernel", "GEM5_GPU_TEST_KERNEL"),
):
    if path := os.environ.get(variable):
        local_resource_args.extend((option, path))

# Follow the existing KVM suites: collect tests only when their host
# prerequisites are present. Compilation failures still fail the fixture.
has_dependencies = (
    os.access("/dev/kvm", os.R_OK | os.W_OK)
    and (rocm_path / "lib" / "llvm" / "bin" / "clang++").is_file()
)
if not has_dependencies:
    log.test_log.message(
        "GPU ISA suites require KVM access and a ROCM_PATH toolchain"
    )

for program, marker in (
    ("division", "division"),
    ("lds_exchange", "lds-exchange"),
):
    if not has_dependencies:
        continue
    gem5_verify_config(
        name=f"gpu-fs-mi300x-isa-{marker}",
        fixtures=(applications,),
        verifiers=(
            verifier.MatchFileRegex(
                re.compile(rf"^GPU ISA {marker} passed: [1-9][0-9]* checks$"),
                ("board.pc.com_1.device",),
            ),
        ),
        config=joinpath(
            config.base_dir,
            "configs",
            "example",
            "gem5_library",
            "x86-mi300x-gpu.py",
        ),
        config_args=(
            "--resource-directory",
            resource_directory,
            "--skip-cache-hash-check",
            "--resource-lock-timeout",
            "3600",
            "--cpu-type",
            "kvm",
            "--num-cus",
            "4",
            "--app",
            joinpath(application_directory, f"{program}.sh"),
            *local_resource_args,
        ),
        valid_isas=(constants.all_compiled_tag, constants.vega_x86_tag),
        valid_hosts=(constants.host_x86_64_tag,),
        valid_variants=(constants.opt_tag,),
        length=constants.long_tag,
        uses_kvm=True,
    )
