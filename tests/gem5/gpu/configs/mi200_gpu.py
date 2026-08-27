# Copyright (c) 2026 The Regents of The University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution; neither the name of
# the copyright holders nor the names of its contributors may be used to
# endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Run the standard-library MI200 GPUFS driver smoke configuration."""

import argparse
from pathlib import Path
from tempfile import TemporaryDirectory

from gpu_fs import (
    gpu_fs_boot_options,
    obtain_gpu_fs_resources,
)

from gem5.components.devices.gpus.amdgpu import MI210
from gem5.components.memory import HBM2Stack
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.prebuilt.viper.board import ViperBoard
from gem5.prebuilt.viper.cpu_cache_hierarchy import ViperCPUCacheHierarchy
from gem5.simulate.simulator import Simulator

parser = argparse.ArgumentParser()
parser.add_argument(
    "--resource-directory",
    type=Path,
    default=Path(__file__).resolve().parent.parent / "resources",
)
args = parser.parse_args()

disk, kernel = obtain_gpu_fs_resources(args.resource_directory)

processor = SimpleProcessor(
    cpu_type=CPUTypes.ATOMIC,
    isa=ISA.X86,
    num_cores=1,
    clk_freq="3GHz",
)
gpu = MI210(
    gpu_memory=HBM2Stack(size="16GiB"),
    # Four CUs retain one complete SQC/scalar-cache group.
    num_cus=4,
)
board = ViperBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=SingleChannelDDR4_2400(size="8GiB"),
    cache_hierarchy=ViperCPUCacheHierarchy(),
    gpus=[gpu],
)

temporary_directory = TemporaryDirectory()
application = Path(temporary_directory.name) / "check-mi200.sh"
application.write_text(
    """#!/bin/bash
set -u

if rocminfo_output=$(rocminfo 2>&1) && \
        grep -Fq -- "gfx90a" <<< "$rocminfo_output"; then
    echo "GPU full-system test passed: gfx90a"
else
    echo "$rocminfo_output"
    echo "GPU full-system test failed: gfx90a"
    exit 1
fi
""",
    encoding="utf-8",
)

kernel_args = board.get_default_kernel_args()
kernel_args.extend(gpu_fs_boot_options())
board.set_kernel_disk_workload(
    kernel=kernel,
    disk_image=disk,
    kernel_args=kernel_args,
    readfile_contents=board.make_gpu_app(gpu, str(application), ""),
)

Simulator(board=board).run()
