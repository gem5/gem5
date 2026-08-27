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

"""Run the standard-library MI355X GPUFS smoke configurations."""

import argparse
import base64
from pathlib import Path
from tempfile import TemporaryDirectory

from gpu_fs import (
    gpu_fs_boot_options,
    obtain_gpu_fs_resources,
)

from gem5.components.devices.gpus.amdgpu import MI355X
from gem5.components.memory import HBM2Stack
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.prebuilt.viper.board import ViperBoard
from gem5.prebuilt.viper.cpu_cache_hierarchy import ViperCPUCacheHierarchy
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator

CHECKPOINT_RESOURCE_VERSION = "1.0.0"

parser = argparse.ArgumentParser()
parser.add_argument(
    "--resource-directory",
    type=Path,
    default=Path(__file__).resolve().parent.parent / "resources",
)
parser.add_argument(
    "--mode",
    choices=("driver", "create-loader", "create-kernel", "restore"),
    default="driver",
)
parser.add_argument(
    "--checkpoint-directory",
    type=Path,
    help="Local MI355X checkpoint to restore",
)
parser.add_argument(
    "--checkpoint-resource",
    help="MI355X CheckpointResource ID to obtain and restore",
)
parser.add_argument(
    "--checkpoint-output",
    type=Path,
    help="Directory in which to save a generated checkpoint",
)
parser.add_argument(
    "--gpu-application-binary",
    type=Path,
    help="Initialized HIP loader used to create the loader checkpoint",
)
parser.add_argument(
    "--gpu-kernel-binary",
    type=Path,
    help="gfx950 code object loaded before creating the final checkpoint",
)
args = parser.parse_args()

if args.checkpoint_directory and args.checkpoint_resource:
    parser.error(
        "--checkpoint-directory and --checkpoint-resource are mutually "
        "exclusive"
    )
if args.mode == "create-loader":
    if not args.checkpoint_output or not args.gpu_application_binary:
        parser.error(
            "create-loader requires --checkpoint-output and "
            "--gpu-application-binary"
        )
elif args.mode == "create-kernel":
    if (
        not args.checkpoint_output
        or not args.checkpoint_directory
        or not args.gpu_kernel_binary
    ):
        parser.error(
            "create-kernel requires --checkpoint-output, "
            "--checkpoint-directory, and --gpu-kernel-binary"
        )
elif args.mode == "restore":
    if not (args.checkpoint_directory or args.checkpoint_resource):
        parser.error(
            "restore requires --checkpoint-directory or "
            "--checkpoint-resource"
        )

disk, kernel = obtain_gpu_fs_resources(args.resource_directory)

processor = SimpleProcessor(
    cpu_type=CPUTypes.ATOMIC,
    isa=ISA.X86,
    num_cores=1,
    clk_freq="3GHz",
)
gpu = MI355X(
    gpu_memory=HBM2Stack(size="16GiB"),
    # This test covers the driver and one kernel, not the full MI355X
    # topology. Four CUs retain one complete SQC/scalar-cache group.
    num_cus=4,
)
board = ViperBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=SingleChannelDDR4_2400(size="8GiB"),
    cache_hierarchy=ViperCPUCacheHierarchy(),
    gpus=[gpu],
)

checkpoint = args.checkpoint_directory
if args.checkpoint_resource:
    checkpoint = obtain_resource(
        args.checkpoint_resource,
        resource_directory=str(args.resource_directory),
        resource_version=CHECKPOINT_RESOURCE_VERSION,
    )

temporary_directory = TemporaryDirectory()
temporary_path = Path(temporary_directory.name)
workload_kwargs = {}


def make_checkpoint_boot_script(application: Path) -> Path:
    script = temporary_path / "checkpoint-boot.sh"
    script.write_text(
        "mkdir -p /proc /sys /dev/pts\n"
        "mount -t proc proc /proc\n"
        "mount -t sysfs sysfs /sys\n"
        "mount -t devpts devpts /dev/pts\n"
        + gpu.get_driver_command()
        + "\n"
        + board.make_gpu_app(gpu, str(application), ""),
        encoding="utf-8",
    )
    return script


if args.mode == "driver":
    application = temporary_path / "check-mi355x.sh"
    application.write_text(
        """#!/bin/bash
set -u

if rocminfo_output=$(rocminfo 2>&1) && \
        grep -Fq -- "gfx950" <<< "$rocminfo_output"; then
    echo "GPU full-system test passed: gfx950"
else
    echo "$rocminfo_output"
    echo "GPU full-system test failed: gfx950"
    exit 1
fi
""",
        encoding="utf-8",
    )
    workload_kwargs["readfile_contents"] = board.make_gpu_app(
        gpu, str(application), ""
    )
elif args.mode == "create-loader":
    # Direct init does not mount the virtual filesystems that ROCr uses to
    # enumerate KFD topology. Supply a complete boot script as a host file so
    # the mounts precede the driver command normally prepended by ViperBoard.
    workload_kwargs["readfile"] = str(
        make_checkpoint_boot_script(args.gpu_application_binary)
    )
elif args.mode == "create-kernel":
    encoded_kernel = temporary_path / "gpu-kernel.b64"
    encoded_kernel.write_text(
        base64.b64encode(args.gpu_kernel_binary.read_bytes()).decode(),
        encoding="ascii",
    )
    workload_kwargs["readfile"] = str(encoded_kernel)

kernel_args = board.get_default_kernel_args()
if args.mode == "driver":
    kernel_args.extend(gpu_fs_boot_options())
else:
    # The checkpoint generator does not need systemd or getty. The GPUFS
    # image's launcher reads and executes the host-provided script.
    kernel_args.append("init=/home/gem5/run_gem5_app.sh")

board.set_kernel_disk_workload(
    kernel=kernel,
    disk_image=disk,
    kernel_args=kernel_args,
    checkpoint=checkpoint,
    **workload_kwargs,
)

simulator = None


def save_checkpoint_and_exit():
    """Save the requested resource checkpoint and stop this stage."""

    simulator.save_checkpoint(args.checkpoint_output)
    return True


on_exit_event = None
if args.mode == "create-loader":
    on_exit_event = {ExitEvent.CHECKPOINT: save_checkpoint_and_exit}
elif args.mode == "create-kernel":
    # The restored loader requests this checkpoint after one GPU kernel has
    # completed successfully. The checkpoint therefore contains initialized
    # ROCr state, but no in-flight GPU dispatch state.
    on_exit_event = {ExitEvent.CHECKPOINT: save_checkpoint_and_exit}

simulator = Simulator(board=board, on_exit_event=on_exit_event)
simulator.run()
