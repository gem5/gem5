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

"""Shared command-line and system construction for x86 GPU examples."""

import argparse
from pathlib import Path

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.memory import HBM2Stack
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import (
    CPUTypes,
    get_cpu_type_from_str,
)
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.prebuilt.viper.board import ViperBoard
from gem5.prebuilt.viper.cpu_cache_hierarchy import ViperCPUCacheHierarchy
from gem5.resources.resource import (
    DiskImageResource,
    FileResource,
    obtain_resource,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

GPUFS_DISK_RESOURCE = "x86-ubuntu-24.04-gpu-img"
GPUFS_KERNEL_RESOURCE = "x86-linux-kernel-6.8.0-gpu"
GPUFS_DISK_RESOURCE_VERSION = "1.0.0"
GPUFS_KERNEL_RESOURCE_VERSION = "1.0.0"


def create_parser(description):
    """Create the common parser used by the x86 GPU examples."""

    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--image",
        type=Path,
        help=(
            "local GPUFS disk image; otherwise obtain "
            f"{GPUFS_DISK_RESOURCE}"
        ),
    )
    parser.add_argument(
        "--kernel",
        type=Path,
        help=(
            "local GPUFS kernel; otherwise obtain " f"{GPUFS_KERNEL_RESOURCE}"
        ),
    )
    parser.add_argument(
        "--app",
        type=Path,
        help="GPU application, Python script, or shell script to run",
    )
    parser.add_argument(
        "--opts",
        default="",
        help="arguments passed to the GPU application",
    )
    parser.add_argument(
        "--resource-directory",
        type=Path,
        help="directory used for resources obtained by gem5",
    )
    parser.add_argument(
        "--disk-resource-version",
        default=GPUFS_DISK_RESOURCE_VERSION,
        help="version of the GPUFS disk image resource",
    )
    parser.add_argument(
        "--kernel-resource-version",
        default=GPUFS_KERNEL_RESOURCE_VERSION,
        help="version of the GPUFS kernel resource",
    )
    parser.add_argument(
        "--cpu-type",
        choices=(CPUTypes.ATOMIC.value, CPUTypes.KVM.value),
        default=CPUTypes.KVM.value,
        help="CPU model used to boot the GPUFS image",
    )
    parser.add_argument(
        "--num-cus",
        type=int,
        help="override the GPU model's default compute-unit count",
    )
    parser.add_argument(
        "--kvm-perf",
        action="store_true",
        help="use KVM perf counters for accurate GPU instructions and cycles",
    )
    parser.add_argument(
        "--kernel-arg",
        action="append",
        default=[],
        help="append an argument to the default Linux kernel command line",
    )
    checkpoint = parser.add_mutually_exclusive_group()
    checkpoint.add_argument(
        "--checkpoint-directory",
        type=Path,
        help="local checkpoint directory to restore",
    )
    checkpoint.add_argument(
        "--checkpoint-resource",
        help="CheckpointResource ID to obtain and restore",
    )
    parser.add_argument(
        "--checkpoint-resource-version",
        help="checkpoint resource version; default is the latest compatible",
    )
    return parser


def _resource_kwargs(args, resource_version):
    kwargs = {"resource_version": resource_version}
    if args.resource_directory:
        args.resource_directory.mkdir(parents=True, exist_ok=True)
        kwargs["resource_directory"] = str(args.resource_directory)
    return kwargs


def obtain_gpu_fs_resources(args):
    """Return the paired GPUFS disk and kernel from local or gem5 resources."""

    disk = (
        DiskImageResource(local_path=args.image, root_partition="1")
        if args.image
        else obtain_resource(
            GPUFS_DISK_RESOURCE,
            **_resource_kwargs(args, args.disk_resource_version),
        )
    )
    kernel = (
        FileResource(local_path=args.kernel)
        if args.kernel
        else obtain_resource(
            GPUFS_KERNEL_RESOURCE,
            **_resource_kwargs(args, args.kernel_resource_version),
        )
    )
    return disk, kernel


def create_board(args, gpu_class):
    """Create the common ViperBoard topology used by the x86 GPU examples."""

    processor = SimpleProcessor(
        cpu_type=get_cpu_type_from_str(args.cpu_type),
        isa=ISA.X86,
        num_cores=1,
        clk_freq="3GHz",
    )
    for core in processor.cores:
        if core.is_kvm_core():
            core.get_simobject().usePerf = args.kvm_perf

    gpu_args = {"gpu_memory": HBM2Stack(size="16GiB")}
    if args.num_cus is not None:
        gpu_args["num_cus"] = args.num_cus
    gpu = gpu_class(**gpu_args)

    board = ViperBoard(
        clk_freq="3GHz",
        processor=processor,
        memory=SingleChannelDDR4_2400(size="8GiB"),
        cache_hierarchy=ViperCPUCacheHierarchy(),
        gpus=[gpu],
    )
    return board, gpu


def obtain_checkpoint(args):
    """Return the local or obtained checkpoint selected by the user."""

    if args.checkpoint_directory:
        return args.checkpoint_directory
    if args.checkpoint_resource:
        return obtain_resource(
            args.checkpoint_resource,
            **_resource_kwargs(args, args.checkpoint_resource_version),
        )
    return None


def set_workload(args, board, gpu, **kwargs):
    """Set the GPUFS workload while permitting checkpoint-tool overrides."""

    disk, kernel = obtain_gpu_fs_resources(args)
    kernel_args = board.get_default_kernel_args()
    kernel_args.extend(args.kernel_arg)
    if args.app:
        kwargs["readfile_contents"] = board.make_gpu_app(
            gpu, str(args.app), args.opts
        )
    board.set_kernel_disk_workload(
        kernel=kernel,
        disk_image=disk,
        kernel_args=kernel_args,
        checkpoint=obtain_checkpoint(args),
        **kwargs,
    )


def run_gpu_example(gpu_class, description):
    """Parse common options and run one x86 GPU full-system simulation."""

    requires(coherence_protocol_required=CoherenceProtocol.GPU_VIPER)
    parser = create_parser(description)
    args = parser.parse_args()
    if not args.app and not (
        args.checkpoint_directory or args.checkpoint_resource
    ):
        parser.error("--app is required unless a checkpoint is being restored")

    board, gpu = create_board(args, gpu_class)
    set_workload(args, board, gpu)
    Simulator(board=board).run()
