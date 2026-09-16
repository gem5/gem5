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

r"""Create the two-stage MI355X checkpoint used by the GPUFS smoke test.

First, create a loader checkpoint from a full GPUFS boot::

    build/ALL/gem5.opt \
        configs/example/gem5_library/x86-mi355x-gpu-checkpoint.py \
        --mode=create-loader \
        --checkpoint-output=m5out/mi355x-loader \
        --gpu-application-binary=/path/to/hip-checkpoint-runner.py

Then restore the loader checkpoint, load and warm the ``gfx950`` kernel,
and create the final smoke-test checkpoint::

    build/ALL/gem5.opt \
        configs/example/gem5_library/x86-mi355x-gpu-checkpoint.py \
        --mode=create-kernel \
        --checkpoint-directory=m5out/mi355x-loader \
        --checkpoint-output=m5out/mi355x-smoke \
        --gpu-kernel-binary=/path/to/gpu-checkpoint-smoke

Both stages default to an Atomic CPU and four MI355X compute units. The
final checkpoint is specific to that topology and warmed kernel; it is not
a general post-boot MI355X checkpoint.
"""

import base64
from pathlib import Path
from tempfile import TemporaryDirectory

from x86_gpu import (
    create_board,
    create_parser,
    set_workload,
)

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.devices.gpus.amdgpu import MI355X
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(coherence_protocol_required=CoherenceProtocol.GPU_VIPER)

parser = create_parser(
    "Create a loader or warmed-kernel checkpoint for the MI355X GPUFS test."
)
parser.set_defaults(cpu_type="atomic", num_cus=4)
parser.add_argument(
    "--mode",
    choices=("create-loader", "create-kernel"),
    required=True,
)
parser.add_argument(
    "--checkpoint-output",
    type=Path,
    required=True,
    help="directory in which to save the generated checkpoint",
)
parser.add_argument(
    "--gpu-application-binary",
    type=Path,
    help="initialized HIP loader used to create the loader checkpoint",
)
parser.add_argument(
    "--gpu-kernel-binary",
    type=Path,
    help="gfx950 code object loaded before creating the final checkpoint",
)
args = parser.parse_args()

if args.app:
    parser.error("--app is not used when creating the checkpoint")

board, gpu = create_board(args, MI355X)

temporary_directory = TemporaryDirectory()
temporary_path = Path(temporary_directory.name)


def make_checkpoint_boot_script(application):
    """Create the direct-init script for the loader checkpoint stage."""

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


def prepare_loader_workload():
    """Validate and prepare the loader-checkpoint workload."""

    if args.checkpoint_directory or args.checkpoint_resource:
        parser.error("create-loader starts from a full boot, not a checkpoint")
    if not args.gpu_application_binary:
        parser.error("create-loader requires --gpu-application-binary")
    # Direct init does not mount the virtual filesystems that ROCr uses to
    # enumerate KFD topology. Mount them before loading the driver.
    return {
        "readfile": str(
            make_checkpoint_boot_script(args.gpu_application_binary)
        )
    }


def prepare_kernel_workload():
    """Validate and prepare the warmed-kernel checkpoint workload."""

    if not (args.checkpoint_directory or args.checkpoint_resource):
        parser.error(
            "create-kernel requires --checkpoint-directory or "
            "--checkpoint-resource"
        )
    if not args.gpu_kernel_binary:
        parser.error("create-kernel requires --gpu-kernel-binary")

    encoded_kernel = temporary_path / "gpu-kernel.b64"
    encoded_kernel.write_text(
        base64.b64encode(args.gpu_kernel_binary.read_bytes()).decode(),
        encoding="ascii",
    )
    return {"readfile": str(encoded_kernel)}


workload_args = {
    "create-loader": prepare_loader_workload,
    "create-kernel": prepare_kernel_workload,
}[args.mode]()

# The GPUFS image's launcher reads and executes the host-provided script.
args.kernel_arg.append("init=/home/gem5/run_gem5_app.sh")
set_workload(args, board, gpu, **workload_args)

simulator = None


def save_checkpoint_and_exit():
    """Save the requested resource checkpoint and stop this stage."""

    simulator.save_checkpoint(args.checkpoint_output)
    return True


simulator = Simulator(
    board=board,
    on_exit_event={ExitEvent.CHECKPOINT: save_checkpoint_and_exit},
)
simulator.run()
