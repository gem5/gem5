# Copyright (c) 2021 Advanced Micro Devices, Inc.
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

# System includes
import argparse
import hashlib
import math

# gem5 related
import m5
from m5.objects import *
from m5.util import addToPath

# gem5 options and objects
addToPath("../../")
from amd import AmdGPUOptions
from common import (
    GPUTLBConfig,
    GPUTLBOptions,
    ObjectList,
    Options,
    Simulation,
)
from ruby import Ruby

# GPU FS related
from system.system import makeGpuFSSystem


def addRunFSOptions(parser):
    parser.add_argument(
        "--script",
        default=None,
        help="Script to execute in the simulated system",
    )
    parser.add_argument(
        "--host-parallel",
        default=False,
        action="store_true",
        help="Run multiple host threads in KVM mode",
    )
    parser.add_argument(
        "--restore-dir",
        type=str,
        default=None,
        help="Directory to restore checkpoints from",
    )
    parser.add_argument(
        "--disk-image",
        default="",
        help="The boot disk image to mount (/dev/sda)",
    )
    parser.add_argument(
        "--second-disk",
        default=None,
        help="The second disk image to mount (/dev/sdb)",
    )
    parser.add_argument("--kernel", default=None, help="Linux kernel to boot")
    parser.add_argument("--gpu-rom", default=None, help="GPU BIOS to load")
    parser.add_argument(
        "--gpu-mmio-trace", default=None, help="GPU MMIO trace to load"
    )
    parser.add_argument(
        "--checkpoint-before-mmios",
        default=False,
        action="store_true",
        help="Take a checkpoint before driver sends MMIOs. "
        "This is used to switch out of KVM mode and into "
        "timing mode required to read the VGA ROM on boot.",
    )
    parser.add_argument(
        "--cpu-topology",
        type=str,
        default="Crossbar",
        help="Network topology to use for CPU side. "
        "Check configs/topologies for complete set",
    )
    parser.add_argument(
        "--gpu-topology",
        type=str,
        default="Crossbar",
        help="Network topology to use for GPU side. "
        "Check configs/topologies for complete set",
    )
    parser.add_argument(
        "--dgpu-mem-size",
        action="store",
        type=str,
        default="16GB",
        help="Specify the dGPU physical memory size",
    )
    parser.add_argument(
        "--dgpu-num-dirs",
        type=int,
        default=1,
        help="Set the number of dGPU directories (memory controllers",
    )
    parser.add_argument(
        "--dgpu-mem-type",
        default="HBM_1000_4H_1x128",
        choices=ObjectList.mem_list.get_names(),
        help="type of memory to use",
    )

    # These are the models that are both supported in gem5 and supported
    # by the versions of ROCm supported by gem5 in full system mode. For
    # other gfx versions there is some support in syscall emulation mode.
    parser.add_argument(
        "--gpu-device",
        default="Vega10",
        choices=["Vega10", "MI100", "MI200", "MI300X"],
        help="GPU model to run: Vega10 (gfx900), MI100 (gfx908), MI200 "
        "(gfx90a), or MI300X (gfx942).",
    )

    parser.add_argument(
        "--debug-at-gpu-task",
        type=int,
        default=-1,
        help="Turn on debug flags starting with this task (counting both blit"
        " and non-blit kernels)",
    )

    parser.add_argument(
        "--exit-at-gpu-task",
        type=int,
        default=-1,
        help="Exit simulation after running this many tasks (counting both "
        "blit and non-blit kernels)",
    )

    parser.add_argument(
        "--exit-after-gpu-kernel",
        type=int,
        default=-1,
        help="Exit simulation after completing this (non-blit) kernel",
    )

    parser.add_argument(
        "--skip-until-gpu-kernel",
        type=int,
        default=0,
        help="Skip (non-blit) kernels until reaching this kernel. Note that "
        "this can impact correctness (the skipped kernels are completely "
        "skipped, not fast forwarded)",
    )

    parser.add_argument(
        "--root-partition",
        type=str,
        default="/dev/sda1",
        help="Root partition of disk image",
    )

    parser.add_argument(
        "--disable-avx",
        action="store_true",
        default=False,
        help="Disables AVX. AVX is used in some ROCm libraries but "
        "does not have checkpointing support yet. If simulation either "
        "creates a checkpoint or restores from one, then AVX needs to "
        "be disabled for correct functionality ",
    )

    parser.add_argument(
        "--kvm-perf",
        default=False,
        action="store_true",
        help="Enable KVM perf counters",
    )

    parser.add_argument(
        "--tcp-rp",
        type=str,
        default="TreePLRURP",
        help="cache replacement policy" "policy for tcp",
    )

    parser.add_argument(
        "--tcc-rp",
        type=str,
        default="TreePLRURP",
        help="cache replacement policy" "policy for tcc",
    )

    # sqc rp both changes sqc rp and scalar cache rp
    parser.add_argument(
        "--sqc-rp",
        type=str,
        default="TreePLRURP",
        help="cache replacement policy" "policy for sqc",
    )


def runGpuFSSystem(args):
    """
    This function can be called by higher level scripts designed to simulate
    specific devices. As a result the scripts typically hard code some args
    that should not be changed by the user.
    """

    # GPUFS is primarily designed to use the X86 KVM CPU. This model needs to
    # use multiple event queues when more than one CPU is simulated. Force it
    # on if that is the case.
    if ObjectList.is_kvm_cpu(ObjectList.cpu_list.get(args.cpu_type)):
        args.host_parallel = True if args.num_cpus > 1 else False

    # These are used by the protocols. They should not be set by the user.
    n_cu = args.num_compute_units
    args.num_sqc = int(math.ceil(float(n_cu) / args.cu_per_sqc))
    args.num_scalar_cache = int(
        math.ceil(float(n_cu) / args.cu_per_scalar_cache)
    )

    # Verify MMIO trace is valid. This is only needed for Vega10 simulations.
    # The md5sum refers to the md5sum of the Vega10 MMIO hardware trace in
    # the gem5-resources repository. By checking it here, we avoid potential
    # errors that would cause the driver not to load and simulations to fail.
    if args.gpu_device == "Vega10":
        mmio_file = open(args.gpu_mmio_trace, "rb")
        mmio_md5 = hashlib.md5(mmio_file.read()).hexdigest()
        if mmio_md5 != "c4ff3326ae8a036e329b8b595c83bd6d":
            m5.util.panic("MMIO file does not match gem5 resources")

    system = makeGpuFSSystem(args)

    root = Root(
        full_system=True,
        system=system,
        time_sync_enable=True,
        time_sync_period="1000us",
    )

    if args.host_parallel:
        root.sim_quantum = int(1e8)

    if args.script is not None:
        system.readfile = args.script

    if args.restore_dir is None:
        m5.instantiate()
    else:
        m5.instantiate(args.restore_dir)

    print("Running the simulation")
    sim_ticks = args.abs_max_tick
    kernels_completed = 0
    tasks_completed = 0
    if args.debug_at_gpu_task != -1:
        m5.trace.disable()

    exit_event = m5.simulate(sim_ticks)

    # Keep executing while there is something to do
    while True:
        if (
            exit_event.getCause() == "m5_exit instruction encountered"
            or exit_event.getCause() == "user interrupt received"
            or exit_event.getCause() == "simulate() limit reached"
        ):
            break
        elif "checkpoint" in exit_event.getCause():
            assert args.checkpoint_dir is not None
            m5.checkpoint(args.checkpoint_dir)
            break
        elif "GPU Kernel Completed" in exit_event.getCause():
            if kernels_completed == args.exit_after_gpu_kernel:
                print(f"Exiting after GPU kernel {kernels_completed}")
                break
            kernels_completed += 1
            tasks_completed += 1
        elif "GPU Blit Kernel Completed" in exit_event.getCause():
            tasks_completed += 1
        elif "Skipping GPU Kernel" in exit_event.getCause():
            print(f"Skipping GPU kernel {kernels_completed}")
            kernels_completed += 1
            tasks_completed += 1
        else:
            print(
                f"Unknown exit event: {exit_event.getCause()}. Continuing..."
            )

        if tasks_completed == args.debug_at_gpu_task:
            print(f"Enabling debug flags @ GPU task {tasks_completed}")
            m5.trace.enable()
        if tasks_completed == args.exit_at_gpu_task:
            print(f"Exiting @ GPU task {tasks_completed}")
            break

        exit_event = m5.simulate(sim_ticks - m5.curTick())

    print(
        "Exiting @ tick %i because %s" % (m5.curTick(), exit_event.getCause())
    )


if __name__ == "__m5_main__":
    # Add gpufs, common, ruby, amdgpu, and gpu tlb args
    parser = argparse.ArgumentParser()
    addRunFSOptions(parser)
    Options.addCommonOptions(parser)
    Ruby.define_options(parser)
    AmdGPUOptions.addAmdGPUOptions(parser)
    GPUTLBOptions.tlb_options(parser)

    args = parser.parse_args()

    runGpuFSSystem(args)
