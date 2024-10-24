# Copyright (c) 2024 Advanced Micro Devices, Inc.
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

"""
Script to run a full system GPU simulation.

Usage:
------
```
scons build/VEGA_X86/gem5.opt
./build/VEGA_X86/gem5.opt
    configs/example/gem5_library/x86-viper-gpu.py
    --image <disk image>
    --kernel <kernel>
    --app <gpu application>
```

Example:
--------
```
./build/VEGA_X86/gem5.opt
    configs/example/gem5_library/x86-viper-gpu.py
    --image ./gem5-resources/src/x86-ubuntu-gpu-ml/disk-image/x86-ubuntu-gpu-ml
    --kernel ./gem5-resources/src/x86-ubuntu-gpu-ml/vmlinux-gpu-ml
    --app ./gem5-resources/src/gpu/square/bin.default/square.default
```
"""

import argparse

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.devices.gpus.amdgpu import MI300X
from gem5.components.memory.single_channel import SingleChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.prebuilt.viper.board import ViperBoard
from gem5.prebuilt.viper.cpu_cache_hierarchy import ViperCPUCacheHierarchy
from gem5.resources.resource import (
    DiskImageResource,
    FileResource,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.GPU_VIPER,
)

# Kernel, disk, and applications are obtained locally.
parser = argparse.ArgumentParser()

parser.add_argument(
    "--image",
    type=str,
    required=True,
    help="Full path to the gem5-resources x86-ubuntu-gpu-ml disk-image.",
)

parser.add_argument(
    "--kernel",
    type=str,
    required=True,
    help="Full path to the gem5-resources vmlinux-gpu-ml kernel.",
)

parser.add_argument(
    "--app",
    type=str,
    required=True,
    help="Path to GPU application, python script, or bash script to run",
)

parser.add_argument(
    "--kvm-perf",
    default=False,
    action="store_true",
    help="Use KVM perf counters to give accurate GPU insts/cycles with KVM",
)

args = parser.parse_args()

# stdlib only supports up to 3GiB currently. This will need to be expanded in
# the future.
memory = SingleChannelDDR4_2400(size="3GiB")

# Note: Only KVM and ATOMIC work due to buggy MOESI_AMD_Base protocol.
processor = SimpleProcessor(cpu_type=CPUTypes.KVM, isa=ISA.X86, num_cores=2)

for core in processor.cores:
    if core.is_kvm_core():
        core.get_simobject().usePerf = args.kvm_perf

# The GPU must be created first so we can assign CPU-side DMA ports to the
# CPU cache hierarchy.
gpu0 = MI300X()

cache_hierarchy = ViperCPUCacheHierarchy(
    l1d_size="32KiB",
    l1d_assoc=8,
    l1i_size="32KiB",
    l1i_assoc=8,
    l2_size="1MiB",
    l2_assoc=16,
    l3_size="16MiB",
    l3_assoc=16,
)

board = ViperBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    gpus=[gpu0],
)

# Example of using a local disk image resource
disk = DiskImageResource(local_path=args.image, root_partition="1")
kernel = FileResource(local_path=args.kernel)

board.set_kernel_disk_workload(
    kernel=kernel,
    disk_image=disk,
    readfile_contents=board.make_gpu_app(gpu0, args.app),
)

simulator = Simulator(board=board)
simulator.run()
