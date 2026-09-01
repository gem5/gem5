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

"""Boot a local AArch64 kernel and disk image with AppleVirtCPU.

Usage
-----

```
scons build/ARM/gem5.opt -j$(sysctl -n hw.ncpu)
./build/ARM/gem5.opt configs/example/apple_virt_kernel_disk.py \
    --kernel /path/to/vmlinux.arm64 \
    --disk-image /path/to/arm64-disk.img \
    --bootloader /path/to/boot.arm64
```

The kernel, disk image, and bootloader must be compatible with the
``VExpress_GEM5_V1`` platform. The simulated terminal is written to
``m5out/board.terminal``.
"""

import argparse

from m5.objects import (
    ArmDefaultRelease,
    VExpress_GEM5_V1,
)

from gem5.components.boards.arm_board import ArmBoard
from gem5.components.cachehierarchies.classic.private_l1_private_l2_cache_hierarchy import (
    PrivateL1PrivateL2CacheHierarchy,
)
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import (
    BootloaderResource,
    DiskImageResource,
    KernelResource,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

parser = argparse.ArgumentParser(
    description="Boot local AArch64 artifacts with AppleVirtCPU."
)
parser.add_argument("--kernel", required=True, help="Path to the ARM kernel.")
parser.add_argument(
    "--disk-image", required=True, help="Path to the ARM disk image."
)
parser.add_argument(
    "--bootloader", required=True, help="Path to the ARM bootloader."
)
parser.add_argument(
    "--root-partition",
    default="1",
    help="Root partition number in the disk image (default: 1).",
)
args = parser.parse_args()

requires(isa_required=ISA.ARM, apple_virt_required=True)

cache_hierarchy = PrivateL1PrivateL2CacheHierarchy(
    l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
)
memory = DualChannelDDR4_2400(size="2GiB")
processor = SimpleProcessor(
    cpu_type=CPUTypes.APPLE_VIRT,
    isa=ISA.ARM,
    num_cores=1,
    clk_freq="3GHz",
)

board = ArmBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
    release=ArmDefaultRelease.for_kvm(),
    platform=VExpress_GEM5_V1(),
)

board.set_kernel_disk_workload(
    kernel=KernelResource(local_path=args.kernel, architecture=ISA.ARM),
    disk_image=DiskImageResource(
        local_path=args.disk_image, root_partition=args.root_partition
    ),
    bootloader=BootloaderResource(
        local_path=args.bootloader, architecture=ISA.ARM
    ),
)

Simulator(board=board).run()
