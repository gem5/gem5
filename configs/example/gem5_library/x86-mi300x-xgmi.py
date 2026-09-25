# Copyright (c) 2025 Advanced Micro Devices, Inc.
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
    configs/example/gem5_library/x86-mi300x-xgmi.py
    --image <disk image>
    --kernel <kernel>
    --app <gpu application>
```
"""

import argparse

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.devices.gpus.amdgpu import MI300X
from gem5.components.devices.gpus.xgmi import xGMIHive
from gem5.components.memory import HBM2Stack
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

requires(coherence_protocol_required=CoherenceProtocol.GPU_VIPER)

parser = argparse.ArgumentParser()
parser.add_argument("--image", type=str, required=True)
parser.add_argument("--kernel", type=str, required=True)
parser.add_argument("--app", type=str, required=True)
parser.add_argument("--kvm-perf", default=False, action="store_true")
args = parser.parse_args()

memory = SingleChannelDDR4_2400(size="8GiB")
processor = SimpleProcessor(cpu_type=CPUTypes.KVM, isa=ISA.X86, num_cores=1)

for core in processor.cores:
    if core.is_kvm_core():
        core.get_simobject().usePerf = args.kvm_perf

hive = xGMIHive()
gpu0 = MI300X(gpu_memory=HBM2Stack(size="4GiB"), num_cus=4, xgmi_hive=hive)
gpu1 = MI300X(gpu_memory=HBM2Stack(size="4GiB"), num_cus=4, xgmi_hive=hive)
gpu0.device.force_p2p_gart_remap = True
gpu0.device.p2p_peer_id = 1
gpu1.device.force_p2p_gart_remap = True
gpu1.device.p2p_peer_id = 0

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
    gpus=[gpu0, gpu1],
    xgmi_hives=[hive],
)

disk = DiskImageResource(local_path=args.image, root_partition="1")
kernel = FileResource(local_path=args.kernel)
board.set_kernel_disk_workload(
    kernel=kernel,
    disk_image=disk,
    readfile_contents=board.make_gpu_app(gpu0, args.app, ""),
)
board.append_kernel_arg("idle=nomwait")
board.append_kernel_arg("clearcpuid=clflushopt")

simulator = Simulator(board=board)
simulator.run()
