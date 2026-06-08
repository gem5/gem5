# Copyright (c) 2021 The Regents of the University of California
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

"""
This script shows an example of running a full system RISCV Ubuntu boot
simulation using the gem5 library. This simulation boots Ubuntu 20.04 using
2 TIMING CPU cores. The simulation ends when the startup is completed
successfully.

Usage
-----

```
scons build/RISCV/gem5.opt
./build/RISCV/gem5.opt \
    configs/example/gem5_library/riscv-ubuntu-run.py
```
"""

import os

import m5
from m5.objects import Root
from m5.util import addToPath

addToPath(os.path.dirname(__file__))


import math

import LookupReader
import NoCbuilder

from m5.objects import DDR4_2400_8x8

from gem5.components.boards.riscv_board import RiscvBoard
from gem5.components.cachehierarchies.ruby.moesi_cmp_directory_cache_hierarchy import (
    MOESICmpDirectoryCacheHierarchy,
)
from gem5.components.memory.memory import ChanneledMemory
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import ISA
from gem5.resources.resource import *
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

# This runs a check to ensure the gem5 binary is compiled for RISCV.
num_cpus = 4
topo = "mesh"
requires(isa_required=ISA.RISCV)

# # With RISCV, we use simple caches.
# from gem5.components.cachehierarchies.classic.private_l1_private_l2_walk_cache_hierarchy import (
#     PrivateL1PrivateL2WalkCacheHierarchy,
# )

# # Here we setup the parameters of the l1 and l2 caches.
# cache_hierarchy = PrivateL1PrivateL2WalkCacheHierarchy(
#     l1d_size="16KiB", l1i_size="16KiB", l2_size="256KiB"
# )

rows = int(math.ceil(math.sqrt(num_cpus)))
filename = "XYrouting.csv"
builder = NoCbuilder.TopologyGraph(
    topo=topo, size=(rows, rows), directory="corners", L2=True
)
builder.compute_XY_routing_table()
builder.write_routing_csv(filename=filename)
lookuplist = LookupReader.read_csv(filename)

cache_hierarchy = MOESICmpDirectoryCacheHierarchy(
    l1d_size="32KiB",
    l1d_assoc=8,
    l1i_size="32KiB",
    l1i_assoc=8,
    l2_size="512KiB",
    l2_assoc=16,
    num_l2_banks=4,
    topo=topo,
    routing_algo=0,
    lookup_table=lookuplist,
)

# Memory: Dual Channel DDR4 2400 DRAM device.

memory = ChanneledMemory(
    dram_interface_class=DDR4_2400_8x8,
    num_channels=4,
    interleaving_size=64,
    size="4GiB",
)

# Here we setup the processor. We use a simple processor.
processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING, isa=ISA.RISCV, num_cores=num_cpus
)

# Here we setup the board. The RiscvBoard allows for Full-System RISCV
# simulations.
board = RiscvBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

fs = False

if fs:
    board.set_kernel_disk_workload(
        kernel=KernelResource(
            local_path="/home/amoghsgk/gem5/resources/riscv-linux-6.8.12-kernel"
        ),
        bootloader=BootloaderResource(
            local_path="/home/amoghsgk/gem5/resources/riscv-bootloader-opensbi-1.3.1"
        ),
        disk_image=DiskImageResource(
            local_path="/home/amoghsgk/gem5/resources/rootfs.img"
        ),
        kernel_args=[
            "console=ttyS0",
            "root={root_value}",
            "disk_device={disk_device}",
            "rw",
            "no_systemd = 1",
        ],
        # checkpoint=CheckpointResource(local_path="./ubuntu-no-systemd_checkpoint/cpt.266844640248"), not working with networks
        readfile_contents="""
        echo "running radix"
        ./RADIX -n 32768 -p 4 -s
        m5 exit
        """,
    )
else:
    board.set_se_binary_workload(
        BinaryResource(
            local_path="/home/amoghsgk/gem5/splash2/codes/kernels/radix/RADIX",
            architecture=ISA.RISCV,
        ),
        arguments=["-n 32768", "-p", str(num_cpus), "-s"],
    )


def exit_event_handler():
    print("First exit: kernel booted")
    if not fs:
        yield True
    yield False  # gem5 is now executing systemd startup
    print("Second exit: Started `after_boot.sh` script")
    # The after_boot.sh script is executed after the kernel and systemd have
    # booted.
    yield False  # gem5 is now executing the `after_boot.sh` script
    print("Third exit: Finished `after_boot.sh` script")
    # The after_boot.sh script will run a script if it is passed via
    # m5 readfile. This is the last exit event before the simulation exits.
    yield True


simulator = Simulator(
    board=board,
    on_exit_event={
        # Here we want override the default behavior for the first m5 exit
        # exit event.
        ExitEvent.EXIT: exit_event_handler()
    },
)
simulator.run()
