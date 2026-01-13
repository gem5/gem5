# Copyright (c) 2023 The University of Edinburgh
# Copyright (c) 2025 Technical University of Munich
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
#

"""
Fetch directed instruction prefetch (FDP) example with std library components

This gem5 configuation script creates a simple simulation setup with a single
O3 CPU model and decoupled front-end. Is serves as a starting point for the
FDP implementation. As workload a simple "Hello World!" program is used.

FDP is tested with the X86, Arm, RISC-V isa which can be specified
using the --isa flag.
This example uses components from the gem5 library.

Usage
-----

```
scons build/ALL/gem5.opt
./build/ALL/gem5.opt \
    configs/example/gem5_library/fdp-hello.py \
    --isa <isa> \
    [--disable-fdp]
```
"""

import argparse

from gem5.components.boards.abstract_board import AbstractBoard
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.two_level_fdp_cache_hierarchy import (
    TwoLevelFDPCacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.decoupled_processor import DecoupledProcessor
from gem5.isas import ISA
from gem5.resources.resource import (
    obtain_resource,
)
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

isa_choices = {
    "X86": ISA.X86,
    "Arm": ISA.ARM,
    "RISCV": ISA.RISCV,
}

workloads = {
    "hello": {
        "Arm": "arm-hello64-static",
        "X86": "x86-hello64-static",
        "RISCV": "riscv-hello",
    },
}


parser = argparse.ArgumentParser(
    description="An example configuration script to run FDP."
)

parser.add_argument(
    "--isa",
    type=str,
    default="X86",
    help="The ISA to simulate.",
    choices=isa_choices.keys(),
)

parser.add_argument(
    "--workload",
    type=str,
    default="hello",
    help="The workload to simulate.",
    choices=workloads.keys(),
)

parser.add_argument(
    "--disable-fdp",
    action="store_true",
    help="Disable FDP to evaluate baseline performance.",
)

args = parser.parse_args()


# This check ensures the gem5 binary is compiled to the correct ISA target.
# If not, an exception will be thrown.
requires(isa_required=isa_choices[args.isa])

# We use a single channel DDR3_1600 memory system
memory = SingleChannelDDR3_1600(size="32MiB")

# 1. Decoupled Prefetcher ------------------------------------------------
cache_hierarchy = TwoLevelFDPCacheHierarchy(
    l1i_size="32KiB",
    l1d_size="32KiB",
    l2_size="1MiB",
    decoupled=not args.disable_fdp,
)

# 2. Decoupled Front-end ------------------------------------------------
# Next setup the decoupled front-end. Its implemented in the O3 core.
# Create the processor with one core
processor = DecoupledProcessor(
    isa=isa_choices[args.isa],
    num_cores=1,
    decoupled=not args.disable_fdp,
)


print(
    f"Running {args.workload} on {args.isa}, "
    f"FDP {'disabled' if args.disable_fdp else 'enabled'}"
)


# The gem5 library simple board which can be used to run simple SE-mode
# simulations.
board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Here we set the workload. In this case we want to run a simple "Hello World!"
# program compiled to the specified ISA. The `Resource` class will automatically
# download the binary from the gem5 Resources cloud bucket if it's not already
# present.
board.set_se_binary_workload(
    obtain_resource(workloads[args.workload][args.isa])
)

# Lastly we run the simulation.
simulator = Simulator(board=board)
simulator.run()

print("Simulation done.")
