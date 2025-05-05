# Copyright (c) 2024 The Regents of the University of California
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
A script to run the CHI protocol with different, user-specified, ISA targets
and number of cores. Sensible SE workloads are used for each supported ISA.
"""

import argparse

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.chi.private_l1_cache_hierarchy import (
    PrivateL1CacheHierarchy,
)
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import (
    ISA,
    get_isa_from_str,
)
from gem5.resources.resource import obtain_resource
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

supported_isa_targets = set({ISA.ARM, ISA.X86, ISA.RISCV})

# Maps the isa under test to the id of the resources that should be used to
# create the workload.
isa_resource_map = {
    ISA.ARM: "arm-matrix-multiply",
    ISA.X86: "x86-matrix-multiply",
    ISA.RISCV: "riscv-matrix-multiply",
}

# States which version of each resource to use.
# The resource versions are typically fixed when testing to ensure that the
# results are consistent even as new resources are added.
resource_version_map = {
    "arm-matrix-multiply": "1.0.0",
    "x86-matrix-multiply": "1.0.0",
    "riscv-matrix-multiply": "1.0.0",
}

assert all(
    target in set(isa_resource_map.keys()) for target in supported_isa_targets
), "One or more supported ISA targets not found in `isa_resource_map`."

assert all(
    id in set(resource_version_map.keys())
    for id in set(isa_resource_map.values())
), (
    "One or more resource ids, specified in `isa_resource_map` not found in "
    "`resource_version_map`."
)


parser = argparse.ArgumentParser(
    description="A script to run the CHI protocol with different, "
    "user-specified, ISA targets and number of codes. Sensible SE workloads "
    " are used for each supported ISA target. "
)

parser.add_argument(
    "isa",
    type=str,
    choices=tuple(isa.value for isa in supported_isa_targets),
    help="The target ISA. I.e., the ISA to run with the CHI protocol on "
    "with the workload.",
)

parser.add_argument(
    "--num-cores",
    type=int,
    default=1,
    required=False,
    help="The number of CPU cores.",
)

parser.add_argument(
    "-r",
    "--resource-directory",
    type=str,
    required=False,
    default=None,
    help="The directory in which resources will be downloaded or exist.",
)

args = parser.parse_args()
isa = get_isa_from_str(args.isa)

requires(isa_required=isa, coherence_protocol_required=CoherenceProtocol.CHI)

cache_hierarchy = PrivateL1CacheHierarchy(size="512KiB", assoc=8)

memory = SingleChannelDDR3_1600(size="32MiB")

processor = SimpleProcessor(
    cpu_type=CPUTypes.TIMING,
    isa=isa,
    num_cores=args.num_cores,
)

board = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

board.set_se_binary_workload(
    binary=obtain_resource(
        resource_id=isa_resource_map[isa],
        resource_version=resource_version_map[isa_resource_map[isa]],
        resource_directory=args.resource_directory,
    )
)

simulator = Simulator(board=board)
simulator.run()
