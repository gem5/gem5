# Copyright (c) 2021-2024 The Regents of the University of California
# Copyright (c) 2022 Google Inc
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
A run script for a very simple Syscall-Execution running simple binaries.
The system has no cache heirarchy and is as "bare-bones" as you can get in
gem5 while still being functinal.
"""

import argparse
import importlib

from m5.util import fatal

from gem5.components.boards.mem_mode import MemMode
from gem5.components.boards.simple_board import SimpleBoard
from gem5.components.cachehierarchies.classic.no_cache import NoCache
from gem5.components.memory import SingleChannelDDR3_1600
from gem5.components.processors.base_cpu_core import BaseCPUCore
from gem5.components.processors.base_cpu_processor import BaseCPUProcessor
from gem5.components.processors.cpu_types import (
    CPUTypes,
    get_cpu_type_from_str,
    get_cpu_types_str_set,
)
from gem5.components.processors.simple_core import SimpleCore
from gem5.components.processors.simple_processor import SimpleProcessor
from gem5.isas import (
    ISA,
    get_isa_from_str,
    get_isas_str_set,
)
from gem5.resources.resource import Resource
from gem5.simulate.simulator import Simulator

supported_isas = {ISA.X86, ISA.ARM, ISA.RISCV}
inst_exit_resource_id_map = {
    ISA.X86: "x86-inst-m5-exit",
    ISA.ARM: "arm-inst-m5-exit",
    ISA.RISCV: "riscv-inst-m5-exit",
}

addr_exit_resource_id_map = {
    ISA.X86: "x86-addr-m5-exit",
    ISA.ARM: "arm-addr-m5-exit",
    ISA.RISCV: None,  # No address exit resource for RISCV
}

assert all(isa in inst_exit_resource_id_map for isa in supported_isas)
assert all(isa in addr_exit_resource_id_map for isa in supported_isas)

parser = argparse.ArgumentParser(
    description="A gem5 script for running simple binaries in SE mode."
)

parser.add_argument(
    "isa",
    type=str,
    choices={isa.value for isa in supported_isas},
    help="The ISA to test the m5_exit instruction on",
)

parser.add_argument(
    "type",
    type=str,
    choices={"inst", "addr"},
    help="The type of m5_exit to test. Address ('addr') or Instruction ('inst')",
)

parser.add_argument(
    "--resource-directory",
    type=str,
    required=False,
    help="The directory in which resources will be downloaded or exist.",
)

args = parser.parse_args()

isa = get_isa_from_str(args.isa)

# Setup the system.
cache_hierarchy = NoCache()
memory = SingleChannelDDR3_1600()

processor = SimpleProcessor(
    cpu_type=CPUTypes.ATOMIC,
    isa=isa,
    num_cores=1,
)

motherboard = SimpleBoard(
    clk_freq="3GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

binary = None


if args.type == "addr":
    assert (
        isa in addr_exit_resource_id_map
    ), f"ISA {isa.value} not in addr_exit_resource_id_map"
    resource_id = addr_exit_resource_id_map[isa]
elif args.type == "inst":
    assert (
        isa in inst_exit_resource_id_map
    ), f"ISA {isa.value} not in addr_inst_resource_id_map"
    resource_id = inst_exit_resource_id_map[isa]

if resource_id is None:
    fatal(
        f"Script does not support ISA {isa.value} "
        f"with m5_exit type {args.type}"
    )

########## Remove this block  when resources are in gem5-resources ############                                                                 ####
from pathlib import Path

resource_id_to_path_map = {
    "x86-addr-m5-exit": Path(
        Path(__file__).parent.parent, "m5_exit", "bin", "x86-addr-m5-exit"
    ),
    "x86-inst-m5-exit": Path(
        Path(__file__).parent.parent, "m5_exit", "bin", "x86-inst-m5-exit"
    ),
    "arm-addr-m5-exit": Path(
        Path(__file__).parent.parent, "m5_exit", "bin", "arm-addr-m5-exit"
    ),
    "arm-inst-m5-exit": Path(
        Path(__file__).parent.parent, "m5_exit", "bin", "arm-inst-m5-exit"
    ),
    "riscv-inst-m5-exit": Path(
        Path(__file__).parent.parent, "m5_exit", "bin", "riscv-inst-m5-exit"
    ),
}

assert all(
    resource_id in resource_id_to_path_map or resource_id is None
    for resource_id in inst_exit_resource_id_map.values()
), "resource_id_to_path_map lacks non-None resource_id"
assert all(
    resource_id in resource_id_to_path_map or resource_id is None
    for resource_id in addr_exit_resource_id_map.values()
), "resource_id_to_path_map lacks non-None resource_id"

from gem5.resources.resource import BinaryResource

binary = BinaryResource(
    local_path=resource_id_to_path_map[resource_id].as_posix()
)
###############################################################################

############# Uncomment this when resources are in gem5-resources #############
###############################################################################
# binary = Resource(
#   resource_id=resource_id,
#    resource_directory=args.resource_directory,
# )
###############################################################################

assert binary is not None, "Binary resource is None"

motherboard.set_se_binary_workload(binary)

# Run the simulation
simulator = Simulator(board=motherboard)
simulator.run()

print(
    "Exiting @ tick {} because {}.".format(
        simulator.get_current_tick(), simulator.get_last_exit_event_cause()
    )
)
