import argparse
import sys
import time

import m5
from m5.objects import *
from m5.objects import Root

from gem5.coherence_protocol import CoherenceProtocol
from gem5.components.boards.x86_board import X86Board
from gem5.components.memory import DualChannelDDR4_2400
from gem5.components.processors.cpu_types import CPUTypes
from gem5.components.processors.simple_switchable_processor import (
    SimpleSwitchableProcessor,
)
from gem5.isas import ISA
from gem5.resources.resource import obtain_resource
from gem5.simulate.exit_event import ExitEvent
from gem5.simulate.simulator import Simulator
from gem5.utils.requires import requires

requires(
    isa_required=ISA.X86,
    coherence_protocol_required=CoherenceProtocol.MESI_TWO_LEVEL,
    kvm_required=True,
)


from gem5.components.cachehierarchies.ruby.mesi_two_level_cache_hierarchy import (
    MESITwoLevelCacheHierarchy,
)


# parse the arguments
def argument_parse():
    parser = argparse.ArgumentParser(description="DSE arguments")

    # location of the binary file
    parser.add_argument(
        "--binary",
        type=str,
        default="/home/abg309/PhD/RCL/gem5-dse/workload/defaultGait/x86/controller_code_cpp",
        help="Location of the binary file",
    )

    # input array
    parser.add_argument(
        "--input",
        type=str,
        nargs="+",
        default=["1.0", "2.0", "3.0"],
        help="Input array",
    )

    args = parser.parse_args()

    return args


# setting up the cache hierarchy
cache_hierarchy = MESITwoLevelCacheHierarchy(
    l1d_size="24KiB",
    l1d_assoc=6,
    l1i_size="32KiB",
    l1i_assoc=8,
    l2_size="512KiB",
    l2_assoc=8,
    num_l2_banks=2,
)


# Memory: Dual Channel DDR4 2400 DRAM device.
# The X86 board only supports 3 GiB of main memory.

memory = DualChannelDDR4_2400(size="3GiB")


# create a processor
processor = SimpleSwitchableProcessor(
    starting_core_type=CPUTypes.KVM,
    switch_core_type=CPUTypes.TIMING,
    isa=ISA.X86,
    num_cores=4,
)


# create a processor
board = X86Board(
    clk_freq="3.4GHz",
    processor=processor,
    memory=memory,
    cache_hierarchy=cache_hierarchy,
)

# Create a system
args = argument_parse()


binary = args.binary

# Use the provided input arguments as the program arguments
arguments = args.input

# Construct a BinaryResource for the local binary path so that
# `set_se_binary_workload` can call `get_local_path()` on it.
from gem5.resources.resource import BinaryResource

binary_resource = BinaryResource(local_path=binary)

# Set the SE workload (Simple Emulation workload) with your custom binary
board.set_se_binary_workload(binary_resource, arguments=arguments)

# Prepare the board and create the Root object by running the board's
# pre-instantiate hook. This ensures `_connect_things` is called so that
# `createCCObject` won't raise the `_connect_things_check` exception.
root = board._pre_instantiate(full_system=False)

m5.instantiate()

# Run the simulation
print("Starting simulation")
exit_event = m5.simulate()

# Output when the simulation ends
print("Exiting @ tick", m5.curTick())
