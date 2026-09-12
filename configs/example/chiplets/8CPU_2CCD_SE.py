# import the m5 (gem5) library created when gem5 is built
import m5
from m5.defines import buildEnv  # type: ignore

# import all of the SimObjects
from m5.objects import *

# Needed for running C++ threads
m5.util.addToPath("../../")  # add /configs/ to path
from typing import TYPE_CHECKING

from chiplets.BaseChipletSystem import BaseChipletSystem
from chiplets.Chiplet import Chiplet
from chiplets.ChipletSystem import ChipletSystem
from common.FileSystemConfig import config_filesystem
from common.Options import *
from network import Network
from ruby import Ruby
from topologies.Mesh_XY import Mesh_XY
from topologies.Pt2Pt import Pt2Pt

if TYPE_CHECKING:
    from src.arch.x86.X86CPU import *
    from src.mem.DRAMInterface import *
    from src.sim.ClockDomain import *
    from src.sim.Process import Process
    from src.sim.Root import Root
    from src.sim.System import System
    from src.sim.VoltageDomain import VoltageDomain
    from src.sim.Workload import *

### Configuration

CACHE_PROTOCOL = buildEnv["PROTOCOL"]

CPU_CLS = X86TimingSimpleCPU
CPU_FREQ = "4GHz"

MEM_CLS = DDR3_1600_8x8
DEFAULT_MEM_CFG = "DDR3_1600_8x8"
MEM_SIZE = "8192MiB"

### Command-Line Argument Handling

import argparse

parser = argparse.ArgumentParser(
    description="A hierarchical chiplet system with a two-level cache and 2 CCDs, each with 4 CPUs."
)

parser.add_argument(
    "binary",
    default="",
    nargs="?",
    type=str,
    help="Path to the binary to execute.",
)

#! key ruby/garnet network options
Ruby.define_options(parser)  # also calls Network.define_options()
# Network.define_options(parser)

options = parser.parse_args()

### System Parameters

# instantiate system SimObject
system = System()

# set system clock
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = CPU_FREQ
system.clk_domain.voltage_domain = VoltageDomain()

## set up system memory

# use timing mode for memory simulation
# (do this unless fast-forwarding or restoring from checkpoint)
system.mem_mode = "timing"

# define a single 512MiB memory range
system.mem_ranges = [AddrRange(MEM_SIZE)]

### CPU

# create the CPUs
# assumed homogenous CPUs

options.num_cpus = 8
system.cpu = [CPU_CLS() for _ in range(options.num_cpus)]
options.ports = options.num_cpus

### Memory

# one directory for the system for example, you will probably want to increase this
options.num_dirs = 1

options.enable_dram_powerdown = False

#! latencies
# default latencies for topology, overridden by chiplet specified latencies
# options.link_latency = 2
# options.router_latency = 1

# small cache
# optionally use this to "encourage" ruby network activity
# options.l1d_size = "256B"
# options.l1i_size = "256B"
# options.l2_size = "512B"
# options.l1d_assoc = 2
# options.l1i_assoc = 2
# options.l2_assoc = 2

# reasonable size cache
options.l1d_size = "4KiB"
options.l1i_size = "16KiB"
options.l2_size = "64KiB"
options.l1d_assoc = 2
options.l1i_assoc = 2
options.l2_assoc = 2
options.num_l2caches = 1
options.cacheline_size = 64

# example of 2-CCD processor with 4 cores per CCD
ccd: list[Chiplet] = []

ccd.append(
    Chiplet(
        system=system,
        full_system=False,
        TopologyClass=Mesh_XY,
        MemoryClass=MEM_CLS,
        inter_node_link_lat=1,
        intra_node_link_lat=1,
        node_main_router_lat=1,
        cores=system.cpu[0:4],
    )
)

ccd.append(
    Chiplet(
        system=system,
        full_system=False,
        TopologyClass=Mesh_XY,
        MemoryClass=MEM_CLS,
        inter_node_link_lat=1,
        intra_node_link_lat=1,
        node_main_router_lat=1,
        cores=system.cpu[4:8],
    )
)

# needed for `Mesh_XY`. because of how mesh topologies currently
# work and how options are passed, this is currently global.
# i.e., ALL meshes created will have two rows.
options.mesh_rows = 2

# meshes also require the mem size to be set in options
options.mem_size = MEM_SIZE

# * create a ChipletSystem containing our CCDs
cs = ChipletSystem(
    system=system,
    full_system=False,
    TopologyClass=Pt2Pt,
    MemoryClass=MEM_CLS,
    inter_node_link_lat=2,
    intra_node_link_lat=1,
    node_main_router_lat=3,
    nodes=ccd,
)

cs.createSystem(options)

print(f"ChipletSystem:\n{cs.to_string()}\n\n")

### run simulation

# specify binary/executable to run
if not options.binary:
    print(
        "No binary file(path) specified gem5 to run, defaulting to hello-world."
    )
    binary = f"tests/test-progs/hello/bin/x86/linux/hello"
elif options.binary == "threads":
    binary = f"tests/test-progs/threads/bin/x86/linux/threads"
else:
    binary = options.binary

# load the binary in syscall emulation mode
system.workload = SEWorkload.init_compatible(binary)

# Set up the pseudo file system for the threads function above
config_filesystem(system)

# create process SimObject
process = Process()
process.cmd = [binary]

# set workload and create workload threads
# same workload for each CPU
for i, cpu in enumerate(system.cpu):
    # print(f"setting workload = {process} for cpu {cpu}")
    cpu.workload = process
    cpu.createThreads()

# instantiate system
root = Root(full_system=False, system=system)
m5.instantiate()

# * uncomment to see routers
# print(f"all cs routers:")
# for r in cs._garnet_network.routers:
#     print(f"    {r.router_id}: {r} @ {hex(id(r))}")

# print(f"cs main router id: {cs._main_router.router_id}; router: {cs._main_router}")
# print(f"ccd[0] main router id: {ccd[0]._main_router.router_id}")
# print(f"ccd[1] main router id: {ccd[1]._main_router.router_id}")

# * uncomment to see clock domain hierarchy
# # must be done after m5.instantiate() to be correct
# print("="*30)
# print("Garnet Network Children with Clock Domains")
# print("="*30)
# BaseChipletSystem.recursivelyPrintClockDomains(cs._garnet_network)
# print("="*30)
# print("\n\n")

# print("="*30)
# print("CCD 0 Children with Clock Domains")
# print("="*30)
# BaseChipletSystem.recursivelyPrintClockDomains(ccd[0])
# print("="*30)
# print("\n\n")

# print("="*30)
# print("CCD 1 Children with Clock Domains")
# print("="*30)
# BaseChipletSystem.recursivelyPrintClockDomains(ccd[1])
# print("="*30)
# print("\n\n")

exit_event = m5.simulate()

print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
