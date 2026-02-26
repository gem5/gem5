# Copyright (c) 2017 Jason Lowe-Power
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

""" This file creates a barebones system and executes 'hello', a simple Hello
World application. Adds a simple memobj between the CPU and the membus.

This config file assumes that the x86 ISA was built.
"""

# import the m5 (gem5) library created when gem5 is built
import m5

# import all of the SimObjects
from m5.objects import *

m5.util.addToPath("../../")
from common import SimpleOpts

# we need to get the cache file in local directory (here)
# import sys
# sys.path.append('../../part1')
# from part1 import *
# from part1.caches import *


class L1Cache(Cache):
    """Simple L1 Cache with default values"""

    assoc = 2
    tag_latency = 2
    data_latency = 2
    response_latency = 2
    mshrs = 4
    tgts_per_mshr = 20

    def __init__(self, options=None):
        super().__init__()
        pass

    def connectBus(self, bus):
        """Connect this cache to a memory-side bus"""
        self.mem_side = bus.cpu_side_ports

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU-side port
        This must be defined in a subclass"""
        raise NotImplementedError


class L1ICache(L1Cache):
    """Simple L1 instruction cache with default values"""

    # Set the default size
    size = "16KiB"

    SimpleOpts.add_option(
        "--l1i_size", help=f"L1 instruction cache size. Default: {size}"
    )

    def __init__(self, opts=None):
        super().__init__(opts)
        if not opts or not opts.l1i_size:
            return
        self.size = opts.l1i_size

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU icache port"""
        self.cpu_side = cpu.icache_port


class L1DCache(L1Cache):
    """Simple L1 data cache with default values"""

    # Set the default size
    size = "64KiB"

    SimpleOpts.add_option(
        "--l1d_size", help=f"L1 data cache size. Default: {size}"
    )

    def __init__(self, opts=None):
        super().__init__(opts)
        if not opts or not opts.l1d_size:
            return
        self.size = opts.l1d_size

    def connectCPU(self, cpu):
        """Connect this cache's port to a CPU dcache port"""
        self.cpu_side = cpu.dcache_port


class L2Cache(Cache):
    """Simple L2 Cache with default values"""

    # Default parameters
    size = "256KiB"
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

    SimpleOpts.add_option("--l2_size", help=f"L2 cache size. Default: {size}")

    def __init__(self, opts=None):
        super().__init__()
        if not opts or not opts.l2_size:
            return
        self.size = opts.l2_size

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


our_default_binary = "/home/docker_share/gem5_mar20/riscv_bin_folder/temp3"
SimpleOpts.add_option("binary", nargs="?", default=our_default_binary)

# options for NoncoherentXBar()
SimpleOpts.add_option("forward_latency", nargs="?", default=1)

args = SimpleOpts.parse_args()

# create the system we are going to simulate
system = System()

# Set the clock frequency of the system (and all of its children)
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = "1GHz"
system.clk_domain.voltage_domain = VoltageDomain()

# Set up the system
system.mem_mode = "timing"  # Use timing accesses
system.mem_ranges = [AddrRange("512MiB")]  # Create an address range

# Create a simple CPU
# system.cpu = X86TimingSimpleCPU()
system.cpu = [RiscvTimingSimpleCPU(), RiscvTimingSimpleCPU()]
# system.cpu = [x86TimingSimpleCPU(), x86TimingSimpleCPU()]


# Create the simple memory object
system.memobj = SimpleMemobj()

# Hook the CPU ports up to the cache
# system.cpu.icache_port = system.memobj.inst_port
# system.cpu.dcache_port = system.memobj.data_port

# create an instruction and data cache for both the cpus
system.cpu[0].icache = L1ICache(args)
system.cpu[0].dcache = L1DCache(args)
system.cpu[1].icache = L1ICache(args)
system.cpu[1].dcache = L1DCache(args)

# connect the instructionand data cache to cpu
system.cpu[0].icache.connectCPU(system.cpu[0])
system.cpu[0].dcache.connectCPU(system.cpu[0])
system.cpu[1].icache.connectCPU(system.cpu[1])
system.cpu[1].dcache.connectCPU(system.cpu[1])

# Crate a memory bus, a coherent crossbar, in this case
# system.l2bus = [L2XBar(), L2XBar()]
ncxbar1 = NoncoherentXBar()
ncxbar1.forward_latency = 1
ncxbar1.frontend_latency = 1
ncxbar1.response_latency = 1
ncxbar1.width = 64
ncxbar2 = NoncoherentXBar()
ncxbar2.forward_latency = 1
ncxbar2.frontend_latency = 1
ncxbar2.response_latency = 1
ncxbar2.width = 64
# system.l2bus = [NoncoherentXBar(), NoncoherentXBar(args)]
system.l2bus = [ncxbar1, ncxbar2]

# connect the L1Icache to l2 bus
system.cpu[0].icache.connectBus(system.l2bus[0])
system.cpu[0].dcache.connectBus(system.l2bus[0])
system.cpu[1].icache.connectBus(system.l2bus[1])
system.cpu[1].dcache.connectBus(system.l2bus[1])

# create an l2 cache for each CPU
system.l2cache = [L2Cache(args), L2Cache(args)]
system.l2cache[0].connectCPUSideBus(system.l2bus[0])
system.l2cache[1].connectCPUSideBus(system.l2bus[1])


# create Simple memory Object
system.memobj = SimpleMemobj()

# Create a memory bus, a coherent crossbar, in this case
system.membus = SystemXBar()

# connect the l2 cache to mem object (withput any bus)
system.l2cache[0].mem_side = system.memobj.inst_port
system.l2cache[1].mem_side = system.memobj.data_port
# system.l2cache[0].connectMemSideBus(system.membus)
# system.l2cache[1].connectMemSideBus(system.membus)

# system.memobj.memPort =


# Connect the memobj
system.memobj.mem_side = system.membus.cpu_side_ports

# create the interrupt controller for the CPU and connect to the membus
system.cpu[0].createInterruptController()
system.cpu[1].createInterruptController()
# system.cpu.interrupts[0].pio = system.membus.mem_side_ports
# system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
# system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

# Create a DDR3 memory controller and connect it to the membus
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

# Connect the system up to the membus
system.system_port = system.membus.cpu_side_ports

# Create a process for a simple "Hello World" application
process = Process()
# Set the command
# grab the specific path to the binary
# thispath = os.path.dirname(os.path.realpath(__file__))
# binpath = os.path.join(
#    thispath, "../../../", "tests/test-progs/hello/bin/x86/linux/hello"
# )

# cmd is a list which begins with the executable (like argv)
# process.cmd = [binpath]
process.cmd = ["/home/docker_share/gem5_mar20/riscv_bin_folder/temp2"]

# Set the cpu to use the process as its workload and create thread contexts
system.cpu[0].workload = process
system.cpu[0].createThreads()
process.cmd = ["/home/docker_share/gem5_mar20/riscv_bin_folder/temp3"]
system.cpu[1].workload = process
system.cpu[1].createThreads()

# system.workload = SEWorkload.init_compatible(binpath)
system.workload = SEWorkload.init_compatible(args.binary)

# set up the root SimObject and start the simulation
root = Root(full_system=False, system=system)
# instantiate all of the objects we've created above
m5.instantiate()

print(f"Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")


# import m5
# from m5.objects import Cache

# Add the common scripts to our path
# m5.util.addToPath("../../")

# from common import SimpleOpts

# Some specific options for caches
# For all options see src/mem/cache/BaseCache.py
