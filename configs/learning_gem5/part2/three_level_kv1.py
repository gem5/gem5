# Copyright (c) 2015 Jason Power
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

""" This file creates a single CPU and a two-level cache system.
This script takes a single parameter which specifies a binary to execute.
If none is provided it executes 'hello' by default (mostly used for testing)

See Part 1, Chapter 3: Adding cache to the configuration script in the
learning_gem5 book for more information about this script.
This file exports options for the L1 I/D and L2 cache sizes.

IMPORTANT: If you modify this file, it's likely that the Learning gem5 book
           also needs to be updated. For now, email Jason <power.jg@gmail.com>

"""

# import the m5 (gem5) library created when gem5 is built
import m5

# import all of the SimObjects
from m5.objects import *

# Add the common scripts to our path
m5.util.addToPath("../../")

# import the caches which we made
from caches import *

# import the SimpleOpts module
from common import SimpleOpts


class L3Cache(Cache):
    """Simple L3 cache with default values"""

    # Default Parameters
    size = "1MiB"
    assoc = 8
    tag_latency = 20
    data_latency = 20
    response_latency = 20
    mshrs = 20
    tgts_per_mshr = 12

    SimpleOpts.add_option("--l3_size", help=f"L3 Cache size. Default:{size}")
    SimpleOpts.add_option(
        "--l3_assoc", help=f"l3 cache Assoc. Default:{assoc}"
    )

    def __init__(self, opts=None):
        super().__init__()
        if not opts:
            return
        if opts.l3_size:
            self.size = opts.l3_size
        if opts.l3_assoc:
            self.assoc = opts.assoc

    def connectCPUSideBus(self, bus):
        self.cpu_side = bus.mem_side_ports

    def connectMemSideBus(self, bus):
        self.mem_side = bus.cpu_side_ports


# Default to running 'hello', use the compiled ISA to find the binary
# grab the specific path to the binary
thispath = os.path.dirname(os.path.realpath(__file__))
default_binary = os.path.join(
    thispath,
    "../../../",
    "tests/test-progs/hello/bin/x86/linux/hello",
)

# Binary to execute
# SimpleOpts.add_option("binary", nargs="?", default=default_binary)
# our_default_elf="/home/docker_share/gem5_mar20/riscv_bin_folder/temp1.elf"
our_default_binary = "/home/docker_share/gem5_mar20/riscv_bin_folder/temp1.o"
SimpleOpts.add_option("binary", nargs="?", default=our_default_binary)

# Finalize the arguments and grab the args so we can pass it on to our objects
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
system.cpu = RiscvTimingSimpleCPU()
# system.cpu = RiscvO3CPU()

# Create an L1 instruction and data cache
system.cpu.icache = L1ICache(args)
system.cpu.dcache = L1DCache(args)

# Connect the instruction and data caches to the CPU
system.cpu.icache.connectCPU(system.cpu)
system.cpu.dcache.connectCPU(system.cpu)

# Create a memory bus, a coherent crossbar, in this case
system.l2bus = L2XBar()

# Hook the CPU ports up to the l2bus
system.cpu.icache.connectBus(system.l2bus)
system.cpu.dcache.connectBus(system.l2bus)

# Create an L2 cache and connect it to the l2bus
system.l2cache = L2Cache(args)
system.l2cache.connectCPUSideBus(system.l2bus)


# create a memory bus, a coherent crossbar, in this case
class L3XBar(L2XBar):
    pass


system.l3bus = L3XBar()

# connect the Mem side ports of the l2 cache to xbar
system.l2cache.connectMemSideBus(system.l3bus)

# create a L3 Cache and connect it to the l3bus
system.l3cache = L3Cache(args)
system.l3cache.connectCPUSideBus(system.l3bus)

# Create a memory bus
system.membus = SystemXBar()

# Connect the L3 cache to the membus
system.l3cache.connectMemSideBus(system.membus)

# create the interrupt controller for the CPU
system.cpu.createInterruptController()

# system.cpu.interrupts[0].pio = system.membus.mem_side_ports
# system.cpu.interrupts[0].int_requestor = system.membus.cpu_side_ports
# system.cpu.interrupts[0].int_responder = system.membus.mem_side_ports

# Connect the system up to the membus
system.system_port = system.membus.cpu_side_ports

# Create a DDR3 memory controller
system.mem_ctrl = MemCtrl()
system.mem_ctrl.dram = DDR3_1600_8x8()
system.mem_ctrl.dram.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.mem_side_ports

system.workload = SEWorkload.init_compatible(args.binary)

# Create a process for a simple "Hello World" application
process = Process()
# Set the command
# cmd is a list which begins with the executable (like argv)
# process.cmd = [args.binary]
# process.cmd = ["/home/docker_share/gem5_mar20/riscv_bin_folder/temp3"]

# process.cmd = ["/home/docker_share/gem5_mar20/riscv_bin_folder/temp2.o\
# ","/home/docker_share/gem5_mar20/riscv_bin_folder/temp1.o"]

# process.cmd = ["/home/docker_share/gem5_mar20/mibench/automotive/\
# qsort/runme_small.sh"]    # did not work

# process.cmd = ["/home/docker_share/gem5_mar20/riscv_bin_folder/\
# temp3 home/docker_share/gem5_mar20/mibench/automotive/qsort/input_small.dat"]
process.cmd = [
    "/home/docker_share/gem5_mar20/mibench/automotive/qsort/qsort_small",
    "/home/docker_share/gem5_mar20/mibench/automotive/qsort/input_small.dat",
]

# Set the cpu to use the process as its workload and create thread contexts
system.cpu.workload = process
system.cpu.createThreads()

# set up the root SimObject and start the simulation
root = Root(full_system=False, system=system)
# instantiate all of the objects we've created above
m5.instantiate()

print(f"Beginning simulation!")
exit_event = m5.simulate()
print(f"Exiting @ tick {m5.curTick()} because {exit_event.getCause()}")
