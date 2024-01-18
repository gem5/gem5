# Copyright (c) 2020 ARM Limited
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

import argparse
import math

import m5
from m5.objects import *
from m5.stats import periodicStatDump
from m5.util import addToPath

addToPath("../")

from common import (
    MemConfig,
    ObjectList,
)

# this script is helpful to sweep the efficiency of a specific memory
# controller configuration, by varying the number of banks accessed,
# and the sequential stride size (how many bytes per activate), and
# observe what bus utilisation (bandwidth) is achieved

parser = argparse.ArgumentParser()

nvm_generators = {"NVM": lambda x: x.createNvm}

# Use a single-channel DDR3-1600 x64 (8x8 topology) by default
parser.add_argument(
    "--nvm-type",
    default="NVM_2400_1x64",
    choices=ObjectList.mem_list.get_names(),
    help="type of memory to use",
)

parser.add_argument(
    "--nvm-ranks",
    "-r",
    type=int,
    default=1,
    help="Number of ranks to iterate across",
)

parser.add_argument(
    "--rd_perc", type=int, default=100, help="Percentage of read commands"
)

parser.add_argument(
    "--mode",
    default="NVM",
    choices=nvm_generators.keys(),
    help="NVM: Random traffic",
)

parser.add_argument(
    "--addr-map",
    choices=ObjectList.dram_addr_map_list.get_names(),
    default="RoRaBaCoCh",
    help="NVM address map policy",
)

args = parser.parse_args()

# at the moment we stay with the default open-adaptive page policy,
# and address mapping

# start with the system itself, using a multi-layer 2.0 GHz
# crossbar, delivering 64 bytes / 3 cycles (one header cycle)
# which amounts to 42.7 GByte/s per layer and thus per port
system = System(membus=IOXBar(width=32))
system.clk_domain = SrcClockDomain(
    clock="2.0GHz", voltage_domain=VoltageDomain(voltage="1V")
)

# we are fine with 256 MB memory for now
mem_range = AddrRange("512MB")
system.mem_ranges = [mem_range]

# do not worry about reserving space for the backing store
system.mmap_using_noreserve = True

# force a single channel to match the assumptions in the DRAM traffic
# generator
args.mem_channels = 1
args.external_memory_system = 0
MemConfig.config_mem(args, system)

# the following assumes that we are using the native memory
# controller with an NVM interface, check to be sure
if not isinstance(system.mem_ctrls[0], m5.objects.MemCtrl):
    fatal("This script assumes the controller is a MemCtrl subclass")
if not isinstance(system.mem_ctrls[0].dram, m5.objects.NVMInterface):
    fatal("This script assumes the memory is a NVMInterface class")

# there is no point slowing things down by saving any data
system.mem_ctrls[0].dram.null = True

# Set the address mapping based on input argument
system.mem_ctrls[0].dram.addr_mapping = args.addr_map

# stay in each state for 0.25 ms, long enough to warm things up, and
# short enough to avoid hitting a refresh
period = 250000000

# stay in each state as long as the dump/reset period, use the entire
# range, issue transactions of the right DRAM burst size, and match
# the DRAM maximum bandwidth to ensure that it is saturated

# get the number of regions
nbr_banks = system.mem_ctrls[0].dram.banks_per_rank.value

# determine the burst length in bytes
burst_size = int(
    (
        system.mem_ctrls[0].dram.devices_per_rank.value
        * system.mem_ctrls[0].dram.device_bus_width.value
        * system.mem_ctrls[0].dram.burst_length.value
    )
    / 8
)


# next, get the page size in bytes
buffer_size = (
    system.mem_ctrls[0].dram.devices_per_rank.value
    * system.mem_ctrls[0].dram.device_rowbuffer_size.value
)

# match the maximum bandwidth of the memory, the parameter is in seconds
# and we need it in ticks (ps)
itt = system.mem_ctrls[0].dram.tBURST.value * 1000000000000

# assume we start at 0
max_addr = mem_range.end

# use min of the page size and 512 bytes as that should be more than
# enough
max_stride = min(256, buffer_size)

# create a traffic generator, and point it to the file we just created
system.tgen = PyTrafficGen()

# add a communication monitor
system.monitor = CommMonitor()

# connect the traffic generator to the bus via a communication monitor
system.tgen.port = system.monitor.cpu_side_port
system.monitor.mem_side_port = system.membus.cpu_side_ports

# connect the system port even if it is not used in this example
system.system_port = system.membus.cpu_side_ports

# every period, dump and reset all stats
periodicStatDump(period)

# run Forrest, run!
root = Root(full_system=False, system=system)
root.system.mem_mode = "timing"

m5.instantiate()


def trace():
    addr_map = ObjectList.dram_addr_map_list.get(args.addr_map)
    generator = nvm_generators[args.mode](system.tgen)
    for stride_size in range(burst_size, max_stride + 1, burst_size):
        for bank in range(1, nbr_banks + 1):
            num_seq_pkts = int(math.ceil(float(stride_size) / burst_size))
            yield generator(
                period,
                0,
                max_addr,
                burst_size,
                int(itt),
                int(itt),
                args.rd_perc,
                0,
                num_seq_pkts,
                buffer_size,
                nbr_banks,
                bank,
                addr_map,
                args.dram_ranks,
            )
    yield system.tgen.createExit(0)


system.tgen.start(trace())

m5.simulate()

print(
    "NVM sweep with burst: %d, banks: %d, max stride: %d"
    % (burst_size, nbr_banks, max_stride)
)
