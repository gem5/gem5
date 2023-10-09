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

from common import ObjectList
from common import MemConfig

# this script is helpful to sweep the efficiency of a specific memory
# controller configuration, by varying the number of banks accessed,
# and the sequential stride size (how many bytes per activate), and
# observe what bus utilisation (bandwidth) is achieved

parser = argparse.ArgumentParser()

hybrid_generators = {"HYBRID": lambda x: x.createHybrid}

# Use a single-channel DDR3-1600 x64 (8x8 topology) by default
parser.add_argument(
    "--nvm-type",
    default="NVM_2400_1x64",
    choices=ObjectList.mem_list.get_names(),
    help="type of memory to use",
)

parser.add_argument(
    "--mem-type",
    default="DDR4_2400_16x4",
    choices=ObjectList.mem_list.get_names(),
    help="type of memory to use",
)

parser.add_argument(
    "--nvm-ranks",
    "-n",
    type=int,
    default=1,
    help="Number of ranks to iterate across",
)

parser.add_argument(
    "--mem-ranks",
    "-r",
    type=int,
    default=2,
    help="Number of ranks to iterate across",
)

parser.add_argument(
    "--rd-perc", type=int, default=100, help="Percentage of read commands"
)

parser.add_argument(
    "--nvm-perc", type=int, default=100, help="Percentage of NVM commands"
)

parser.add_argument(
    "--mode",
    default="HYBRID",
    choices=hybrid_generators.keys(),
    help="Hybrid: Random DRAM + NVM traffic",
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

# set 2 ranges, the first, smaller range for DDR
# the second, larger (1024) range for NVM
# the NVM range starts directly after the DRAM range
system.mem_ranges = [
    AddrRange("128MB"),
    AddrRange(Addr("128MB"), size="1024MB"),
]

# do not worry about reserving space for the backing store
system.mmap_using_noreserve = True

# force a single channel to match the assumptions in the DRAM traffic
# generator
args.mem_channels = 1
args.external_memory_system = 0
args.hybrid_channel = True
MemConfig.config_mem(args, system)

# the following assumes that we are using the native controller
# with NVM and DRAM interfaces, check to be sure
if not isinstance(system.mem_ctrls[0], m5.objects.HeteroMemCtrl):
    fatal("This script assumes the controller is a HeteroMemCtrl subclass")
if not isinstance(system.mem_ctrls[0].dram, m5.objects.DRAMInterface):
    fatal("This script assumes the first memory is a DRAMInterface subclass")
if not isinstance(system.mem_ctrls[0].nvm, m5.objects.NVMInterface):
    fatal("This script assumes the second memory is a NVMInterface subclass")

# there is no point slowing things down by saving any data
system.mem_ctrls[0].dram.null = True
system.mem_ctrls[0].nvm.null = True

# Set the address mapping based on input argument
system.mem_ctrls[0].dram.addr_mapping = args.addr_map
system.mem_ctrls[0].nvm.addr_mapping = args.addr_map

# stay in each state for 0.25 ms, long enough to warm things up, and
# short enough to avoid hitting a refresh
period = 250000000

# stay in each state as long as the dump/reset period, use the entire
# range, issue transactions of the right burst size, and match
# the maximum bandwidth to ensure that it is saturated

# get the number of banks
nbr_banks_dram = system.mem_ctrls[0].dram.banks_per_rank.value

# determine the burst length in bytes
burst_size_dram = int(
    (
        system.mem_ctrls[0].dram.devices_per_rank.value
        * system.mem_ctrls[0].dram.device_bus_width.value
        * system.mem_ctrls[0].dram.burst_length.value
    )
    / 8
)

# next, get the page size in bytes
page_size_dram = (
    system.mem_ctrls[0].dram.devices_per_rank.value
    * system.mem_ctrls[0].dram.device_rowbuffer_size.value
)

# get the number of regions
nbr_banks_nvm = system.mem_ctrls[0].nvm.banks_per_rank.value

# determine the burst length in bytes
burst_size_nvm = int(
    (
        system.mem_ctrls[0].nvm.devices_per_rank.value
        * system.mem_ctrls[0].nvm.device_bus_width.value
        * system.mem_ctrls[0].nvm.burst_length.value
    )
    / 8
)


burst_size = max(burst_size_dram, burst_size_nvm)

# next, get the page size in bytes
buffer_size_nvm = (
    system.mem_ctrls[0].nvm.devices_per_rank.value
    * system.mem_ctrls[0].nvm.device_rowbuffer_size.value
)

# match the maximum bandwidth of the memory, the parameter is in seconds
# and we need it in ticks (ps)
itt = (
    min(
        system.mem_ctrls[0].dram.tBURST.value,
        system.mem_ctrls[0].nvm.tBURST.value,
    )
    * 1000000000000
)

# assume we start at 0 for DRAM
max_addr_dram = system.mem_ranges[0].end
min_addr_nvm = system.mem_ranges[1].start
max_addr_nvm = system.mem_ranges[1].end

# use min of the page size and 512 bytes as that should be more than
# enough
max_stride = min(256, buffer_size_nvm, page_size_dram)

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
    generator = hybrid_generators[args.mode](system.tgen)
    for stride_size in range(burst_size, max_stride + 1, burst_size):
        num_seq_pkts_dram = int(
            math.ceil(float(stride_size) / burst_size_dram)
        )
        num_seq_pkts_nvm = int(math.ceil(float(stride_size) / burst_size_nvm))
        yield generator(
            period,
            0,
            max_addr_dram,
            burst_size_dram,
            min_addr_nvm,
            max_addr_nvm,
            burst_size_nvm,
            int(itt),
            int(itt),
            args.rd_perc,
            0,
            num_seq_pkts_dram,
            page_size_dram,
            nbr_banks_dram,
            nbr_banks_dram,
            num_seq_pkts_nvm,
            buffer_size_nvm,
            nbr_banks_nvm,
            nbr_banks_nvm,
            addr_map,
            args.mem_ranks,
            args.nvm_ranks,
            args.nvm_perc,
        )

    yield system.tgen.createExit(0)


system.tgen.start(trace())

m5.simulate()

print("Hybrid DRAM + NVM sweep with max_stride: %d" % (max_stride))
print("NVM burst: %d, NVM banks: %d" % (burst_size_nvm, nbr_banks_nvm))
print("DRAM burst: %d, DRAM banks: %d" % (burst_size_dram, nbr_banks_dram))
