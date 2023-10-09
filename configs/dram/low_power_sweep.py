# Copyright (c) 2014-2015, 2017, 2019 ARM Limited
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

import m5
from m5.objects import *
from m5.stats import periodicStatDump
from m5.util import addToPath

addToPath("../")

from common import ObjectList
from common import MemConfig

# This script aims at triggering low power state transitions in the DRAM
# controller. The traffic generator is used in DRAM mode and traffic
# states target a different levels of bank utilization and strides.
# At the end after sweeping through bank utilization and strides, we go
# through an idle state with no requests to enforce self-refresh.

parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)

# Use a single-channel DDR4-2400 in 16x4 configuration by default
parser.add_argument(
    "--mem-type",
    default="DDR4_2400_16x4",
    choices=ObjectList.mem_list.get_names(),
    help="type of memory to use",
)

parser.add_argument(
    "--mem-ranks",
    "-r",
    type=int,
    default=1,
    help="Number of ranks to iterate across",
)

parser.add_argument(
    "--page-policy",
    "-p",
    choices=["close_adaptive", "open_adaptive"],
    default="close_adaptive",
    help="controller page policy",
)

parser.add_argument(
    "--itt-list",
    "-t",
    default="1 20 100",
    help="a list of multipliers for the max value of itt, " 'e.g. "1 20 100"',
)

parser.add_argument(
    "--rd-perc", type=int, default=100, help="Percentage of read commands"
)

parser.add_argument(
    "--addr-map",
    choices=m5.objects.AddrMap.vals,
    default="RoRaBaCoCh",
    help="DRAM address map policy",
)

parser.add_argument(
    "--idle-end",
    type=int,
    default=50000000,
    help="time in ps of an idle period at the end ",
)

args = parser.parse_args()

# Start with the system itself, using a multi-layer 2.0 GHz
# crossbar, delivering 64 bytes / 3 cycles (one header cycle)
# which amounts to 42.7 GByte/s per layer and thus per port.
system = System(membus=IOXBar(width=32))
system.clk_domain = SrcClockDomain(
    clock="2.0GHz", voltage_domain=VoltageDomain(voltage="1V")
)

# We are fine with 256 MB memory for now.
mem_range = AddrRange("256MB")
# Start address is 0
system.mem_ranges = [mem_range]

# Do not worry about reserving space for the backing store
system.mmap_using_noreserve = True

# Force a single channel to match the assumptions in the DRAM traffic
# generator
args.mem_channels = 1
args.external_memory_system = 0
args.tlm_memory = 0
args.elastic_trace_en = 0
MemConfig.config_mem(args, system)

# Sanity check for memory controller class.
if not isinstance(system.mem_ctrls[0], m5.objects.MemCtrl):
    fatal("This script assumes the controller is a MemCtrl subclass")
if not isinstance(system.mem_ctrls[0].dram, m5.objects.DRAMInterface):
    fatal("This script assumes the memory is a DRAMInterface subclass")

# There is no point slowing things down by saving any data.
system.mem_ctrls[0].dram.null = True

# enable DRAM low power states
system.mem_ctrls[0].dram.enable_dram_powerdown = True

# Set the address mapping based on input argument
system.mem_ctrls[0].dram.addr_mapping = args.addr_map
system.mem_ctrls[0].dram.page_policy = args.page_policy

# We create a traffic generator state for each param combination we want to
# test. Each traffic generator state is specified in the config file and the
# generator remains in the state for specific period. This period is 0.25 ms.
# Stats are dumped and reset at the state transition.
period = 250000000

# We specify the states in a config file input to the traffic generator.
cfg_file_name = "lowp_sweep.cfg"
cfg_file_path = os.path.dirname(__file__) + "/" + cfg_file_name
cfg_file = open(cfg_file_path, "w")

# Get the number of banks
nbr_banks = int(system.mem_ctrls[0].dram.banks_per_rank.value)

# determine the burst size in bytes
burst_size = int(
    (
        system.mem_ctrls[0].dram.devices_per_rank.value
        * system.mem_ctrls[0].dram.device_bus_width.value
        * system.mem_ctrls[0].dram.burst_length.value
    )
    / 8
)

# next, get the page size in bytes (the rowbuffer size is already in bytes)
page_size = (
    system.mem_ctrls[0].dram.devices_per_rank.value
    * system.mem_ctrls[0].dram.device_rowbuffer_size.value
)

# Inter-request delay should be such that we can hit as many transitions
# to/from low power states as possible to. We provide a min and max itt to the
# traffic generator and it randomises in the range. The parameter is in
# seconds and we need it in ticks (ps).
itt_min = system.mem_ctrls[0].dram.tBURST.value * 1000000000000

# The itt value when set to (tRAS + tRP + tCK) covers the case where
# a read command is delayed beyond the delay from ACT to PRE_PDN entry of the
# previous command. For write command followed by precharge, this delay
# between a write and power down entry will be tRCD + tCL + tWR + tRP + tCK.
# As we use this delay as a unit and create multiples of it as bigger delays
# for the sweep, this parameter works for reads, writes and mix of them.
pd_entry_time = (
    system.mem_ctrls[0].dram.tRAS.value
    + system.mem_ctrls[0].dram.tRP.value
    + system.mem_ctrls[0].dram.tCK.value
) * 1000000000000

# We sweep itt max using the multipliers specified by the user.
itt_max_str = args.itt_list.strip().split()
itt_max_multiples = [int(x) for x in itt_max_str]
if len(itt_max_multiples) == 0:
    fatal("String for itt-max-list detected empty\n")

itt_max_values = [pd_entry_time * m for m in itt_max_multiples]

# Generate request addresses in the entire range, assume we start at 0
max_addr = mem_range.end

# For max stride, use min of the page size and 512 bytes as that should be
# more than enough
max_stride = min(512, page_size)
mid_stride = 4 * burst_size
stride_values = [burst_size, mid_stride, max_stride]

# be selective about bank utilization instead of going from 1 to the number of
# banks
bank_util_values = [1, int(nbr_banks / 2), nbr_banks]

# Next we create the config file, but first a comment
cfg_file.write(
    """# STATE state# period mode=DRAM
# read_percent start_addr end_addr req_size min_itt max_itt data_limit
# stride_size page_size #banks #banks_util addr_map #ranks\n"""
)

addr_map = m5.objects.AddrMap.map[args.addr_map]

nxt_state = 0
for itt_max in itt_max_values:
    for bank in bank_util_values:
        for stride_size in stride_values:
            cfg_file.write(
                "STATE %d %d %s %d 0 %d %d "
                "%d %d %d %d %d %d %d %d %d\n"
                % (
                    nxt_state,
                    period,
                    "DRAM",
                    args.rd_perc,
                    max_addr,
                    burst_size,
                    itt_min,
                    itt_max,
                    0,
                    stride_size,
                    page_size,
                    nbr_banks,
                    bank,
                    addr_map,
                    args.mem_ranks,
                )
            )
            nxt_state = nxt_state + 1

# State for idle period
idle_period = args.idle_end
cfg_file.write("STATE %d %d IDLE\n" % (nxt_state, idle_period))

# Init state is state 0
cfg_file.write("INIT 0\n")

# Go through the states one by one
for state in range(1, nxt_state + 1):
    cfg_file.write("TRANSITION %d %d 1\n" % (state - 1, state))

# Transition from last state to itself to not break the probability math
cfg_file.write("TRANSITION %d %d 1\n" % (nxt_state, nxt_state))
cfg_file.close()

# create a traffic generator, and point it to the file we just created
system.tgen = TrafficGen(config_file=cfg_file_path)

# add a communication monitor
system.monitor = CommMonitor()

# connect the traffic generator to the bus via a communication monitor
system.tgen.port = system.monitor.cpu_side_port
system.monitor.mem_side_port = system.membus.cpu_side_ports

# connect the system port even if it is not used in this example
system.system_port = system.membus.cpu_side_ports

# every period, dump and reset all stats
periodicStatDump(period)

root = Root(full_system=False, system=system)
root.system.mem_mode = "timing"

m5.instantiate()

# Simulate for exactly as long as it takes to go through all the states
# This is why sim exists.
m5.simulate(nxt_state * period + idle_period)
print("--- Done DRAM low power sweep ---")
print("Fixed params - ")
print(
    "\tburst: %d, banks: %d, max stride: %d, itt min: %s ns"
    % (burst_size, nbr_banks, max_stride, itt_min)
)
print("Swept params - ")
print("\titt max multiples input:", itt_max_multiples)
print("\titt max values", itt_max_values)
print("\tbank utilization values", bank_util_values)
print("\tstride values:", stride_values)
print("Traffic gen config file:", cfg_file_name)
