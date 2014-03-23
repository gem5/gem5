# Copyright (c) 2014 ARM Limited
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
# Authors: Andreas Hansson

import optparse

import m5
from m5.objects import *
from m5.util import addToPath
from m5.internal.stats import periodicStatDump

addToPath('../common')

import MemConfig

# this script is helpful to sweep the efficiency of a specific memory
# controller configuration, by varying the number of banks accessed,
# and the sequential stride size (how many bytes per activate), and
# observe what bus utilisation (bandwidth) is achieved

parser = optparse.OptionParser()

# Use a single-channel DDR3-1600 x64 by default
parser.add_option("--mem-type", type="choice", default="ddr3_1600_x64",
                  choices=MemConfig.mem_names(),
                  help = "type of memory to use")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# at the moment we stay with the default open-adaptive page policy,
# and address mapping

# start with the system itself, using a multi-layer 1 GHz
# bus/crossbar, delivering 64 bytes / 5 cycles (one header cycle)
# which amounts to 12.8 GByte/s per layer and thus per port
system = System(membus = NoncoherentBus(width = 16))
system.clk_domain = SrcClockDomain(clock = '1GHz',
                                   voltage_domain =
                                   VoltageDomain(voltage = '1V'))

# we are fine with 256 MB memory for now
mem_range = AddrRange('256MB')
system.mem_ranges = [mem_range]

# force a single channel to match the assumptions in the DRAM traffic
# generator
options.mem_channels = 1
MemConfig.config_mem(options, system)

# the following assumes that we are using the native DRAM
# controller, check to be sure
if not isinstance(system.mem_ctrls[0], m5.objects.DRAMCtrl):
    fatal("This script assumes the memory is a DRAMCtrl subclass")

# for now the generator assumes a single rank
system.mem_ctrls[0].ranks_per_channel = 1

# stay in each state for 0.25 ms, long enough to warm things up, and
# short enough to avoid hitting a refresh
period = 250000000

# this is where we go off piste, and print the traffic generator
# configuration that we will later use, crazy but it works
cfg_file_name = "configs/dram/sweep.cfg"
cfg_file = open(cfg_file_name, 'w')

# stay in each state as long as the dump/reset period, use the entire
# range, issue transactions of the right DRAM burst size, and match
# the DRAM maximum bandwidth to ensure that it is saturated

# get the number of banks
nbr_banks = system.mem_ctrls[0].banks_per_rank.value

# determine the burst length in bytes
burst_size = int((system.mem_ctrls[0].devices_per_rank.value *
                  system.mem_ctrls[0].device_bus_width.value *
                  system.mem_ctrls[0].burst_length.value) / 8)

# next, get the page size in bytes
page_size = system.mem_ctrls[0].devices_per_rank.value * \
    system.mem_ctrls[0].device_rowbuffer_size.value

# match the maximum bandwidth of the memory, the parameter is in ns
# and we need it in ticks
itt = system.mem_ctrls[0].tBURST.value * 1000000000000

# assume we start at 0
max_addr = mem_range.end

# now we create the state by iterating over the stride size from burst
# size to min of the page size and 1 kB, and from using only a single
# bank up to the number of banks available
nxt_state = 0
for bank in range(1, nbr_banks + 1):
    for stride_size in range(burst_size, min(1024, page_size) + 1, burst_size):
        cfg_file.write("STATE %d %d DRAM 100 0 %d "
                       "%d %d %d %d %d %d %d %d 1\n" %
                       (nxt_state, period, max_addr, burst_size, itt, itt, 0,
                        stride_size, page_size, nbr_banks, bank))
        nxt_state = nxt_state + 1

cfg_file.write("INIT 0\n")

# go through the states one by one
for state in range(1, nxt_state):
    cfg_file.write("TRANSITION %d %d 1\n" % (state - 1, state))

cfg_file.write("TRANSITION %d %d 1\n" % (nxt_state - 1, nxt_state - 1))

cfg_file.close()

# create a traffic generator, and point it to the file we just created
system.tgen = TrafficGen(config_file = cfg_file_name)

# add a communication monitor
system.monitor = CommMonitor()

# connect the traffic generator to the bus via a communication monitor
system.tgen.port = system.monitor.slave
system.monitor.master = system.membus.slave

# connect the system port even if it is not used in this example
system.system_port = system.membus.slave

# every period, dump and reset all stats
periodicStatDump(period)

# run Forrest, run!
root = Root(full_system = False, system = system)
root.system.mem_mode = 'timing'

m5.instantiate()
m5.simulate(nxt_state * period)
