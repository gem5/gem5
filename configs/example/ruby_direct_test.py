# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2009 Advanced Micro Devices, Inc.
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
import argparse
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath

addToPath("../")

from common import Options
from ruby import Ruby

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)

parser = argparse.ArgumentParser()
Options.addNoISAOptions(parser)

parser.add_argument(
    "--requests", metavar="N", default=100, help="Stop after N requests"
)
parser.add_argument(
    "-f",
    "--wakeup_freq",
    metavar="N",
    default=10,
    help="Wakeup every N cycles",
)
parser.add_argument(
    "--test-type",
    default="SeriesGetx",
    choices=["SeriesGetx", "SeriesGets", "SeriesGetMixed", "Invalidate"],
    help="Type of test",
)
parser.add_argument(
    "--percent-writes",
    type=int,
    default=100,
    help="percentage of accesses that should be writes",
)

#
# Add the ruby specific and protocol specific args
#
Ruby.define_options(parser)
args = parser.parse_args()

#
# Select the direct test generator
#
if args.test_type == "SeriesGetx":
    generator = SeriesRequestGenerator(
        num_cpus=args.num_cpus, percent_writes=100
    )
elif args.test_type == "SeriesGets":
    generator = SeriesRequestGenerator(
        num_cpus=args.num_cpus, percent_writes=0
    )
elif args.test_type == "SeriesGetMixed":
    generator = SeriesRequestGenerator(
        num_cpus=args.num_cpus, percent_writes=args.percent_writes
    )
elif args.test_type == "Invalidate":
    generator = InvalidateGenerator(num_cpus=args.num_cpus)
else:
    print("Error: unknown direct test generator")
    sys.exit(1)

# Create the M5 system.
system = System(mem_ranges=[AddrRange(args.mem_size)])


# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)

system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)

# Create the ruby random tester
system.cpu = RubyDirectedTester(
    requests_to_complete=args.requests, generator=generator
)

# the ruby tester reuses num_cpus to specify the
# number of cpu ports connected to the tester object, which
# is stored in system.cpu. because there is only ever one
# tester object, num_cpus is not necessarily equal to the
# size of system.cpu
cpu_list = [system.cpu] * args.num_cpus
Ruby.create_system(args, False, system, cpus=cpu_list)

# Since Ruby runs at an independent frequency, create a seperate clock
system.ruby.clk_domain = SrcClockDomain(
    clock=args.ruby_clock, voltage_domain=system.voltage_domain
)

assert args.num_cpus == len(system.ruby._cpu_ports)

for ruby_port in system.ruby._cpu_ports:
    #
    # Tie the ruby tester ports to the ruby cpu ports
    #
    system.cpu.cpuPort = ruby_port.in_ports

# -----------------------
# run simulation
# -----------------------

root = Root(full_system=False, system=system)
root.system.mem_mode = "timing"

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency("1ns")

# instantiate configuration
m5.instantiate()

# simulate until program terminates
exit_event = m5.simulate(args.abs_max_tick)

print("Exiting @ tick", m5.curTick(), "because", exit_event.getCause())
