# Copyright (c) 2010-2015 Advanced Micro Devices, Inc.
# All rights reserved.
#
# For use for simulation and test purposes only
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
from __future__ import absolute_import

import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath
import os, optparse, sys

addToPath('../')

from common import Options
from ruby import Ruby

# Get paths we might need.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)

parser = optparse.OptionParser()
Options.addNoISAOptions(parser)

parser.add_option("--maxloads", metavar="N", default=100,
                  help="Stop after N loads")
parser.add_option("-f", "--wakeup_freq", metavar="N", default=10,
                  help="Wakeup every N cycles")
parser.add_option("-u", "--num-compute-units", type="int", default=1,
                  help="number of compute units in the GPU")
parser.add_option("--num-cp", type="int", default=0,
                  help="Number of GPU Command Processors (CP)")
# not super important now, but to avoid putting the number 4 everywhere, make
# it an option/knob
parser.add_option("--cu-per-sqc", type="int", default=4, help="number of CUs \
                  sharing an SQC (icache, and thus icache TLB)")
parser.add_option("--simds-per-cu", type="int", default=4, help="SIMD units" \
                  "per CU")
parser.add_option("--wf-size", type="int", default=64,
                  help="Wavefront size(in workitems)")
parser.add_option("--wfs-per-simd", type="int", default=10, help="Number of " \
                  "WF slots per SIMD")

#
# Add the ruby specific and protocol specific options
#
Ruby.define_options(parser)

exec(compile( \
    open(os.path.join(config_root, "common", "Options.py")).read(), \
    os.path.join(config_root, "common", "Options.py"), 'exec'))

(options, args) = parser.parse_args()

#
# Set the default cache size and associativity to be very small to encourage
# races between requests and writebacks.
#
options.l1d_size="256B"
options.l1i_size="256B"
options.l2_size="512B"
options.l3_size="1kB"
options.l1d_assoc=2
options.l1i_assoc=2
options.l2_assoc=2
options.l3_assoc=2

# This file can support multiple compute units
assert(options.num_compute_units >= 1)
n_cu = options.num_compute_units

options.num_sqc = int((n_cu + options.cu_per_sqc - 1) // options.cu_per_sqc)

if args:
     print("Error: script doesn't take any positional arguments")
     sys.exit(1)

#
# Create the ruby random tester
#

# Check to for the GPU_RfO protocol.  Other GPU protocols are non-SC and will
# not work with the Ruby random tester.
assert(buildEnv['PROTOCOL'] == 'GPU_RfO')

# The GPU_RfO protocol does not support cache flushes
check_flush = False

tester = RubyTester(check_flush=check_flush,
                    checks_to_complete=options.maxloads,
                    wakeup_frequency=options.wakeup_freq,
                    deadlock_threshold=1000000)

#
# Create the M5 system.  Note that the Memory Object isn't
# actually used by the rubytester, but is included to support the
# M5 memory size == Ruby memory size checks
#
system = System(cpu=tester, mem_ranges=[AddrRange(options.mem_size)])

# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage=options.sys_voltage)

system.clk_domain = SrcClockDomain(clock=options.sys_clock,
                                   voltage_domain=system.voltage_domain)

Ruby.create_system(options, False, system)

# Create a seperate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(clock=options.ruby_clock,
                                       voltage_domain=system.voltage_domain)

tester.num_cpus = len(system.ruby._cpu_ports)

#
# The tester is most effective when randomization is turned on and
# artifical delay is randomly inserted on messages
#
system.ruby.randomization = True

for ruby_port in system.ruby._cpu_ports:

    #
    # Tie the ruby tester ports to the ruby cpu read and write ports
    #
    if ruby_port.support_data_reqs and ruby_port.support_inst_reqs:
        tester.cpuInstDataPort = ruby_port.slave
    elif ruby_port.support_data_reqs:
        tester.cpuDataPort = ruby_port.slave
    elif ruby_port.support_inst_reqs:
        tester.cpuInstPort = ruby_port.slave

    # Do not automatically retry stalled Ruby requests
    ruby_port.no_retry_on_stall = True

    #
    # Tell each sequencer this is the ruby tester so that it
    # copies the subblock back to the checker
    #
    ruby_port.using_ruby_tester = True

# -----------------------
# run simulation
# -----------------------

root = Root( full_system = False, system = system )
root.system.mem_mode = 'timing'

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency('1ns')

# instantiate configuration
m5.instantiate()

# simulate until program terminates
exit_event = m5.simulate(options.abs_max_tick)

print('Exiting @ tick', m5.curTick(), 'because', exit_event.getCause())
