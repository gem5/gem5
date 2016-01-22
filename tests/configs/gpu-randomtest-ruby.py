#
#  Copyright (c) 2010-2015 Advanced Micro Devices, Inc.
#  All rights reserved.
#
#  For use for simulation and test purposes only
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#  this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors
#  may be used to endorse or promote products derived from this software
#  without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#  Author: Brad Beckmann
#

import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath
import os, optparse, sys

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)
addToPath(config_root+'/configs/common')
addToPath(config_root+'/configs/ruby')
addToPath(config_root+'/configs/topologies')

import Ruby
import Options

parser = optparse.OptionParser()
Options.addCommonOptions(parser)

# add the gpu specific options expected by the the gpu and gpu_RfO
parser.add_option("-u", "--num-compute-units", type="int", default=8,
                  help="number of compute units in the GPU")
parser.add_option("--num-cp", type="int", default=0,
                  help="Number of GPU Command Processors (CP)")
parser.add_option("--simds-per-cu", type="int", default=4, help="SIMD units" \
                  "per CU")
parser.add_option("--wf-size", type="int", default=64,
                  help="Wavefront size(in workitems)")
parser.add_option("--wfs-per-simd", type="int", default=10, help="Number of " \
                  "WF slots per SIMD")

# Add the ruby specific and protocol specific options
Ruby.define_options(parser)

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
options.num_compute_units=8
options.num_sqc=2

# Check to for the GPU_RfO protocol.  Other GPU protocols are non-SC and will
# not work with the Ruby random tester.
assert(buildEnv['PROTOCOL'] == 'GPU_RfO')

#
# create the tester and system, including ruby
#
tester = RubyTester(check_flush = False, checks_to_complete = 100,
                    wakeup_frequency = 10, num_cpus = options.num_cpus)

# We set the testers as cpu for ruby to find the correct clock domains
# for the L1 Objects.
system = System(cpu = tester)

# Dummy voltage domain for all our clock domains
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)
system.clk_domain = SrcClockDomain(clock = '1GHz',
                                   voltage_domain = system.voltage_domain)

system.mem_ranges = AddrRange('256MB')

Ruby.create_system(options, False, system)

# Create a separate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(clock = '1GHz',
                                        voltage_domain = system.voltage_domain)

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
    # Tell the sequencer this is the ruby tester so that it
    # copies the subblock back to the checker
    #
    ruby_port.using_ruby_tester = True

# -----------------------
# run simulation
# -----------------------

root = Root(full_system = False, system = system )
root.system.mem_mode = 'timing'

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency('1ns')
