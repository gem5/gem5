# Copyright (c) 2009-2011 Advanced Micro Devices, Inc.
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
#
# Authors: Brad Beckmann

#
# Full system configuraiton for ruby
#

import optparse
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal

addToPath('../common')
addToPath('../ruby')
addToPath('../topologies')

import Ruby

from FSConfig import *
from SysPaths import *
from Benchmarks import *
import Options
import Simulation

parser = optparse.OptionParser()
Options.addCommonOptions(parser)
Options.addFSOptions(parser)

# Add the ruby specific and protocol specific options
Ruby.define_options(parser)

(options, args) = parser.parse_args()
options.ruby = True

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

if options.benchmark:
    try:
        bm = Benchmarks[options.benchmark]
    except KeyError:
        print "Error benchmark %s has not been defined." % options.benchmark
        print "Valid benchmarks are: %s" % DefinedBenchmarks
        sys.exit(1)
else:
    bm = [SysConfig(disk=options.disk_image, mem=options.mem_size)]

# Check for timing mode because ruby does not support atomic accesses
if not (options.cpu_type == "detailed" or options.cpu_type == "timing"):
    print >> sys.stderr, "Ruby requires TimingSimpleCPU or O3CPU!!"
    sys.exit(1)
(CPUClass, test_mem_mode, FutureClass) = Simulation.setCPUClass(options)

TestMemClass = Simulation.setMemClass(options)

if buildEnv['TARGET_ISA'] == "alpha":
    system = makeLinuxAlphaRubySystem(test_mem_mode, bm[0])
elif buildEnv['TARGET_ISA'] == "x86":
    system = makeLinuxX86System(test_mem_mode, options.num_cpus, bm[0], True)
    Simulation.setWorkCountOptions(system, options)
else:
    fatal("incapable of building non-alpha or non-x86 full system!")
system.cache_line_size = options.cacheline_size

# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage = options.sys_voltage)

system.clk_domain = SrcClockDomain(clock = options.sys_clock,
                                   voltage_domain = system.voltage_domain)

if options.kernel is not None:
    system.kernel = binary(options.kernel)

if options.script is not None:
    system.readfile = options.script

system.cpu = [CPUClass(cpu_id=i) for i in xrange(options.num_cpus)]

# Create a source clock for the CPUs and set the clock period
system.cpu_clk_domain = SrcClockDomain(clock = options.cpu_clock,
                                       voltage_domain = system.voltage_domain)

Ruby.create_system(options, system, system.piobus, system._dma_ports)

# Create a seperate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(clock = options.ruby_clock,
                                        voltage_domain = system.voltage_domain)

for (i, cpu) in enumerate(system.cpu):
    #
    # Tie the cpu ports to the correct ruby system ports
    #
    cpu.clk_domain = system.cpu_clk_domain
    cpu.createThreads()
    cpu.createInterruptController()

    cpu.icache_port = system.ruby._cpu_ruby_ports[i].slave
    cpu.dcache_port = system.ruby._cpu_ruby_ports[i].slave

    if buildEnv['TARGET_ISA'] == "x86":
        cpu.itb.walker.port = system.ruby._cpu_ruby_ports[i].slave
        cpu.dtb.walker.port = system.ruby._cpu_ruby_ports[i].slave

        cpu.interrupts.pio = system.ruby._cpu_ruby_ports[i].master
        cpu.interrupts.int_master = system.ruby._cpu_ruby_ports[i].slave
        cpu.interrupts.int_slave = system.ruby._cpu_ruby_ports[i].master

    system.ruby._cpu_ruby_ports[i].access_phys_mem = True

# Create the appropriate memory controllers and connect them to the
# PIO bus
system.mem_ctrls = [TestMemClass(range = r) for r in system.mem_ranges]
for i in xrange(len(system.mem_ctrls)):
    system.mem_ctrls[i].port = system.piobus.master

root = Root(full_system = True, system = system)
Simulation.run(options, root, system, FutureClass)
