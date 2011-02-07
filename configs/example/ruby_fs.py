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

import os
import optparse
import sys
from os.path import join as joinpath

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, panic

if not buildEnv['FULL_SYSTEM']:
    panic("This script requires full-system mode (*_FS).")

addToPath('../common')
addToPath('../ruby')

import Ruby

from FSConfig import *
from SysPaths import *
from Benchmarks import *
import Simulation
from Caches import *

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)

parser = optparse.OptionParser()

# Benchmark options
parser.add_option("-b", "--benchmark", action="store", type="string",
                  dest="benchmark",
                  help="Specify the benchmark to run. Available benchmarks: %s"\
                  % DefinedBenchmarks)
parser.add_option("-o", "--options", default="",
    help='The options to pass to the binary, use " " around the entire string')
parser.add_option("-i", "--input", default="", help="Read stdin from a file.")
parser.add_option("--output", default="", help="Redirect stdout to a file.")
parser.add_option("--errout", default="", help="Redirect stderr to a file.")

#
# Add the ruby specific and protocol specific options
#
Ruby.define_options(parser)

execfile(os.path.join(config_root, "common", "Options.py"))

(options, args) = parser.parse_args()

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
    bm = [SysConfig()]

#
# currently ruby fs only works in simple timing mode because ruby does not
# support atomic accesses by devices.  Also ruby_fs currently assumes
# that is running a checkpoints that were created by ALPHA_FS under atomic
# mode.  Since switch cpus are not defined in these checkpoints, we don't
# fast forward with the atomic cpu and instead set the FutureClass to None.
# Therefore the cpus resolve to the correct names and unserialize correctly.
#
class CPUClass(TimingSimpleCPU): pass
test_mem_mode = 'timing'
FutureClass = None

CPUClass.clock = options.clock

if buildEnv['TARGET_ISA'] == "alpha":
    system = makeLinuxAlphaRubySystem(test_mem_mode, bm[0])
    system.ruby = Ruby.create_system(options,
                                     system,
                                     system.piobus,
                                     system.dma_devices)
elif buildEnv['TARGET_ISA'] == "x86":
    system = makeLinuxX86System(test_mem_mode, options, bm[0], True)
    system.ruby = Ruby.create_system(options,
                                     system,
                                     system.piobus)
else:
    fatal("incapable of building non-alpha or non-x86 full system!")

system.cpu = [CPUClass(cpu_id=i) for i in xrange(options.num_cpus)]

for (i, cpu) in enumerate(system.cpu):
    #
    # Tie the cpu ports to the correct ruby system ports
    #
    cpu.icache_port = system.ruby.cpu_ruby_ports[i].port
    cpu.dcache_port = system.ruby.cpu_ruby_ports[i].port
    if buildEnv['TARGET_ISA'] == "x86":
        cpu.itb.walker.port = system.ruby.cpu_ruby_ports[i].port
        cpu.dtb.walker.port = system.ruby.cpu_ruby_ports[i].port
        cpu.interrupts.pio = system.piobus.port
        cpu.interrupts.int_port = system.piobus.port

root = Root(system = system)

Simulation.run(options, root, system, FutureClass)
