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

addToPath('../../tests/configs/')
addToPath('../common')

import ruby_config

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

# ruby options
parser.add_option("--ruby-debug", action="store_true")
parser.add_option("--ruby-debug-file", default="", help="Ruby debug out file (stdout if blank)")
parser.add_option("--protocol", default="", help="Ruby protocol compiled into binary")


# ruby host memory experimentation
parser.add_option("--cache_size", type="int")
parser.add_option("--cache_assoc", type="int")
parser.add_option("--map_levels", type="int")

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
assert(options.timing)
class CPUClass(TimingSimpleCPU): pass
test_mem_mode = 'timing'
FutureClass = None

CPUClass.clock = '1GHz'

#
# Since we are running in timing mode, set the number of M5 ticks to ruby ticks
# to the cpu clock frequency
#
M5_to_ruby_tick = '1000t'

np = options.num_cpus

# check for max instruction count
if options.max_inst:
    max_inst = options.max_inst
else:
    max_inst = 0
    
# set cache size
if options.cache_size:
    cache_size = options.cache_size
else:
    cache_size = 32768 # 32 kB is default
    
# set cache assoc
if options.cache_assoc:
    cache_assoc = options.cache_assoc
else:
    cache_assoc = 8 # 8 is default
    
# set map levels
if options.map_levels:
    map_levels = options.map_levels
else:
    map_levels = 4 # 4 levels is the default

if options.protocol == "MOESI_hammer":
    ruby_config_file = "MOESI_hammer-homogeneous.rb"
elif options.protocol == "MOESI_CMP_token":
    ruby_config_file = "TwoLevel_SplitL1UnifiedL2.rb"
elif options.protocol == "MI_example":
    ruby_config_file = "MI_example-homogeneous.rb"
else:
    print "Error: unsupported ruby protocol"
    sys.exit(1)

#
# Currently, since ruby configuraiton is separate from m5, we need to manually
# tell ruby that two dma ports are created by makeLinuxAlphaRubySystem().
# Eventually, this will be fix with a unified configuration system.
#
rubymem = ruby_config.generate(ruby_config_file,
                               np,
                               np,
                               128,
                               False,
                               cache_size,
                               cache_assoc,
                               map_levels,
                               2,
                               M5_to_ruby_tick)

if options.ruby_debug == True:
  rubymem.debug = True
  rubymem.debug_file = options.ruby_debug_file

system = makeLinuxAlphaRubySystem(test_mem_mode, rubymem, bm[0])

system.cpu = [CPUClass(cpu_id=i) for i in xrange(np)]

if options.l2cache:
    print "Error: -l2cache incompatible with ruby, must configure it ruby-style"
    sys.exit(1)

if options.caches:
    print "Error: -caches incompatible with ruby, must configure it ruby-style"
    sys.exit(1)

for i in xrange(np):
    system.cpu[i].connectMemPorts(system.physmem)

    if options.fastmem:
        system.cpu[i].physmem_port = system.physmem.port

root = Root(system = system)

Simulation.run(options, root, system, FutureClass)
