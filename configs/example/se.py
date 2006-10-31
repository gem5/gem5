# Copyright (c) 2006 The Regents of The University of Michigan
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
# Authors: Steve Reinhardt

# Simple test script
#
# "m5 test.py"

import m5
from m5.objects import *
import os, optparse, sys
m5.AddToPath('../common')
import Simulation
from Caches import *

# Get paths we might need.  It's expected this file is in m5/configs/example.
config_path = os.path.dirname(os.path.abspath(__file__))
config_root = os.path.dirname(config_path)
m5_root = os.path.dirname(config_root)
print m5_root
print config_path
print config_root


parser = optparse.OptionParser()

# Benchmark options
parser.add_option("-c", "--cmd",
                  default=os.path.join(m5_root, "tests/test-progs/hello/bin/alpha/linux/hello"),
                  help="The binary to run in syscall emulation mode.")
parser.add_option("-o", "--options", default="",
                  help="The options to pass to the binary, use \" \" around the entire\
                        string.")
parser.add_option("-i", "--input", default="",
                  help="A file of input to give to the binary.")

execfile(os.path.join(config_root, "common", "Options.py"))

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

process = LiveProcess()
process.executable = options.cmd
process.cmd = options.cmd + " " + options.options
if options.input != "":
    process.input = options.input

if options.detailed:
    #check for SMT workload
    workloads = options.cmd.split(';')
    if len(workloads) > 1:
        process = []
        smt_idx = 0
        inputs = []

        if options.input != "":
            inputs = options.input.split(';')

        for wrkld in workloads:
            smt_process = LiveProcess()
            smt_process.executable = wrkld
            smt_process.cmd = wrkld + " " + options.options
            if inputs and inputs[smt_idx]:
                smt_process.input = inputs[smt_idx]
            process += [smt_process, ]
            smt_idx += 1


if options.timing:
    CPUClass = TimingSimpleCPU
    test_mem_mode = 'timing'
elif options.detailed:
    CPUClass = DerivO3CPU
    test_mem_mode = 'timing'
else:
    CPUClass = AtomicSimpleCPU
    test_mem_mode = 'atomic'

CPUClass.clock = '2GHz'

np = options.num_cpus

system = System(cpu = [CPUClass(cpu_id=i) for i in xrange(np)],
                physmem = PhysicalMemory(range=AddrRange("512MB")),
                membus = Bus(), mem_mode = test_mem_mode)

system.physmem.port = system.membus.port

for i in xrange(np):
    if options.caches and not options.standard_switch:
        system.cpu[i].addPrivateSplitL1Caches(L1Cache(size = '32kB'),
                                              L1Cache(size = '64kB'))
    system.cpu[i].connectMemPorts(system.membus)
    system.cpu[i].mem = system.physmem
    system.cpu[i].workload = process

root = Root(system = system)

Simulation.run(options, root, system)
