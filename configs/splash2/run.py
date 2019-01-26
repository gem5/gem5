# Copyright (c) 2005-2007 The Regents of The University of Michigan
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
# Authors: Ron Dreslinski

# Splash2 Run Script
#

from __future__ import print_function

import os
import optparse
import sys

import m5
from m5.objects import *

# --------------------
# Define Command Line Options
# ====================

parser = optparse.OptionParser()

parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("-m", "--maxtick", type="int")
parser.add_option("-n", "--numcpus",
                  help="Number of cpus in total", type="int")
parser.add_option("-f", "--frequency",
                  default = "1GHz",
                  help="Frequency of each CPU")
parser.add_option("--l1size",
                  default = "32kB")
parser.add_option("--l1latency",
                  default = "1ns")
parser.add_option("--l2size",
                  default = "256kB")
parser.add_option("--l2latency",
                  default = "10ns")
parser.add_option("--rootdir",
                  help="Root directory of Splash2",
                  default="/dist/splash2/codes")
parser.add_option("-b", "--benchmark",
                  help="Splash 2 benchmark to run")

(options, args) = parser.parse_args()

if args:
    print("Error: script doesn't take any positional arguments")
    sys.exit(1)

if not options.numcpus:
    print("Specify the number of cpus with -n")
    sys.exit(1)

# --------------------
# Define Splash2 Benchmarks
# ====================
class Cholesky(Process):
    cwd = options.rootdir + '/kernels/cholesky'
    executable = options.rootdir + '/kernels/cholesky/CHOLESKY'
    cmd = ['CHOLESKY', '-p' +  str(options.numcpus),
            options.rootdir + '/kernels/cholesky/inputs/tk23.O']

class FFT(Process):
    cwd = options.rootdir + '/kernels/fft'
    executable = options.rootdir + '/kernels/fft/FFT'
    cmd = ['FFT', '-p', str(options.numcpus), '-m18']

class LU_contig(Process):
    executable = options.rootdir + '/kernels/lu/contiguous_blocks/LU'
    cmd = ['LU', '-p', str(options.numcpus)]
    cwd = options.rootdir + '/kernels/lu/contiguous_blocks'

class LU_noncontig(Process):
    executable = options.rootdir + '/kernels/lu/non_contiguous_blocks/LU'
    cmd = ['LU', '-p', str(options.numcpus)]
    cwd = options.rootdir + '/kernels/lu/non_contiguous_blocks'

class Radix(Process):
    executable = options.rootdir + '/kernels/radix/RADIX'
    cmd = ['RADIX', '-n524288', '-p', str(options.numcpus)]
    cwd = options.rootdir + '/kernels/radix'

class Barnes(Process):
    executable = options.rootdir + '/apps/barnes/BARNES'
    cmd = ['BARNES']
    input = options.rootdir + '/apps/barnes/input.p' + str(options.numcpus)
    cwd = options.rootdir + '/apps/barnes'

class FMM(Process):
    executable = options.rootdir + '/apps/fmm/FMM'
    cmd = ['FMM']
    if str(options.numcpus) == '1':
        input = options.rootdir + '/apps/fmm/inputs/input.2048'
    else:
        input = options.rootdir + '/apps/fmm/inputs/input.2048.p' + str(options.numcpus)
    cwd = options.rootdir + '/apps/fmm'

class Ocean_contig(Process):
    executable = options.rootdir + '/apps/ocean/contiguous_partitions/OCEAN'
    cmd = ['OCEAN', '-p', str(options.numcpus)]
    cwd = options.rootdir + '/apps/ocean/contiguous_partitions'

class Ocean_noncontig(Process):
    executable = options.rootdir + '/apps/ocean/non_contiguous_partitions/OCEAN'
    cmd = ['OCEAN', '-p', str(options.numcpus)]
    cwd = options.rootdir + '/apps/ocean/non_contiguous_partitions'

class Raytrace(Process):
    executable = options.rootdir + '/apps/raytrace/RAYTRACE'
    cmd = ['RAYTRACE', '-p' + str(options.numcpus),
           options.rootdir + '/apps/raytrace/inputs/teapot.env']
    cwd = options.rootdir + '/apps/raytrace'

class Water_nsquared(Process):
    executable = options.rootdir + '/apps/water-nsquared/WATER-NSQUARED'
    cmd = ['WATER-NSQUARED']
    if options.numcpus==1:
        input = options.rootdir + '/apps/water-nsquared/input'
    else:
        input = options.rootdir + '/apps/water-nsquared/input.p' + str(options.numcpus)
    cwd = options.rootdir + '/apps/water-nsquared'

class Water_spatial(Process):
    executable = options.rootdir + '/apps/water-spatial/WATER-SPATIAL'
    cmd = ['WATER-SPATIAL']
    if options.numcpus==1:
        input = options.rootdir + '/apps/water-spatial/input'
    else:
        input = options.rootdir + '/apps/water-spatial/input.p' + str(options.numcpus)
    cwd = options.rootdir + '/apps/water-spatial'

# --------------------
# Base L1 Cache Definition
# ====================

class L1(Cache):
    latency = options.l1latency
    mshrs = 12
    tgts_per_mshr = 8

# ----------------------
# Base L2 Cache Definition
# ----------------------

class L2(Cache):
    latency = options.l2latency
    mshrs = 92
    tgts_per_mshr = 16
    write_buffers = 8

# ----------------------
# Define the cpus
# ----------------------

busFrequency = Frequency(options.frequency)

if options.timing:
    cpus = [TimingSimpleCPU(cpu_id = i,
                            clock=options.frequency)
            for i in range(options.numcpus)]
elif options.detailed:
    cpus = [DerivO3CPU(cpu_id = i,
                       clock=options.frequency)
            for i in range(options.numcpus)]
else:
    cpus = [AtomicSimpleCPU(cpu_id = i,
                            clock=options.frequency)
            for i in range(options.numcpus)]

# ----------------------
# Create a system, and add system wide objects
# ----------------------
system = System(cpu = cpus, physmem = SimpleMemory(),
                membus = SystemXBar(clock = busFrequency))
system.clock = '1GHz'

system.toL2bus = L2XBar(clock = busFrequency)
system.l2 = L2(size = options.l2size, assoc = 8)

# ----------------------
# Connect the L2 cache and memory together
# ----------------------

system.physmem.port = system.membus.master
system.l2.cpu_side = system.toL2bus.master
system.l2.mem_side = system.membus.slave
system.system_port = system.membus.slave

# ----------------------
# Connect the L2 cache and clusters together
# ----------------------
for cpu in cpus:
    cpu.addPrivateSplitL1Caches(L1(size = options.l1size, assoc = 1),
                                L1(size = options.l1size, assoc = 4))
    # connect cpu level-1 caches to shared level-2 cache
    cpu.connectAllPorts(system.toL2bus, system.membus)


# ----------------------
# Define the root
# ----------------------

root = Root(full_system = False, system = system)

# --------------------
# Pick the correct Splash2 Benchmarks
# ====================
if options.benchmark == 'Cholesky':
    root.workload = Cholesky()
elif options.benchmark == 'FFT':
    root.workload = FFT()
elif options.benchmark == 'LUContig':
    root.workload = LU_contig()
elif options.benchmark == 'LUNoncontig':
    root.workload = LU_noncontig()
elif options.benchmark == 'Radix':
    root.workload = Radix()
elif options.benchmark == 'Barnes':
    root.workload = Barnes()
elif options.benchmark == 'FMM':
    root.workload = FMM()
elif options.benchmark == 'OceanContig':
    root.workload = Ocean_contig()
elif options.benchmark == 'OceanNoncontig':
    root.workload = Ocean_noncontig()
elif options.benchmark == 'Raytrace':
    root.workload = Raytrace()
elif options.benchmark == 'WaterNSquared':
    root.workload = Water_nsquared()
elif options.benchmark == 'WaterSpatial':
    root.workload = Water_spatial()
else:
    print("The --benchmark environment variable was set to something "
          "improper. Use Cholesky, FFT, LUContig, LUNoncontig, Radix, "
          "Barnes, FMM, OceanContig, OceanNoncontig, Raytrace, WaterNSquared, "
          "or WaterSpatial", file=sys.stderr)
    sys.exit(1)

# --------------------
# Assign the workload to the cpus
# ====================

for cpu in cpus:
    cpu.workload = root.workload

# ----------------------
# Run the simulation
# ----------------------

if options.timing or options.detailed:
    root.system.mem_mode = 'timing'

# instantiate configuration
m5.instantiate()

# simulate until program terminates
if options.maxtick:
    exit_event = m5.simulate(options.maxtick)
else:
    exit_event = m5.simulate(m5.MaxTick)

print('Exiting @ tick', m5.curTick(), 'because', exit_event.getCause())

