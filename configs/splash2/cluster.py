# Copyright (c) 2006-2007 The Regents of The University of Michigan
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

# Simple test script
#
# "m5 test.py"

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
parser.add_option("-c", "--numclusters",
                  help="Number of clusters", type="int")
parser.add_option("-n", "--numcpus",
                  help="Number of cpus in total", type="int")
parser.add_option("-f", "--frequency",
                  default = "1GHz",
                  help="Frequency of each CPU")
parser.add_option("--l1size",
                  default = "32kB")
parser.add_option("--l1latency",
                  default = 1)
parser.add_option("--l2size",
                  default = "256kB")
parser.add_option("--l2latency",
                  default = 10)
parser.add_option("--rootdir",
                  help="ROot directory of Splash2",
                  default="/dist/splash2/codes/")
parser.add_option("-b", "--benchmark",
                  help="Splash 2 benchmark to run")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# --------------------
# Define Splash2 Benchmarks
# ====================
class Cholesky(Process):
        executable = options.rootdir + '/kernels/cholesky/CHOLESKY'
        cmd = 'CHOLESKY -p' + str(options.numcpus) + ' '\
             + options.rootdir + '/kernels/cholesky/inputs/tk23.O'

class FFT(Process):
        executable = options.rootdir + 'kernels/fft/FFT'
        cmd = 'FFT -p' + str(options.numcpus) + ' -m18'

class LU_contig(Process):
        executable = options.rootdir + 'kernels/lu/contiguous_blocks/LU'
        cmd = 'LU -p' + str(options.numcpus)

class LU_noncontig(Process):
        executable = options.rootdir + 'kernels/lu/non_contiguous_blocks/LU'
        cmd = 'LU -p' + str(options.numcpus)

class Radix(Process):
        executable = options.rootdir + 'kernels/radix/RADIX'
        cmd = 'RADIX -n524288 -p' + str(options.numcpus)

class Barnes(Process):
        executable = options.rootdir + 'apps/barnes/BARNES'
        cmd = 'BARNES'
        input = options.rootdir + 'apps/barnes/input.p' + str(options.numcpus)

class FMM(Process):
        executable = options.rootdir + 'apps/fmm/FMM'
        cmd = 'FMM'
        input = options.rootdir + 'apps/fmm/inputs/input.2048.p' + str(options.numcpus)

class Ocean_contig(Process):
        executable = options.rootdir + 'apps/ocean/contiguous_partitions/OCEAN'
        cmd = 'OCEAN -p' + str(options.numcpus)

class Ocean_noncontig(Process):
        executable = options.rootdir + 'apps/ocean/non_contiguous_partitions/OCEAN'
        cmd = 'OCEAN -p' + str(options.numcpus)

class Raytrace(Process):
        executable = options.rootdir + 'apps/raytrace/RAYTRACE'
        cmd = 'RAYTRACE -p' + str(options.numcpus) + ' ' \
             + options.rootdir + 'apps/raytrace/inputs/teapot.env'

class Water_nsquared(Process):
        executable = options.rootdir + 'apps/water-nsquared/WATER-NSQUARED'
        cmd = 'WATER-NSQUARED'
        input = options.rootdir + 'apps/water-nsquared/input.p' + str(options.numcpus)

class Water_spatial(Process):
        executable = options.rootdir + 'apps/water-spatial/WATER-SPATIAL'
        cmd = 'WATER-SPATIAL'
        input = options.rootdir + 'apps/water-spatial/input.p' + str(options.numcpus)


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
# Define the clusters with their cpus
# ----------------------
class Cluster:
    pass

cpusPerCluster = options.numcpus/options.numclusters

busFrequency = Frequency(options.frequency)
busFrequency *= cpusPerCluster

all_cpus = []
all_l1s = []
all_l1buses = []
if options.timing:
    clusters = [ Cluster() for i in xrange(options.numclusters)]
    for j in xrange(options.numclusters):
        clusters[j].id = j
    for cluster in clusters:
        cluster.clusterbus = L2XBar(clock=busFrequency)
        all_l1buses += [cluster.clusterbus]
        cluster.cpus = [TimingSimpleCPU(cpu_id = i + cluster.id,
                                        clock=options.frequency)
                        for i in xrange(cpusPerCluster)]
        all_cpus += cluster.cpus
        cluster.l1 = L1(size=options.l1size, assoc = 4)
        all_l1s += [cluster.l1]
elif options.detailed:
    clusters = [ Cluster() for i in xrange(options.numclusters)]
    for j in xrange(options.numclusters):
        clusters[j].id = j
    for cluster in clusters:
        cluster.clusterbus = L2XBar(clock=busFrequency)
        all_l1buses += [cluster.clusterbus]
        cluster.cpus = [DerivO3CPU(cpu_id = i + cluster.id,
                                   clock=options.frequency)
                        for i in xrange(cpusPerCluster)]
        all_cpus += cluster.cpus
        cluster.l1 = L1(size=options.l1size, assoc = 4)
        all_l1s += [cluster.l1]
else:
    clusters = [ Cluster() for i in xrange(options.numclusters)]
    for j in xrange(options.numclusters):
        clusters[j].id = j
    for cluster in clusters:
        cluster.clusterbus = L2XBar(clock=busFrequency)
        all_l1buses += [cluster.clusterbus]
        cluster.cpus = [AtomicSimpleCPU(cpu_id = i + cluster.id,
                                        clock=options.frequency)
                        for i in xrange(cpusPerCluster)]
        all_cpus += cluster.cpus
        cluster.l1 = L1(size=options.l1size, assoc = 4)
        all_l1s += [cluster.l1]

# ----------------------
# Create a system, and add system wide objects
# ----------------------
system = System(cpu = all_cpus, l1_ = all_l1s, l1bus_ = all_l1buses,
                physmem = SimpleMemory(),
                membus = SystemXBar(clock = busFrequency))
system.clock = '1GHz'

system.toL2bus = L2XBar(clock = busFrequency)
system.l2 = L2(size = options.l2size, assoc = 8)

# ----------------------
# Connect the L2 cache and memory together
# ----------------------

system.physmem.port = system.membus.master
system.l2.cpu_side = system.toL2bus.slave
system.l2.mem_side = system.membus.master

# ----------------------
# Connect the L2 cache and clusters together
# ----------------------
for cluster in clusters:
    cluster.l1.cpu_side = cluster.clusterbus.master
    cluster.l1.mem_side = system.toL2bus.slave
    for cpu in cluster.cpus:
        cpu.icache_port = cluster.clusterbus.slave
        cpu.dcache_port = cluster.clusterbus.slave

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
    m5.util.panic("""
The --benchmark environment variable was set to something improper.
Use Cholesky, FFT, LUContig, LUNoncontig, Radix, Barnes, FMM, OceanContig,
OceanNoncontig, Raytrace, WaterNSquared, or WaterSpatial
""")

# --------------------
# Assign the workload to the cpus
# ====================

for cluster in clusters:
    for cpu in cluster.cpus:
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

print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()

