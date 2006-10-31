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
# Authors: Ali Saidi

import optparse, os, sys

import m5
from m5.objects import *
m5.AddToPath('../common')
from FSConfig import *
from SysPaths import *
from Benchmarks import *

if not m5.build_env['FULL_SYSTEM']:
    m5.panic("This script requires full-system mode (ALPHA_FS).")

parser = optparse.OptionParser()

# Benchmark options
parser.add_option("--dual", action="store_true",
                  help="Simulate two systems attached with an ethernet link")
parser.add_option("-b", "--benchmark", action="store", type="string",
                  dest="benchmark",
                  help="Specify the benchmark to run. Available benchmarks: %s"\
                  % DefinedBenchmarks)

# system options
parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("-n", "--num_cpus", type="int", default=1)
parser.add_option("--caches", action="store_true")

# Run duration options
parser.add_option("-m", "--maxtick", type="int")
parser.add_option("--maxtime", type="float")

# Metafile options
parser.add_option("--etherdump", action="store", type="string", dest="etherdump",
                  help="Specify the filename to dump a pcap capture of the" \
                  "ethernet traffic")

# Checkpointing options
###Note that performing checkpointing via python script files will override
###checkpoint instructions built into binaries.
parser.add_option("--take_checkpoints", action="store", type="string",
                  help="<M,N> will take checkpoint at cycle M and every N cycles \
                  thereafter")
parser.add_option("--max_checkpoints", action="store", type="int",
                  help="the maximum number of checkpoints to drop",
                  default=5)
parser.add_option("--checkpoint_dir", action="store", type="string",
                  help="Place all checkpoints in this absolute directory")
parser.add_option("-r", "--checkpoint_restore", action="store", type="int",
                  help="restore from checkpoint <N>")

# CPU Switching - default switch model goes from a checkpoint
# to a timing simple CPU with caches to warm up, then to detailed CPU for
# data measurement
parser.add_option("-s", "--standard_switch", action="store_true",
                  help="switch from one cpu mode to another")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

class MyCache(BaseCache):
    assoc = 2
    block_size = 64
    latency = 1
    mshrs = 10
    tgts_per_mshr = 5
    protocol = CoherenceProtocol(protocol='moesi')

# driver system CPU is always simple... note this is an assignment of
# a class, not an instance.
DriveCPUClass = AtomicSimpleCPU
drive_mem_mode = 'atomic'

# system under test can be any of these CPUs
if options.detailed:
    TestCPUClass = DerivO3CPU
    test_mem_mode = 'timing'
elif options.timing:
    TestCPUClass = TimingSimpleCPU
    test_mem_mode = 'timing'
else:
    TestCPUClass = AtomicSimpleCPU
    test_mem_mode = 'atomic'

TestCPUClass.clock = '2GHz'
DriveCPUClass.clock = '2GHz'

if options.benchmark:
    try:
        bm = Benchmarks[options.benchmark]
    except KeyError:
        print "Error benchmark %s has not been defined." % options.benchmark
        print "Valid benchmarks are: %s" % DefinedBenchmarks
        sys.exit(1)
else:
    if options.dual:
        bm = [SysConfig(), SysConfig()]
    else:
        bm = [SysConfig()]

test_sys = makeLinuxAlphaSystem(test_mem_mode, bm[0])
np = options.num_cpus
test_sys.cpu = [TestCPUClass(cpu_id=i) for i in xrange(np)]
for i in xrange(np):
    if options.caches and not options.standard_switch:
        test_sys.cpu[i].addPrivateSplitL1Caches(MyCache(size = '32kB'),
                                                  MyCache(size = '64kB'))
    test_sys.cpu[i].connectMemPorts(test_sys.membus)

if len(bm) == 2:
    drive_sys = makeLinuxAlphaSystem(drive_mem_mode, bm[1])
    drive_sys.cpu = DriveCPUClass(cpu_id=0)
    drive_sys.cpu.connectMemPorts(drive_sys.membus)
    root = makeDualRoot(test_sys, drive_sys, options.etherdump)
elif len(bm) == 1:
    root = Root(clock = '1THz', system = test_sys)
else:
    print "Error I don't know how to create more than 2 systems."
    sys.exit(1)

if options.standard_switch:
    switch_cpus = [TimingSimpleCPU(defer_registration=True, cpu_id=(np+i)) for i in xrange(np)]
    switch_cpus1 = [DerivO3CPU(defer_registration=True, cpu_id=(2*np+i)) for i in xrange(np)]
    for i in xrange(np):
        switch_cpus[i].system =  test_sys
        switch_cpus1[i].system =  test_sys
        switch_cpus[i].clock = TestCPUClass.clock
        switch_cpus1[i].clock = TestCPUClass.clock
        if options.caches:
            switch_cpus[i].addPrivateSplitL1Caches(MyCache(size = '32kB'),
                                                    MyCache(size = '64kB'))

        switch_cpus[i].connectMemPorts(test_sys.membus)
        root.switch_cpus = switch_cpus
        root.switch_cpus1 = switch_cpus1
        switch_cpu_list = [(test_sys.cpu[i], switch_cpus[i]) for i in xrange(np)]
        switch_cpu_list1 = [(switch_cpus[i], switch_cpus1[i]) for i in xrange(np)]

m5.instantiate(root)

if options.checkpoint_dir:
    cptdir = options.checkpoint_dir
else:
    cptdir = os.getcwd()

if options.checkpoint_restore:
    from os.path import isdir
    from os import listdir, getcwd
    import re

    if not isdir(cptdir):
        m5.panic("checkpoint dir %s does not exist!" % cptdir)

    dirs = listdir(cptdir)
    expr = re.compile('cpt.([0-9]*)')
    cpts = []
    for dir in dirs:
        match = expr.match(dir)
        if match:
            cpts.append(match.group(1))

    cpts.sort(lambda a,b: cmp(long(a), long(b)))

    if options.checkpoint_restore > len(cpts):
        m5.panic('Checkpoint %d not found' % options.checkpoint_restore)

    m5.restoreCheckpoint(root, "/".join([cptdir, "cpt.%s" % cpts[options.checkpoint_restore - 1]]))

if options.standard_switch:
    exit_event = m5.simulate(1000)
    ## when you change to Timing (or Atomic), you halt the system given
    ## as argument.  When you are finished with the system changes
    ## (including switchCpus), you must resume the system manually.
    ## You DON'T need to resume after just switching CPUs if you haven't
    ## changed anything on the system level.
    m5.changeToTiming(test_sys)
    m5.switchCpus(switch_cpu_list)
    m5.resume(test_sys)

    exit_event = m5.simulate(500000000000)
    m5.switchCpus(switch_cpu_list1)

if options.maxtick:
    maxtick = options.maxtick
elif options.maxtime:
    simtime = int(options.maxtime * root.clock.value)
    print "simulating for: ", simtime
    maxtick = simtime
else:
    maxtick = -1

num_checkpoints = 0

exit_cause = ''

if options.take_checkpoints:
    [when, period] = options.take_checkpoints.split(",", 1)
    when = int(when)
    period = int(period)

    exit_event = m5.simulate(when)
    while exit_event.getCause() == "checkpoint":
        exit_event = m5.simulate(when - m5.curTick())

    if exit_event.getCause() == "simulate() limit reached":
        m5.checkpoint(root, cptdir + "cpt.%d")
        num_checkpoints += 1

    sim_ticks = when
    exit_cause = "maximum %d checkpoints dropped" % options.max_checkpoints
    while num_checkpoints < options.max_checkpoints:
        if (sim_ticks + period) > maxtick and maxtick != -1:
            exit_event = m5.simulate(maxtick - sim_ticks)
            exit_cause = exit_event.getCause()
            break
        else:
            exit_event = m5.simulate(period)
            sim_ticks += period
            while exit_event.getCause() == "checkpoint":
                exit_event = m5.simulate(period - m5.curTick())
            if exit_event.getCause() == "simulate() limit reached":
                m5.checkpoint(root, cptdir + "cpt.%d")
                num_checkpoints += 1

else: #no checkpoints being taken via this script
    exit_event = m5.simulate(maxtick)

    while exit_event.getCause() == "checkpoint":
        m5.checkpoint(root, cptdir + "cpt.%d")
        num_checkpoints += 1
        if num_checkpoints == options.max_checkpoints:
            exit_cause =  "maximum %d checkpoints dropped" % options.max_checkpoints
            break

        if maxtick == -1:
            exit_event = m5.simulate(maxtick)
        else:
            exit_event = m5.simulate(maxtick - m5.curTick())

        exit_cause = exit_event.getCause()

if exit_cause == '':
    exit_cause = exit_event.getCause()
print 'Exiting @ cycle', m5.curTick(), 'because ', exit_cause
