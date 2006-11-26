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
# Authors: Lisa Hsu

from os import getcwd
from os.path import join as joinpath
import m5
from m5.objects import *
m5.AddToPath('../common')
from Caches import L1Cache

def setCPUClass(options):

    atomic = False
    if options.timing:
        TmpClass = TimingSimpleCPU
    elif options.detailed:
        if not options.caches:
            print "O3 CPU must be used with caches"
            sys.exit(1)
        TmpClass = DerivO3CPU
    else:
        TmpClass = AtomicSimpleCPU
        atomic = True

    CPUClass = None
    test_mem_mode = 'atomic'

    if not atomic:
        if options.checkpoint_restore:
            CPUClass = TmpClass
            TmpClass = AtomicSimpleCPU
        else:
            test_mem_mode = 'timing'

    return (TmpClass, test_mem_mode, CPUClass)


def run(options, root, testsys, cpu_class):
    if options.maxtick:
        maxtick = options.maxtick
    elif options.maxtime:
        simtime = int(options.maxtime * root.clock.value)
        print "simulating for: ", simtime
        maxtick = simtime
    else:
        maxtick = m5.MaxTick

    if options.checkpoint_dir:
        cptdir = options.checkpoint_dir
    else:
        cptdir = getcwd()

    np = options.num_cpus
    max_checkpoints = options.max_checkpoints
    switch_cpus = None

    if cpu_class:
        switch_cpus = [cpu_class(defer_registration=True, cpu_id=(np+i))
                       for i in xrange(np)]

        for i in xrange(np):
            switch_cpus[i].system =  testsys
            if not m5.build_env['FULL_SYSTEM']:
                switch_cpus[i].workload = testsys.cpu[i].workload
            switch_cpus[i].clock = testsys.cpu[0].clock

        root.switch_cpus = switch_cpus
        switch_cpu_list = [(testsys.cpu[i], switch_cpus[i]) for i in xrange(np)]

    if options.standard_switch:
        switch_cpus = [TimingSimpleCPU(defer_registration=True, cpu_id=(np+i))
                       for i in xrange(np)]
        switch_cpus_1 = [DerivO3CPU(defer_registration=True, cpu_id=(2*np+i))
                        for i in xrange(np)]

        for i in xrange(np):
            switch_cpus[i].system =  testsys
            switch_cpus_1[i].system =  testsys
            if not m5.build_env['FULL_SYSTEM']:
                switch_cpus[i].workload = testsys.cpu[i].workload
                switch_cpus_1[i].workload = testsys.cpu[i].workload
            switch_cpus[i].clock = testsys.cpu[0].clock
            switch_cpus_1[i].clock = testsys.cpu[0].clock

            if not options.caches:
                # O3 CPU must have a cache to work.
                switch_cpus_1[i].addPrivateSplitL1Caches(L1Cache(size = '32kB'),
                                                         L1Cache(size = '64kB'))
                switch_cpus_1[i].connectMemPorts(testsys.membus)


            testsys.switch_cpus = switch_cpus
            testsys.switch_cpus_1 = switch_cpus_1
            switch_cpu_list = [(testsys.cpu[i], switch_cpus[i]) for i in xrange(np)]
            switch_cpu_list1 = [(switch_cpus[i], switch_cpus_1[i]) for i in xrange(np)]

    m5.instantiate(root)

    if options.checkpoint_restore:
        from os.path import isdir
        from os import listdir
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

        cpt_num = options.checkpoint_restore

        if cpt_num > len(cpts):
            m5.panic('Checkpoint %d not found' % cpt_num)

        m5.restoreCheckpoint(root,
                             joinpath(cptdir, "cpt.%s" % cpts[cpt_num - 1]))

    if options.standard_switch or cpu_class:
        exit_event = m5.simulate(10000)

        ## when you change to Timing (or Atomic), you halt the system given
        ## as argument.  When you are finished with the system changes
        ## (including switchCpus), you must resume the system manually.
        ## You DON'T need to resume after just switching CPUs if you haven't
        ## changed anything on the system level.

        m5.changeToTiming(testsys)
        m5.switchCpus(switch_cpu_list)
        m5.resume(testsys)

        if options.standard_switch:
            exit_event = m5.simulate(options.warmup)
            m5.switchCpus(switch_cpu_list1)

    num_checkpoints = 0
    exit_cause = ''

    ## Checkpoints being taken via the command line at <when> and at subsequent
    ## periods of <period>.  Checkpoint instructions received from the benchmark running
    ## are ignored and skipped in favor of command line checkpoint instructions.
    if options.take_checkpoints:
        [when, period] = options.take_checkpoints.split(",", 1)
        when = int(when)
        period = int(period)

        exit_event = m5.simulate(when)
        while exit_event.getCause() == "checkpoint":
            exit_event = m5.simulate(when - m5.curTick())

        if exit_event.getCause() == "simulate() limit reached":
            m5.checkpoint(root, joinpath(cptdir, "cpt.%d"))
            num_checkpoints += 1

        sim_ticks = when
        exit_cause = "maximum %d checkpoints dropped" % max_checkpoints
        while num_checkpoints < max_checkpoints:
            if (sim_ticks + period) > maxtick:
                exit_event = m5.simulate(maxtick - sim_ticks)
                exit_cause = exit_event.getCause()
                break
            else:
                exit_event = m5.simulate(period)
                sim_ticks += period
                while exit_event.getCause() == "checkpoint":
                    exit_event = m5.simulate(sim_ticks - m5.curTick())
                if exit_event.getCause() == "simulate() limit reached":
                    m5.checkpoint(root, joinpath(cptdir, "cpt.%d"))
                    num_checkpoints += 1

    else: #no checkpoints being taken via this script
        exit_event = m5.simulate(maxtick)

        while exit_event.getCause() == "checkpoint":
            m5.checkpoint(root, joinpath(cptdir, "cpt.%d"))
            num_checkpoints += 1
            if num_checkpoints == max_checkpoints:
                exit_cause =  "maximum %d checkpoints dropped" % max_checkpoints
                break

            exit_event = m5.simulate(maxtick - m5.curTick())
            exit_cause = exit_event.getCause()

    if exit_cause == '':
        exit_cause = exit_event.getCause()
    print 'Exiting @ cycle %i because %s' % (m5.curTick(), exit_cause)

