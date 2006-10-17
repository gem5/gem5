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

parser.add_option("-d", "--detailed", action="store_true")
parser.add_option("-t", "--timing", action="store_true")
parser.add_option("-m", "--maxtick", type="int")
parser.add_option("--maxtime", type="float")
parser.add_option("--dual", action="store_true",
                  help="Simulate two systems attached with an ethernet link")
parser.add_option("-b", "--benchmark", action="store", type="string",
                  dest="benchmark",
                  help="Specify the benchmark to run. Available benchmarks: %s"\
                  % DefinedBenchmarks)
parser.add_option("--etherdump", action="store", type="string", dest="etherdump",
                  help="Specify the filename to dump a pcap capture of the" \
                  "ethernet traffic")
parser.add_option("--checkpoint_dir", action="store", type="string",
                  help="Place all checkpoints in this absolute directory")
parser.add_option("-c", "--checkpoint", action="store", type="int",
                  help="restore from checkpoint <N>")

(options, args) = parser.parse_args()

if args:
    print "Error: script doesn't take any positional arguments"
    sys.exit(1)

# client system CPU is always simple... note this is an assignment of
# a class, not an instance.
ClientCPUClass = AtomicSimpleCPU
client_mem_mode = 'atomic'

if options.detailed:
    ServerCPUClass = DerivO3CPU
    server_mem_mode = 'timing'
elif options.timing:
    ServerCPUClass = TimingSimpleCPU
    server_mem_mode = 'timing'
else:
    ServerCPUClass = AtomicSimpleCPU
    server_mem_mode = 'atomic'

ServerCPUClass.clock = '2GHz'
ClientCPUClass.clock = '2GHz'

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

server_sys = makeLinuxAlphaSystem(server_mem_mode, bm[0])
server_sys.cpu = ServerCPUClass(cpu_id=0)
server_sys.cpu.connectMemPorts(server_sys.membus)
server_sys.cpu.mem = server_sys.physmem

if len(bm) == 2:
    client_sys = makeLinuxAlphaSystem(client_mem_mode, bm[1])
    client_sys.cpu = ClientCPUClass(cpu_id=0)
    client_sys.cpu.connectMemPorts(client_sys.membus)
    client_sys.cpu.mem = client_sys.physmem
    root = makeDualRoot(server_sys, client_sys, options.etherdump)
elif len(bm) == 1:
    root = Root(clock = '1THz', system = server_sys)
else:
    print "Error I don't know how to create more than 2 systems."
    sys.exit(1)

m5.instantiate(root)

if options.checkpoint:
    from os.path import isdir
    from os import listdir, getcwd
    import re
    if options.checkpoint_dir:
        cptdir = options.checkpoint_dir
    else:
        cptdir = getcwd()

    if not isdir(cptdir):
        m5.panic("checkpoint dir %s does not exist!" % cptdir)

    dirs = listdir(cptdir)
    expr = re.compile('cpt.([0-9]*)')
    cpts = []
    for dir in dirs:
        match = expr.match(dir)
        if match:
            cpts.append(match.group(1))

    if options.checkpoint > len(cpts):
        m5.panic('Checkpoint %d not found' % options.checkpoint)

    m5.restoreCheckpoint(root, "/".join([cptdir, "cpt.%s" % cpts[options.checkpoint - 1]]))

if options.maxtick:
    maxtick = options.maxtick
elif options.maxtime:
    simtime = int(options.maxtime * root.clock.value)
    print "simulating for: ", simtime
    maxtick = simtime
else:
    maxtick = -1

exit_event = m5.simulate(maxtick)

while exit_event.getCause() == "checkpoint":
    if options.checkpoint_dir:
        m5.checkpoint(root, "/".join([options.checkpoint_dir, "cpt.%d"]))
    else:
        m5.checkpoint(root, "cpt.%d")

    if maxtick == -1:
        exit_event = m5.simulate(maxtick)
    else:
        exit_event = m5.simulate(maxtick - m5.curTick())

print 'Exiting @ cycle', m5.curTick(), 'because', exit_event.getCause()
