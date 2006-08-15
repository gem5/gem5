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

from SysPaths import *

class Machine:
    def __init__(self, script=None, mem=None, disk=None):
        self.scriptname = script
        self.diskname = disk
        self.memsize = mem

    def script(self):
        if self.scriptname:
            return script(self.scriptname)
        else:
            return ''

    def mem(self):
        if self.memsize:
            return self.memsize
        else:
            return '128MB'

    def disk(self):
        if self.diskname:
            return disk(self.diskname)
        else:
            return env.get('LINUX_IMAGE', disk('linux-latest.img'))

#Benchmarks are defined as a key in a dict which is a list of Machines
# The first defined machine is the test system, the others are driving systems
# Currently there is only support for 1 or 2 machines

Benchmarks = {}
Benchmarks['PovrayBench']       = [Machine('povray-bench.rcS', '512MB', 'povray.img')]
Benchmarks['PovrayAutumn']      = [Machine('povray-autumn.rcS', '512MB', 'povray.img')]
Benchmarks['NetperfStream']     = [Machine('netperf-stream-client.rcS'),
                                   Machine('netperf-server.rcS')]
Benchmarks['NetperfStreamNT']   = [Machine('netperf-stream-nt-client.rcS'),
                                   Machine('netperf-server.rcS')]
Benchmarks['NetperfMaerts']     = [Machine('netperf-maerts-client.rcS'),
                                   Machine('netperf-server.rcS')]
Benchmarks['SurgeStandard']     = [Machine('surge-server.rcS', '512MB'),
                                   Machine('surge-client.rcS', '256MB')]
Benchmarks['SurgeSpecweb']      = [Machine('spec-surge-server.rcS', '512MB'),
                                   Machine('spec-surge-client.rcS', '256MB')]
Benchmarks['Nhfsstone']         = [Machine('nfs-server-nhfsstone.rcS', '512MB'),
                                   Machine('nfs-client-nhfsstone.rcS')]
Benchmarks['Nfs']               = [Machine('nfs-server.rcS', '900MB'),
                                   Machine('nfs-client-dbench.rcS')]
Benchmarks['NfsTcp']            = [Machine('nfs-server.rcS', '900MB'),
                                   Machine('nfs-client-tcp.rcS')]
Benchmarks['IScsiInitiator']    = [Machine('iscsi-client.rcS', '512MB'),
                                   Machine('iscsi-server.rcS', '512MB')]
Benchmarks['IScsiTarget']       = [Machine('iscsi-server.rcS', '512MB'),
                                   Machine('iscsi-client.rcS', '512MB')]
Benchmarks['Validation']        = [Machine('iscsi-server.rcS', '512MB'),
                                   Machine('iscsi-client.rcS', '512MB')]
Benchmarks['Ping']              = [Machine('ping-server.rcS',),
                                   Machine('ping-client.rcS')]


Benchmarks['ValAccDelay']	= [Machine('devtime.rcS', '512MB')]
Benchmarks['ValAccDelay2']	= [Machine('devtimewmr.rcS', '512MB')]
Benchmarks['ValMemLat']         = [Machine('micro_memlat.rcS', '512MB')]
Benchmarks['ValMemLat2MB']	= [Machine('micro_memlat2mb.rcS', '512MB')]
Benchmarks['ValMemLat8MB']	= [Machine('micro_memlat8mb.rcS', '512MB')]
Benchmarks['ValMemLat']         = [Machine('micro_memlat8.rcS', '512MB')]
Benchmarks['ValTlbLat']         = [Machine('micro_tlblat.rcS', '512MB')]
Benchmarks['ValSysLat']         = [Machine('micro_syscall.rcS', '512MB')]
Benchmarks['ValCtxLat']         = [Machine('micro_ctx.rcS', '512MB')]
Benchmarks['ValStream']         = [Machine('micro_stream.rcS', '512MB')]
Benchmarks['ValStreamScale']	= [Machine('micro_streamscale.rcS', '512MB')]
Benchmarks['ValStreamCopy']	= [Machine('micro_streamcopy.rcS', '512MB')]

benchs = Benchmarks.keys()
benchs.sort()
DefinedBenchmarks = ", ".join(benchs)
