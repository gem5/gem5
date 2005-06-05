# Copyright (c) 2005 The Regents of The University of Michigan
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

from m5 import *
import Splash2

if 'SYSTEM' not in env:
    panic("The SYSTEM environment variable must be set!\ne.g -ESYSTEM=Detailed\n")

if env['SYSTEM'] == 'Simple':
    from SimpleConfig import *
    BaseCPU.workload = Parent.workload
    SimpleStandAlone.cpu = [ CPU() for i in xrange(int(env['NP'])) ]
    root = SimpleStandAlone()
elif env['SYSTEM'] == 'Detailed':
    from DetailedConfig import *
    BaseCPU.workload = Parent.workload
    DetailedStandAlone.cpu = [ DetailedCPU() for i in xrange(int(env['NP'])) ]
    root = DetailedStandAlone()
else:
    panic("The SYSTEM environment variable was set to something improper.\n Use Simple or Detailed\n")

if 'BENCHMARK' not in env:
        panic("The BENCHMARK environment variable must be set!\ne.g. -EBENCHMARK=Cholesky\n")

if env['BENCHMARK'] == 'Cholesky':
    root.workload = Splash2.Cholesky()
elif env['BENCHMARK'] == 'FFT':
    root.workload = Splash2.FFT()
elif env['BENCHMARK'] == 'LUContig':
    root.workload = Splash2.LU_contig()
elif env['BENCHMARK'] == 'LUNoncontig':
    root.workload = Splash2.LU_noncontig()
elif env['BENCHMARK'] == 'Radix':
    root.workload = Splash2.Radix()
elif env['BENCHMARK'] == 'Barnes':
    root.workload = Splash2.Barnes()
elif env['BENCHMARK'] == 'FMM':
    root.workload = Splash2.FMM()
elif env['BENCHMARK'] == 'OceanContig':
    root.workload = Splash2.Ocean_contig()
elif env['BENCHMARK'] == 'OceanNoncontig':
    root.workload = Splash2.Ocean_noncontig()
elif env['BENCHMARK'] == 'Raytrace':
    root.workload = Splash2.Raytrace()
elif env['BENCHMARK'] == 'WaterNSquared':
    root.workload = Splash2.Water_nsquared()
elif env['BENCHMARK'] == 'WaterSpatial':
    root.workload = Splash2.Water_spatial()
else:
    panic("The BENCHMARK environment variable was set to something" \
          +" improper.\nUse Cholesky, FFT, LUContig, LUNoncontig, Radix" \
          +", Barnes, FMM, OceanContig,\nOceanNoncontig, Raytrace," \
          +" WaterNSquared, or WaterSpatial\n")
