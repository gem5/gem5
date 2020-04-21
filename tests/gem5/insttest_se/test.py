# Copyright (c) 2020 The Regents of the University of California
# All Rights Reserved.
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
# Authors: Bobby R. Bruce

'''
Test file for the insttest binary running on the RISCV and SPARC
'''
from testlib import *

test_progs = {
    'riscv': ('insttest-rv64a', 'insttest-rv64c', 'insttest-rv64d',
        'insttest-rv64f', 'insttest-rv64i', 'insttest-rv64m'),
    'sparc': ('insttest',)
}
#o3-timing  simple-atomic  simple-timing
cpu_types = {
    'riscv' : ('AtomicSimpleCPU', 'TimingSimpleCPU', 'DerivO3CPU', 'MinorCPU'),
    'sparc' : ('AtomicSimpleCPU', 'TimingSimpleCPU')
}
supported_os = {
    'riscv' : ('linux',),
    'sparc' : ('linux',)
}

if config.bin_path:
    base_path = config.bin_path
else:
    base_path = joinpath(absdirpath(__file__), '..', 'test-progs')

urlbase = config.resource_url + '/test-progs/insttest/bin/'
for isa in test_progs:
    for binary in test_progs[isa]:
        for  operating_s in supported_os[isa]:
            import os
            url = urlbase + isa + '/' + operating_s + '/' + binary
            path = joinpath(base_path, isa, operating_s, binary)

            try:
                program = DownloadedProgram(url, path, binary)
            except:
                continue

            ref_path = joinpath(getcwd(), 'ref', isa, operating_s, binary)
            verifiers = (
                verifier.MatchStdoutNoPerf(joinpath(ref_path, 'simout')),
            )

            for cpu in cpu_types[isa]:

                gem5_verify_config(
                    name='test-'+binary + '-' + operating_s + '-' + cpu,
                    fixtures=(program,),
                    verifiers=verifiers,
                    config=joinpath(config.base_dir, 'configs',
                        'example','se.py'),
                    config_args=['--cmd', joinpath(path, binary),
                        '--cpu-type', cpu, '--caches'],
                    valid_isas=(isa.upper(),),
                )
