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

'''
Test file for the m5threads atomic test
'''
from testlib import *

cpu_types = ('DerivO3CPU', 'TimingSimpleCPU')

if config.bin_path:
    base_path = config.bin_path
else:
    base_path = joinpath(absdirpath(__file__), '..', 'test-progs',
                         'test_atomic', 'bin')

binary = 'test_atomic'
url = config.resource_url + '/current/test-progs/pthread/bin/' + binary
DownloadedProgram(url, base_path, binary)

verifiers = (
    verifier.MatchStdoutNoPerf(joinpath(getcwd(), 'ref/sparc/linux/simout')),
)

for cpu in cpu_types:
    gem5_verify_config(
        name='test-atomic-' + cpu,
        verifiers=verifiers,
        config=joinpath(getcwd(), 'atomic_system.py'),
        config_args=['--cpu-type', cpu,
                     '--num-cores', '8',
                     '--cmd', joinpath(base_path, binary)],
        valid_isas=('SPARC',),
        valid_hosts=constants.supported_hosts,
    )
