# Copyright (c) 2017 Mark D. Hill and David A. Wood
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
# Authors: Sean Wilson

'''
Test file for the util m5 exit assembly instruction.
'''
from testlib import *

test_progs = {
    'x86': ('hello64-static', 'hello64-dynamic', 'hello32-static'),
    'arm': ('hello64-static', 'hello32-static'),
}

urlbase = 'http://gem5.org/dist/current/test-progs/hello/bin/'
for isa in test_progs:
    for binary in test_progs[isa]:
        import os
        url = urlbase + isa + '/linux/' + binary
        path = joinpath(absdirpath(__file__), '..', 'test-progs', 'hello',
                        'bin', isa, 'linux')
        hello_program = DownloadedProgram(url, path, binary)

        ref_path = joinpath(getcwd(), 'ref')

        verifiers = (
                verifier.MatchStdoutNoPerf(joinpath(ref_path, 'simout')),
        )

        gem5_verify_config(
                name='test'+binary,
                fixtures=(hello_program,),
                verifiers=verifiers,
                config=joinpath(config.base_dir, 'configs', 'example','se.py'),
                config_args=['--cmd', joinpath(path, binary)],
                valid_isas=(isa.upper(),),
        )
