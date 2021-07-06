# Copyright (c) 2021 Huawei International
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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
Test file for the hdf5 stats.
It just runs an SE simulation with the hdf5 stats and checks that the
simulation succeeds and the stats file exists.
No specific checks on the stats are performed.
'''
import re
import os
from testlib import *

ok_exit_regex = re.compile(
r'Exiting @ tick \d+ because exiting with last active thread context'
)

path = joinpath(config.bin_path, 'test-progs', 'hello', 'bin', 'arm', 'linux')
filename = 'hello'
url = (config.resource_url + '/test-progs/hello/bin/arm/linux/hello')
test_program = DownloadedProgram(url, path, filename)

stdout_verifier = verifier.MatchRegex(ok_exit_regex)
h5_verifier = verifier.CheckH5StatsExist()
gem5_verify_config(
    name='hdf5_test',
    verifiers=[stdout_verifier, h5_verifier],
    fixtures=(test_program,),
    config=os.path.join(config.base_dir, 'configs', 'example','se.py'),
    config_args=['--cmd', joinpath(test_program.path, filename)],
    gem5_args=['--stats-file=h5://stats.h5'],
    valid_isas=(constants.arm_tag,)
)
