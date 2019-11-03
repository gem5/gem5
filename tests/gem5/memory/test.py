# Copyright (c) 2018 The Regents of the University of California.
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
# Authors: Jason Lowe-Power

'''
Test file for simple memory test
TODO: Add stats checking
'''
from testlib import *

gem5_verify_config(
    name='simple_mem_default',
    verifiers=(), # No need for verfiers this will return non-zero on fail
    config=joinpath(getcwd(), 'simple-run.py'),
    config_args = [],
    valid_isas=(constants.null_tag,),
)

simple_mem_params = [
        ('inf-bandwidth', {'bandwidth': '0GB/s'}),
        ('low-latency', {'latency': '1ns'}),
        ('high-latency', {'latency': '1us'}),
        ('low-bandwidth', {'bandwidth': '1MB/s'}),
        ('high-var', {'latency_var': '100ns'})
        ]


for name, params in simple_mem_params:
    args = ['--' + key + '=' + val for key,val in params.iteritems()]

    gem5_verify_config(
        name='simple_mem_' + name,
        verifiers=(), # No need for verfiers this will return non-zero on fail
        config=joinpath(getcwd(), 'simple-run.py'),
        config_args = args,
        valid_isas=(constants.null_tag,),
        ) # This tests for validity as well as performance

gem5_verify_config(
    name='memtest',
    verifiers=(), # No need for verfiers this will return non-zero on fail
    config=joinpath(getcwd(), 'memtest-run.py'),
    config_args = [],
    valid_isas=(constants.null_tag,),
)
