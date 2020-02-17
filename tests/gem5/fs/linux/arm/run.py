# Copyright (c) 2012, 2019-2020 ARM Limited
# All rights reserved
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
# Copyright (c) 2006-2007 The Regents of The University of Michigan
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

from __future__ import print_function

import sys
import os
import os.path
from os.path import join as joinpath

import m5

def run_test(root):
    """Default run_test implementations. Scripts can override it."""

    # instantiate configuration
    m5.instantiate()

    # simulate until program terminates
    exit_event = m5.simulate()
    print('Exiting @ tick', m5.curTick(), 'because', exit_event.getCause())

config = sys.argv[1]
os.environ['M5_PATH'] = sys.argv[2]

# path setup
gem5_root = joinpath(os.path.dirname(__file__), '..', '..', '..', '..', '..')
sys.path.append(joinpath(gem5_root, 'configs'))
tests_root = joinpath(gem5_root, 'tests')
sys.path.append(joinpath(tests_root, 'configs'))

exec(compile(open(config).read(), config, 'exec'))

system = root.system
system.readfile = os.path.join(tests_root, 'halt.sh')

# The CPU can either be a list of CPUs or a single object.
if isinstance(system.cpu, list):
    [ cpu.createThreads() for cpu in system.cpu ]
else:
    system.cpu.createThreads()

# Since we're in batch mode, don't allow tcp socket connections
m5.disableAllListeners()

run_test(root)
