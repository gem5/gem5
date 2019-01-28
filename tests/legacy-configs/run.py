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
New version of the run.py script. For this, all dependencies should be
handled outside of the script.

.. warning:: This script is NOT the recommended way to handle configurations
    for new tests. This exists for legacy support only. New Tests should
    either use configs from the normal gem5 configs or create their own for
    a test.
'''
import argparse
import sys
import os
from os.path import abspath, join as joinpath, dirname

import m5

# Add the normal gem5 config path to system path.
# This requirement should be removed if possible from all legacy scripts, but
# I've left it here for now.
sys.path.insert(0, abspath(joinpath(dirname(__file__), '../../configs')))

# set default maxtick... script can override
# -1 means run forever
maxtick = m5.MaxTick

def run_test(root):
    """Default run_test implementations. Scripts can override it."""

    # instantiate configuration
    m5.instantiate()

    # simulate until program terminates
    exit_event = m5.simulate(maxtick)
    print 'Exiting @ tick', m5.curTick(), 'because', exit_event.getCause()

test_progs = os.environ.get('M5_TEST_PROGS', '/dist/m5/regression/test-progs')

# Since we're in batch mode, dont allow tcp socket connections
m5.disableAllListeners()

parser = argparse.ArgumentParser()
parser.add_argument('--cmd',
                    action='store',
                    type=str,
                    help='Command to pass to the test system')
parser.add_argument('--executable',
                    action='store',
                    type=str,
                    help='Executable to pass to the test system')
parser.add_argument('--config',
                    action='append',
                    type=str,
                    help='A config file to initialize the system with.'\
                    + ' If more than one given, loads them in order given.')
args = parser.parse_args()

executable = args.executable

for config in args.config:
    exec(compile(open(config).read(), config, 'exec'))

# Initialize all CPUs in a system
def initCPUs(sys):
    def initCPU(cpu):
        # We might actually have a MemTest object or something similar
        # here that just pretends to be a CPU.
        try:
            cpu.createThreads()
        except:
            pass

    # The CPU attribute doesn't exist in some cases, e.g. the Ruby testers.
    if not hasattr(sys, "cpu"):
        return

    # The CPU can either be a list of CPUs or a single object.
    if isinstance(sys.cpu, list):
        [ initCPU(cpu) for cpu in sys.cpu ]
    else:
        initCPU(sys.cpu)

# TODO: Might want to automatically place the cmd and executable on the
# cpu[0].workload, although I think most legacy configs do this automatically
# or somewhere in their `test.py` config.


# We might be creating a single system or a dual system. Try
# initializing the CPUs in all known system attributes.
for sysattr in [ "system", "testsys", "drivesys" ]:
    if hasattr(root, sysattr):
        initCPUs(getattr(root, sysattr))

run_test(root)
