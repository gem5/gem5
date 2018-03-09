# Copyright (c) 2012 ARM Limited
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
#
# Authors: Steve Reinhardt

from __future__ import print_function

import os
import sys
import re
import string

from os.path import join as joinpath
import os.path
import os

import m5

def skip_test(reason=""):
    """Signal that a test should be skipped and optionally print why.

    Keyword arguments:
      reason -- Reason why the test failed. Output is omitted if empty.
    """

    if reason:
        print("Skipping test: %s" % reason)
    sys.exit(2)

def has_sim_object(name):
    """Test if a SimObject exists in the simulator.

    Arguments:
      name -- Name of SimObject (string)

    Returns: True if the object exists, False otherwise.
    """

    try:
        cls = getattr(m5.objects, name)
        return issubclass(cls, m5.objects.SimObject)
    except AttributeError:
        return False

def require_sim_object(name, fatal=False):
    """Test if a SimObject exists and abort/skip test if not.

    Arguments:
      name -- Name of SimObject (string)

    Keyword arguments:
      fatal -- Set to True to indicate that the test should fail
               instead of being skipped.
    """

    if has_sim_object(name):
        return
    else:
        msg = "Test requires the '%s' SimObject." % name
        if fatal:
            m5.fatal(msg)
        else:
            skip_test(msg)


def require_file(path, fatal=False, mode=os.F_OK):
    """Test if a file exists and abort/skip test if not.

    Arguments:
      path -- File to test for.

    Keyword arguments:
      fatal -- Set to True to indicate that the test should fail
               instead of being skipped.
      modes -- Mode to test for, default to existence. See the
               Python documentation for os.access().
    """

    if os.access(path, mode):
        return
    else:
        msg = "Test requires '%s'" % path
        if not os.path.exists(path):
            msg += " which does not exist."
        else:
            msg += " which has incorrect permissions."

        if fatal:
            m5.fatal(msg)
        else:
            skip_test(msg)

def require_kvm(kvm_dev="/dev/kvm", fatal=False):
    """Test if KVM is available.

    Keyword arguments:
      kvm_dev -- Device to test (normally /dev/kvm)
      fatal -- Set to True to indicate that the test should fail
               instead of being skipped.
    """

    require_sim_object("BaseKvmCPU", fatal=fatal)
    require_file(kvm_dev, fatal=fatal, mode=os.R_OK | os.W_OK)

def run_test(root):
    """Default run_test implementations. Scripts can override it."""

    # instantiate configuration
    m5.instantiate()

    # simulate until program terminates
    exit_event = m5.simulate(maxtick)
    print('Exiting @ tick', m5.curTick(), 'because', exit_event.getCause())

# Since we're in batch mode, dont allow tcp socket connections
m5.disableAllListeners()

# single "path" arg encodes everything we need to know about test
(category, mode, name, isa, opsys, config) = sys.argv[1].split('/')[-6:]

# find path to directory containing this file
tests_root = os.path.dirname(__file__)
test_progs = os.environ.get('M5_TEST_PROGS', '/dist/m5/regression/test-progs')
if not os.path.isdir(test_progs):
    test_progs = joinpath(tests_root, 'test-progs')

# generate path to binary file
def binpath(app, file=None):
    # executable has same name as app unless specified otherwise
    if not file:
        file = app
    return joinpath(test_progs, app, 'bin', isa, opsys, file)

# generate path to input file
def inputpath(app, file=None):
    # input file has same name as app unless specified otherwise
    if not file:
        file = app
    return joinpath(test_progs, app, 'input', file)

def srcpath(path):
    """Path to file in gem5's source tree"""
    return joinpath(os.path.dirname(__file__), "..", path)

# build configuration
sys.path.append(joinpath(tests_root, 'configs'))
test_filename = config
# for ruby configurations, remove the protocol name from the test filename
if re.search('-ruby', test_filename):
    test_filename = test_filename.split('-ruby')[0]+'-ruby'
execfile(joinpath(tests_root, 'configs', test_filename + '.py'))

# set default maxtick... script can override
# -1 means run forever
maxtick = m5.MaxTick

# tweak configuration for specific test
sys.path.append(joinpath(tests_root, category, mode, name))
execfile(joinpath(tests_root, category, mode, name, 'test.py'))

# Initialize all CPUs in a system
def initCPUs(sys):
    def initCPU(cpu):
        # We might actually have a MemTest object or something similar
        # here that just pretends to be a CPU.
        try:
            cpu.createThreads()
        except:
            pass

    # The CPU attribute doesn't exist in some cases, e.g. the Ruby
    # testers.
    if not hasattr(sys, "cpu"):
        return

    # The CPU can either be a list of CPUs or a single object.
    if isinstance(sys.cpu, list):
        [ initCPU(cpu) for cpu in sys.cpu ]
    else:
        initCPU(sys.cpu)

# We might be creating a single system or a dual system. Try
# initializing the CPUs in all known system attributes.
for sysattr in [ "system", "testsys", "drivesys" ]:
    if hasattr(root, sysattr):
        initCPUs(getattr(root, sysattr))

run_test(root)
