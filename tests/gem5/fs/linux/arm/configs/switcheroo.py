# Copyright (c) 2012 ARM Limited
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

import m5
from m5.objects import *

import _m5

m5.util.addToPath("../configs/")
from base_caches import *


class Sequential:
    """Sequential CPU switcher.

    The sequential CPU switches between all CPUs in a system in
    order. The CPUs in the system must have been prepared for
    switching, which in practice means that only one CPU is switched
    in. base_config.BaseFSSwitcheroo can be used to create such a
    system.
    """

    def __init__(self, cpus):
        self.first_cpu = None
        for cpuno, cpu in enumerate(cpus):
            if not cpu.switched_out:
                if self.first_cpu != None:
                    fatal("More than one CPU is switched in")
                self.first_cpu = cpuno

        if self.first_cpu == None:
            fatal("The system contains no switched in CPUs")

        self.cur_cpu = self.first_cpu
        self.cpus = cpus

    def next(self):
        self.cur_cpu = (self.cur_cpu + 1) % len(self.cpus)
        return self.cpus[self.cur_cpu]

    def first(self):
        return self.cpus[self.first_cpu]


def run_test(root, switcher=None, freq=1000, verbose=False):
    """Test runner for CPU switcheroo tests.

    The switcheroo test runner is used to switch CPUs in a system that
    has been prepared for CPU switching. Such systems should have
    multiple CPUs when they are instantiated, but only one should be
    switched in. Such configurations can be created using the
    base_config.BaseFSSwitcheroo class.

    A CPU switcher object is used to control switching. The default
    switcher sequentially switches between all CPUs in a system,
    starting with the CPU that is currently switched in.

    Unlike most other test runners, this one automatically configures
    the memory mode of the system based on the first CPU the switcher
    reports.

    Keyword Arguments:
      switcher -- CPU switcher implementation. See Sequential for
                  an example implementation.
      period -- Switching frequency in Hz.
      verbose -- Enable output at each switch (suppressed by default).
    """

    if switcher == None:
        switcher = Sequential(root.system.cpu)

    current_cpu = switcher.first()
    system = root.system
    system.mem_mode = type(current_cpu).memory_mode()

    # Suppress "Entering event queue" messages since we get tons of them.
    # Worse yet, they include the timestamp, which makes them highly
    # variable and unsuitable for comparing as test outputs.
    if not verbose:
        _m5.core.setLogLevel(_m5.core.LogLevel.WARN)

    # instantiate configuration
    m5.instantiate()

    # Determine the switching period, this has to be done after
    # instantiating the system since the time base must be fixed.
    period = m5.ticks.fromSeconds(1.0 / freq)
    while True:
        exit_event = m5.simulate(period)
        exit_cause = exit_event.getCause()

        if exit_cause == "simulate() limit reached":
            next_cpu = switcher.next()

            if verbose:
                print("Switching CPUs...")
                print(f"Next CPU: {type(next_cpu)}")
            m5.drain()
            if current_cpu != next_cpu:
                m5.switchCpus(
                    system, [(current_cpu, next_cpu)], verbose=verbose
                )
            else:
                print(
                    "Source CPU and destination CPU are the same,"
                    " skipping..."
                )
            current_cpu = next_cpu
        elif (
            exit_cause == "target called exit()"
            or exit_cause == "m5_exit instruction encountered"
        ):
            sys.exit(0)
        else:
            print(f"Test failed: Unknown exit cause: {exit_cause}")
            sys.exit(1)
