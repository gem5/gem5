# Copyright (c) 2017 Jason Power
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

""" This file creates a set of Ruby caches, the Ruby network, and a simple
point-to-point topology for the RubyRandomTester to use.
See Part 3 in the Learning gem5 book:
http://gem5.org/documentation/learning_gem5/part3/MSIintro

IMPORTANT: If you modify this file, it's likely that the Learning gem5 book
           also needs to be updated. For now, email Jason <jason@lowepower.com>

"""

from m5.defines import buildEnv
from m5.util import fatal

from m5.objects import *

from msi_caches import L1Cache, DirController, MyNetwork


class TestCacheSystem(RubySystem):
    def __init__(self):
        if buildEnv["PROTOCOL"] != "MSI":
            fatal("This system assumes MSI from learning gem5!")

        super().__init__()

    def setup(self, system, tester, mem_ctrls):
        """Set up the Ruby cache subsystem. Note: This can't be done in the
        constructor because many of these items require a pointer to the
        ruby system (self). This causes infinite recursion in initialize()
        if we do this in the __init__.
        Setting up for running the RubyRandomTester is a little different
        than when we're using CPUs.
        """
        num_testers = tester.num_cpus

        # Ruby's global network.
        self.network = MyNetwork(self)

        # MSI uses 3 virtual networks
        self.number_of_virtual_networks = 3
        self.network.number_of_virtual_networks = 3

        self.controllers = [
            L1Cache(system, self, self) for i in range(num_testers)
        ] + [DirController(self, system.mem_ranges, mem_ctrls)]

        self.sequencers = [
            RubySequencer(
                version=i,
                # I/D cache is combined and grab from ctrl
                dcache=self.controllers[i].cacheMemory,
                clk_domain=self.clk_domain,
            )
            for i in range(num_testers)
        ]

        for i, c in enumerate(self.controllers[0 : len(self.sequencers)]):
            c.sequencer = self.sequencers[i]

        self.num_of_sequencers = len(self.sequencers)

        # Create the network and connect the controllers.
        # NOTE: This is quite different if using Garnet!
        self.network.connectControllers(self.controllers)
        self.network.setup_buffers()

        # Set up a proxy port for the system_port. Used for load binaries and
        # other functional-only things.
        self.sys_port_proxy = RubyPortProxy()
        system.system_port = self.sys_port_proxy.in_ports

        # Connect up the sequencers to the random tester
        for seq in self.sequencers:
            if seq.support_data_reqs and seq.support_inst_reqs:
                tester.cpuInstDataPort = seq.in_ports
            elif seq.support_data_reqs:
                tester.cpuDataPort = seq.in_ports
            elif seq.support_inst_reqs:
                tester.cpuInstDataPort = seq.in_ports

            # Do not automatically retry stalled Ruby requests
            seq.no_retry_on_stall = True

            # Tell each sequencer this is the ruby tester so that it
            # copies the subblock back to the checker
            seq.using_ruby_tester = True
