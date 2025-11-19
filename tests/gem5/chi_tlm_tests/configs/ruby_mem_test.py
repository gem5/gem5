# Copyright (c) 2024 Arm Limited
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
# Copyright (c) 2006-2007 The Regents of The University of Michigan
# Copyright (c) 2009 Advanced Micro Devices, Inc.
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

import argparse
import importlib
import os
import sys

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.tlm_chi import *
from m5.util import addToPath

m5.util.addToPath(os.path.join(m5.util.repoPath(), "configs"))
from common import Options
from ruby import (
    CHI,
    Ruby,
)
from ruby.CHI_config import (
    CHI_MN,
    CHI_Node,
    Versions,
)


class TLM_RNF(CHI_Node):
    # The CHI controller can be a child of this object or another if
    # 'parent' is specified
    def __init__(self, ruby_system, parent):
        super().__init__(ruby_system)

        self._cntrl = TlmController(
            version=Versions.getVersion(CHI_Cache_Controller),
            ruby_system=ruby_system,
        )

        parent.chi_controller = self._cntrl
        self.connectController(self._cntrl)

    def getSequencers(self):
        return []

    def getAllControllers(self):
        return [self._cntrl]

    def getNetworkSideControllers(self):
        return [self._cntrl]


def rnf_gen(options, ruby_system, cpus):
    return [TLM_RNF(ruby_system, cpu) for cpu in system.cpu]


def mn_gen(options, ruby_system, cpus):
    all_rnf_cntrls = []
    for rnf in ruby_system.rnf:
        all_rnf_cntrls.extend(rnf.getAllControllers())
    return [CHI_MN(ruby_system, all_rnf_cntrls)]


def suite_importer(file_path):
    """
    Used to import the suite file.
    :param file_path: Path of the file to import
    """
    module_name = os.path.basename(file_path)
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def create_system(options, system):
    system.ruby = RubySystem()

    # Instantiate the network object
    # so that the controllers can connect to it.
    system.ruby.network = SimpleNetwork(
        ruby_system=system.ruby,
        topology=options.topology,
        routers=[],
        ext_links=[],
        int_links=[],
        netifs=[],
    )

    bootmem = None
    dma_ports = []
    (cpu_sequencers, dir_cntrls, topology) = CHI.create_system(
        options, False, system, dma_ports, bootmem, system.ruby, system.cpu
    )

    # Create the network topology
    topology.makeTopology(
        options, system.ruby.network, SimpleIntLink, SimpleExtLink, Switch
    )

    system.ruby.network.setup_buffers()

    # Create a port proxy for connecting the system port. This is
    # independent of the protocol and kept in the protocol-agnostic
    # part (i.e. here).
    # Give the system port proxy a SimObject parent without creating a
    # full-fledged controller
    system.sys_port_proxy = RubyPortProxy(ruby_system=system.ruby)

    # Connect the system port for loading of binaries etc
    system.system_port = system.sys_port_proxy.in_ports

    Ruby.setup_memory_controllers(system, system.ruby, dir_cntrls, options)

    system.ruby.number_of_virtual_networks = (
        system.ruby.network.number_of_virtual_networks
    )
    system.ruby._cpu_ports = cpu_sequencers
    system.ruby.num_of_sequencers = len(cpu_sequencers)


parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument(
    "suite",
    type=str,
    help="Path to the suite file",
)
Options.addNoISAOptions(parser)

#
# Add the ruby specific and protocol specific options
#
Ruby.define_options(parser)

args = parser.parse_args()

#
# Configuring 4GiBs of SimpleMemory
#
args.mem_type = "SimpleMemory"
args.mem_size = "4GiB"

#
# Currently ruby does not support atomic or uncacheable accesses
#
cpus = [TlmGenerator(cpu_id=i) for i in range(args.num_cpus)]

for cpu in cpus:
    suite = suite_importer(args.suite)
    suite.test_all(cpu)

system = System(
    cpu=cpus,
    clk_domain=SrcClockDomain(clock=args.sys_clock),
    mem_ranges=[AddrRange(args.mem_size)],
)
m5.util.addToPath("../common")

# Hooking up the RN-F generation callback
system._rnf_gen = rnf_gen
system._mn_gen = mn_gen

create_system(args, system)

# Create a top-level voltage domain and clock domain
system.voltage_domain = VoltageDomain(voltage=args.sys_voltage)
system.clk_domain = SrcClockDomain(
    clock=args.sys_clock, voltage_domain=system.voltage_domain
)
# Create a seperate clock domain for Ruby
system.ruby.clk_domain = SrcClockDomain(
    clock=args.ruby_clock, voltage_domain=system.voltage_domain
)

# To make unit-tests reproducible, we disable randomization
system.ruby.randomization = False

# -----------------------
# run simulation
# -----------------------

root = Root(full_system=False, system=system)
root.system.mem_mode = "timing"

# Not much point in this being higher than the L1 latency
m5.ticks.setGlobalFrequency("1ns")

# instantiate configuration
m5.instantiate()

# simulate until program terminates
exit_event = m5.simulate(args.abs_max_tick)

print("Exiting @ tick", m5.curTick(), "because", exit_event.getCause())
