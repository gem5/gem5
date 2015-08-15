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
#
# Authors: Brad Beckmann

import math
import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath, fatal

import MemConfig
addToPath('../topologies')

def define_options(parser):
    # By default, ruby uses the simple timing cpu
    parser.set_defaults(cpu_type="timing")

    parser.add_option("--ruby-clock", action="store", type="string",
                      default='2GHz',
                      help="Clock for blocks running at Ruby system's speed")

    parser.add_option("--access-backing-store", action="store_true", default=False,
                      help="Should ruby maintain a second copy of memory")

    # Options related to cache structure
    parser.add_option("--ports", action="store", type="int", default=4,
                      help="used of transitions per cycle which is a proxy \
                            for the number of ports.")

    # ruby network options
    parser.add_option("--topology", type="string", default="Crossbar",
                      help="check configs/topologies for complete set")
    parser.add_option("--mesh-rows", type="int", default=1,
                      help="the number of rows in the mesh topology")
    parser.add_option("--garnet-network", type="choice",
                      choices=['fixed', 'flexible'], help="'fixed'|'flexible'")
    parser.add_option("--network-fault-model", action="store_true", default=False,
                      help="enable network fault model: see src/mem/ruby/network/fault_model/")

    # ruby mapping options
    parser.add_option("--numa-high-bit", type="int", default=0,
                      help="high order address bit to use for numa mapping. " \
                           "0 = highest bit, not specified = lowest bit")

    parser.add_option("--recycle-latency", type="int", default=10,
                      help="Recycle latency for ruby controller input buffers")

    protocol = buildEnv['PROTOCOL']
    exec "import %s" % protocol
    eval("%s.define_options(parser)" % protocol)

def setup_memory_controllers(system, ruby, dir_cntrls, options):
    ruby.block_size_bytes = options.cacheline_size
    ruby.memory_size_bits = 48
    block_size_bits = int(math.log(options.cacheline_size, 2))

    if options.numa_high_bit:
        numa_bit = options.numa_high_bit
    else:
        # if the numa_bit is not specified, set the directory bits as the
        # lowest bits above the block offset bits, and the numa_bit as the
        # highest of those directory bits
        dir_bits = int(math.log(options.num_dirs, 2))
        numa_bit = block_size_bits + dir_bits - 1

    index = 0
    mem_ctrls = []
    crossbars = []

    # Sets bits to be used for interleaving.  Creates memory controllers
    # attached to a directory controller.  A separate controller is created
    # for each address range as the abstract memory can handle only one
    # contiguous address range as of now.
    for dir_cntrl in dir_cntrls:
        dir_cntrl.directory.numa_high_bit = numa_bit

        crossbar = None
        if len(system.mem_ranges) > 1:
            crossbar = IOXBar()
            crossbars.append(crossbar)
            dir_cntrl.memory = crossbar.slave

        for r in system.mem_ranges:
            mem_ctrl = MemConfig.create_mem_ctrl(
                MemConfig.get(options.mem_type), r, index, options.num_dirs,
                int(math.log(options.num_dirs, 2)), options.cacheline_size)

            mem_ctrls.append(mem_ctrl)

            if crossbar != None:
                mem_ctrl.port = crossbar.master
            else:
                mem_ctrl.port = dir_cntrl.memory

        index += 1

    system.mem_ctrls = mem_ctrls

    if len(crossbars) > 0:
        ruby.crossbars = crossbars


def create_topology(controllers, options):
    """ Called from create_system in configs/ruby/<protocol>.py
        Must return an object which is a subclass of BaseTopology
        found in configs/topologies/BaseTopology.py
        This is a wrapper for the legacy topologies.
    """
    exec "import %s as Topo" % options.topology
    topology = eval("Topo.%s(controllers)" % options.topology)
    return topology

def create_system(options, full_system, system, piobus = None, dma_ports = []):

    system.ruby = RubySystem()
    ruby = system.ruby

    # Set the network classes based on the command line options
    if options.garnet_network == "fixed":
        NetworkClass = GarnetNetwork_d
        IntLinkClass = GarnetIntLink_d
        ExtLinkClass = GarnetExtLink_d
        RouterClass = GarnetRouter_d
        InterfaceClass = GarnetNetworkInterface_d

    elif options.garnet_network == "flexible":
        NetworkClass = GarnetNetwork
        IntLinkClass = GarnetIntLink
        ExtLinkClass = GarnetExtLink
        RouterClass = GarnetRouter
        InterfaceClass = GarnetNetworkInterface

    else:
        NetworkClass = SimpleNetwork
        IntLinkClass = SimpleIntLink
        ExtLinkClass = SimpleExtLink
        RouterClass = Switch
        InterfaceClass = None

    # Instantiate the network object so that the controllers can connect to it.
    network = NetworkClass(ruby_system = ruby, topology = options.topology,
            routers = [], ext_links = [], int_links = [], netifs = [])
    ruby.network = network

    protocol = buildEnv['PROTOCOL']
    exec "import %s" % protocol
    try:
        (cpu_sequencers, dir_cntrls, topology) = \
             eval("%s.create_system(options, full_system, system, dma_ports,\
                                    ruby)"
                  % protocol)
    except:
        print "Error: could not create sytem for ruby protocol %s" % protocol
        raise

    # Create a port proxy for connecting the system port. This is
    # independent of the protocol and kept in the protocol-agnostic
    # part (i.e. here).
    sys_port_proxy = RubyPortProxy(ruby_system = ruby)

    # Give the system port proxy a SimObject parent without creating a
    # full-fledged controller
    system.sys_port_proxy = sys_port_proxy

    # Connect the system port for loading of binaries etc
    system.system_port = system.sys_port_proxy.slave

    # Create the network topology
    topology.makeTopology(options, network, IntLinkClass, ExtLinkClass,
            RouterClass)

    if options.garnet_network is None:
        assert(NetworkClass == SimpleNetwork)
        assert(RouterClass == Switch)
        network.setup_buffers()

    if InterfaceClass != None:
        netifs = [InterfaceClass(id=i) for (i,n) in enumerate(network.ext_links)]
        network.netifs = netifs

    if options.network_fault_model:
        assert(options.garnet_network == "fixed")
        network.enable_fault_model = True
        network.fault_model = FaultModel()

    setup_memory_controllers(system, ruby, dir_cntrls, options)

    # Connect the cpu sequencers and the piobus
    if piobus != None:
        for cpu_seq in cpu_sequencers:
            cpu_seq.pio_master_port = piobus.slave
            cpu_seq.mem_master_port = piobus.slave

            if buildEnv['TARGET_ISA'] == "x86":
                cpu_seq.pio_slave_port = piobus.master

    ruby.number_of_virtual_networks = ruby.network.number_of_virtual_networks
    ruby._cpu_ports = cpu_sequencers
    ruby.num_of_sequencers = len(cpu_sequencers)

    # Create a backing copy of physical memory in case required
    if options.access_backing_store:
        ruby.access_backing_store = True
        ruby.phys_mem = SimpleMemory(range=system.mem_ranges[0],
                                     in_addr_map=False)

def send_evicts(options):
    # currently, 2 scenarios warrant forwarding evictions to the CPU:
    # 1. The O3 model must keep the LSQ coherent with the caches
    # 2. The x86 mwait instruction is built on top of coherence invalidations
    if options.cpu_type == "detailed" or buildEnv['TARGET_ISA'] == 'x86':
        return True
    return False
