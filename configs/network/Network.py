# Copyright (c) 2016 Georgia Institute of Technology
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
# Authors: Tushar Krishna

import math
import m5
from m5.objects import *
from m5.defines import buildEnv
from m5.util import addToPath, fatal

def define_options(parser):
    # By default, ruby uses the simple timing cpu
    parser.set_defaults(cpu_type="timing")

    parser.add_option("--topology", type="string", default="Crossbar",
                      help="check configs/topologies for complete set")
    parser.add_option("--mesh-rows", type="int", default=0,
                      help="the number of rows in the mesh topology")
    parser.add_option("--network", type="choice", default="simple",
                      choices=['simple', 'garnet2.0'],
                      help="'simple'|'garnet2.0'")
    parser.add_option("--router-latency", action="store", type="int",
                      default=1,
                      help="""number of pipeline stages in the garnet router.
                            Has to be >= 1.
                            Can be over-ridden on a per router basis
                            in the topology file.""")
    parser.add_option("--link-latency", action="store", type="int", default=1,
                      help="""latency of each link the simple/garnet networks.
                            Has to be >= 1.
                            Can be over-ridden on a per link basis
                            in the topology file.""")
    parser.add_option("--link-width-bits", action="store", type="int",
                      default=128,
                      help="width in bits for all links inside garnet.")
    parser.add_option("--vcs-per-vnet", action="store", type="int", default=4,
                      help="""number of virtual channels per virtual network
                            inside garnet network.""")
    parser.add_option("--routing-algorithm", action="store", type="int",
                      default=0,
                      help="""routing algorithm in network.
                            0: weight-based table
                            1: XY (for Mesh. see garnet2.0/RoutingUnit.cc)
                            2: Custom (see garnet2.0/RoutingUnit.cc""")
    parser.add_option("--network-fault-model", action="store_true",
                      default=False,
                      help="""enable network fault model:
                            see src/mem/ruby/network/fault_model/""")


def create_network(options, ruby):

    # Set the network classes based on the command line options
    if options.network == "garnet2.0":
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

    # Instantiate the network object
    # so that the controllers can connect to it.
    network = NetworkClass(ruby_system = ruby, topology = options.topology,
            routers = [], ext_links = [], int_links = [], netifs = [])

    return (network, IntLinkClass, ExtLinkClass, RouterClass, InterfaceClass)

def init_network(options, network, InterfaceClass):

    if options.network == "garnet2.0":
        network.num_rows = options.mesh_rows
        network.vcs_per_vnet = options.vcs_per_vnet
        network.ni_flit_size = options.link_width_bits / 8
        network.routing_algorithm = options.routing_algorithm

    if options.network == "simple":
        network.setup_buffers()

    if InterfaceClass != None:
        netifs = [InterfaceClass(id=i) \
                  for (i,n) in enumerate(network.ext_links)]
        network.netifs = netifs

    if options.network_fault_model:
        assert(options.network == "garnet2.0")
        network.enable_fault_model = True
        network.fault_model = FaultModel()
