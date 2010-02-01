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
# Authors: Steve Reinhardt
#          Brad Beckmann

from m5.params import *
from m5.SimObject import SimObject

class Link(SimObject):
    type = 'Link'
    latency = Param.Int(1, "")
    bw_multiplier = Param.Int("")
    weight = Param.Int(1, "")

class ExtLink(Link):
    type = 'ExtLink'
    ext_node = Param.RubyController("External node")
    int_node = Param.Int("ID of internal node")
    bw_multiplier = 64

class IntLink(Link):
    type = 'IntLink'
    node_a = Param.Int("ID of internal node on one end")
    node_b = Param.Int("ID of internal node on other end")
    bw_multiplier = 16

class Topology(SimObject):
    type = 'Topology'
    ext_links = VectorParam.ExtLink("Links to external nodes")
    int_links = VectorParam.IntLink("Links between internal nodes")
    num_int_nodes = Param.Int("Nunber of internal nodes")
    print_config = Param.Bool(False,
        "display topology config in the stats file")

def makeCrossbar(nodes):
    ext_links = [ExtLink(ext_node=n, int_node=i)
                 for (i, n) in enumerate(nodes)]
    xbar = len(nodes) # node ID for crossbar switch
    int_links = [IntLink(node_a=i, node_b=xbar) for i in range(len(nodes))]    
    return Topology(ext_links=ext_links, int_links=int_links,
                    num_int_nodes=len(nodes)+1)

def makeMesh(nodes, num_routers, num_rows):
    #
    # There must be an evenly divisible number of cntrls to routers
    # Also, obviously the number or rows must be <= the number of routers
    #
    cntrls_per_router, remainder = divmod(len(nodes), num_routers)
    assert(num_rows <= num_routers)
    num_columns = int(num_routers / num_rows)
    assert(num_columns * num_rows == num_routers)

    #
    # Add all but the remainder nodes to the list of nodes to be uniformly
    # distributed across the network.
    #
    network_nodes = []
    remainder_nodes = []
    for node_index in xrange(len(nodes)):
        if node_index < (len(nodes) - remainder):
            network_nodes.append(nodes[node_index])
        else:
            remainder_nodes.append(nodes[node_index])

    #
    # Connect each node to the appropriate router
    #
    ext_links = []
    for (i, n) in enumerate(network_nodes):
        cntrl_level, router_id = divmod(i, num_routers)
        assert(cntrl_level < cntrls_per_router)
        ext_links.append(ExtLink(ext_node=n, int_node=router_id))

    #
    # Connect the remainding nodes to router 0.  These should only be DMA nodes.
    #
    for (i, node) in enumerate(remainder_nodes):
        assert(node.type == 'DMA_Controller')
        assert(i < remainder)
        ext_links.append(ExtLink(ext_node=node, int_node=0))
    
    #
    # Create the mesh links.  First row (east-west) links then column
    # (north-south) links
    #
    int_links = []
    for row in xrange(num_rows):
        for col in xrange(num_columns):
            if (col + 1 < num_columns):
                east_id = col + (row * num_columns)
                west_id = (col + 1) + (row * num_columns)
                int_links.append(IntLink(node_a=east_id,
                                         node_b=west_id,
                                         weight=1))
    for col in xrange(num_columns):
        for row in xrange(num_rows):
            if (row + 1 < num_rows):
                north_id = col + (row * num_columns)
                south_id = col + ((row + 1) * num_columns)
                int_links.append(IntLink(node_a=north_id,
                                         node_b=south_id,
                                         weight=2))

    return Topology(ext_links=ext_links,
                    int_links=int_links,
                    num_int_nodes=num_routers)

class RubyNetwork(SimObject):
    type = 'RubyNetwork'
    cxx_class = 'Network'
    abstract = True
    number_of_virtual_networks = Param.Int(10, "");
    topology = Param.Topology("");
    buffer_size = Param.Int(0,
        "default buffer size; 0 indicates infinite buffering");
    endpoint_bandwidth = Param.Int(10000, "");
    adaptive_routing = Param.Bool(True, "");
    link_latency = Param.Int(1,
        "local memory latency ?? NetworkLinkLatency");
    control_msg_size = Param.Int(8, "");
