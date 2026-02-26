# Copyright (c) 2021 The Regents of the University of California.
# All Rights Reserved
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

from m5.objects import (
    SimpleExtLink,
    SimpleIntLink,
    SimpleNetwork,
    Switch,
)


class SimplePt2Pt(SimpleNetwork):
    """A simple point-to-point network. This doesn't not use garnet."""

    def __init__(self, ruby_system):
        super().__init__()
        self.netifs = []

        # TODO: These should be in a base class
        # https://gem5.atlassian.net/browse/GEM5-1039
        self.ruby_system = ruby_system

    def connectControllers(self, controllers):
        """Connect all of the controllers to routers and connec the routers
        together in a point-to-point network.
        """
        # Create one router/switch per controller in the system
        self.routers = [Switch(router_id=i) for i in range(len(controllers))]

        # Make a link from each controller to the router. The link goes
        # externally to the network.
        self.ext_links = [
            SimpleExtLink(link_id=i, ext_node=c, int_node=self.routers[i])
            for i, c in enumerate(controllers)
        ]

        # Make an "internal" link (internal to the network) between every pair
        # of routers.
        link_count = 0
        int_links = []
        for ri in self.routers:
            for rj in self.routers:
                if ri == rj:
                    continue  # Don't connect a router to itself!
                link_count += 1
                int_links.append(
                    SimpleIntLink(link_id=link_count, src_node=ri, dst_node=rj)
                )
        self.int_links = int_links


# keshav. we are designing this class to use garnet ntw and
# our desired topology
from m5.objects import (
    GarnetExtLink,
    GarnetIntLink,
    GarnetNetwork,
    GarnetNetworkInterface,
    GarnetRouter,
)


# class SimplePt2PtV2(GarnetNetwork):
class GarnetPt2Pt(GarnetNetwork):
    """A simple Pt2Pt network based on garnet"""

    def __init__(self, ruby_system):
        super().__init__()
        self.netifs = []
        self.ruby_system = ruby_system

    def connectControllers(self, controllers):

        # create one router/switch per controller in the system
        # self.routers = [GarnetRouter(vcs_per_vnet=16,virt_nets=5,width=16) for i in range(len(controllers))]
        self.routers = [
            GarnetRouter(
                router_id=i, latency=5, vcs_per_vnet=16, virt_nets=3, width=16
            )
            for i in range(len(controllers))
        ]

        # Make a link from each controller to the router. The link goes
        # externally to the router.
        link_count = 0
        self.ext_links = [
            GarnetExtLink(
                link_id=i, int_node=self.routers[i], ext_node=c, latency=1
            )
            for i, c in enumerate(controllers)
        ]
        self.netifs = [
            GarnetNetworkInterface(id=i)
            for (i, n) in enumerate(self.ext_links)
        ]

        # Make a iternal link, (internal to the network) btw router pairs
        link_count = len(controllers)
        int_links = []
        for ri in self.routers:
            for rj in self.routers:
                if ri == rj:
                    continue  # Don't connect a router to itself!
                link_count += 1
                int_links.append(
                    GarnetIntLink(
                        link_id=link_count, src_node=ri, dst_node=rj, latency=1
                    )
                )
        self.int_links = int_links


# Copyright (c) 2010 Advanced Micro Devices, Inc.
#               2016 Georgia Institute of Technology
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

#### old imports
# from common import FileSystemConfig
# from topologies.BaseTopology import SimpleTopology

# from m5.objects import *
# from m5.params import *
#### old import ends

# Creates a generic Mesh assuming an equal number of cache
# and directory controllers.
# XY routing is enforced (using link weights)
# to guarantee deadlock freedom.


# class Mesh_XY(SimpleTopology):
class Garnet_Mesh_XY(GarnetNetwork):
    # description = "Mesh_XY"
    def __init__(self, ruby_system):
        super().__init__()
        self.netifs = []
        self.ruby_system = ruby_system
        # self.mesh_rows=3;   self.link_latency=1;
        # self.router_latency=5;  self.vcs_per_vnet=16;
        # self.virt_nets=3;   self.width=16;

    # def __init__(self, controllers):
    #    self.nodes = controllers

    # Makes a generic mesh
    # assuming an equal number of cache and directory cntrls

    # def makeTopology(self, options, network, IntLink, ExtLink, Router):
    def connectControllers(self, controllers):
        nodes = controllers
        # nodes = self.nodes

        num_routers = min(len(controllers), pow(3, 2))
        # num_routers = min(len(controllers),pow(self.mesh_rows,2));
        # num_routers = options.num_cpus
        num_rows = 3
        # num_rows = self.mesh_rows;   #num_rows = options.mesh_rows

        # default values for link latency and router latency.
        # Can be over-ridden on a per link/router basis
        # link_latency = options.link_latency  # used by simple and garnet
        # link_latency = self.link_latency  # used by simple and garnet
        link_latency = 1  # used by simple and garnet
        # router_latency = options.router_latency  # only used by garnet
        # router_latency = self.router_latency  # only used by garnet
        router_latency = 5  # only used by garnet

        # There must be an evenly divisible number of cntrls to routers
        # Also, obviously the number or rows must be <= the number of routers
        cntrls_per_router, remainder = divmod(len(nodes), num_routers)
        assert num_rows > 0 and num_rows <= num_routers
        num_columns = int(num_routers / num_rows)
        assert num_columns * num_rows == num_routers

        # Create the routers in the mesh
        routers = [
            # Router(router_id=i, latency=router_latency)
            # GarnetRouter(router_id=i, latency=self.router_latency,
            GarnetRouter(
                router_id=i,
                latency=5,
                # vcs_per_vnet=self.vcs_per_vnet,virt_nets=self.virt_nets,width=self.width)
                vcs_per_vnet=16,
                virt_nets=3,
                width=16,
            )
            for i in range(num_routers)
        ]
        self.routers = routers
        # network.routers = routers

        # link counter to set unique link ids
        link_count = 0

        # Add all but the remainder nodes to the list of nodes to be uniformly
        # distributed across the network.
        network_nodes = []
        remainder_nodes = []
        for node_index in range(len(nodes)):
            if node_index < (len(nodes) - remainder):
                network_nodes.append(nodes[node_index])
            else:
                remainder_nodes.append(nodes[node_index])

        # Connect each node to the appropriate router
        ext_links = []
        for i, n in enumerate(network_nodes):
            cntrl_level, router_id = divmod(i, num_routers)
            assert cntrl_level < cntrls_per_router
            ext_links.append(
                # ExtLink(
                GarnetExtLink(
                    link_id=link_count,
                    ext_node=n,
                    int_node=routers[router_id],
                    latency=link_latency,
                )
            )
            link_count += 1

        # Connect the remainding nodes to router 0.  These should only be
        # DMA nodes.
        for i, node in enumerate(remainder_nodes):
            print(f"node.type:{node.type}")
            # assert node.type == "DMA_Controller"
            assert i < remainder
            ext_links.append(
                # ExtLink(
                GarnetExtLink(
                    link_id=link_count,
                    ext_node=node,
                    int_node=routers[0],
                    latency=link_latency,
                )
            )
            link_count += 1

        self.ext_links = ext_links
        # network.ext_links = ext_links

        # Create the mesh links.
        int_links = []

        # East output to West input links (weight = 1)
        for row in range(num_rows):
            for col in range(num_columns):
                if col + 1 < num_columns:
                    east_out = col + (row * num_columns)
                    west_in = (col + 1) + (row * num_columns)
                    int_links.append(
                        # IntLink(
                        GarnetIntLink(
                            link_id=link_count,
                            src_node=routers[east_out],
                            dst_node=routers[west_in],
                            src_outport="East",
                            dst_inport="West",
                            latency=link_latency,
                            weight=1,
                        )
                    )
                    link_count += 1

        # West output to East input links (weight = 1)
        for row in range(num_rows):
            for col in range(num_columns):
                if col + 1 < num_columns:
                    east_in = col + (row * num_columns)
                    west_out = (col + 1) + (row * num_columns)
                    int_links.append(
                        # IntLink(
                        GarnetIntLink(
                            link_id=link_count,
                            src_node=routers[west_out],
                            dst_node=routers[east_in],
                            src_outport="West",
                            dst_inport="East",
                            latency=link_latency,
                            weight=1,
                        )
                    )
                    link_count += 1

        # North output to South input links (weight = 2)
        for col in range(num_columns):
            for row in range(num_rows):
                if row + 1 < num_rows:
                    north_out = col + (row * num_columns)
                    south_in = col + ((row + 1) * num_columns)
                    int_links.append(
                        # IntLink(
                        GarnetIntLink(
                            link_id=link_count,
                            src_node=routers[north_out],
                            dst_node=routers[south_in],
                            src_outport="North",
                            dst_inport="South",
                            latency=link_latency,
                            weight=1,  # 2,#1,#2,
                        )
                    )
                    link_count += 1

        # South output to North input links (weight = 2)
        for col in range(num_columns):
            for row in range(num_rows):
                if row + 1 < num_rows:
                    north_in = col + (row * num_columns)
                    south_out = col + ((row + 1) * num_columns)
                    int_links.append(
                        # IntLink(
                        GarnetIntLink(
                            link_id=link_count,
                            src_node=routers[south_out],
                            dst_node=routers[north_in],
                            src_outport="South",
                            dst_inport="North",
                            latency=link_latency,
                            weight=1,  # 2,#1,#2,
                        )
                    )
                    link_count += 1

        self.int_links = int_links
        # network.int_links = int_links
        self.netifs = [
            GarnetNetworkInterface(id=i)
            for (i, n) in enumerate(self.ext_links)
        ]

    """
    # Register nodes with filesystem
    def registerTopology(self, options):
        for i in range(options.num_cpus):
            FileSystemConfig.register_node(
                [i], MemorySize(options.mem_size) // options.num_cpus, i
            )
    """
