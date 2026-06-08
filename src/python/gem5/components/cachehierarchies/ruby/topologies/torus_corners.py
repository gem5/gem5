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

import math

from m5.objects import (
    GarnetExtLink,
    GarnetIntLink,
    GarnetNetwork,
    GarnetRouter,
)


class TorusCorners(GarnetNetwork):
    """torus with directory on corners using garnet."""

    def __init__(self, ruby_system, lookup):
        super().__init__()
        self.netifs = []
        self.ruby_system = ruby_system
        self.ni_flit_size = 16
        self.vcs_per_vnet = 4
        self.routing_algorithm = 0
        self.lookup_table = lookup

    def connectControllers(self, controllers):
        """Connect all of the controllers to routers and connect the routers
        together in a mesh network.
        """
        link_latency = 1
        l1_nodes = []
        l2_nodes = []  # length 4
        dir_nodes = []  # length 4
        dma_nodes = []
        # Create one router/switch per l1 controller in the system
        dma_count = 0
        for controller in controllers:
            if controller.type == "DMA_Controller":
                dma_count += 1
        num_routers = int(len(controllers) - 8 - dma_count)
        for node_index in range(len(controllers)):
            if node_index < num_routers:
                l1_nodes.append(controllers[node_index])
            elif node_index < num_routers + 4:
                l2_nodes.append(controllers[node_index])
            elif node_index < num_routers + 8:
                dir_nodes.append(controllers[node_index])
            else:
                dma_nodes.append(controllers[node_index])

        num_rows = int(math.ceil(math.sqrt(num_routers)))
        self.num_rows = num_rows
        self.routers = [
            GarnetRouter(router_id=n, latency=1) for n in range(num_routers)
        ]

        link_count = 0
        # Connect each node to the appropriate router
        ext_links = []
        for i, n in enumerate(l1_nodes):
            cntrl_level, router_id = divmod(i, num_routers)
            ext_links.append(
                GarnetExtLink(
                    link_id=link_count,
                    ext_node=n,
                    int_node=self.routers[router_id],
                    latency=link_latency,
                )
            )
            link_count += 1

        # Connect the dir and l2 nodes to the corners.
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=dir_nodes[0],
                int_node=self.routers[0],
                latency=link_latency,
            )
        )
        link_count += 1
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=dir_nodes[1],
                int_node=self.routers[num_rows - 1],
                latency=link_latency,
            )
        )
        link_count += 1
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=dir_nodes[2],
                int_node=self.routers[num_routers - num_rows],
                latency=link_latency,
            )
        )
        link_count += 1
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=dir_nodes[3],
                int_node=self.routers[num_routers - 1],
                latency=link_latency,
            )
        )
        link_count += 1
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=l2_nodes[0],
                int_node=self.routers[0],
                latency=link_latency,
            )
        )
        link_count += 1
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=l2_nodes[1],
                int_node=self.routers[num_rows - 1],
                latency=link_latency,
            )
        )
        link_count += 1
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=l2_nodes[2],
                int_node=self.routers[num_routers - num_rows],
                latency=link_latency,
            )
        )
        link_count += 1
        ext_links.append(
            GarnetExtLink(
                link_id=link_count,
                ext_node=l2_nodes[3],
                int_node=self.routers[num_routers - 1],
                latency=link_latency,
            )
        )
        link_count += 1

        # Connect the remainding nodes to router 0.  These should only be
        # DMA nodes.in case of garnet standalone we want to remove assert
        for i, node in enumerate(dma_nodes):
            # assert node.type == "DMA_Controller"
            ext_links.append(
                GarnetExtLink(
                    link_id=link_count,
                    ext_node=node,
                    int_node=self.routers[0],
                    latency=link_latency,
                )
            )
            link_count += 1

        self.ext_links = ext_links

        # Make an "internal" link (internal to the network) between every pair
        # of routers.
        link_count = 0
        int_links = []

        # East output to West input links (weight = 1)
        for row in range(num_rows):
            for col in range(num_rows):
                east_out = col + (row * num_rows)
                west_in = ((col + 1) % num_rows) + (row * num_rows)
                int_links.append(
                    GarnetIntLink(
                        link_id=link_count,
                        src_node=self.routers[east_out],
                        dst_node=self.routers[west_in],
                        src_outport=west_in,
                        dst_inport=east_out,
                        latency=link_latency,
                        weight=1,
                    )
                )
                link_count += 1

        # West output to East input links (weight = 1)
        for row in range(num_rows):
            for col in range(num_rows):
                east_in = col + (row * num_rows)
                west_out = ((col + 1) % num_rows) + (row * num_rows)
                int_links.append(
                    GarnetIntLink(
                        link_id=link_count,
                        src_node=self.routers[west_out],
                        dst_node=self.routers[east_in],
                        src_outport=east_in,
                        dst_inport=west_out,
                        latency=link_latency,
                        weight=1,
                    )
                )
                link_count += 1

        # North output to South input links (weight = 2)
        for col in range(num_rows):
            for row in range(num_rows):
                north_out = col + (row * num_rows)
                south_in = col + (((row + 1) % num_rows) * num_rows)
                int_links.append(
                    GarnetIntLink(
                        link_id=link_count,
                        src_node=self.routers[north_out],
                        dst_node=self.routers[south_in],
                        src_outport=south_in,
                        dst_inport=north_out,
                        latency=link_latency,
                        weight=2,
                    )
                )
                link_count += 1

        # South output to North input links (weight = 2)
        for col in range(num_rows):
            for row in range(num_rows):
                north_in = col + (row * num_rows)
                south_out = col + (((row + 1) % num_rows) * num_rows)
                int_links.append(
                    GarnetIntLink(
                        link_id=link_count,
                        src_node=self.routers[south_out],
                        dst_node=self.routers[north_in],
                        src_outport=north_in,
                        dst_inport=south_out,
                        latency=link_latency,
                        weight=2,
                    )
                )
                link_count += 1

        self.int_links = int_links
