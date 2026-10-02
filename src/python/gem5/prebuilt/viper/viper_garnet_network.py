# Copyright (c) 2026, University of Wisconsin
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE

from m5.objects import (
    GarnetExtLink,
    GarnetIntLink,
    GarnetNetwork,
    GarnetNetworkInterface,
    GarnetRouter,
)


class GarnetPt2Pt(GarnetNetwork):
    def __init__(self, ruby_system):
        super().__init__()
        self.netifs = []
        self.ruby_system = ruby_system

    def connect(self, controllers):
        self.routers = [
            GarnetRouter(router_id=i) for i in range(len(controllers))
        ]

        # Make a link from each controller to the router. The link goes
        # externally to the network.
        self.ext_links = [
            GarnetExtLink(link_id=i, ext_node=c, int_node=self.routers[i])
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
                    GarnetIntLink(link_id=link_count, src_node=ri, dst_node=rj)
                )
        self.int_links = int_links
        self.netifs = [
            GarnetNetworkInterface(id=i)
            for (i, n) in enumerate(self.ext_links)
        ]


class GarnetDoubleCrossbar(GarnetNetwork):

    def __init__(self, ruby_system):
        super().__init__()
        self.netifs = []
        self.ruby_system = ruby_system

    def connect(self, controllers):
        l2_xbar_types = (
            "GPU_VIPER_TCP_Controller",
            "GPU_VIPER_SQC_Controller",
            "GPU_VIPER_TCC_Controller",
        )
        soc_xbar_types = (
            "GPU_VIPER_DMA_Controller",
            "GPU_VIPER_Directory_Controller",
        )

        # Create one router per controller plus a crossbar for L2 controllers
        # and a crossbar for SoC controllers.
        routers = [GarnetRouter(router_id=i) for i in range(len(controllers))]
        routers.append(GarnetRouter(router_id=len(routers)))
        routers.append(GarnetRouter(router_id=len(routers)))
        self.routers = routers

        # Routers 0 ... N-2 connect to the individual controllers
        self.ext_links = [
            GarnetExtLink(link_id=i, ext_node=c, int_node=self.routers[i])
            for i, c in enumerate(controllers)
        ]

        # Connect compute unit components and L2s to L2 crossbar in both
        # directions.
        l2_xbar_id = len(controllers)
        soc_xbar_id = l2_xbar_id + 1
        int_links = []

        for ext_link in self.ext_links:
            if ext_link.ext_node.type in l2_xbar_types:
                int_links.append(
                    GarnetIntLink(
                        link_id=len(int_links),
                        src_node=ext_link.int_node,
                        dst_node=self.routers[l2_xbar_id],
                    )
                )
                int_links.append(
                    GarnetIntLink(
                        link_id=len(int_links),
                        src_node=self.routers[l2_xbar_id],
                        dst_node=ext_link.int_node,
                    )
                )
            elif ext_link.ext_node.type in soc_xbar_types:
                int_links.append(
                    GarnetIntLink(
                        link_id=len(int_links),
                        src_node=ext_link.int_node,
                        dst_node=self.routers[soc_xbar_id],
                    )
                )
                int_links.append(
                    GarnetIntLink(
                        link_id=len(int_links),
                        src_node=self.routers[soc_xbar_id],
                        dst_node=ext_link.int_node,
                    )
                )

        # Connect L2 xbar to SoC xbar.
        int_links.append(
            GarnetIntLink(
                link_id=len(int_links),
                src_node=self.routers[l2_xbar_id],
                dst_node=self.routers[soc_xbar_id],
            )
        )
        int_links.append(
            GarnetIntLink(
                link_id=len(int_links),
                src_node=self.routers[soc_xbar_id],
                dst_node=self.routers[l2_xbar_id],
            )
        )

        # Finalize network int_links for unproxy
        self.int_links = int_links
        self.netifs = [
            GarnetNetworkInterface(id=i)
            for (i, n) in enumerate(self.ext_links)
        ]
