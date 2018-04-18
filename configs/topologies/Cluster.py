# Copyright (c) 2012 Advanced Micro Devices, Inc.
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
# Authors: Jason Power

from __future__ import print_function
from __future__ import absolute_import

from .BaseTopology import BaseTopology

class Cluster(BaseTopology):
    """ A cluster is a group of nodes which are all one hop from eachother
        Clusters can also contain other clusters
        When creating this kind of topology, return a single cluster (usually
        the root cluster) from create_system in configs/ruby/<protocol>.py
    """

    _num_int_links = 0
    _num_ext_links = 0
    _num_routers = 0

    # Below methods for auto counting
    @classmethod
    def num_int_links(cls):
        cls._num_int_links += 1
        return cls._num_int_links - 1
    @classmethod
    def num_ext_links(cls):
        cls._num_ext_links += 1
        return cls._num_ext_links - 1
    @classmethod
    def num_routers(cls):
        cls._num_routers += 1
        return cls._num_routers - 1

    def __init__(self, intBW=0, extBW=0, intLatency=0, extLatency=0):
        """ internalBandwidth is bandwidth of all links within the cluster
            externalBandwidth is bandwidth from this cluster to any cluster
                connecting to it.
            internal/externalLatency are similar
            **** When creating a cluster with sub-clusters, the sub-cluster
                 external bandwidth overrides the internal bandwidth of the
                 super cluster
        """
        self.nodes = []
        self.router = None # created in makeTopology
        self.intBW = intBW
        self.extBW = extBW
        self.intLatency = intLatency
        self.extLatency = extLatency

    def add(self, node):
        self.nodes.append(node)

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        """ Recursively make all of the links and routers
        """

        # make a router to connect all of the nodes
        self.router = Router(router_id=self.num_routers())
        network.routers.append(self.router)

        for node in self.nodes:
            if type(node) == Cluster:
                node.makeTopology(options, network, IntLink,
                                  ExtLink, Router)

                # connect this cluster to the router
                link_out = IntLink(link_id=self.num_int_links(), src_node=self.router,
                           dst_node=node.router)
                link_in = IntLink(link_id=self.num_int_links(), src_node=node.router,
                                  dst_node=self.router)

                if node.extBW:
                    link_out.bandwidth_factor = node.extBW
                    link_in.bandwidth_factor = node.extBW

                # if there is an internal b/w for this node
                # and no ext b/w to override
                elif self.intBW:
                    link_out.bandwidth_factor = self.intBW
                    link_in.bandwidth_factor = self.intBW

                if node.extLatency:
                    link_out.latency = node.extLatency
                    link_in.latency = node.extLatency
                elif self.intLatency:
                    link_out.latency = self.intLatency
                    link_in.latency = self.intLatency

                network.int_links.append(link_out)
                network.int_links.append(link_in)
            else:
                # node is just a controller,
                # connect it to the router via a ext_link
                link = ExtLink(link_id=self.num_ext_links(), ext_node=node,
                        int_node=self.router)

                if self.intBW:
                    link.bandwidth_factor = self.intBW
                if self.intLatency:
                    link.latency = self.intLatency

                network.ext_links.append(link)

    def __len__(self):
        return len([i for i in self.nodes if type(i) != Cluster]) + \
               sum([len(i) for i in self.nodes if type(i) == Cluster])
