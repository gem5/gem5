from common import FileSystemConfig
from topologies.BaseTopology import SimpleTopology

from m5.objects import *
from m5.params import *


class HierarchicalRing(SimpleTopology):
    description = "HierarchicalRing"

    def __init__(self, controllers):
        self.nodes = controllers

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes

        num_routers = options.num_cpus
        link_latency = options.link_latency
        router_latency = options.router_latency

        cntrls_per_router, remainder = divmod(len(nodes), num_routers)

        routers = [
            Router(router_id=i, latency=router_latency)
            for i in range(num_routers)
        ]
        network.routers = routers

        link_count = 0

        network_nodes = []
        remainder_nodes = []
        for node_index in range(len(nodes)):
            if node_index < (len(nodes) - remainder):
                network_nodes.append(nodes[node_index])
            else:
                remainder_nodes.append(nodes[node_index])

        ext_links = []
        for i, n in enumerate(network_nodes):
            cntrl_level, router_id = divmod(i, num_routers)
            assert cntrl_level < cntrls_per_router
            ext_links.append(
                ExtLink(
                    link_id=link_count,
                    ext_node=n,
                    int_node=routers[router_id],
                    latency=link_latency,
                )
            )
            link_count += 1

        for i, node in enumerate(remainder_nodes):
            assert node.type == "DMA_Controller"
            assert i < remainder
            ext_links.append(
                ExtLink(
                    link_id=link_count,
                    ext_node=node,
                    int_node=routers[0],
                    latency=link_latency,
                )
            )
            link_count += 1

        network.ext_links = ext_links

        int_links = []

        def add_bidirectional_link(
            u, v, weight, src_out="East", dst_in="West"
        ):
            nonlocal link_count
            int_links.append(
                IntLink(
                    link_id=link_count,
                    src_node=routers[u],
                    dst_node=routers[v],
                    src_outport=src_out,
                    dst_inport=dst_in,
                    latency=link_latency,
                    weight=weight,
                )
            )
            link_count += 1
            int_links.append(
                IntLink(
                    link_id=link_count,
                    src_node=routers[v],
                    dst_node=routers[u],
                    src_outport=dst_in,
                    dst_inport=src_out,
                    latency=link_latency,
                    weight=weight,
                )
            )
            link_count += 1

        # 4 Local Base Rings (Weight 1)
        base_rings = [
            [0, 1, 2, 3],
            [4, 5, 6, 7],
            [8, 9, 10, 11],
            [12, 13, 14, 15],
        ]
        for ring in base_rings:
            n = len(ring)
            for i in range(n):
                u = ring[i]
                v = ring[(i + 1) % n]
                add_bidirectional_link(
                    u, v, weight=1, src_out="East", dst_in="West"
                )

        # 1 Global Connecting Ring (Weight 2) between routers 0, 4, 8, 12
        global_ring = [0, 4, 8, 12]
        n_global = len(global_ring)
        for i in range(n_global):
            u = global_ring[i]
            v = global_ring[(i + 1) % n_global]
            add_bidirectional_link(
                u, v, weight=2, src_out="North", dst_in="South"
            )

        network.int_links = int_links

    def registerTopology(self, options):
        for i in range(options.num_cpus):
            FileSystemConfig.register_node(
                [i], MemorySize(options.mem_size) // options.num_cpus, i
            )
