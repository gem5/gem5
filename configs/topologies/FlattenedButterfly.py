from common import FileSystemConfig
from topologies.BaseTopology import SimpleTopology

from m5.objects import *
from m5.params import *


class FlattenedButterfly(SimpleTopology):
    description = "FlattenedButterfly"

    def __init__(self, controllers):
        self.nodes = controllers

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        nodes = self.nodes

        num_routers = options.num_cpus
        num_rows = options.mesh_rows

        link_latency = options.link_latency
        router_latency = options.router_latency

        cntrls_per_router, remainder = divmod(len(nodes), num_routers)
        assert num_rows > 0 and num_rows <= num_routers
        num_columns = int(num_routers / num_rows)
        assert num_columns * num_rows == num_routers

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

        # Horizontal (Row) Links - Weight 1
        for row in range(num_rows):
            for i in range(num_columns):
                for j in range(i + 1, num_columns):
                    u = i + (row * num_columns)
                    v = j + (row * num_columns)
                    int_links.append(
                        IntLink(
                            link_id=link_count,
                            src_node=routers[u],
                            dst_node=routers[v],
                            src_outport="East",
                            dst_inport="West",
                            latency=link_latency,
                            weight=1,
                        )
                    )
                    link_count += 1
                    int_links.append(
                        IntLink(
                            link_id=link_count,
                            src_node=routers[v],
                            dst_node=routers[u],
                            src_outport="West",
                            dst_inport="East",
                            latency=link_latency,
                            weight=1,
                        )
                    )
                    link_count += 1

        # Vertical (Column) Links - Weight 2
        for col in range(num_columns):
            for i in range(num_rows):
                for j in range(i + 1, num_rows):
                    u = col + (i * num_columns)
                    v = col + (j * num_columns)
                    int_links.append(
                        IntLink(
                            link_id=link_count,
                            src_node=routers[u],
                            dst_node=routers[v],
                            src_outport="North",
                            dst_inport="South",
                            latency=link_latency,
                            weight=2,
                        )
                    )
                    link_count += 1
                    int_links.append(
                        IntLink(
                            link_id=link_count,
                            src_node=routers[v],
                            dst_node=routers[u],
                            src_outport="South",
                            dst_inport="North",
                            latency=link_latency,
                            weight=2,
                        )
                    )
                    link_count += 1

        network.int_links = int_links

    def registerTopology(self, options):
        for i in range(options.num_cpus):
            FileSystemConfig.register_node(
                [i], MemorySize(options.mem_size) // options.num_cpus, i
            )
