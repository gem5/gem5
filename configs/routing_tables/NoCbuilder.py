import csv

import networkx as nx

try:
    from routing_tables import (
        GraphSetup,
        createGc,
        createGr,
    )
except:
    import createGc
    import createGr
    import GraphSetup

import os

dirname = os.path.dirname(os.path.abspath(__file__))


class CommunicationGraph:
    def __init__(self, app=None, mappingFlag=True, filename="tgff_64_64.txt"):
        if app is None:
            self.GC = GraphSetup.createGc_from_txt(filename, mappingFlag)
        else:
            self.GC = createGc.createGc(mappingFlag, app)
        self.cycles = 100000000

    def convertToPacketsPerTick(self, rate, str="KB"):
        tickrate = pow(10, 12)  # default tickrate in gem5
        # assuming 1 packet is 72bytes as defines by ruby write requests
        if str == "KB":
            byterate = rate * 1024
        elif str == "MB":
            byterate = rate * 1024 * 1024
        elif str == "GB":
            byterate = rate * 1024 * 1024 * 1024
        else:
            print("invalid data rate")

        packetrate = byterate / 72  # packets/sec

        return packetrate / tickrate  # packs/tick

    def num_cpus(self):
        return self.GC.number_of_nodes()

    def get_destinations(self, node):
        if self.GC.has_node(node):
            neighbors = sorted(
                self.GC[node].items(), key=lambda edge: edge[1]["demand"]
            )
            return [neighbor for neighbor, _metadata in neighbors]
        else:
            return []

    def get_rates(self, node):
        if self.GC.has_node(node):
            neighbors = sorted(
                self.GC[node].items(), key=lambda edge: edge[1]["demand"]
            )
            return [_metadata["demand"] for neighbor, _metadata in neighbors]
        else:
            return [0]

    def get_total_rate(self, node):
        return self.convertToPacketsPerTick(sum(self.get_rates(node)), "MB")


class TopologyGraph:
    # def create_mesh_2d(self,n_rows=4, n_cols=4):
    #     G = nx.DiGraph()

    #     # Add nodes
    #     for r in range(n_rows):
    #         for c in range(n_cols):
    #             node_id = r * n_cols + c
    #             G.add_node(node_id, pos=(c, -r),latency=1)  # store positions for plotting

    #     # Add edges (bi-directional)
    #     for r in range(n_rows):
    #         for c in range(n_cols):
    #             node = r * n_cols + c
    #             # Right neighbor
    #             if c < n_cols - 1:
    #                 right = r * n_cols + (c + 1)
    #                 G.add_edge(node, right,traffic=100,latency=1)
    #                 G.add_edge(right, node,traffic=100,latency=1)
    #             # Down neighbor
    #             if r < n_rows - 1:
    #                 down = (r + 1) * n_cols + c
    #                 G.add_edge(node, down,traffic=1000,latency=1)
    #                 G.add_edge(down, node,traffic=1000,latency=1)
    #     return G

    def __init__(self, topo="mesh", size=(3, 3), directory="all", L2=False):
        self.GR = createGr.createGr(topo, size[0], size[1])
        if topo == "torus":
            corners = [
                0,
                size[0] - 2,
                size[0] * (size[0] - 2),
                size[0] * (size[0] - 2) + (size[0] - 2),
            ]
        else:
            corners = [
                0,
                size[0] - 1,
                size[0] * (size[0] - 1),
                (size[0] * size[0]) - 1,
            ]
        for node, atr in self.GR.nodes(data=True):
            atr["latency"] = 1
            atr["core"] = True
            if L2 and node in corners:
                atr["l2"] = True
            else:
                atr["l2"] = False
            if directory == "all":
                atr["directory"] = True
            elif directory == "corners":
                if node in corners:
                    atr["directory"] = True
                else:
                    atr["directory"] = False

        for src, dst, atr in self.GR.edges(data=True):
            atr["latency"] = 1

        self.mesh_width = size[0]
        self.routing_table = {}  # (src, dst) -> [route]

    def get_num_routers(self):
        return self.GR.number_of_nodes()

    def get_routers(self):
        return self.GR.nodes

    def get_edges(self):
        return self.GR.edges

    def plot(self):
        createGr.plot_net_topology_graph(Gr=self.GR)

    def compute_routing_table(self):
        for src in self.GR.nodes:
            for dst in self.GR.nodes:
                if src == dst:
                    continue

                self.routing_table[(src, dst)] = nx.shortest_path(
                    self.GR, src, dst
                )

    def compute_west_first_routing_table(self):
        width = self.mesh_width
        coords = {n: (n % width, n // width) for n in self.GR.nodes}

        for src in self.GR.nodes:
            for dst in self.GR.nodes:
                if src == dst:
                    continue

                path = []
                curr = src
                dx, dy = coords[dst]

                while curr != dst:
                    cx, cy = coords[curr]

                    if cx > dx:
                        next_node = curr - 1
                    elif cy < dy:
                        next_node = curr + width
                    elif cy > dy:
                        next_node = curr - width
                    elif cx < dx:
                        next_node = curr + 1
                    else:
                        break

                    path.append(next_node)
                    curr = next_node

                self.routing_table[(src, dst)] = path

    def compute_XY_routing_table(self):

        width = self.mesh_width
        coords = {n: (n % width, n // width) for n in self.GR.nodes}
        for src in self.GR.nodes:
            for dst in self.GR.nodes:
                if src == dst:
                    continue

                path = []
                curr = src

                while curr != dst:

                    cx, cy = coords[curr]
                    dx, dy = coords[dst]

                    if cx < dx:
                        next_node = curr + 1
                    elif cx > dx:
                        next_node = curr - 1
                    elif cy < dy:
                        next_node = curr + width
                    else:
                        next_node = curr - width

                    path.append(next_node)
                    curr = next_node

                self.routing_table[(src, dst)] = path

    def write_routing_csv(self, filename):
        with open(os.path.join(dirname, filename), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["src", "dst", "curr", "next_hop"])

            for (src, dst), next_hop in sorted(self.routing_table.items()):

                for i in range(len(next_hop)):
                    if i == 0:
                        writer.writerow([src, dst, src, next_hop[0]])
                    else:
                        writer.writerow(
                            [src, dst, next_hop[i - 1], next_hop[i]]
                        )


import math

if __name__ == "__main__":
    builder = TopologyGraph(topo="mesh", size=(4, 4))
    print([(n, data) for n, data in (builder.get_routers())(data=True)])
    for n, data in (builder.get_routers())(data=True):
        if data["directory"]:
            print(n, data)
