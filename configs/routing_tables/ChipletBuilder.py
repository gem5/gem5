import os
import itertools
import csv

import networkx as nx
try:
    import createGr
except:
    from routing_tables import createGr

dirname = os.path.dirname(os.path.abspath(__file__))


class Routing:
    def __init__(self,graph,size):
        self.graph = graph
        self.size = size

    def compute_XY(self):
        width = self.size[0]
        routing_table = {}
        coords = {n: (n % width, n // width) for n in self.graph}
        for src in self.graph:
            for dst in self.graph:
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

                routing_table[(src, dst)] = path
        return routing_table

class BaseNetwork:
    def __init__(self, clock="1Ghz"):
        self.clock = clock
        self.graph = nx.DiGraph()

    def add_router_options(
        self, router_latency=1, cores=True, directory="all"
    ):
        min_deg = min(dict(self.graph.degree()).values())
        corners = [n for n, d in self.graph.degree() if d == min_deg]
        for node, atr in self.graph.nodes(data=True):
            atr["latency"] = router_latency
            atr["clock"] = self.clock
            atr["core"] = cores
            atr["l2"] = False
            if directory == "all":
                atr["directory"] = True
            elif directory == "corners":
                if node in corners:
                    atr["directory"] = True
                else:
                    atr["directory"] = False
            elif directory == "none":
                atr["directory"] = False

    def add_link_options(self, link_latency=1):
        for src, dst, atr in self.graph.edges(data=True):
            atr["latency"] = link_latency
            atr["clock"] = self.clock
            atr["cdc"] = 0


class Chiplet(BaseNetwork):
    def __init__(self, topo="mesh", size=(4, 4)):
        super().__init__()
        self.size = size
        self.graph = createGr.createGr(topo, size[0], size[1])
        self.add_link_options()
        self.add_router_options()
        route = Routing(self.graph,size)
        self.routing_table = route.compute_XY()
        self.interposers = []

    def find_interposers(self):
        """
        add complex interposer selection logic here
        """
        # by default pick 4 nodes in pairs on 2 opposite edges adjacent to corners  
        x = self.size[0]
        y = self.size[1]
        return [1,x-1,(y-1)*x+1, x*y -2]
    
    def compute_closest_interposers(self,MTR=True):
        closest_interposer = {}
        for (src, dst), path in self.routing_table.items():

            if dst not in self.interposers:
                continue

            cost = len(path)
            if MTR:
                penalty = False
                #for default interposers
                #no parallel exits
                y = self.size[1]
                if len(path)>1:
                    prev_node = path[-2]
                else:
                    prev_node = src
                if prev_node == dst + y or prev_node == dst - y:
                    penalty = True
                #increase cost if the MTR not agreeing
                if penalty:
                    cost+=1000

            if src not in closest_interposer or cost < len(closest_interposer[src]):
                closest_interposer[src] = path
        return closest_interposer
    
    def compute_interposer_to_dest(self,MTR=True):
        destination_routes = {}
        for (src, dst), path in self.routing_table.items():
            if src not in self.interposers:
                continue
            
            destination_routes[(src,dst)] = path

            if MTR:
                """
                MTR logic to avoid deadlocks by banning certain paths
                """
                #for default interposers
                #no left on one side no rights on opposite
                new_path = []
                if sorted(self.interposers).index(src)<(len(self.interposers)/2):
                    if path[0] == src+1:
                        new_path.append(src+self.size[1]) #force a y path
                        new_path.extend(self.routing_table[src+self.size[1],dst])
                        destination_routes[(src,dst)] = new_path
                
                if sorted(self.interposers).index(src)>=(len(self.interposers)/2):
                    if path[0] == src+-1:
                        new_path.append(src-self.size[1]) #force a y path
                        new_path.extend(self.routing_table[src-self.size[1],dst])
                        destination_routes[(src,dst)] = new_path
        
        return destination_routes            

class NOI(BaseNetwork):
    def __init__(self, topo="mesh", size=(4, 4)):
        super().__init__()
        self.graph = createGr.createGr(topo, size[0], size[1])
        self.add_link_options()
        self.add_router_options(cores=False, directory="none")
        route = Routing(self.graph,size)
        self.routing_table = route.compute_XY()
        self.association_table = []
        self.routing = {}
    
    def compute_routes(self,mapping):
        for src in self.graph:
            for chiplet_id,destinations in enumerate(self.association_table):
                for des in destinations:
                    if src==mapping[des]:
                        continue
                    if (src,chiplet_id) not in self.routing:
                        self.routing[(src,chiplet_id)] = self.routing_table[(src,mapping[des])]
                    if len(self.routing_table[(src,mapping[des])])<len(self.routing[(src,chiplet_id)]):
                        self.routing[(src,chiplet_id)] = self.routing_table[(src,mapping[des])]
class ChipletSystem:
    def __init__(self):
        self.chiplets = []
        self.baseNetwork = None
        self.graph = nx.DiGraph()
        self.routing_table = {}
        self.sizes = []
        self.closest_interposer = {}
        self.destination_routes = {}
        self.interposer_connections = {}

    def connect_chiplets(self):
        interposer_nodes = []
        next_id = 0

        for chiplet in self.chiplets:
            mapping = {n: next_id + idx for idx, n in enumerate(chiplet.graph.nodes())}
            next_id += len(chiplet.graph.nodes())

            new_routing = {
                (mapping[src], mapping[dst]): [mapping[x] for x in path]
                for (src, dst), path in chiplet.routing_table.items()}
            chiplet.routing_table = new_routing
            self.routing_table.update(chiplet.routing_table)
            self.graph = nx.compose(self.graph, nx.relabel_nodes(chiplet.graph, mapping,False))
            chiplet.interposers = [mapping[n] for n in chiplet.find_interposers()]
            interposer_nodes.extend(chiplet.interposers)
            self.baseNetwork.association_table.append(chiplet.interposers)
            self.closest_interposer.update(chiplet.compute_closest_interposers())
            self.destination_routes.update(chiplet.compute_interposer_to_dest())

        mapping = {
            n: next_id + idx for idx, n in enumerate(self.baseNetwork.graph.nodes())
        }        
        new_routing = {
                (mapping[src], mapping[dst]): [mapping[x] for x in path]
                for (src, dst), path in self.baseNetwork.routing_table.items()}
        self.baseNetwork.routing_table = new_routing
        self.graph = nx.compose(self.graph, nx.relabel_nodes(self.baseNetwork.graph, mapping,False))

        for i, node in enumerate(interposer_nodes):
            target = mapping[i]
            self.interposer_connections[node] = target
            self.interposer_connections[target] = node
            self.graph.add_edge(
                node,
                target,
                latency=1,
                clock=self.baseNetwork.clock,
                direction="interposer",
            )
            self.graph.add_edge(
                target,
                node,
                latency=1,
                clock=self.baseNetwork.clock,
                direction="interposer",
            )
    
        self.baseNetwork.compute_routes(self.interposer_connections)

    def create_chiplets(self, num_chiplets):
        for i in range(num_chiplets):
            self.chiplets.append(Chiplet())
        self.sizes = list(itertools.accumulate(c.size[0] * c.size[1] for c in self.chiplets))

    def merge_routing(self):   
        for src in range(self.sizes[-1]):
            for dst in range(self.sizes[-1]):
                if src == dst:
                    continue
                if (src,dst) in self.routing_table:
                    continue
                
                path = []

                #extend to closest interposer
                path.extend(self.closest_interposer[src])
                interposer_entry = self.interposer_connections[path[-1]]
                path.append(interposer_entry)
                
                #caclulate interposer path
                chiplet_id = len([x for x in self.sizes if x <= dst])
                path.extend(self.baseNetwork.routing[(interposer_entry,chiplet_id)])
                interposer_exit = self.interposer_connections[path[-1]]
                path.append(interposer_exit)

                #extend to destination
                if path[-1]!=dst:
                    path.extend(self.destination_routes[(path[-1],dst)])

                self.routing_table[(src,dst)]=path


                

class ChipletBuilder:
    def __init__(self, num_chiplets,filename="chipletRouting.csv"):
        self.system = ChipletSystem()
        self.system.create_chiplets(num_chiplets)
        self.system.baseNetwork = NOI()
        self.system.connect_chiplets()
        self.system.merge_routing()
        self.GR = self.system.graph
        self.write_routing_csv(filename)

    def get_num_routers(self):
        return self.GR.number_of_nodes()

    def get_routers(self):
        return self.GR.nodes

    def get_edges(self):
        return self.GR.edges
    
    def write_routing_csv(self, filename):
        with open(os.path.join(dirname, filename), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["src", "dst", "curr", "next_hop"])

            for (src, dst), next_hop in sorted(self.system.routing_table.items()):

                for i in range(len(next_hop)):
                    if i == 0:
                        writer.writerow([src, dst, src, next_hop[0]])
                    else:
                        writer.writerow(
                            [src, dst, next_hop[i - 1], next_hop[i]]
                        )
    
if __name__ == "__main__":
    builder = ChipletBuilder(4)
