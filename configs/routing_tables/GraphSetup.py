import csv
import os
import re

import networkx as nx

# Get the absolute path to the directory containing this script
script_dir = os.path.dirname(os.path.abspath(__file__))


LINE_RE = re.compile(
    r"PE\s+(\d+)\s*->\s*PE\s*(\d+)\s*" r"d=\s*(\d+)\s*" r"hops=\s*(\d+)\s*(.*)"
)

SW_RE = re.compile(r"sw(\d+)")


def parse_routing_file(filename):
    """
    Returns list of dictionaries:
    {
        'src_pe': int,
        'dst_pe': int,
        'demand': float,
        'hops': int,
        'switch_path': [sw0, sw1, ...]
    }
    """
    flows = []
    filename = os.path.join(script_dir, filename)
    with open(filename) as f:
        for line in f:
            line = line.strip()

            if not line.startswith("PE"):
                continue

            m = LINE_RE.match(line)
            if not m:
                continue

            src_pe = int(m.group(1))
            dst_pe = int(m.group(2))
            demand = float(m.group(3))
            hops = int(m.group(4))
            path_str = m.group(5)

            switch_path = [int(x) for x in SW_RE.findall(path_str)]

            flows.append(
                {
                    "src_pe": src_pe,
                    "dst_pe": dst_pe,
                    "demand": demand,
                    "hops": hops,
                    "switch_path": switch_path,
                }
            )

    return flows


def createGc_from_txt(filename, mapping=True):
    """
    Graph with PE ids as nodes.
    """
    Gc = nx.DiGraph()

    flows = parse_routing_file(filename)

    if mapping:
        for flow in flows:
            path = flow["switch_path"]

            if len(path) < 2:
                continue

            src_sw = path[0]
            dst_sw = path[-1]

            Gc.add_edge(
                src_sw,
                dst_sw,
                demand=flow["demand"],
            )
        createRoutingTable_from_txt(flows)
    else:
        for flow in flows:
            Gc.add_edge(
                flow["src_pe"],
                flow["dst_pe"],
                demand=flow["demand"],
            )
    return Gc


def createRoutingTable_from_txt(flows):
    with open(os.path.join(script_dir, "ASroute.csv"), "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["src", "dst", "curr", "next_hop"])

        for flow in flows:
            path = flow["switch_path"]
            for i in range(len(path) - 1):
                writer.writerow([path[0], path[-1], path[i], path[i + 1]])
    return


if __name__ == "__main__":
    Gc = createGc_from_txt("tgff_64_64.txt", True)
    print(Gc.nodes)
    print(sum(data["demand"] for _, _, data in Gc.edges(data=True)))
