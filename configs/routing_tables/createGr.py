import networkx as nx

Edge = tuple[int, int]  # Bidirected edge (src, dst)
EdgeWithDict = tuple[
    int, int, dict[str, str]
]  # Bidirected edge (src, dst, *attr)


def _validate_dims(m: int, n: int) -> None:
    # m: number of rows, n: number of columns
    if m < 1 or n < 1:
        raise ValueError(
            f"Invalid network size: m={m}, n={n}. Both must be >= 1."
        )


def create_mesh_edges(m: int, n: int) -> list[EdgeWithDict]:
    """
    2D mesh: each node connects to its existing N/S/E/W neighbors (directed edges).
    Note: this generates edges in both directions (because each node adds its neighbor edges),
    matching the behavior of the original script.
    """
    _validate_dims(m, n)
    edges: list[EdgeWithDict] = []

    for i in range(m * n):
        # left/right
        if i % n:  # not in first column
            edges.append((i, i - 1, {"direction": "left"}))
        if (i + 1) % n:  # not in last column
            edges.append((i, i + 1, {"direction": "right"}))

        # up/down
        if i >= n:  # not in first row
            edges.append((i, i - n, {"direction": "up"}))
        if i <= n * (m - 1) - 1:  # not in last row
            edges.append((i, i + n, {"direction": "down"}))

    return edges


def create_torus_edges(m: int, n: int) -> list[Edge]:
    """
    2D torus: mesh plus wrap-around edges on left/right and top/bottom.
    Matches the logic of the original script.
    """
    _validate_dims(m, n)
    edges: list[Edge] = []

    for i in range(m * n):
        # left/right with wrap
        if i % n:
            edges.append((i, i - 1, {"direction": "left"}))
        else:
            edges.append((i, i + n - 1, {"direction": "left"}))

        if (i + 1) % n:
            edges.append((i, i + 1, {"direction": "right"}))
        else:
            edges.append((i, i - n + 1, {"direction": "right"}))

        # up/down with wrap
        if i >= n:
            edges.append((i, i - n, {"direction": "up"}))
        else:
            edges.append((i, i + (m - 1) * n, {"direction": "up"}))

        if i <= n * (m - 1) - 1:
            edges.append((i, i + n, {"direction": "down"}))
        else:
            edges.append((i, i - (m - 1) * n, {"direction": "down"}))

    return edges


def create_hexagonal_edges(m: int, n: int) -> list[Edge]:
    """
    "Hexagonal" variant from the original script:
    - starts with mesh-like N/S/E/W edges (no wrap)
    - adds an extra diagonal between i and (i + n + 1) (both directions)
      for nodes that are not on the last row and not in the last column.
    """
    _validate_dims(m, n)
    edges: list[Edge] = []

    for i in range(m * n):
        # left/right
        if i % n:
            edges.append((i, i - 1))
        if (i + 1) % n:
            edges.append((i, i + 1))

        # up/down
        if i >= n:
            edges.append((i, i - n))
        if i <= n * (m - 1) - 1:
            edges.append((i, i + n))

        # extra diagonal (both directions), as in your original
        is_not_last_row = i <= n * (m - 1) - 1
        is_not_last_col = ((i + 1) % n) != 0
        if is_not_last_row and is_not_last_col:
            j = i + n + 1
            edges.append((i, j))
            edges.append((j, i))

    return edges


def available_topologies() -> list[str]:
    return ["mesh", "torus", "hexagonal"]


def createGr(topology: str, m: int, n: int) -> nx.DiGraph:
    """
    Create a directed routing graph Gr for a given topology and grid dimensions.

    Node attributes:
      - pos: (row, col) where row = node_id // n, col = node_id % n

    Edge attributes:
      - bw: bandwidth capacity (default: float('inf'))
    """
    topo = topology.strip().lower()

    if topo == "mesh":
        edges = create_mesh_edges(m, n)
    elif topo == "torus":
        edges = create_torus_edges(m, n)
    elif topo == "hexagonal":
        edges = create_hexagonal_edges(m, n)
    else:
        raise ValueError(
            f"Invalid topology '{topology}'. Choose from {available_topologies()}."
        )

    Gr = nx.DiGraph()
    Gr.add_nodes_from(range(m * n))
    Gr.add_edges_from(edges)

    # Set node positions
    for node in Gr.nodes():
        col, row = divmod(node, n)
        Gr.nodes[node]["pos"] = (row, col)

    # Set edge bandwidth to infinity by default
    for src, dst in Gr.edges():
        Gr[src][dst]["bw"] = float("inf")

    return Gr


def plot_net_topology_graph(
    Gr: nx.DiGraph,
    title: str = "Network Topology",
    layer_cols: int | None = None,
):
    """
    Plot Gr.
    - If layer_cols is None: uses spring_layout.
    - Else: uses multipartite_layout with layers derived from node_id // layer_cols.
      (Set layer_cols=n to match grid columns.)
    """
    import matplotlib.pyplot as plt

    if layer_cols is None:
        pos = nx.spring_layout(Gr, seed=1)
    else:
        layers = {node: node // layer_cols for node in Gr.nodes()}
        nx.set_node_attributes(Gr, layers, "layer")
        pos = nx.multipartite_layout(Gr, subset_key="layer")

    plt.title(title)
    nx.draw(Gr, pos, with_labels=True, connectionstyle="arc3,rad=0.05")
    plt.show()


if __name__ == "__main__":
    for topo in available_topologies():
        m, n = 4, 8
        Gr = createGr(topo, m, n)

        print(
            f"{topo}: nodes={Gr.number_of_nodes()}, edges={Gr.number_of_edges()}"
        )
        print(f"{topo}: sample nodes={sorted(list(Gr.nodes(data=True)))[:4]}")
        print(
            f"{topo}: sample edges with bw={sorted(list(Gr.edges(data=True)))[:6]}"
        )

        # Use layer_cols=n so the layout matches the grid width by default
        plot_net_topology_graph(Gr, title=f"Topology: {topo}", layer_cols=n)
