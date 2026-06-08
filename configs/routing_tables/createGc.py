from math import (
    ceil,
    sqrt,
)
from typing import (
    Dict,
    List,
    Optional,
    Tuple,
)

import networkx as nx

CommEdge = Tuple[int, int, float]  # (src, dst, demand)
Edge = Tuple[int, int]
_APP_MAPPING: Dict[str, List[Edge]] = {
    "PIP": [
        (0, 1),
        (4, 0),
        (1, 2),
        (2, 5),
        (3, 4),
        (6, 6),
        (5, 3),
        (7, 7),
    ],
    "MP3": [
        (0, 10),
        (1, 9),
        (2, 14),
        (8, 11),
        (4, 5),
        (3, 6),
        (5, 4),
        (6, 8),
        (7, 0),
        (9, 7),
        (12, 3),
        (10, 1),
        (11, 2),
    ],
    "MWD": [
        (0, 8),
        (1, 9),
        (2, 13),
        (5, 10),
        (6, 11),
        (9, 7),
        (4, 4),
        (3, 0),
        (7, 5),
        (8, 6),
        (10, 2),
        (11, 3),
    ],
    "MPEG4": [
        (4, 5),
        (0, 4),
        (1, 2),
        (2, 8),
        (3, 1),
        (8, 7),
        (9, 9),
        (10, 6),
        (5, 0),
        (6, 10),
        (7, 11),
        (11, 14),
    ],
    "VOPD": [
        (0, 7),
        (1, 3),
        (2, 2),
        (3, 1),
        (4, 5),
        (15, 0),
        (5, 6),
        (6, 10),
        (7, 11),
        (8, 14),
        (9, 15),
        (11, 9),
        (10, 13),
        (12, 8),
        (13, 4),
        (14, 12),
    ],
}
_APP_COMM: Dict[str, List[CommEdge]] = {
    "PIP": [
        (0, 4, 64),
        (1, 0, 128),
        (1, 2, 64),
        (2, 3, 64),
        (3, 6, 64),
        (4, 5, 64),
        (5, 6, 64),
        (6, 7, 64),
    ],
    "MWD": [
        (0, 1, 64),
        (1, 2, 128),
        (1, 5, 96),
        (5, 6, 96),
        (6, 9, 96),
        (0, 4, 128),
        (3, 4, 96),
        (4, 7, 96),
        (7, 8, 96),
        (8, 9, 96),
        (8, 10, 64),
        (10, 11, 64),
    ],
    "MP3": [
        (0, 1, 2083),
        (0, 2, 4060),
        (0, 8, 25),
        (1, 4, 1000),
        (2, 3, 500),
        (3, 4, 1000),
        (4, 5, 870),
        (5, 6, 150),
        (5, 7, 180),
        (8, 9, 2083),
        (9, 12, 10),
        (10, 11, 4060),
        (11, 12, 500),
    ],
    "MPEG4": [
        (4, 0, 190),
        (0, 4, 190),
        (4, 1, 0.5),
        (1, 4, 0.5),
        (4, 2, 60),
        (2, 4, 60),
        (4, 3, 600),
        (3, 4, 600),
        (4, 8, 0.5),
        (8, 4, 0.5),
        (4, 9, 910),
        (9, 4, 910),
        (4, 10, 32),
        (10, 4, 32),
        (2, 5, 40),
        (5, 2, 40),
        (3, 5, 40),
        (5, 4, 40),
        (6, 9, 670),
        (9, 6, 670),
        (6, 10, 173),
        (10, 6, 173),
        (6, 7, 250),
        (7, 6, 250),
        (6, 11, 500),
        (11, 6, 500),
    ],
    "VOPD": [
        (0, 1, 70),
        (1, 2, 362),
        (2, 3, 362),
        (3, 4, 362),
        (3, 15, 49),
        (15, 4, 27),
        (4, 5, 357),
        (5, 6, 353),
        (6, 7, 300),
        (7, 8, 313),
        (7, 9, 500),
        (8, 9, 313),
        (9, 8, 94),
        (11, 5, 16),
        (11, 8, 16),
        (10, 11, 16),
        (11, 12, 16),
        (12, 13, 157),
        (13, 14, 16),
        (14, 10, 16),
        (14, 12, 16),
    ],
    "DVOPD": [
        (0, 1, 70),
        (1, 2, 362),
        (2, 3, 362),
        (3, 4, 362),
        (3, 14, 49),
        (14, 4, 27),
        (4, 5, 357),
        (5, 6, 353),
        (6, 7, 300),
        (7, 9, 500),
        (7, 8, 313),
        (8, 9, 313),
        (9, 8, 94),
        (10, 5, 16),
        (10, 8, 16),
        (10, 11, 16),
        (11, 12, 157),
        (12, 13, 16),
        (13, 10, 16),
        (13, 11, 16),
        (10, 31, 540),
        (30, 0, 126),
        (30, 15, 126),
        (15, 16, 70),
        (16, 17, 362),
        (17, 18, 362),
        (18, 19, 362),
        (18, 29, 49),
        (29, 19, 27),
        (19, 20, 357),
        (20, 21, 353),
        (21, 22, 300),
        (22, 23, 313),
        (22, 24, 500),
        (23, 24, 313),
        (24, 23, 94),
        (25, 20, 16),
        (25, 23, 16),
        (25, 26, 16),
        (25, 31, 540),
        (26, 27, 157),
        (27, 28, 16),
        (28, 25, 16),
        (28, 26, 16),
    ],
}


def available_apps() -> List[str]:
    return sorted(_APP_COMM.keys())


def create_app_communication(app: str) -> List[CommEdge]:
    key = app.strip().upper()
    if key not in _APP_COMM:
        raise ValueError(
            f"Unknown app '{app}'. Supported: {', '.join(available_apps())}"
        )
    return list(_APP_COMM[key])


def createGc(Mapping: bool, app: str) -> nx.DiGraph:
    Gc = nx.DiGraph()
    mapped_comm = create_app_communication(app)
    if Mapping:
        mapping = dict(_APP_MAPPING[app])
        mapped_comm = [
            (mapping[src], mapping[dst], weight)
            for src, dst, weight in create_app_communication(app)
        ]
    for src, dst, demand in mapped_comm:
        Gc.add_edge(int(src), int(dst), demand=float(demand))
    return Gc


def plot_communication_graph(
    Gc: nx.DiGraph,
    title: str = "Communication Graph",
    layer_cols: Optional[int] = 4,
):
    import matplotlib.pyplot as plt

    if layer_cols is None:
        pos = nx.spring_layout(Gc, seed=1)
    else:
        layers = {node: node // layer_cols for node in Gc.nodes()}
        nx.set_node_attributes(Gc, layers, "layer")
        pos = nx.multipartite_layout(Gc, subset_key="layer")

    nx.draw(Gc, pos, with_labels=True, connectionstyle="arc3,rad=0.1")
    nx.draw_networkx_edge_labels(
        Gc, pos, edge_labels=nx.get_edge_attributes(Gc, "demand")
    )
    plt.title(title)
    plt.show()


if __name__ == "__main__":
    for app in available_apps():
        Gc = createGc(app)  # rplace with app to run for now

        total_nodes = Gc.number_of_nodes()
        total_edges = Gc.number_of_edges()
        total_demand = sum(
            data["demand"] for _, _, data in Gc.edges(data=True)
        )

        print(
            f"{app}: Nodes={total_nodes}, Edges={total_edges}, Total Demand={total_demand}"
        )

        # Uncomment to visualize:
        plot_communication_graph(
            Gc,
            title=f"Communication Graph: {app}",
            layer_cols=ceil(sqrt(len(Gc))),
        )
