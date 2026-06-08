import matplotlib.pyplot as plt
import networkx as nx


def create_mesh_2d(n_rows, n_cols):
    """
    Create a 2D mesh NoC as a directed graph with bidirectional links.
    Each node is numbered row-major: 0,1,2,...
    """
    G = nx.DiGraph()

    # Add nodes
    for r in range(n_rows):
        for c in range(n_cols):
            node_id = r * n_cols + c
            G.add_node(node_id, pos=(c, -r))  # store positions for plotting

    # Add edges (bi-directional)
    for r in range(n_rows):
        for c in range(n_cols):
            node = r * n_cols + c
            # Right neighbor
            if c < n_cols - 1:
                right = r * n_cols + (c + 1)
                G.add_edge(node, right)
                G.add_edge(right, node)
            # Down neighbor
            if r < n_rows - 1:
                down = (r + 1) * n_cols + c
                G.add_edge(node, down)
                G.add_edge(down, node)

    return G


n_rows, n_cols = 4, 4
mesh = create_mesh_2d(n_rows, n_cols)

# Visualize
nx.grid_2d_graph
pos = nx.get_node_attributes(mesh, "pos")
nx.draw(
    mesh,
    pos,
    with_labels=True,
    node_size=600,
    node_color="lightblue",
    arrowsize=20,
)
plt.title("2D Mesh NoC (Bidirectional)")
plt.show()
