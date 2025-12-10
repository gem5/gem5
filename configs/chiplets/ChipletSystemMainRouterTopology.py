from enum import Enum


class ChipletSystemMainRouterTopology(Enum):
    """
    `Chiplet[System]` is currently implemented so that nodes (or, in
    other words, the main routers) are connected via `GarnetIntLink`s.
    Thus, they cannot use the template topologies in `configs/topologies/`
    to generate a topology because those require the nodes to be
    `RubyController`s (which `GarnetRouter`s are not).
    Instead, there are presently two options for connecting chiplets.
    Either all cross-chiplet requests must travel through the parent
    ChipletSystem (`ParentOnly`), or the main routers of all nodes are
    connected to all other nodes on the same level of the hierarchy in
    a point-to-point topology (`Pt2Pt`).
    """

    ParentOnly = "ParentOnly"
    Pt2Pt = "Pt2Pt"
    # todo: implement a general mesh topolgoy
