from __future__ import annotations

from typing import (
    TYPE_CHECKING,
    Self,
    Sequence,
    Type,
)

from m5.defines import buildEnv  # type: ignore
from m5.objects import *
from m5.util import (
    addToPath,
    fatal,
)

if TYPE_CHECKING:
    # below import only used to type check BaseChipletSystem.parent
    from chiplets.ChipletSystem import ChipletSystem

    from src.sim.System import System
    from src.sim.SubSystem import SubSystem
    from src.mem.AbstractMemory import AbstractMemory
    from src.mem.ruby.network.garnet.GarnetLink import *
    from src.mem.ruby.network.garnet.GarnetNetwork import *

from abc import ABC

from topologies.BaseTopology import BaseTopology


class BaseChipletSystem(ABC):
    """
    See ChipletSystem and Chiplet. BaseChipletSystem acts
    as a vessel for shared functionality between the two.
    Do not instantiate directly; instead use a subclass.
    """

    # TODO: should probably inherit SubSystem to prevent issues...
    # TODO: ...with (homogeneous) hierarchies

    # ruby/cache coherence protocol string
    protocol: str

    # the gem5 `System` SimObject that this object is part of
    _system: System

    # if full-system simulation mode is enabled in gem5
    _is_full_system: bool

    # topology class. used internally to create the topology.
    # Ruby.py currently requires a name (string) to be specified
    # to create the topology, instead of a class.
    # Since Ruby.py may be deprecated, we use a class to enable
    # easier refactoring later on if needed.
    _topology_cls: type[BaseTopology]

    _memory_cls: type[AbstractMemory]

    # list of *direct* nodes/children (ChipletSystems or Chiplets)
    nodes: list[BaseChipletSystem]

    # topology of the children for *only this layer of the hierarchy*
    topology: BaseTopology | None

    # default latency (in cycles) of the links between nodes
    # for this layer of abstraction (i.e., direct child nodes)
    default_inter_node_link_lat: int

    # same as above, but for the routers corresponding to the links
    default_inter_node_router_lat: int

    # parent ChipletSystem (if present)
    parent: ChipletSystem | None

    # list of nodes on the same hierarchical level of this node's parent
    # ChipletSystem to which this node has *direct* connections
    connected_nodes: list[BaseChipletSystem]

    # `GarnetNetwork` for this object
    _garnet_network: GarnetNetwork

    def __init__(
        self,
        system: System,
        full_system: bool,
        TopologyClass: type[BaseTopology],
        MemoryClass: type[AbstractMemory],
        nodes: Sequence[BaseChipletSystem],
        inter_node_link_lat: int,
        inter_node_router_lat: int,
    ):
        """
        Create a `BaseChipletSystem`.
        This should only be invoked by the constructors of
        `ChipletSystem`, `Chiplet`, or other subclasses.

        Args:
            system (System):
                The gem5 `System` SimObject that this `BaseChipletSystem`
                is part of.
            full_system (bool):
                If gem5 is running in full-system simulation mode.
            TopologyClass (Type[BaseTopology]):
                The class, derived from `BaseTopology`, corresponding
                to the topology to be used in this `ChipletSystem`.
                Only the root `ChipletSystem` needs to specify this.
            MemoryClass (Type[AbstractMemory]):
                The class corresponding to the memory configuration
                for the gem5 `System` this `ChipletSystem` is part of.
            nodes (list[BaseChipletSystem]):
                Optional list of nodes. May be passed to constructor
                or added separately using `addNode()`.
            inter_node_link_lat (int):
                The default latency (in cycles) of the links between
                nodes for this layer of abstraction.
            inter_node_router_lat (int):
                The default latency (in cycles) of the routers
                corresponding to the the links between nodes
                for this layer of abstraction.
        """
        # SubSystem.__init__(self)

        self.protocol = buildEnv["PROTOCOL"]

        self.nodes = list(nodes)

        self._system = system
        self._is_full_system = full_system

        self._topology_cls = TopologyClass
        self._memory_cls = MemoryClass

        self.topology = None  # created in `createSystem()`

        self.default_inter_node_link_lat = inter_node_link_lat
        self.default_inter_node_router_lat = inter_node_router_lat
        self.connected_nodes = []

    def addNode(
        self,
    ):
        """
        Add a node/child to this `Chiplet[System]`
        """

        # todo: implement
        raise NotImplementedError

    def getRoot(self):
        """
        Retrieve a reference to the root `ChipletSystem`,
        i.e., the `ChipletSystem` at the top of the hierarchy.
        """

        if self.parent == None:
            return self
        else:
            return self.parent.getRoot()

    def getIntLink(
        self, node1: BaseChipletSystem, node2: BaseChipletSystem
    ) -> GarnetIntLink | None:
        """
        Retrieve the Garnet link object, if one exists, between the two
        specified nodes, which must be children of this `Chiplet[System]`.
        Note that the `Chiplet[System]` abstracts the `GarnetIntLink`,
        since an actual `GarnetIntLink` is between two routers, not two
        `Chiplet[System]`s directly.

        Args:
            node1 (BaseChipletSystem): One end of the link.
            node2 (BaseChipletSystem): The other end of the link.
        """

        if (
            (len(self.nodes) == 0)
            or node1 not in self.nodes
            or node2 not in self.nodes
        ):
            # no nodes, or one or both nodes are not children of `self`
            return None

        # todo: get the GarnetIntLink

        return None

    def connect(self, node: BaseChipletSystem):
        """
        Register a connection between this `ChipletSystem` or `Chiplet`
        and another `ChipletSystem` or `Chiplet` sharing the same parent.

        Args:
            node (BaseChipletSystem): The node to connect to.
        """

        if self.parent is None:
            fatal(
                "This BaseChipletSystem has no parent, "
                "but the user attempted to connect it "
                "to another BaseChipletSystem."
            )

        if self.parent != node.parent:
            fatal(
                "User attempted to connect two BaseChipletSystems "
                "that do not share a parent ChipletSystem."
            )

        if node not in self.connected_nodes:
            self.connected_nodes.append(node)

        if self not in node.connected_nodes:
            node.connected_nodes.append(self)

        # ? instantiate a GarnetLink if one is not already present?

    def connectChildren(
        self, node1: BaseChipletSystem, node2: BaseChipletSystem
    ):
        """
        Register a connection between two specified `ChipletSystem`s
        or `Chiplet`s *which are children of this `ChipletSystem`.*

        Args:
            node1 (BaseChipletSystem): The node to connect from.
            node2 (BaseChipletSystem): The node to connect to.
        """

        node1.connect(node2)
