from __future__ import annotations

from typing import (
    TYPE_CHECKING,
    Self,
    Sequence,
    Type,
)

from chiplets.BaseChipletSystem import (
    BaseChipletSystem,
    FakeGarnetNetwork,
)
from network.Network import init_network
from topologies.BaseTopology import BaseTopology

from m5.defines import buildEnv  # type: ignore
from m5.objects import *
from m5.util import (
    fatal,
    panic,
)

if TYPE_CHECKING:
    from argparse import Namespace

    from src.cpu.BaseCPU import BaseCPU
    from src.mem.AbstractMemory import AbstractMemory
    from src.mem.ruby.network.garnet.GarnetNetwork import *
    from src.mem.ruby.system.RubySystem import RubySystem
    from src.sim.SubSystem import SubSystem
    from src.sim.System import System


class Chiplet(BaseChipletSystem):
    """
    A Chiplet denotes a node at the innermost level of a ChipletSystem.
    A Chiplet does not have any child ChipletSystems; all of its children
    are processor objects (cores) -- in other words, "leaf" nodes.
    """

    type = "Chiplet"
    abstract = False

    def __init__(
        self,
        system: System,
        full_system: bool,
        TopologyClass: type[BaseTopology],
        MemoryClass: type[AbstractMemory],
        inter_node_link_lat: int,
        intra_node_link_lat: int,
        node_main_router_lat: int,
        cores: Sequence[BaseCPU],
        cache_controller_link_lat: int = -1,
        cache_controller_router_lat: int = -1,
        dir_controller_link_lat: int = -1,
        dir_controller_router_lat: int = -1,
    ):
        """
        Instantiate a `Chiplet` object.

        Args:
            system (System):
                The gem5 `System` SimObject that this `Chiplet`
                is part of.
            full_system (bool):
                If gem5 is running in full-system simulation mode.
            TopologyClass (Type[BaseTopology]):
                The class, derived from `BaseTopology`, corresponding
                to the topology to be used in this level of the
                `ChipletSystem` hierarchy.
            MemoryClass (Type[AbstractMemory]):
                The class corresponding to the memory configuration
                for the gem5 `System` this `ChipletSystem` is part of.
            inter_node_link_lat (int):
                The default latency (in cycles) of the links between
                nodes for this layer of abstraction.
            intra_node_link_lat (int):
                The default latency (in cycles) of the links between
                each routers in the same level of the hierarchy;
                e.g., between the main router and the router corresponding
                to a directory controller or cache controller.
            node_main_router_lat (int):
                The default latency (in cycles) of the main router
                for this `Chiplet[System]`. This is also the router
                corresponding to the the links between nodes
                for this layer of abstraction.
            nodes (Sequence[BaseChipletSystem]):
                Optional list of nodes. May be passed to constructor
                or added separately using `addNode()`.
            cache_controller_link_lat (int, optional):
                The default latency (in cycles) of the links between
                each cache controller and its corresponding router.
                Defaults to `inter_node_link_lat`.
            cache_controller_router_lat (int, optional):
                The default latency (in cycles) of the router
                corresponding to each cache controller.
                Defaults to `inter_node_router_lat`.
            dir_controller_link_lat (int, optional):
                The default latency (in cycles) of the links between
                each directory controller and its corresponding router.
                Defaults to `inter_node_link_lat`.
            dir_controller_router_lat (int, optional):
                The default latency (in cycles) of the router
                corresponding to each directory controller.
                Defaults to `inter_node_router_lat`.
        """
        super().__init__(
            # self,
            system=system,
            full_system=full_system,
            TopologyClass=TopologyClass,
            MemoryClass=MemoryClass,
            nodes=cores,
            inter_node_link_lat=inter_node_link_lat,
            node_main_router_lat=node_main_router_lat,
            intra_node_link_lat=intra_node_link_lat,
            cache_controller_link_lat=cache_controller_link_lat,
            cache_controller_router_lat=cache_controller_router_lat,
            dir_controller_link_lat=dir_controller_link_lat,
            dir_controller_router_lat=dir_controller_router_lat,
        )

        self._garnet_network = FakeGarnetNetwork()

    # * alias Chiplet._cores to Chiplet._nodes
    def get_cores(self):
        return self._nodes

    def set_cores(self, cores):
        self._nodes = cores

    _cores = property(get_cores, set_cores)

    def createChiplet(
        self,
        options: Namespace,
        l2_is_private: bool,
    ):
        # self._garnet_network = FakeGarnetNetwork()

        root = self.getRoot()
        if root == self:  # could use `self.isRoot()` but alr have `root`
            panic("Chiplet is root, this should never happen!")

        # at this time, `create_system()` should've been called by
        # the root `ChipletSystem`, so the meta topology should be
        # populated with all controllers
        all_controllers = root._meta_topology.nodes  # type: ignore

        # now let's figure out which controllers belong to this Chiplet
        for c in all_controllers:
            controller_cpu = self.getControllerCPU(c, l2_is_private)

            if controller_cpu is None:
                continue
            elif controller_cpu in self._cores:
                self._topology_controllers.append(c)
                # print(
                #     f"Chiplet {self.id} found controller {c} "
                #     f"(v{c.version})"
                # )

        #! create topology
        # need to populate `self._topology_controllers` first
        self._createTopology(options, self._topology_controllers)

    def to_string(self):
        import textwrap

        parent_id = "None"
        if hasattr(self, "_parent_sys") and self._parent_sys is not None:
            parent_id = self._parent_sys.id
        return f"""{self.__class__.__name__} (ID: {self.id}) [
            Parent ID: {parent_id}
            Protocol: {self.protocol}
            Topology: {self._topology_cls.__name__}
            Connections (IDs): {[int(f"{n.id}") for n in self._connected_nodes]}
            Cores: [
        {textwrap.indent(
            "\n        ".join([f"{n}\n" for n in self._nodes]),
            "        "
        ).rstrip('\n')
        }
            ]:{self.id}c
        ]:{self.id}"""
