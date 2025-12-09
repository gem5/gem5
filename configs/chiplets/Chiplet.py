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
        inter_node_router_lat: int,
        cores: Sequence[BaseCPU],
    ):
        super().__init__(
            # self,
            system=system,
            full_system=full_system,
            TopologyClass=TopologyClass,
            MemoryClass=MemoryClass,
            nodes=cores,
            inter_node_link_lat=inter_node_link_lat,
            inter_node_router_lat=inter_node_router_lat,
        )

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
        self._garnet_network = FakeGarnetNetwork()

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
                self._ruby_controllers.append(c)
                # print(
                #     f"Chiplet {self.id} found controller {c} "
                #     f"(v{c.version})"
                # )

        #! create topology
        # need to populate `self._ruby_controllers` first
        self._createTopology(options, self._ruby_controllers)

    def createChipletOld(
        self,
        options: Namespace,
        l2_is_private: bool,
    ):
        import traceback

        fatal(f"do not use this function! tb: {traceback.print_stack()}")
        # print(f"createChiplet() called for chiplet ID {self.id}")

        # todo: currently this is basically just duplicate code from...
        # todo: ...`ChipletSystem`, refactor somehow
        # Create the `GarnetNetwork` for this level of the hierarchy
        self._garnet_network = GarnetNetwork(
            ruby_system=self.getRoot()._ruby_system,
            topology=self._topology_cls.__name__,
            routers=[],
            ext_links=[],
            int_links=[],
            netifs=[],
            ignore_mesh_chk=True,
        )

        # set `SimObject` parent hierarchy
        # make sure not to set root `GarnetNetwork` parent
        # if we did, it would break Ruby because it expects
        # the gem5 `System` to have a `.network` param
        if not self._garnet_network.has_parent():
            # print(f"setting gem5 parent for GarnetNetwork of "
            #       f"Chiplet ID {self.id}")
            self._garnet_network.set_parent(self, "network")

        root = self.getRoot()
        if root == self:
            panic("Chiplet is root, this should never happen!")
        all_controllers = root._meta_topology.nodes  # type: ignore

        for c in all_controllers:
            controller_cpu = self.getControllerCPU(c, l2_is_private)

            if controller_cpu is None:
                continue
            elif controller_cpu in self._cores:
                self._ruby_controllers.append(c)
                # print(
                #     f"Chiplet {self.id} found controller {c} "
                #     f"(v{c.version})"
                # )

        #! create topology
        # need to populate `self._ruby_controllers` first
        self._createTopology(options, self._ruby_controllers)

        # self._defineMainRouterParams()

        self._connectHierarchyGarnet()

        # * initialize network
        # `Network.py` does this without any major obstacles to
        # what we're doing in `ChipletSystem` as of now
        #! effectively requires `.int_links` and `.ext_links` of
        #! our `GarnetNetwork` to be set before this
        # todo: we might have to adapt this so that we can add ...
        # todo: ...links after calling `createSystem()`, not sure
        options.network = "garnet"  # only required by `Network.py`
        init_network(
            options=options,
            network=self._garnet_network,
            InterfaceClass=GarnetNetworkInterface,
        )

        self._fixAllGarnetObjectParams()

        # if hasattr(self._garnet_network, "number_of_virtual_networks"):
        #     print(f"Chiplet{self.id}'s GarnetNetwork has param "
        #           f"number_of_virtual_networks = "
        #           f"{self._garnet_network.number_of_virtual_networks}")
        # else:
        #     print(f"Chiplet{self.id}'s GarnetNetwork has NO param "
        #           f"number_of_virtual_networks!")

        # self._defineMainRouterParams()

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
