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
    from argparse import Namespace

    # below import only used to type check BaseChipletSystem.parent
    from chiplets.ChipletSystem import ChipletSystem
    from src.cpu.BaseCPU import BaseCPU
    from src.sim.System import System
    from src.sim.SubSystem import SubSystem
    from src.mem.ruby.system.RubySystem import RubySystem
    from src.mem.AbstractMemory import AbstractMemory
    from src.mem.ruby.slicc_interface.Controller import RubyController
    from src.mem.ruby.network.garnet.GarnetLink import *
    from src.mem.ruby.network.garnet.GarnetNetwork import *

from abc import ABC

from topologies.BaseTopology import BaseTopology


class BaseChipletSystem(ABC):
    """
    See `ChipletSystem` and `Chiplet`. `BaseChipletSystem`
    acts as a vessel for shared functionality between the two.
    Do not instantiate directly; instead use a subclass.
    """

    # TODO: should probably inherit `SubSystem` to prevent issues...
    # TODO: ...with (homogeneous) hierarchies

    # ruby/cache coherence protocol string
    protocol: str

    # the gem5 `System` SimObject that this object is part of
    _system: System

    # if full-system simulation mode is enabled in gem5
    _is_full_system: bool

    # topology class. used internally to create the topology.
    # `Ruby.py` currently requires a name (string) to be specified
    # to create the topology, instead of a class.
    # Since `Ruby.py` may be deprecated, we use a class to enable
    # easier refactoring later on if needed.
    _topology_cls: type[BaseTopology]

    _memory_cls: type[AbstractMemory]

    # list of *direct* nodes/children (`ChipletSystem`s or `Chiplet`s)
    nodes: list[BaseChipletSystem]

    # topology of the children for *only this layer of the hierarchy*
    topology: BaseTopology | None

    # default latency (in cycles) of the links between nodes
    # for this layer of abstraction (i.e., direct child nodes)
    default_inter_node_link_lat: int

    # same as above, but for the routers corresponding to the links
    default_inter_node_router_lat: int

    # parent `ChipletSystem` (if present)
    parent: ChipletSystem | None

    # list of nodes on the same hierarchical level of this node's parent
    # `ChipletSystem` to which this node has *direct* connections
    connected_nodes: list[BaseChipletSystem]

    # main router for this node/`Chiplet[System]`
    # used to make connections to parent and siblings
    _main_router: GarnetRouter

    # Garnet link (bidirectional pair/tuple) to/from parent (respectively)
    _parent_link: tuple[GarnetIntLink, GarnetIntLink]

    # list of Garnet link pairs for siblings
    _sibling_links: list[tuple[GarnetIntLink, GarnetIntLink]]

    # `GarnetNetwork` for this object
    _garnet_network: GarnetNetwork

    # class-local reference to `self.system.ruby`
    # (gem5 system SimObject's RubySystem)
    _ruby_system: RubySystem

    # `RubyController`s that correspond to this `Chiplet[System]`
    _ruby_controllers: list[RubyController]

    # `GarnetExtLink`s that connect this object to parent `RubyController`s
    _ruby_controller_links: list[GarnetExtLink]

    # unique ID for each
    __id = 0

    @classmethod
    def _generate_id(cls):
        cls.__id += 1
        return cls.__id - 1

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
            nodes (Sequence[BaseChipletSystem]):
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

        self._main_router = GarnetRouter(
            router_id=-1,  # temporary, is replaced later
            latency=self.default_inter_node_router_lat,
            # the following are default parameters
            # gem5 complains if I don't include these params
            # vcs_per_vnet=4,
        )

        self._sibling_links = []

        self._ruby_controllers = []
        self._ruby_controller_links = []

        self.id = BaseChipletSystem._generate_id()

    def __getitem__(self, index):
        return self.nodes[index]

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

        if not hasattr(self, "parent") or self.parent == None:
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

        # todo: get the GarnetIntLinks

        return None

    def _getRouterParent(self, router: GarnetRouter, startRoot: bool = True):
        # if we aren't already at the root, start search from there
        if startRoot and hasattr(self, "parent"):
            return self.getRoot()._getRouterParent(router, startRoot=False)

        # grab routers and search
        routers = self._garnet_network.routers
        if router in routers:
            return self
        else:
            for node in self.nodes:
                if issubclass(node.__class__, BaseChipletSystem):
                    res = node._getRouterParent(router, startRoot=False)
                    if res is not None:
                        return res
            return None

    def getAllLinksString(self):
        str = ""
        ext_links = self._garnet_network.ext_links
        for link in ext_links:
            int_parent = self._getRouterParent(link.int_node)
            str += f"ext link {link.link_id} ({link}):\n"
            str += f"    ext node: {link.ext_node}; "
            str += f"int node: {link.int_node} "
            if int_parent is not None:
                str += f"(c{int_parent.id}:"
                str += f"r{link.int_node.router_id})"
            str += "\n"
        int_links = self._garnet_network.int_links
        for link in int_links:
            src_parent = self._getRouterParent(link.src_node)
            dst_parent = self._getRouterParent(link.dst_node)
            str += f"int link {link.link_id} ({link}):\n"
            str += "    from "
            if src_parent is not None:
                str += f"c{src_parent.id}:"
            str += f"r{link.src_node.router_id} "
            str += "to "
            if dst_parent is not None:
                str += f"c{dst_parent.id}:"
            str += f"r{link.dst_node.router_id}"
            str += "\n"

        return str

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

    def to_string(self):
        raise NotImplementedError

    def getControllerCPU(
        self,
        controller: RubyController,
        l2_is_private: bool = False,
    ) -> BaseCPU | None:
        # todo: improve this
        # `RubyController` doesn't seem to provide any way to identify
        # the corresponding CPU reliably, so right now we assume that
        # cpu index corresponds to L1 (or L2, if private) controller index.
        name = f"{controller}"
        # example name: `<orphan System>.ruby.l1_cntrl0`
        if "l1_cntrl" in name or (l2_is_private and "l2_cntrl" in name):
            idx = name[-1]  # last digit
            return self._system.cpu[int(idx)]
        else:
            return None  # doesn't correspond to a particular CPU

    def _createTopology(self, options: Namespace):
        # for reference, this is what you would do for a flat topology
        if False:
            self.topology = self._topology_cls(
                self.getRoot()._meta_topology.nodes  # type: ignore
            )

        self.topology = self._topology_cls(
            self._ruby_controllers  # type: ignore
        )

        # if mesh we need to temporarily adjust CPU count option
        from topologies.Mesh_westfirst import Mesh_westfirst
        from topologies.Mesh_XY import Mesh_XY
        from topologies.MeshDirCorners_XY import MeshDirCorners_XY

        if (
            self._topology_cls is Mesh_westfirst
            or self._topology_cls is Mesh_XY
            or self._topology_cls is MeshDirCorners_XY
        ):
            # temp set CPU count option to number of nodes
            options.num_cpus = len(self.nodes)

        # * setup topology
        # latter 3 args are the literal classes for the
        # corresponding objects. they can technically be
        # based on either `GarnetNetwork` or `SimpleNetwork`,
        # but we're using Garnet and don't allow Simple.
        self.topology.makeTopology(
            options=options,
            network=self._garnet_network,
            IntLink=GarnetIntLink,
            ExtLink=GarnetExtLink,
            Router=GarnetRouter,
        )

        if (
            self._topology_cls is Mesh_westfirst
            or self._topology_cls is Mesh_XY
            or self._topology_cls is MeshDirCorners_XY
        ):
            # set CPU count option back to what it was
            options.num_cpus = len(self._system.cpu)
            # ? this might interact very strangely with
            # ? `registerTopology()`

        # if in SE mode, register topology with (faux) filesystem
        # only some topologies implement this (as of now, only
        # `Mesh_XY` and `MeshDirCorners_XY` do.)
        if not self._is_full_system:
            # notes: `Mesh_XY` requires
            # `options.num_cpus` and `options.mem_size`.
            # `MeshDirCorners_XY` requires only `options.mem_size`.
            self.topology.registerTopology(options)

        gn = self._garnet_network
        gn.routers = gn.routers + [self._main_router]

    def _biLinkGarnetRouters(
        self, router1: GarnetRouter, router2: GarnetRouter
    ):
        num_links = len(self._garnet_network.int_links) + len(
            self._garnet_network.ext_links
        )
        print(
            f"found {num_links} internal links for network {self._garnet_network}"
        )

        l12 = GarnetIntLink(
            link_id=num_links,
            src_node=router1,
            dst_node=router2,
            latency=self.default_inter_node_link_lat,
        )

        l21 = GarnetIntLink(
            link_id=num_links + 1,
            src_node=router2,
            dst_node=router1,
            latency=self.default_inter_node_link_lat,
        )

        # self._garnet_network.int_links.append(l12)
        # self._garnet_network.int_links.append(l21)
        self._garnet_network.int_links += l12
        self._garnet_network.int_links += l21

        return l12, l21

    def _connectParentGarnet(self, copy_links_to_parent: bool = True):
        if hasattr(self, "parent") and self.parent is not None:
            self._parent_link = self._biLinkGarnetRouters(
                self._main_router, self.parent._main_router
            )
            # if copy_links_to_parent:
            #     pgn = self.parent._garnet_network
            #     pgn.int_links.append(self._parent_link[0])
            #     pgn.int_links.append(self._parent_link[1])

    def _connectSiblingGarnet(self, sibl: BaseChipletSystem):
        l_to, l_from = self._biLinkGarnetRouters(
            self._main_router, sibl._main_router
        )
        self._sibling_links.append((l_to, l_from))

    def _connectRubyControllerGarnet(
        self, ctrl: RubyController, link_lat: int = -1
    ):
        if link_lat == -1:
            link_lat = self.default_inter_node_link_lat
        num_links = len(self._garnet_network.int_links) + len(
            self._garnet_network.ext_links
        )
        print(
            f"found {num_links} external links for network {self._garnet_network}"
        )
        link = GarnetExtLink(
            link_id=num_links,
            ext_node=ctrl,
            int_node=self._main_router,
            latency=link_lat,
        )
        self._garnet_network.ext_links += link
        self._ruby_controller_links.append(link)
        return link

    def _defineMainRouterParams(self):
        """
        For some reason gem5 hates it when these aren't automatically
        defined for `_main_router`, probably because it was instantiated
        seperately. Should just nee to be called before `m5.instantiate()`.
        """
        # gem5 will complain if params aren't defined for main router
        mr = self._main_router
        gn = self._garnet_network
        # mr.vcs_per_vnet = gn.vcs_per_vnet
        # mr.virt_nets = self._garnet_network.number_of_virtual_networks
        # mr.ni_flit_size = gn.ni_flit_size
        mr.eventq_index = gn.eventq_index

    def _connectHierarchyGarnet(self):
        """
        Should be called after `_createTopology()`.
        Adds the main router to the `GarnetNetwork` list and
        creates connections between the main routers of the current
        level of the hierarchy and its parents/siblings.
        """
        # topologies tend to overwrite garnet links/routers entirely
        # so just create our links (and add our main router) afterwards
        num_routers = len(self._garnet_network.routers)
        self._main_router.router_id = num_routers - 1
        self._garnet_network.routers.append(self._main_router)
        self._garnet_network.mainrouter = self._main_router  # load bearing
        self._connectParentGarnet()
        if hasattr(self, "parent") and self.parent is not None:
            for node in self.parent.nodes:
                if node is not self:
                    self._connectSiblingGarnet(node)
            for rc in self.parent._ruby_controllers:
                self._connectRubyControllerGarnet(rc)
        # setattr(
        #     self._garnet_network,
        #     f'mainrouterc{self.id}',
        #     self._main_router
        # )
        self._combineHierarchyGarnetNetworks(self._garnet_network)

    def _mergeGarnetNetworks(
        self, merge_into: GarnetNetwork, merge_from: GarnetNetwork
    ):
        mi_router_ct = len(merge_into.routers)
        mi_link_ct = len(merge_into.int_links) + len(merge_into.ext_links)
        for r in merge_from.routers:
            # ensure unique IDs in resulting network
            mi_router_ct += 1
            r.router_id = mi_router_ct
            merge_into.routers.append(r)

        for el in merge_from.ext_links:
            mi_link_ct += 1
            el.link_id = mi_link_ct
            merge_into.ext_links.append(el)

        for il in merge_from.int_links:
            mi_link_ct += 1
            il.link_id = mi_link_ct
            merge_into.int_links.append(il)

    def _combineHierarchyGarnetNetworks(self, merge_into: GarnetNetwork):
        """
        Call from parent `ChipletSystem` to combine all `GarnetNetwork`s
        after topologies have been created.

        Args:
            merge_into (GarnetNetwork):
                The `GarnetNetwork` to merge into. This should always be
                the `GarnetNetwork` of the root `ChipletSystem`, so that
                recursive calls continue merging properly.
        """
        for n in self.nodes:
            if hasattr(n, "_garnet_network"):
                self._mergeGarnetNetworks(merge_into, n._garnet_network)
            if hasattr(n, "nodes"):
                n._combineHierarchyGarnetNetworks(merge_into)
