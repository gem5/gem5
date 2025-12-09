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
    panic,
    warn,
)

if TYPE_CHECKING:
    from argparse import Namespace

    # below import only used to type check BaseChipletSystem.parent
    from chiplets.ChipletSystem import (
        ChipletSystem,
        ChipletGarnetObjectInfo,
    )
    from src.cpu.BaseCPU import BaseCPU
    from src.sim.System import System
    from src.sim.SubSystem import SubSystem
    from src.mem.ruby.system.RubySystem import RubySystem
    from src.mem.AbstractMemory import AbstractMemory
    from src.mem.ruby.slicc_interface.Controller import RubyController
    from src.mem.ruby.system.Sequencer import RubySequencer
    from src.mem.ruby.network.garnet.GarnetLink import *
    from src.mem.ruby.network.garnet.GarnetNetwork import *

from topologies.BaseTopology import BaseTopology


class FakeGarnetNetwork:
    """
    Used to collect routers and links from topologies without
    actually instantiating a `GarnetNetwork` C++ object.
    !! It is assumed that topologies only use the `network`
    !! param to add routers and links to the `GarnetNetwork`.
    This is currently true for all existing topologies, but
    if it is ever not true for a topology you use, it will break.
    """

    routers: list[GarnetRouter]
    ext_links: list[GarnetExtLink]
    int_links: list[GarnetIntLink]

    def __init__(
        self,
        routers: list[GarnetRouter] = [],
        ext_links: list[GarnetExtLink] = [],
        int_links: list[GarnetIntLink] = [],
    ):
        self.routers = routers
        self.ext_links = ext_links
        self.int_links = int_links


class BaseChipletSystem(SubSystem):
    """
    See `ChipletSystem` and `Chiplet`. `BaseChipletSystem`
    acts as a vessel for shared functionality between the two.
    Do not instantiate directly; instead use a subclass.
    """

    type = "BaseChipletSystem"
    abstract = True

    # ID for this `Chiplet[System]`
    id = Param.Int("ID in relation to other Chiplet[System]s")

    # ruby/cache coherence protocol string
    protocol = Param.String("String name corresponding to Ruby protocol")

    # *
    # * Latency parameters
    # *

    # default latency (in cycles) of the links between nodes
    # for this layer of abstraction (i.e., direct child nodes)
    default_inter_node_link_lat = Param.Int(
        "default latency (in cycles) of the links between nodes "
        "in this level of the hierarchy"
    )

    # same as above, but for the routers corresponding to the links
    default_inter_node_router_lat = Param.Int(
        "default latency (in cycles) of the routers corresponding to "
        "the links between nodes in this level of the hierarchy"
    )

    # *
    # * Attributes related to gem5 logistics
    # *

    # the gem5 `System` SimObject that this object is part of
    _system: System

    # if full-system simulation mode is enabled in gem5
    _is_full_system: bool

    # memory class of the gem5 system
    _memory_cls: type[AbstractMemory]

    # *
    # * Topology and Nodes
    # *

    # topology of the children for *only this layer of the hierarchy*
    _topology: BaseTopology | None

    # topology class. used internally to create the topology.
    # legacy Ruby (`Ruby.py`) requires a name (string) to be specified
    # to create the topology; this is obtained from the class.
    # hierarchical chiplet construction uses the class directly.
    _topology_cls: type[BaseTopology]

    # list of *direct* nodes/children (`ChipletSystem`s or `Chiplet`s)
    _nodes: list[BaseChipletSystem] | list[BaseCPU]

    # parent `ChipletSystem` (if present)
    _parent_sys: ChipletSystem | None

    # list of nodes on the same hierarchical level of this node's parent
    # `ChipletSystem` to which this node has *direct* connections
    _connected_nodes: list[BaseChipletSystem]

    # *
    # * Links, Routers, and Controllers
    # *

    # main router for this node/`Chiplet[System]`
    # used to make connections to parent and siblings
    _main_router: GarnetRouter

    # Garnet link (bidirectional pair/tuple) to/from parent (respectively)
    _parent_link: tuple[GarnetIntLink, GarnetIntLink]

    # list of Garnet link pairs for siblings
    _sibling_links: list[tuple[GarnetIntLink, GarnetIntLink]]

    # `GarnetNetwork` for this object
    # ! will always be fake except for root after `createSystem()` is done
    _garnet_network: GarnetNetwork | FakeGarnetNetwork

    # `RubyController`s that correspond to this `Chiplet[System]`
    # includes all controllers used to create the topology
    _ruby_controllers: list[RubyController]

    # `GarnetExtLink`s that connect this object to parent `RubyController`s
    _ruby_controller_links: list[GarnetExtLink]
    # _ruby_controller_router_map: list[tuple[RubyController, GarnetRouter]]

    # directory controllers in this `Chiplet[System]`
    # these are returned by `create_system()` of the Ruby protocol
    _dir_controllers: list[RubyController]
    # _dir_controller_link_map: list[tuple[RubyController, GarnetExtLink]]
    # _dir_controller_router_map: list[tuple[RubyController, GarnetRouter]]

    # controllers that are not directory controllers
    # e.g., LLC controller
    _non_dir_controllers: list[RubyController]

    # Ruby CPU sequencers for this `Chiplet[System]`
    # includes sequencers for all CPUs
    _cpu_sequencers: list[RubySequencer]

    # *
    # * Other
    # *

    # used to generate a unique ID for each `Chiplet[System]`
    # note that this is global to all classes that inherit from
    # `BaseChipletSystem`, regardless of their actual class.
    # do not override `_generate_id()`.
    __id = 0

    @classmethod
    def _generate_id(cls):
        """
        Used to generate a unique ID for each `Chiplet[System]`.
        The ID is unique among all instances of classes that
        inherit from `BaseChipletSystem`.
        !! Do not override this method !!

        Returns:
            int: an ID for the `Chiplet[System]`.
        """
        cls.__id += 1
        return cls.__id - 1

    # *
    # * Constructor
    # *

    def __init__(
        self,
        system: System,
        full_system: bool,
        TopologyClass: type[BaseTopology],
        MemoryClass: type[AbstractMemory],
        nodes: Sequence[BaseChipletSystem] | Sequence[BaseCPU],
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
            nodes (Sequence[BaseChipletSystem] | Sequence[BaseCPU]):
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
        super().__init__()

        self.protocol = buildEnv["PROTOCOL"]

        self._nodes = list(nodes)

        self._system = system
        self._is_full_system = full_system

        self._topology_cls = TopologyClass
        self._memory_cls = MemoryClass

        self._topology = None  # created in `createSystem()`

        self.default_inter_node_link_lat = inter_node_link_lat
        self.default_inter_node_router_lat = inter_node_router_lat
        self._connected_nodes = []

        self._main_router = GarnetRouter(
            router_id=0,  # some hijinks occur with this later
            latency=self.default_inter_node_router_lat,
            # the following are default parameters
            # gem5 complains if I don't include these params
            vcs_per_vnet=4,
        )

        self._sibling_links = []

        self._ruby_controllers = []
        self._ruby_controller_links = []
        # self._ruby_controller_router_map = []

        self._dir_controllers = []
        # self._dir_controller_link_map = []
        # self._dir_controller_router_map = []

        self._non_dir_controllers = []

        self.id = BaseChipletSystem._generate_id()

    def __getitem__(self, index):
        return self._nodes[index]

    def addNode(
        self,
    ):
        """
        Add a node/child to this `Chiplet[System]`
        """

        # todo: implement
        warn(
            "As of now, Chiplet[System] nodes must be specified "
            "at instantiation time in the constructor."
        )
        raise NotImplementedError

    # *
    # * Root ChipletSystem Methods
    # *

    def getRoot(self) -> ChipletSystem:
        """
        Retrieve a reference to the root `ChipletSystem`,
        i.e., the `ChipletSystem` at the top of the hierarchy.
        """

        if (
            not hasattr(self, "_parent_sys") or self._parent_sys == None
        ) and isinstance(self, ChipletSystem):
            return self
        elif self._parent_sys is not None:
            return self._parent_sys.getRoot()
        else:  # major failure
            if not hasattr(self, "_parent_sys"):
                par_str = " attribute doesn't exist"
            else:  # is none
                par_str = " was None"
            was_cs_or_not_str = ""
            if not isinstance(self, ChipletSystem):
                was_cs_or_not_str = "not "
            fatal(
                f"Failed to retrieve root ChipletSystem for "
                f"{self.__class__.__name__} ID {self.id}. "
                f"_parent_sys {par_str}. "
                f"Self was {was_cs_or_not_str}ChipletSystem."
            )
            # below will never be reached, just to satisfy type checker
            assert self is ChipletSystem
            return self

    def isRoot(self):
        """
        Determine if this `Chiplet[System]` is the root or not.
        """
        return not hasattr(self, "_parent_sys") or self._parent_sys == None

    # *
    # * Debugging Methods
    # *

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

    def to_string(self):
        raise NotImplementedError

    # *
    # * Network and Controller Classification
    # *

    def _getRouterParent(self, router: GarnetRouter, startRoot: bool = True):
        # if we aren't already at the root, start search from there
        if startRoot and hasattr(self, "_parent_sys"):
            return self.getRoot()._getRouterParent(router, startRoot=False)

        # grab routers and search
        routers = self._garnet_network.routers
        if router in routers:
            return self
        else:
            for node in self._nodes:
                if issubclass(node.__class__, BaseChipletSystem):
                    res = node._getRouterParent(router, startRoot=False)
                    if res is not None:
                        return res
            return None

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

    def isDirController(self, controller: RubyController):
        name = f"{controller}"
        # example name: `<orphan System>.ruby.dir_cntrl0`
        return "dir_cntrl" in name

    # *
    # * `_createTopology()`
    # *

    def _createTopology(
        self,
        options: Namespace,
        controllers: list[RubyController],
    ):
        """
        Instantiate and make the topology corresponding to
        `self._topology_cls` (should've been specified in constructor),
        using the specified `controllers`.

        ! also connects all the passed `controllers` to the
        ! main router of this `Chiplet[System]`

        In the hierarchy scheme (using `_createChipletHierarchy()`),
        the topology is used with a `FakeGarnetNetwork` to create the
        topological structure of each node in the hierarchy without
        instantiating an actual `GarnetNetwork`. Then all routers and
        links are collected and instantiated into one root network.
        See `_createChipletHierarchy()` for more details.

        Args:
            options (Namespace): _description_
            controllers (list[RubyController]):
                This should usually be `self._ruby_controllers`.
                May or may not include the directory controllers
                depending on what was passed to `createSystem()`.
        """
        # for reference, this is what you would do for a flat topology
        if False:
            self._topology = self._topology_cls(
                self.getRoot()._meta_topology.nodes  # type: ignore
            )

        # actually instantiate the topology
        self._topology = self._topology_cls(controllers)  # type: ignore

        # if mesh, we need to temporarily adjust CPU count option
        # so that the topology is generated correctly
        # and its assertions don't fail.
        # mesh topology understandably assumes it is in a vacuum
        # and that number of routers = number of CPUs
        from topologies.Mesh_westfirst import Mesh_westfirst
        from topologies.Mesh_XY import Mesh_XY
        from topologies.MeshDirCorners_XY import MeshDirCorners_XY

        if (
            self._topology_cls is Mesh_westfirst
            or self._topology_cls is Mesh_XY
            or self._topology_cls is MeshDirCorners_XY
        ):
            # temp set CPU count option to number of nodes
            options.num_cpus = len(self._nodes)

        if not isinstance(self._garnet_network, FakeGarnetNetwork):
            warn(
                "_createTopology() called, but self._garnet_network "
                "is not a FakeGarnetNetwork. Either using legacy ruby, "
                "user is calling methods in places they shouldn't be "
                "called, or something is very broken."
            )

        # * setup topology
        # latter 3 args are the literal classes for the
        # corresponding objects. they can technically be
        # from either `GarnetNetwork` or `SimpleNetwork`,
        # but we're using Garnet and don't allow Simple.
        self._topology.makeTopology(
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
            self._topology.registerTopology(options)

        # grab reference to my network
        # is ok (and expected) for this to be a `FakeGarnetNetwork`
        # since all we care about here is iterating and adding
        gnw = self._garnet_network

        # iterate through ext_links and map controllers to routers
        # note that this is (one or more)-to-one, i.e., more than
        # one controller can map to a given router.
        # the point of this is t

        # ! connect to main router
        for r in gnw.routers:
            print(
                f"Linking Routers with main for "
                f"{self.__class__.__name__} ID {self.id}"
            )
            self._biLinkGarnetRouters(self._main_router, r)

        # * add main router to my routers
        gnw.routers += self._main_router

        for c in controllers:
            r = self._findRouterFromController(c, False)
            if r is not None:
                print(f"    controller {c} :: router {r.router_id} {r}")

    # *
    # * Low-Level Network Connection/Instantiation Methods
    # *

    def _findExtLinkFromController(
        self,
        controller: RubyController,
        searchFromRoot: bool = True,
    ) -> GarnetExtLink | None:
        if searchFromRoot and not self.isRoot():
            return self.getRoot()._findExtLinkFromController(controller, False)
            # don't need to set searchFromRoot = False but why not

        ext_links = self._garnet_network.ext_links
        for ext_link in ext_links:
            if controller == ext_link.ext_node:
                return ext_link

        return None

    def _findRouterFromController(
        self,
        controller: RubyController,
        searchFromRoot: bool = True,
    ) -> GarnetRouter | None:
        ext_link = self._findExtLinkFromController(controller, searchFromRoot)
        if ext_link is not None:
            return ext_link.int_node
        else:
            return None

    def _findIntLinkFromRouters(
        self,
        routerFrom: GarnetRouter,
        routerTo: GarnetRouter,
        searchFromRoot: bool = True,
    ) -> GarnetIntLink | None:
        if searchFromRoot and not self.isRoot():
            return self.getRoot()._findIntLinkFromRouters(
                routerFrom, routerTo, False
            )
            # don't need to set searchFromRoot = False but why not

        int_links = self._garnet_network.int_links
        for int_link in int_links:
            if (
                int_link.src_node == routerFrom
                and int_link.dst_node == routerTo
            ):
                return int_link
        return None

    def _biLinkGarnetRouters(
        self,
        router1: GarnetRouter,
        router2: GarnetRouter,
        # auto_reg_parent: bool = True,
    ):
        num_links = len(self._garnet_network.int_links) + len(
            self._garnet_network.ext_links
        )
        # print(
        #     f"found {num_links} links for network {self._garnet_network}"
        # )

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

        # TODO NEW HIERARCHY: remove this
        # if auto_reg_parent:
        #     l12.set_parent(self._garnet_network, f"int_links{num_links}")
        #     l21.set_parent(self._garnet_network, f"int_links{num_links + 1}")

        return l12, l21

    def _connectParentGarnet(self, copy_links_to_parent: bool = True):
        if hasattr(self, "_parent_sys") and self._parent_sys is not None:
            self._parent_link = self._biLinkGarnetRouters(
                self._main_router, self._parent_sys._main_router
            )
            # if copy_links_to_parent:
            #     pgn = self.parent._garnet_network
            #     pgn.int_links.append(self._parent_link[0])
            #     pgn.int_links.append(self._parent_link[1])

    def _connectSiblingGarnet(self, sibl: BaseChipletSystem):
        warn("This may cause unexpected behavior.")
        l_to, l_from = self._biLinkGarnetRouters(
            self._main_router, sibl._main_router
        )
        self._sibling_links.append((l_to, l_from))

    def _connectChildrenGarnet(
        self, node1: BaseChipletSystem, node2: BaseChipletSystem
    ):
        """
        Don't call this directly; instead call `connectChildren()`.
        This function doesn't do any checks (is relatively unsafe).

        Args:
            node1 (BaseChipletSystem): The node to connect from.
            node2 (BaseChipletSystem): The node to connect to.
        """
        l12, l21 = self._biLinkGarnetRouters(
            node1._main_router, node2._main_router
        )
        node1._sibling_links.append((l12, l21))
        node2._sibling_links.append((l12, l21))

    def _connectRubyControllerGarnet(
        self,
        ctrl: RubyController,
        router: GarnetRouter | None = None,
        link_lat: int = -1,
        auto_reg_parent: bool = True,
    ):
        """
        Creates and registers a `GarnetExtLink` between the specified
        `RubyController` object and the specified `GarnetRouter`
        (defaults to the main router for this `Chiplet[System]`).

        Args:
            ctrl (RubyController):
                The `RubyController` to link from.
                This is the external node of the resulting `GarnetExtLink`.
            router (GarnetRouter, optional):
                The `GarnetRouter` to link the controller to.
                This is the internal node of the resulting `GarnetExtLink`.
                Defaults to None, which results in using the main router.
                Usually this should be specified only when creating a link
                to a directory controller, which does things differently.
            link_lat (int, optional):
                The link latency for the resulting `GarnetExtLink`.
                Defaults to -1, which results in using the default link
                latency stored in this `Chiplet[System]`.
            auto_reg_parent (bool, optional):
                Whether to automatically register the parent of the
                resultant `GarnetExtLink`. Defaults to `True`, and usually
                it should be left that way.

        Returns:
            GarnetExtLink:
                The `GarnetExtLink` between the specified `RubyController`
                and `GarnetRouter`.
        """

        # default to configured inter node link latency if not specified
        if link_lat == -1:
            link_lat = self.default_inter_node_link_lat

        # use main router if not specified
        if router is None:
            router = self._main_router

        num_links = len(self._garnet_network.int_links) + len(
            self._garnet_network.ext_links
        )
        # print(
        #     f"found {num_links} links for network {self._garnet_network}"
        # )
        link = GarnetExtLink(
            link_id=num_links,
            ext_node=ctrl,
            int_node=router,
            latency=link_lat,
        )

        self._garnet_network.ext_links += link

        # TODO NEW HIERARCHY: remove this
        # if self.isDirController(ctrl):
        #     self._dir_controller_link_map.append(link)
        # else:
        #     self._ruby_controller_links.append(link)

        # TODO NEW HIERARCHY: remove this
        # if auto_reg_parent:
        #     link.set_parent(self._garnet_network, f"ext_links{num_links}")
        return link

    def _getNodeLinks(
        self, node1: BaseChipletSystem, node2: BaseChipletSystem
    ) -> tuple[GarnetIntLink | None, GarnetIntLink | None]:
        """
        Retrieve the Garnet link objects, if they exist, between the two
        specified nodes, which must be children of this `Chiplet[System]`.
        Note that the `Chiplet[System]` abstracts the `GarnetIntLink`,
        since an actual `GarnetIntLink` is between two routers, not two
        `Chiplet[System]`s directly.

        Args:
            node1 (BaseChipletSystem): One end of the link.
            node2 (BaseChipletSystem): The other end of the link.
        """

        if (
            (len(self._nodes) == 0)
            or node1 not in self._nodes
            or node2 not in self._nodes
        ):
            # no nodes, or one or both nodes are not children of `self`
            warn(
                f"_getNodeLinks() was called, but either there are no "
                f"nodes (actual len = {len(self._nodes)}), or one of "
                f"the nodes is not a child of this Chiplet[System] "
                f"(node1 is child? {node1 in self._nodes}) "
                f"(node2 is child? {node2 in self._nodes})."
            )
            return (None, None)

        n1r = node1._main_router
        n2r = node2._main_router

        l12 = self._findIntLinkFromRouters(n1r, n2r)
        l21 = self._findIntLinkFromRouters(n2r, n1r)

        return (l12, l21)

    # *
    # * High-Level Chiplet-to-Chiplet Connection Methods
    # *

    def connect(self, node: BaseChipletSystem):
        """
        Register a connection between this `ChipletSystem` or `Chiplet`
        and another `ChipletSystem` or `Chiplet` sharing the same parent.

        Args:
            node (BaseChipletSystem): The node to connect to.
        """

        if self._parent_sys is None:
            fatal(
                f"This {self.__class__.__name__} has no parent, "
                f"but the user attempted to connect it "
                f"to another {node.__class__.__name__}."
            )
        assert self._parent_sys is not None

        if self._parent_sys != node._parent_sys:
            fatal(
                "User attempted to connect two nodes "
                "that do not share a parent ChipletSystem."
            )

        self._parent_sys.connectChildren(self, node)

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

        if node1 not in self._nodes:
            fatal(
                f"User attempted to connect from node {node1} to {node2}, "
                f"but it is not a child of this {self.__class__.__name__}!"
            )

        if node2 not in self._nodes:
            fatal(
                f"User attempted to connect to node {node2} from {node1}, "
                f"but it is not a child of this {self.__class__.__name__}!"
            )

        # check if they are already connected
        (l12, l21) = self._getNodeLinks(node1, node2)
        if l12 is not None and l21 is not None:
            warn(
                f"Attempted to connect to node {node2} from {node1}, "
                f"but these nodes are already connected."
            )
            return
        elif l12 is not None or l21 is not None:
            panic(
                f"Attempted to connect to node {node2} from {node1}, "
                f"but only one direction of int link was found. "
                f"Link 1->2: {l12}; Link 2->1: {l21}"
                f"(one of these should be None)."
            )
        elif l12 is None and l21 is None:

            if (
                node2 not in node1._connected_nodes
                and node1 not in node2._connected_nodes
            ):
                self._connectChildrenGarnet(node1, node2)

            if node2 not in node1._connected_nodes:
                node1._connected_nodes.append(node2)

            if node1 not in node2._connected_nodes:
                node2._connected_nodes.append(node1)

        else:
            panic("This shouldn't be possible to reach in connectChildren()!")

    # *
    # * Granular Connection Configuration Methods
    # *

    def enableNodeLinkSerDes(
        self,
        node1: BaseChipletSystem,
        node2: BaseChipletSystem,
        enable_node_1: bool = True,
        enable_node_2: bool = True,
    ):
        """
        Enable the serializer-deserializer units on either end of
        the links between the two specified nodes (`ChipletSystem`s
        or `Chiplet`s) *which are children of this `ChipletSystem`.*

        Args:
            node1 (BaseChipletSystem): One node.
            node2 (BaseChipletSystem): The other node.
            enable_node_1 (bool):
                Whether to enable SerDes on node 1. Defaults to True.
            enable_node_2 (bool):
                Whether to enable SerDes on node 2. Defaults to True.
        """
        (l12, l21) = self._getNodeLinks(node1, node2)

        if l12 is None and l21 is None:
            warn(
                f"int links between {node1.__class__.__name__}{node1.id} "
                f"and {node2.__class__.__name__}{node2.id} do not exist!"
            )
            return

        if l12 is not None:
            if enable_node_1:
                l12.src_serdes = True
            if enable_node_2:
                l12.dst_serdes = True

            if l21 is None:
                warn(
                    f"int link from {node1.__class__.__name__}{node1.id} "
                    f"to {node2.__class__.__name__}{node2.id} exists "
                    f"but int link in other direction does not. "
                )

        if l21 is not None:
            if enable_node_2:
                l21.src_serdes = True
            if enable_node_1:
                l21.dst_serdes = True

            if l12 is None:
                warn(
                    f"int link from {node1.__class__.__name__}{node1.id} "
                    f"to {node2.__class__.__name__}{node2.id} exists "
                    f"but int link in other direction does not. "
                )

    def enableNodeLinkCDC(
        self,
        node1: BaseChipletSystem,
        node2: BaseChipletSystem,
        enable_node_1: bool = True,
        enable_node_2: bool = True,
    ):
        """
        Enable the clock domain crossing units on either end of
        the links between the two specified nodes (`ChipletSystem`s
        or `Chiplet`s) *which are children of this `ChipletSystem`.*

        Args:
            node1 (BaseChipletSystem): One node.
            node2 (BaseChipletSystem): The other node.
            enable_node_1 (bool):
                Whether to enable CDC on node 1. Defaults to True.
            enable_node_2 (bool):
                Whether to enable CDC on node 2. Defaults to True.
        """
        (l12, l21) = self._getNodeLinks(node1, node2)

        if l12 is None and l21 is None:
            warn(
                f"int links between {node1.__class__.__name__}{node1.id} "
                f"and {node2.__class__.__name__}{node2.id} do not exist!"
            )
            return

        if l12 is not None:
            if enable_node_1:
                l12.src_cdc = True
            if enable_node_2:
                l12.dst_cdc = True

            if l21 is None:
                warn(
                    f"int link from {node1.__class__.__name__}{node1.id} "
                    f"to {node2.__class__.__name__}{node2.id} exists "
                    f"but int link in other direction does not. "
                )

        if l21 is not None:
            if enable_node_2:
                l21.src_cdc = True
            if enable_node_1:
                l21.dst_cdc = True

            if l12 is None:
                warn(
                    f"int link from {node1.__class__.__name__}{node1.id} "
                    f"to {node2.__class__.__name__}{node2.id} exists "
                    f"but int link in other direction does not. "
                )

    def setNodeLinkFlitWidth(
        self,
        node1: BaseChipletSystem,
        node2: BaseChipletSystem,
        width: int,
        set_width_link_1_to_2: bool = True,
        set_width_link_2_to_1: bool = True,
    ):
        """
        Adjust flit size (width) of the links between the two specified
        nodes (`ChipletSystem`s or `Chiplet`s) *which are children of
        this `ChipletSystem`.*

        Note that since internal links are unidirectional, the user has
        the option to change the flit size for only one of the links
        (i.e., only for traffic from node1 to node2, or vice versa)
        by setting the set_width_link_1_to_2 (or ...2_to_1) flags.

        By default, both unidirectional links comprising the connection
        between node1 and node2 will have their width/flit size changed.

        Args:
            node1 (BaseChipletSystem): _description_
            node2 (BaseChipletSystem): _description_
            width (int): The flit width to set on the link.
            set_width_link_1_to_2 (bool, optional):
                Whether to set the width of the unidirectional
                internal link from node1 to node2.
                Defaults to True.
            set_width_link_2_to_1 (bool, optional):
                Whether to set the width of the unidirectional
                internal link from node2 to node1.
                Defaults to True.
        """
        (l12, l21) = self._getNodeLinks(node1, node2)

        if l12 is None and l21 is None:
            warn(
                f"int links between {node1.__class__.__name__}{node1.id} "
                f"and {node2.__class__.__name__}{node2.id} do not exist!"
            )
            return

        if l12 is not None:
            if set_width_link_1_to_2:
                l12.width = width

            if l21 is None:
                warn(
                    f"int link from {node1.__class__.__name__}{node1.id} "
                    f"to {node2.__class__.__name__}{node2.id} exists "
                    f"but int link in other direction does not. "
                )

        if l21 is not None:
            if set_width_link_2_to_1:
                l21.width = width

            if l12 is None:
                warn(
                    f"int link from {node1.__class__.__name__}{node1.id} "
                    f"to {node2.__class__.__name__}{node2.id} exists "
                    f"but int link in other direction does not. "
                )

    # *
    # * Deprecated
    # *

    def _defineMainRouterParams(self):
        """
        For some reason gem5 hates it when these aren't automatically
        defined for `_main_router`, probably because it was instantiated
        seperately. Should just nee to be called before `m5.instantiate()`.
        """
        assert isinstance(self._garnet_network, GarnetNetwork)
        # gem5 will complain if params aren't defined for main router
        mr = self._main_router
        gn = self._garnet_network
        if int(mr.router_id) == -1:
            num_routers = len(self._garnet_network.routers)
            mr.router_id = num_routers - 1
        mr.vcs_per_vnet = gn.vcs_per_vnet
        # mr.virt_nets = self._garnet_network.number_of_virtual_networks
        # mr.ni_flit_size = gn.ni_flit_size
        # mr.eventq_index = gn.eventq_index

    def _fixAllGarnetObjectParams(self):

        import traceback

        fatal(f"do not use this function! tb: {traceback.print_stack()}")

        assert isinstance(self._garnet_network, GarnetNetwork)

        def _recursiveUnproxyParams(sim_obj, set_clk_domain=False):
            if len(sim_obj) > 1:
                for i, obj in enumerate(sim_obj):
                    _recursiveUnproxyParams(obj, set_clk_domain)
                return
            if not isSimObject(sim_obj):
                warn(f"{sim_obj} is not a SimObject.")
                return
            if set_clk_domain:
                if hasattr(sim_obj, "clk_domain"):
                    # warn(f"setting clock domain for {sim_obj}")
                    sim_obj.clk_domain = self._system.clk_domain
                # if hasattr(sim_obj, 'vcs_per_vnet'):
                #     vcs = self._garnet_network.vcs_per_vnet
                #     warn(f"setting vcs_per_vnet for {sim_obj} = {vcs}")
                #     sim_obj.vcs_per_vnet = vcs
            sim_obj.unproxyParams()
            children = sim_obj._children.values()
            for child in children:
                _recursiveUnproxyParams(child, set_clk_domain)

        # fix network params
        if not hasattr(self._garnet_network, "number_of_virtual_networks"):
            self._garnet_network.number_of_virtual_networks = (
                self._system.ruby.network.number_of_virtual_networks
            )

        # fix router params
        for r in self._garnet_network.routers:
            # print(f"setting clk_domain for {r}")
            r.clk_domain = self._system.clk_domain
            # print(f"setting eqidx = 0 for {r}")
            # default for all `SimObject`s should be 0
            r.eventq_index = 0
            r.power_state.eventq_index = r.eventq_index
            r.unproxyParams()

        # fix int link params
        for il in self._garnet_network.int_links:
            # il.clk_domain = self._system.clk_domain
            il.eventq_index = 0
            _recursiveUnproxyParams(il)
            # il.unproxyParams()
            # il_attrs = dir(il)
            # for attr_str in il_attrs:
            #     attr = getattr(il, attr)
            #     if isSimObject(attr):
            #         attr.unproxyParams()
            # if hasattr(il, 'credit_link'):
            #     il.credit_link.unproxyParams()
            #     il.credit_link.power_state.unproxyParams()
            # #     il.credit_link.clk_domain = self._system.clk_domain

        # fix ext link params
        for el in self._garnet_network.ext_links:
            el.eventq_index = 0
            _recursiveUnproxyParams(el, True)

    def _connectHierarchyGarnet(self):
        """
        Should be called after `_createTopology()`.
        Adds the main router to the `GarnetNetwork` list and
        creates connections between the main routers of the current
        level of the hierarchy and its parents/siblings.
        """
        import traceback

        fatal(f"do not use this function! tb: {traceback.print_stack()}")
        # topologies tend to overwrite garnet links/routers entirely
        # so just create our links (and add our main router) afterwards
        num_routers = len(self._garnet_network.routers)
        if self._main_router.router_id == -1:
            self._main_router.router_id = num_routers  # - 1
        if self._main_router not in self._garnet_network.routers:
            self._garnet_network.routers.append(self._main_router)
        # self._garnet_network.mainrouter = self._main_router  # load bearing
        self._connectParentGarnet()
        if hasattr(self, "_parent_sys") and self._parent_sys is not None:
            for node in self._parent_sys._nodes:
                if node is not self:
                    self._parent_sys.connectChildren(self, node)
                    # self._connectSiblingGarnet(node)
            # for rc in self._parent_sys._ruby_controllers:
            #     self._connectRubyControllerGarnet(rc)
        # setattr(
        #     self._garnet_network,
        #     f'mainrouterc{self.id}',
        #     self._main_router
        # )
        # setattr(
        #     self._garnet_network,
        #     f'routers{self.id}',
        #     self._main_router
        # )

    def _mergeGarnetNetworks(
        self, merge_into: GarnetNetwork, merge_from: GarnetNetwork
    ):
        import traceback

        fatal(f"do not use this function! tb: {traceback.print_stack()}")
        mi_router_ct = len(merge_into.routers)
        mi_link_ct = len(merge_into.int_links) + len(merge_into.ext_links)
        for r in merge_from.routers:
            # ensure unique IDs in resulting network
            r.router_id = mi_router_ct
            merge_into.routers.append(r)
            mi_router_ct += 1

        merge_from.routers = []

        for el in merge_from.ext_links:
            el.link_id = mi_link_ct
            merge_into.ext_links.append(el)
            mi_link_ct += 1

        merge_from.ext_links = []

        for il in merge_from.int_links:
            il.link_id = mi_link_ct
            merge_into.int_links.append(il)
            mi_link_ct += 1

        merge_from.int_links = []

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
        import traceback

        fatal(f"do not use this function! tb: {traceback.print_stack()}")

        for n in self._nodes:
            if hasattr(n, "_garnet_network"):
                assert isinstance(n._garnet_network, GarnetNetwork)
                self._mergeGarnetNetworks(merge_into, n._garnet_network)
            if hasattr(n, "_nodes"):
                n._combineHierarchyGarnetNetworks(merge_into)
