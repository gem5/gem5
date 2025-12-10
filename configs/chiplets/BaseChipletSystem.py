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
    from src.sim.ClockDomain import ClockDomain
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
    protocol = Param.String("String name corresponding to the Ruby protocol")

    # *
    # * Latency parameters
    # *

    # default latency (in cycles) of the links between nodes
    # for this layer of abstraction (i.e., direct child nodes)
    default_inter_node_link_lat = Param.Int(
        "Default latency (in cycles) of the links between nodes "
        "in this level of the hierarchy."
    )

    # same as above, but for links within a ChipletSystem that
    # aren't from one child node to another
    default_intra_node_link_lat = Param.Int(
        "Default latency (in cycles) of the routers corresponding to "
        "the links between routers *within one node* in this level of "
        "the hierarchy."
    )

    # same as default_inter_node_link_lat,
    # but for the routers corresponding to the links
    default_node_main_router_lat = Param.Int(
        "Default latency (in cycles) of the routers corresponding to "
        "the links between nodes in this level of the hierarchy."
    )

    default_cache_controller_link_lat = Param.Int(
        "Default latency (in cycles) of the links between "
        "each cache controller and its corresponding router. "
        "(*not* between that router and the main router of this "
        "`ChipletSystem`). Defaults to `default_inter_node_link_lat`."
    )

    default_cache_controller_router_lat = Param.Int(
        "The default latency (in cycles) of the router "
        "corresponding to each cache controller. "
        "Defaults to `default_node_main_router_lat`."
    )

    default_dir_controller_link_lat = Param.Int(
        "Default latency (in cycles) of the links between "
        "each directory controller and its corresponding router "
        "(*not* between that router and the main router of this "
        "`ChipletSystem`). Defaults to `default_inter_node_link_lat`."
    )

    default_dir_controller_router_lat = Param.Int(
        "The default latency (in cycles) of the router "
        "corresponding to each dir controller. "
        "Defaults to `default_node_main_router_lat`."
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
    # corresponds to `_connected_nodes`, which holds the relevant node refs
    _sibling_links: list[tuple[GarnetIntLink, GarnetIntLink]]

    # `GarnetNetwork` for this object
    # ! will always be fake except for root after `createSystem()` is done
    _garnet_network: GarnetNetwork | FakeGarnetNetwork

    # `RubyController`s corresponding to the topology of this
    # `Chiplet[System]` specifically. in other words, this
    # includes all controllers used to generate the core topology via
    # `_createTopology()`. However, it may not include *all* controllers
    # present in this `Chiplet[System]` depending on the parameters
    # specified when calling `createSystem()`.
    _topology_controllers: list[RubyController]

    # directory controllers in this `Chiplet[System]`
    # these are returned by `create_system()` of the Ruby protocol
    _dir_controllers: list[RubyController]

    # controllers that are not directory controllers
    # e.g., LLC controller
    # does not include *all* cache controllers in the system
    _cache_controllers: list[RubyController]

    # Ruby CPU sequencers for this `Chiplet[System]`
    # includes sequencers for all CPUs
    # should only be populated at the root
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
        intra_node_link_lat: int,
        node_main_router_lat: int,
        cache_controller_link_lat: int,
        cache_controller_router_lat: int,
        dir_controller_link_lat: int,
        dir_controller_router_lat: int,
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
                to the topology to be used in this level of the
                `ChipletSystem` hierarchy.
            MemoryClass (Type[AbstractMemory]):
                The class corresponding to the memory configuration
                for the gem5 `System` this `ChipletSystem` is part of.
            nodes (Sequence[BaseChipletSystem] | Sequence[BaseCPU]):
                Optional list of nodes. May be passed to constructor
                or added separately using `addNode()`.
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
            cache_controller_link_lat (int, optional):
                The default latency (in cycles) of the links between
                each cache controller and its corresponding router.
                Defaults to `inter_node_link_lat`.
            cache_controller_router_lat (int, optional):
                The default latency (in cycles) of the router
                corresponding to each cache controller.
                Defaults to `node_main_router_lat`.
            dir_controller_link_lat (int, optional):
                The default latency (in cycles) of the links between
                each directory controller and its corresponding router.
                Defaults to `inter_node_link_lat`.
            dir_controller_router_lat (int, optional):
                The default latency (in cycles) of the router
                corresponding to each directory controller.
                Defaults to `node_main_router_lat`.
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
        self.default_intra_node_link_lat = intra_node_link_lat
        self.default_node_main_router_lat = node_main_router_lat

        if cache_controller_link_lat == -1:
            self.default_cache_controller_link_lat = (
                self.default_inter_node_link_lat
            )

        if cache_controller_router_lat == -1:
            self.default_cache_controller_router_lat = (
                self.default_node_main_router_lat
            )

        if dir_controller_link_lat == -1:
            self.default_dir_controller_link_lat = (
                self.default_inter_node_link_lat
            )

        if dir_controller_router_lat == -1:
            self.default_dir_controller_router_lat = (
                self.default_node_main_router_lat
            )

        self._connected_nodes = []

        self._main_router = GarnetRouter(
            router_id=0,  # some hijinks occur with this later
            latency=self.default_node_main_router_lat,
            # the following are default parameters
            # gem5 complains if I don't include these params
            vcs_per_vnet=4,
        )

        self._sibling_links = []

        self._topology_controllers = []
        self._dir_controllers = []
        self._cache_controllers = []

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

    def getRoot(self):  # -> ChipletSystem:
        """
        Retrieve a reference to the root `ChipletSystem`,
        i.e., the `ChipletSystem` at the top of the hierarchy.
        """

        if (
            not hasattr(self, "_parent_sys") or self._parent_sys == None
        ):  # and isinstance(self, ChipletSystem):
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
            # assert self is ChipletSystem
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

    @classmethod
    def recursivelyPrintClockDomains(cls, base):
        print(f"Starting clock domain traversal from {base}.")
        cls._traverseClockDomains(base, True, False)

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
                This should usually be `self._topology_controllers`.
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

        # assert options.network == "garnet"

        # * set topology latencies
        # the topology will configure router and link latencies based
        # on `options`, so we have to set that here.
        options.router_latency = self.default_node_main_router_lat
        options.link_latency = self.default_intra_node_link_lat
        # we don't really care what these values were because all
        # our latency values are specified elsewhere,
        # and we (probably) don't want them to be all uniform either.

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
        print(
            f"Linking Routers with main for "
            f"{self.__class__.__name__} ID {self.id}"
        )
        for r in gnw.routers:
            self._biLinkGarnetRouters(self._main_router, r)

        # * add main router to my routers
        gnw.routers += self._main_router

        for c in controllers:
            r = self._getRouterFromController(c, False)
            if r is not None:
                print(f"    controller {c} :: router {r.router_id} {r}")

    # *
    # * Low-Level Network Connection/Instantiation Methods
    # *

    def _getExtLinkFromController(
        self,
        controller: RubyController,
        searchFromRoot: bool = True,
    ) -> GarnetExtLink | None:
        if searchFromRoot and not self.isRoot():
            return self.getRoot()._getExtLinkFromController(controller, False)
            # don't need to set searchFromRoot = False but why not

        ext_links = self._garnet_network.ext_links
        for ext_link in ext_links:
            if controller == ext_link.ext_node:
                return ext_link

        return None

    def _getRouterFromController(
        self,
        controller: RubyController,
        searchFromRoot: bool = True,
    ) -> GarnetRouter | None:
        ext_link = self._getExtLinkFromController(controller, searchFromRoot)
        if ext_link is not None:
            return ext_link.int_node
        else:
            return None

    def _getIntLinkFromRouters(
        self,
        routerFrom: GarnetRouter,
        routerTo: GarnetRouter,
        searchFromRoot: bool = True,
    ) -> GarnetIntLink | None:
        if searchFromRoot and not self.isRoot():
            return self.getRoot()._getIntLinkFromRouters(
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

    def _getRoutersFromIntLink(
        self, int_link: GarnetIntLink
    ) -> tuple[GarnetRouter | None, GarnetRouter | None]:
        if int_link is None:
            return (None, None)

        return int_link.src_node, int_link.dst_node

    def _setLinkLatency(
        self,
        link: GarnetIntLink | GarnetExtLink,
        new_latency: int,
    ):
        """
        Manually set the latency for a specific network link.
        If using externally to `Chiplet[System]`, find the link
        with `_getIntLinkFromRouters()` for internal links between
        two routers, or `_getExtLinkFromController()` for external
        links between a router and a `RubyController`.

        Args:
            link (GarnetIntLink | GarnetExtLink):
                The link for which to set the latency.
            new_latency (int): The new latency to set.
        """
        link.latency = new_latency

    def _biLinkGarnetRouters(
        self,
        router1: GarnetRouter,
        router2: GarnetRouter,
        link_latency: int = -1,
    ) -> tuple[GarnetIntLink | None, GarnetIntLink | None]:
        """
        Create internal links (`GarnetIntLink`s) between two specified
        routers in a `GarnetNetwork` (can be a `FakeGarnetNetwork`).

        Args:
            router1 (GarnetRouter): The first router to link.
            router2 (GarnetRouter): The second router to link.
            link_latency (int, optional):
                The link latency.
                Defaults to `self.default_inter_node_link_lat`.

        Returns:
            tuple[GarnetIntLink | None, GarnetIntLink | None]:
                the internal links, if successfully created.
        """
        if link_latency == -1:
            link_latency = self.default_inter_node_link_lat

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
            latency=link_latency,
        )

        l21 = GarnetIntLink(
            link_id=num_links + 1,
            src_node=router2,
            dst_node=router1,
            latency=link_latency,
        )

        # self._garnet_network.int_links.append(l12)
        # self._garnet_network.int_links.append(l21)
        self._garnet_network.int_links += l12
        self._garnet_network.int_links += l21

        return l12, l21

    def _connectParentGarnet(self):
        warn("It is recommended to connect from parent instead of child.")
        if hasattr(self, "_parent_sys") and self._parent_sys is not None:
            self._parent_link = self._biLinkGarnetRouters(
                self._main_router, self._parent_sys._main_router
            )

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
    ) -> GarnetExtLink:
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
                Defaults to the main router.
                Usually this should be specified only when creating a link
                to a directory controller, which does things differently.
            link_lat (int, optional):
                The link latency for the resulting `GarnetExtLink`.
                Defaults to -1, which results in using the default link
                latency stored in this `Chiplet[System]`.

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

        l12 = self._getIntLinkFromRouters(n1r, n2r)
        l21 = self._getIntLinkFromRouters(n2r, n1r)

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

        if hasattr(self, "_created"):
            if self._created:
                warn(
                    f"connectChildren() was called after createSystem()! "
                    f"This may result in unexpected behavior! "
                    f"Nodes: {node1}; {node2}"
                )

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
        by setting the `set_width_link_1_to_2` (or `...2_to_1`) flags.

        By default, both unidirectional links comprising the connection
        between node1 and node2 will have their width/flit size changed.

        Args:
            node1 (BaseChipletSystem): One node.
            node2 (BaseChipletSystem): The other node.
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

    def setNodeLatencies(
        self,
        node1: BaseChipletSystem,
        node2: BaseChipletSystem,
        link_latency: int = -1,
        router_latency: int = -1,
        set_lat_link_1_to_2: bool = True,
        set_lat_link_2_to_1: bool = True,
        set_lat_router_1: bool = True,
        set_lat_router_2: bool = True,
    ):
        """
        Adjust latency of the links (and their routers) between the two
        specified nodes (`ChipletSystem`s or `Chiplet`s) *which are
        children of this `ChipletSystem`.*

        Note that since internal links are unidirectional, the user has
        the option to change the latencies for only one of the links
        (i.e., only for traffic from node1 to node2, or vice versa)
        by setting the `set_lat_link_1_to_2` (or `...2_to_1`) flags, and
        similarly for the routers with set_lat_router_1 (or `..._2`).

        Args:
            node1 (BaseChipletSystem): One node.
            node2 (BaseChipletSystem): The other node.
            link_latency (int):
                The link latency to set.
                If none is provided, it will not be set.
            router_latency (int):
                The router latency to set.
                If none is provided, it will not be set.
            set_lat_link_1_to_2 (bool, optional):
                Whether to set the latency of the unidirectional
                internal link from node1 to node2.
                Defaults to True.
            set_lat_link_2_to_1 (bool, optional):
                Whether to set the latency of the unidirectional
                internal link from node2 to node1.
                Defaults to True.
            set_lat_router_1 (bool, optional):
                Whether to set the latency of the router on
                node1's end of the internal links.
                Defaults to True.
            set_lat_router_2 (bool, optional):
                Whether to set the latency of the router on
                node2's end of the internal links.
                Defaults to True.
        """
        (l12, l21) = self._getNodeLinks(node1, node2)

        if l12 is None and l21 is None:
            warn(
                f"int links between {node1.__class__.__name__}{node1.id} "
                f"and {node2.__class__.__name__}{node2.id} do not exist!"
            )
            return

        if link_latency != -1:
            if l12 is not None:
                if set_lat_link_1_to_2:
                    l12.width = link_latency

                if l21 is None:
                    warn(
                        f"int link "
                        f"from {node1.__class__.__name__}{node1.id} "
                        f"to {node2.__class__.__name__}{node2.id} exists "
                        f"but int link in other direction does not. "
                    )

            if l21 is not None:
                if set_lat_link_2_to_1:
                    l21.width = link_latency

                if l12 is None:
                    warn(
                        f"int link "
                        f"from {node1.__class__.__name__}{node1.id} "
                        f"to {node2.__class__.__name__}{node2.id} exists "
                        f"but int link in other direction does not. "
                    )

        if router_latency != -1:
            node1_router = None
            node2_router = None
            if l12 is not None:
                node1_router, node2_router = self._getRoutersFromIntLink(l12)
            elif l21 is not None:
                node1_router, node2_router = self._getRoutersFromIntLink(l21)

            if node1_router is None:
                warn(
                    f"main router for node1 may be missing! "
                    f"latency not set for router belonging to "
                    f"node {node1}"
                )
            elif set_lat_router_1:
                node1_router.latency = router_latency

            if node2_router is None:
                warn(
                    f"main router for node2 may be missing! "
                    f"latency not set for router belonging to "
                    f"node {node1}"
                )
            elif set_lat_router_2:
                node2_router.latency = router_latency

    @classmethod
    def setClockDomain(
        cls,
        obj: BaseChipletSystem | BaseCPU | GarnetRouter | GarnetExtLink,
        new_clk_domain: ClockDomain,
        recursive: bool = False,
        print_clk_domains: bool = False,
    ):
        """
        Set the clock domain of the specified `SimObject` (typically a
        `BaseChipletSystem`, CPU, or Garnet router/link).
        Optionally, recursively traverse down `SimObject` hierarchy
        starting at `obj`, setting a new (specified) clock domain
        for any `SimObject` that has a clock domain.

        Note that this is a class method, not an instance method.
        This method does NOT automatically set the clock domain of the
        `BaseChipletSystem` it is called on, or its children (although
        this method should be called from the class, not any instance).
        Instead, a base `SimObject` `obj` must be specified.

        Args:
            obj (BaseChipletSystem|BaseCPU|GarnetRouter|GarnetExtLink):
                `SimObject` to set the clock domain of, or, if `recursive`,
                to traverse from. Specified types are generally what
                might be used. This method should be used alongside
                `enableNodeLinkCDC()`, `_findExtLinkFromController()`,
                `_getNodeLinks()`, and/or `_findRouterFromController()`,
                as these methods will be needed to find `obj` and enable
                the CDC units corresponding to the appropriate links.
            new_clk_domain (ClockDomain):
                Any clock domains found will be updated/set to
                this new clock domain.
            recursive (bool, optional):
                Whether to recursively traverse the `SimObject` hierarchy
                and explicitly set all child clock domains.
                Generally this should be done if the clock domains are not
                properly inherited (you can check this using the method
                `recursivelyPrintClockDomains()`). Typically, `SimObject`s
                should automatically inherit the clock domain of their
                parent, so setting the parent object's clock domain should
                be sufficient. Defaults to `False`.
            print_clk_domains (bool, optional):
                Whether to print the clock domain(s) *before* they are set.
        """
        if print_clk_domains:
            print(f"Setting clock domain for {obj} to {new_clk_domain}.")
        if hasattr(obj, "clk_domain"):
            if print_clk_domains:
                print(f"{obj} : {obj.clk_domain}")
            if obj.clk_domain == new_clk_domain:
                warn(
                    f"The clock domain of {obj} is already set to "
                    f"{new_clk_domain}!"
                )
            obj.clk_domain = new_clk_domain
        elif hasattr(obj, "_clk_domain"):
            if print_clk_domains:
                print(f"{obj} : {obj._clk_domain}")
            if obj._clk_domain == new_clk_domain:
                warn(
                    f"The clock domain of {obj} is already set to "
                    f"{new_clk_domain}!"
                )
            obj._clk_domain = new_clk_domain
        if recursive:
            if print_clk_domains:
                print(f"Starting clock domain traversal from {obj}.")
            cls._traverseClockDomains(
                obj, print_clk_domains, True, new_clk_domain
            )

    @classmethod
    def _traverseClockDomains(
        cls,
        base: BaseChipletSystem | BaseCPU | GarnetRouter | GarnetExtLink,
        print_clk_domains: bool = True,
        set_clk_domains: bool = False,
        new_clk_domain: ClockDomain | None = None,
    ):
        """
        Recursively traverse down `SimObject` hierarchy starting at `base`.
        Optionally print found clock domains or recursively set new ones.

        Args:
            base (BaseChipletSystem|BaseCPU|GarnetRouter|GarnetExtLink):
                `SimObject` to traverse from. Specified types are generally
                what might be used. This method should be used alongside
                `enableNodeLinkCDC()`, `_findExtLinkFromController()`,
                `_getNodeLinks()`, and/or `_findRouterFromController()`,
                as these methods will be needed to find `base` and enable
                the CDC units corresponding to the appropriate links.
            print_clk_domains (bool, optional):
                Whether to print the clock domains that are found during
                traversal. Defaults to True. Note that if also setting
                new clock domains, `m5.instantiate()` should probably be
                called before checking the hierarchy.
            set_clk_domains (bool, optional):
                Whether to set the clock domains found during traversal
                to `new_clk_domain`. Defaults to False. Has no effect
                if `new_clk_domain` is unspecified (`None`).
            new_clk_domain (ClockDomain, optional):
                If `set` is `True`, any clock domains found will be updated
                to this new clock domain. Defaults to None. Has no effect
                if `set` is `False`.
        """
        if not print_clk_domains and not set_clk_domains:
            warn(
                f"_traverseClockDomains() was called, but no actions "
                f"were specified (base = {base})."
            )
            return
        if set_clk_domains is True and new_clk_domain is None:
            warn(
                f"_traverseClockDomains() was called, and "
                f"`set_clock_domains` was set to True, but no "
                f"clock domain was specified (base = {base})."
            )
            return

        if hasattr(base, "clk_domain"):
            if print_clk_domains:
                print(f"{base} : {base.clk_domain}")
            if set_clk_domains:
                base.clk_domain = new_clk_domain
        elif hasattr(base, "_clk_domain"):
            if print_clk_domains:
                print(f"{base} : {base._clk_domain}")
            if set_clk_domains:
                base._clk_domain = new_clk_domain

        if isSimObject(base):
            # * check for SimObject children
            # print(f"{base} is SimObject")
            for child in base._children.values():
                #   print(f"   child: {child}")
                cls._traverseClockDomains(
                    child, print_clk_domains, set_clk_domains, new_clk_domain
                )

            # * check for `ChipletSystem` children
            if hasattr(base, "_nodes"):
                for node in base._nodes:
                    # but don't print/set redundantly
                    if node not in base._children.values():
                        cls._traverseClockDomains(
                            node,
                            print_clk_domains,
                            set_clk_domains,
                            new_clk_domain,
                        )
        elif isSimObjectSequence(base):
            # * handle lists of SimObjects
            # print(f"{base} is SimObject Vector")
            for obj in base:
                if isinstance(obj, list):
                    # list in a list
                    # type checker seems to think this will happen
                    # not sure about that
                    for child in obj:
                        cls._traverseClockDomains(
                            child,
                            print_clk_domains,
                            set_clk_domains,
                            new_clk_domain,
                        )
                else:  # not a list in a list
                    for child in obj._children.values():
                        # print(f"   child: {child}")
                        cls._traverseClockDomains(
                            child,
                            print_clk_domains,
                            set_clk_domains,
                            new_clk_domain,
                        )
