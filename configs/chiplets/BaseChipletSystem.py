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
    warn,
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

from topologies.BaseTopology import BaseTopology


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
    _nodes: list[BaseChipletSystem]

    # topology of the children for *only this layer of the hierarchy*
    _topology: BaseTopology | None

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

    # parent `ChipletSystem` (if present)
    _parent_sys: ChipletSystem | None

    # list of nodes on the same hierarchical level of this node's parent
    # `ChipletSystem` to which this node has *direct* connections
    _connected_nodes: list[BaseChipletSystem]

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
    # excludes directory controllers
    _ruby_controllers: list[RubyController]

    # `GarnetExtLink`s that connect this object to parent `RubyController`s
    _ruby_controller_link_map: list[GarnetExtLink]
    _ruby_controller_router_map: list[tuple[RubyController, GarnetRouter]]

    # directory controllers in this `Chiplet[System]`
    _dir_controllers: list[RubyController]
    _dir_controller_link_map: list[tuple[RubyController, GarnetExtLink]]
    _dir_controller_router_map: list[tuple[RubyController, GarnetRouter]]

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
            router_id=-1,  # temporary, is replaced later
            latency=self.default_inter_node_router_lat,
            # the following are default parameters
            # gem5 complains if I don't include these params
            vcs_per_vnet=4,
        )

        self._sibling_links = []

        self._ruby_controllers = []
        self._ruby_controller_link_map = []
        self._ruby_controller_router_map = []

        self._dir_controllers = []
        self._dir_controller_link_map = []
        self._dir_controller_router_map = []

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
        raise NotImplementedError

    def getRoot(self):
        """
        Retrieve a reference to the root `ChipletSystem`,
        i.e., the `ChipletSystem` at the top of the hierarchy.
        """

        if not hasattr(self, "_parent_sys") or self._parent_sys == None:
            return self
        else:
            return self._parent_sys.getRoot()

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
            (len(self._nodes) == 0)
            or node1 not in self._nodes
            or node2 not in self._nodes
        ):
            # no nodes, or one or both nodes are not children of `self`
            return None

        # todo: get the GarnetIntLinks

        return None

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

    def _createTopology(self, options: Namespace):
        # for reference, this is what you would do for a flat topology
        if False:
            self._topology = self._topology_cls(
                self.getRoot()._meta_topology.nodes  # type: ignore
            )

        self._topology = self._topology_cls(
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
            options.num_cpus = len(self._nodes)

        # * setup topology
        # latter 3 args are the literal classes for the
        # corresponding objects. they can technically be
        # based on either `GarnetNetwork` or `SimpleNetwork`,
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

        # if `num_rows > 0` we will fail an assertion
        # when the C++ objects are instantiated
        # (`m_num_rows * m_num_cols == m_routers.size()`)
        # (which will not be true since we added routers)
        print(f"NUM ROWS is currently {self._garnet_network.num_rows}")
        self._garnet_network.num_rows = 0
        print(f"NUM ROWS is {self._garnet_network.num_rows} after set = 0")

        # if in SE mode, register topology with (faux) filesystem
        # only some topologies implement this (as of now, only
        # `Mesh_XY` and `MeshDirCorners_XY` do.)
        if not self._is_full_system:
            # notes: `Mesh_XY` requires
            # `options.num_cpus` and `options.mem_size`.
            # `MeshDirCorners_XY` requires only `options.mem_size`.
            self._topology.registerTopology(options)

        gn = self._garnet_network

        # store mappings of controller to router
        # also connect to main router
        print(
            f"Linking Routers with main for "
            f"{self.__class__.__name__} ID {self.id}"
        )
        for c, r in zip(self._ruby_controllers, gn.routers, strict=True):
            print(f"    controller {c} :: router {r.router_id} {r}")
            self._ruby_controller_router_map.append((c, r))
            self._biLinkGarnetRouters(self._main_router, r)

        gn.routers = gn.routers + [self._main_router]

    def _biLinkGarnetRouters(
        self,
        router1: GarnetRouter,
        router2: GarnetRouter,
        auto_reg_parent: bool = True,
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

        if auto_reg_parent:
            l12.set_parent(self._garnet_network, f"int_links{num_links}")
            l21.set_parent(self._garnet_network, f"int_links{num_links + 1}")

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

        Args:
            node1 (BaseChipletSystem): The node to connect from.
            node2 (BaseChipletSystem): The node to connect to.
        """
        l12, l21 = self._biLinkGarnetRouters(
            node1._main_router, node2._main_router
        )
        node1._sibling_links.append((l12, l21))
        node2._sibling_links.append((l12, l21))

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

        if (
            node2 not in node1._connected_nodes
            and node1 not in node2._connected_nodes
        ):
            self._connectChildrenGarnet(node1, node2)

        if node2 not in node1._connected_nodes:
            node1._connected_nodes.append(node2)

        if node1 not in node2._connected_nodes:
            node2._connected_nodes.append(node1)

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

        if self.isDirController(ctrl):
            self._dir_controller_link_map.append(link)
        else:
            self._ruby_controller_link_map.append(link)

        if auto_reg_parent:
            link.set_parent(self._garnet_network, f"ext_links{num_links}")
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
        if int(mr.router_id) == -1:
            num_routers = len(self._garnet_network.routers)
            mr.router_id = num_routers - 1
        mr.vcs_per_vnet = gn.vcs_per_vnet
        # mr.virt_nets = self._garnet_network.number_of_virtual_networks
        # mr.ni_flit_size = gn.ni_flit_size
        # mr.eventq_index = gn.eventq_index

    def _fixAllGarnetObjectParams(self):

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
        mi_router_ct = len(merge_into.routers)
        mi_link_ct = len(merge_into.int_links) + len(merge_into.ext_links)
        for r in merge_from.routers:
            # ensure unique IDs in resulting network
            r.router_id = mi_router_ct
            merge_into.routers.append(r)
            mi_router_ct += 1

        # merge_from.routers = []

        for el in merge_from.ext_links:
            el.link_id = mi_link_ct
            merge_into.ext_links.append(el)
            mi_link_ct += 1

        # merge_from.ext_links = []

        for il in merge_from.int_links:
            il.link_id = mi_link_ct
            merge_into.int_links.append(il)
            mi_link_ct += 1

        # merge_from.int_links = []

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
        for n in self._nodes:
            if hasattr(n, "_garnet_network"):
                self._mergeGarnetNetworks(merge_into, n._garnet_network)
            if hasattr(n, "_nodes"):
                n._combineHierarchyGarnetNetworks(merge_into)
