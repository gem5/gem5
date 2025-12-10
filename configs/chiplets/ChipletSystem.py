from argparse import Namespace  # for type checking
from importlib import import_module
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
from chiplets.Chiplet import Chiplet
from chiplets.ChipletSystemMainRouterTopology import (
    ChipletSystemMainRouterTopology as CSMRT,
)
from common import (
    FileSystemConfig,
    MemConfig,
    ObjectList,
)
from network.Network import init_network
from ruby import Ruby
from topologies.BaseTopology import BaseTopology
from topologies.ChipletMetaTopology import ChipletMetaTopology

from m5.defines import buildEnv  # type: ignore
from m5.objects import *
from m5.util import (
    addToPath,
    fatal,
    panic,
    warn,
)

if TYPE_CHECKING:
    from src.cpu.BaseCPU import BaseCPU
    from src.dev.Device import DmaDevice
    from src.mem.AbstractMemory import AbstractMemory
    from src.mem.ruby.network.garnet.GarnetLink import *
    from src.mem.ruby.network.garnet.GarnetNetwork import *
    from src.mem.ruby.network.Network import RubyNetwork
    from src.mem.ruby.slicc_interface.Controller import RubyController
    from src.mem.ruby.system.RubySystem import RubySystem
    from src.mem.ruby.system.Sequencer import (
        RubyPortProxy,
        RubySequencer,
    )
    from src.mem.SimpleMemory import SimpleMemory
    from src.mem.XBar import IOXBar
    from src.sim.System import System


class ChipletGarnetObjectInfo:
    """
    Used to hold information corresponding to a Garnet/Ruby object that has
    been aggregated into the main `GarnetNetwork` of a `ChipletSystem`.
    This should be the value of a `dict` whose keys are the corresponding
    Garnet/Ruby object (e.g., router or link).
    """

    def __init__(
        self,
        original_parent: BaseChipletSystem | BaseCPU,
        original_id: int = -1,
        description: str = "",
    ):
        """
        Args:
            original_parent (BaseChipletSystem | BaseCPU):
                A reference to the original parent of the object
                corresponding to this `GarnetObjectInfo` instance.
            original_id (int, optional):
                If the object corresponding to this `GarnetObjectInfo`
                instance has an ID attribute, and it was changed in
                the network aggregation, set this to the original ID.
                Defaults to -1.
            description (str, optional):
                A string description of what the corresponding object
                represents.
        """
        self.original_parent = original_parent
        self.original_id = original_id
        self.description = description


class ChipletSystem(BaseChipletSystem):
    """
    A `ChipletSystem` encapsulates a collection of child `Chiplet`s and/or
    `ChipletSystem`s. It acts as an abstraction of a physical processor
    package, a multi-chip module (MCM), or CCD, insofar as its role in
    containing multiple processors (heterogeneous or homogenous) across
    multiple chiplets linked by interconnects. Uses Ruby and Garnet.

    As of now, it is intended to be used to build a hierarchy bottom-up,
    i.e., first create `Chiplet`s with the system CPUs, then instantiate
    `ChipletSystem`s, passing in those `Chiplet`s into the constructor as
    nodes, and so on.
    """

    #! see BaseChipletSystem for additional important attributes

    type = "ChipletSystem"
    abstract = False
    clk_domain = Param.ClockDomain(Parent.clk_domain, "Clock domain")

    # class-local reference to `self.system.ruby`
    # (gem5 system SimObject's RubySystem)
    _ruby_system: RubySystem

    _root_garnet_network: GarnetNetwork

    # whether createSystem() has been called (successfully)
    _created: bool

    # see `createSystem()` or `class ChipletMetaTopology`
    _meta_topology: ChipletMetaTopology

    # storage for objects for network aggregation
    _root_all_routers_dict: dict[GarnetRouter, ChipletGarnetObjectInfo]
    _root_all_ext_links_dict: dict[GarnetExtLink, ChipletGarnetObjectInfo]
    _root_all_int_links_dict: dict[GarnetIntLink, ChipletGarnetObjectInfo]

    _global_router_count: int
    _global_link_count: int

    # store direct ref to main routers
    _root_main_routers_dict: dict[BaseChipletSystem, GarnetRouter]

    # store fake garnet networks (mostly for the one for the root)
    _root_fake_garnet_networks_dict: dict[BaseChipletSystem, FakeGarnetNetwork]

    def __init__(
        self,
        system: System,
        full_system: bool,
        TopologyClass: Type[BaseTopology],
        MemoryClass: Type[AbstractMemory],
        inter_node_link_lat: int,
        intra_node_link_lat: int,
        node_main_router_lat: int,
        nodes: Sequence[BaseChipletSystem] = [],
        cache_controller_link_lat: int = -1,
        cache_controller_router_lat: int = -1,
        dir_controller_link_lat: int = -1,
        dir_controller_router_lat: int = -1,
    ):
        """
        Instantiate a `ChipletSystem` object.

        Args:
            system (System):
                The gem5 `System` SimObject that this `ChipletSystem`
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
            system=system,
            full_system=full_system,
            TopologyClass=TopologyClass,
            MemoryClass=MemoryClass,
            nodes=nodes,
            inter_node_link_lat=inter_node_link_lat,
            node_main_router_lat=node_main_router_lat,
            intra_node_link_lat=intra_node_link_lat,
            cache_controller_link_lat=cache_controller_link_lat,
            cache_controller_router_lat=cache_controller_router_lat,
            dir_controller_link_lat=dir_controller_link_lat,
            dir_controller_router_lat=dir_controller_router_lat,
        )

        # * set gem5 SimObject parent-child relationship
        if not self.has_parent():
            self.set_parent(
                self._system, f"{self.__class__.__name__}{self.id}"
            )

        # * set gem5 `SimObject` parent property for nodes
        # * set _parent_sys property for children
        # currently only adding nodes in constructor is really supported
        for node in self._nodes:
            node.set_parent(self, f"{node.__class__.__name__}{node.id}")
            if not hasattr(node, "_parent_sys"):
                node._parent_sys = self

        self._global_router_count = 0
        self._global_link_count = 0

        self._root_all_routers_dict = {}
        self._root_all_ext_links_dict = {}
        self._root_all_int_links_dict = {}

        self._root_main_routers_dict = {}
        self._root_fake_garnet_networks_dict = {}

        self._garnet_network = FakeGarnetNetwork()

        self._main_router = self.create_and_register_router(
            latency=node_main_router_lat,
            description=f"{self.__class__.__name__}{self.id} main router",
        )

        self._created = False
        self._ruby_system = None

    # * helper methods for retrieving data on Garnet network objects
    def getRootRouters(self) -> list[GarnetRouter]:
        return list(self.getRoot()._root_all_routers_dict.keys())

    def getRootExtLinks(self) -> list[GarnetExtLink]:
        return list(self.getRoot()._root_all_ext_links_dict.keys())

    def getRootIntLinks(self) -> list[GarnetIntLink]:
        return list(self.getRoot()._root_all_int_links_dict.keys())

    def getGarnetObjectInfo(self, object):
        info = None

        if isinstance(object, GarnetRouter):
            routers = self.getRootRouters()
            if object in routers:
                info = self._root_all_routers_dict[object]

        if isinstance(object, GarnetExtLink):
            ext_links = self.getRootExtLinks()
            if object in ext_links:
                info = self._root_all_ext_links_dict[object]

        if isinstance(object, GarnetIntLink):
            int_links = self.getRootIntLinks()
            if object in int_links:
                info = self._root_all_int_links_dict[object]

        return info

    def getOriginalParent(self, object):
        info = self.getGarnetObjectInfo(object)
        if info is not None:
            return info.original_parent

    # * helper methods for creating and registering network objects
    def _registerNetworkObject(
        self,
        object: GarnetRouter | GarnetExtLink | GarnetIntLink,
        info: ChipletGarnetObjectInfo,
    ):
        """
        Registers a Garnet network object (router or link) in
        the root dictionaries mapping routers/links to information
        such as original parent, original ID, and description.

        ! Does NOT add the specified object to any `GarnetNetwork`
        ! or `FakeGarnetNetwork`!

        Args:
            object (GarnetRouter | GarnetExtLink | GarnetIntLink):
                The network object to register.
            info (ChipletGarnetObjectInfo): _description_
        """
        if isinstance(object, GarnetRouter):
            self.getRoot()._root_all_routers_dict[object] = info

        if isinstance(object, GarnetExtLink):
            self.getRoot()._root_all_ext_links_dict[object] = info

        if isinstance(object, GarnetIntLink):
            self.getRoot()._root_all_int_links_dict[object] = info

    def _generate_router_id(self):
        self._global_router_count += 1
        return self._global_router_count - 1

    def _generate_link_id(self):
        self._global_link_count += 1
        return self._global_link_count - 1

    def create_and_register_router(
        self,
        latency: int = -1,
        vcs_per_vnet: int = 4,
        description: str = "",
        add_router_to_self: bool = False,
    ) -> GarnetRouter:
        if latency == -1:
            latency = self.default_node_main_router_lat

        root = self.getRoot()

        new_router_id = root._generate_router_id()

        new_router = GarnetRouter(
            router_id=new_router_id,
            latency=latency,
            vcs_per_vnet=vcs_per_vnet,
        )

        new_router_info = ChipletGarnetObjectInfo(
            original_parent=self,
            original_id=new_router_id,
            description=description,
        )

        root._registerNetworkObject(new_router, new_router_info)

        if add_router_to_self:
            self._garnet_network.routers.append(new_router)

        return new_router

    def create_and_register_and_add_router(
        self,
        latency: int = -1,
        vcs_per_vnet: int = 4,
        description: str = "",
    ) -> GarnetRouter:
        return self.create_and_register_router(
            latency, vcs_per_vnet, description
        )

    def _mergeFakeGarnetNetworks(
        self,
        net1: FakeGarnetNetwork,
        net2: FakeGarnetNetwork,
        add_to_dict_if_missing: bool = True,
        skip_duplicates: bool = True,
    ):
        local_router_id = len(net1.routers)
        local_link_id = len(net1.ext_links) + len(net1.int_links)

        root = self.getRoot()

        for router in net2.routers:
            if root is not None:
                if router in root._root_all_routers_dict.keys():
                    info = root._root_all_routers_dict[router]
                    info.original_id = router.router_id
                elif add_to_dict_if_missing:
                    root._registerNetworkObject(
                        router,
                        ChipletGarnetObjectInfo(
                            # assume that callee is parent of `net1`
                            original_parent=self,
                            original_id=router.router_id,
                            description=f"combined into "
                            f"{net1} from {net2}",
                        ),
                    )
            if router in net1.routers:
                if skip_duplicates:
                    continue
                else:
                    warn(
                        f"adding duplicate router {router}"
                        f"during FakeGarnetNetwork merge"
                    )
            router.router_id = local_router_id
            local_router_id += 1
            net1.routers.append(router)

        for ext_link in net2.ext_links:
            if root is not None:
                if ext_link in root._root_all_ext_links_dict.keys():
                    info = root._root_all_ext_links_dict[ext_link]
                    info.original_id = ext_link.link_id
                elif add_to_dict_if_missing:
                    root._registerNetworkObject(
                        ext_link,
                        ChipletGarnetObjectInfo(
                            # assume that callee is parent of `net1`
                            original_parent=self,
                            original_id=ext_link.link_id,
                            description=f"combined into "
                            f"{net1} from {net2}",
                        ),
                    )
            if ext_link in net1.ext_links:
                if skip_duplicates:
                    continue
                else:
                    warn(
                        f"adding duplicate ext_link {ext_link}"
                        f"during FakeGarnetNetwork merge"
                    )
            ext_link.link_id = local_link_id
            local_link_id += 1
            net1.ext_links.append(ext_link)

        for int_link in net2.int_links:
            if root is not None:
                if int_link in root._root_all_int_links_dict.keys():
                    info = root._root_all_int_links_dict[int_link]
                    info.original_id = int_link.link_id
                elif add_to_dict_if_missing:
                    root._registerNetworkObject(
                        int_link,
                        ChipletGarnetObjectInfo(
                            # assume that callee is parent of `net1`
                            original_parent=self,
                            original_id=int_link.link_id,
                            description=f"combined into "
                            f"{net1} from {net2}",
                        ),
                    )
            if int_link in net1.int_links:
                if skip_duplicates:
                    continue
                else:
                    warn(
                        f"adding duplicate int_link {int_link}"
                        f"during FakeGarnetNetwork merge"
                    )
            int_link.link_id = local_link_id
            local_link_id += 1
            net1.int_links.append(int_link)

    def _rootAccumulateNetworkObjects(
        self,
        net: FakeGarnetNetwork,
    ):
        """
        Given a `FakeGarnetNetwork`, add all of its network objects
        to the root network object dictionaries.
        After this, accessing the lists via the appropriate getters
        (e.g., `self.getRootRouters()`) should include the objects
        in `net` appended to any existing registered objects.

        Args:
            net (FakeGarnetNetwork):
                The `FakeGarnetNetwork` to accumulate objects from.
        """
        root_existing = FakeGarnetNetwork(
            self.getRootRouters(),
            self.getRootExtLinks(),
            self.getRootIntLinks(),
        )
        self._mergeFakeGarnetNetworks(
            root_existing,
            net,
            add_to_dict_if_missing=True,
            skip_duplicates=True,
        )

    def _rootReAssignAllIDs(self):
        root = self.getRoot()
        rnet = root._garnet_network

        # reset counters
        root._global_router_count = 0
        root._global_link_count = 0

        for router in root.getRootRouters():
            str = f"router id {router.router_id}"
            info = root._root_all_routers_dict[router]
            info.original_id = router.router_id
            router.router_id = root._generate_router_id()
            str += f", new id = {router.router_id}; info = {info}"
            print(str)

        for ext_link in root.getRootExtLinks():
            str = f"ext_link id {ext_link.link_id}"
            info = root._root_all_ext_links_dict[ext_link]
            info.original_id = ext_link.link_id
            ext_link.link_id = root._generate_link_id()
            str += f", new id = {ext_link.link_id}; info = {info}"
            print(str)

        for int_link in root.getRootIntLinks():
            str = f"int_link id {int_link.link_id}"
            info = root._root_all_int_links_dict[int_link]
            info.original_id = int_link.link_id
            int_link.link_id = root._generate_link_id()
            str += f", new id = {int_link.link_id}; info = {info}"
            print(str)

    # * helper methods for `createSystem()`
    def _createLegacyRubyChiplet(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        piobus: IOXBar | None,
        dma_ports: list[DmaDevice],
        bootmem: AbstractMemory | None,
    ):
        """
        Creates a `ChipletSystem` using `Ruby.py`'s `create_system()`
        directly. Probably won't work well with hierarchical topologies.

        Args:
            See `_createChipletHierarchy()`.
        """
        # warn if hierarchical topology
        for node in self._nodes:
            if isinstance(node, ChipletSystem):
                warn(
                    "Using legacy ruby (Ruby.py) with hierarchical "
                    "chiplet topology. Unexpected behavior may occur."
                )

        # at top level, run `Ruby.create_system()`
        # the way Ruby seems to be set up right now,
        # this should only run once, and not sure if
        # hierarchical topologies are plausible
        if not hasattr(self, "_parent_sys"):
            # configure options that Ruby.py expects
            options.network = "garnet"
            options.topology = self._topology_cls.__name__
            options.mem_type = self._memory_cls.__name__

            # create ruby system
            Ruby.create_system(
                options=options,
                full_system=self._is_full_system,
                system=self._system,
                piobus=piobus,
                dma_ports=dma_ports,
                bootmem=bootmem,
            )

            self._x86_connect_ruby_ports()

    def _instantiateRootGarnetNetwork(self):
        if not self.isRoot():
            fatal(
                f"User tried to instantiate GarnetNetwork for "
                f"non-root ChipletSystem (ID {self.id})."
            )

        if hasattr(self, "_garnet_network") and isinstance(
            self._garnet_network, GarnetNetwork
        ):
            fatal(
                f"GarnetNetwork already exists for ChipletSystem "
                f"ID {self.id}!"
            )

        # Create the `GarnetNetwork` for the entire `ChipletSystem`
        root_garnet_network = GarnetNetwork(
            ruby_system=self._ruby_system,
            topology=self._topology_cls.__name__,
            routers=[],  # populate later
            ext_links=[],  # populate later
            int_links=[],  # populate later
            netifs=[],  # will be populated by `init_network()`
            ignore_mesh_chk=True,
        )
        #! routers, ext/int links, and netifs will be assigned
        #! in `_initializeRootGarnetNetwork()`, after being
        #! created via topology templates.

        # * ignore_mesh_chk:
        # if `num_rows > 0` we will fail an assertion
        # when the C++ objects are instantiated
        # (`m_num_rows * m_num_cols == m_routers.size()`)
        # (which will not be true since we added routers)
        # so we had to add an override to prevent this
        # TODO NEW HIERARCHY: this might not be needed anymore

        return root_garnet_network

    def _initializeRootGarnetNetwork(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
    ):
        assert isinstance(self._garnet_network, GarnetNetwork)

        # * populate `GarnetNetwork` with routers and links.
        # these should've been created since calling
        # `_instantiateRootGarnetNetwork()` by all `Chiplet[System]`s
        # using `_createTopology()` (and thus `makeTopology()`).
        self._garnet_network.routers = self.getRootRouters()
        self._garnet_network.ext_links = self.getRootExtLinks()
        self._garnet_network.int_links = self.getRootIntLinks()

        # * initialize network
        # `Network.py:init_network()` does what we need without
        # any major problems as of now
        options.network = "garnet"  # only required by `Network.py`
        #! `init_network` requires the int and ext links to be set.
        #! it also instantiates all the requisite network interfaces.
        init_network(
            options=options,
            network=self._garnet_network,
            InterfaceClass=GarnetNetworkInterface,
        )

    def _instantiateRubySystem(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
    ):
        if self.isRoot():  # redundant but just in case
            # instantiate `RubySystem`, to be "created" next
            self._system.ruby = RubySystem()
            self._ruby_system = self._system.ruby

            # Generate pseudo filesystem
            FileSystemConfig.config_filesystem(self._system, options)
        else:
            fatal(
                f"User tried to instantiate Ruby system on non-root "
                f"ChipletSystem (ID {self.id}). Do not call "
                f"_instantiateRubySystem() from outside ChipletSystem."
            )

        if self.getRoot()._ruby_system is None:
            panic(
                f"RubySystem is None for "
                f"{self.__class__.__name__} {self.id}!"
            )

    def _rubyProtocolCreateSystem(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
    ):
        """
        Creates the Ruby system.
        The `RubySystem` object must be instantiated first.
        ! `self._system.ruby.network` must be set to a valid
        ! `GarnetNetwork` before calling this method.
        Sets the following attributes of this `ChipletSystem`:
         - `self._cpu_sequencers`
         - `self._dir_controllers`
         - `self._meta_topology`

        Args:
            See `_createChipletHierarchy()`.
        """
        if not hasattr(self._system, "ruby"):
            if hasattr(self, "_ruby_system"):
                panic("RubySystem was not properly assigned to gem5 System!")
            else:
                fatal(
                    "User tried to create Ruby system, but it hasn't been "
                    "instantiated. Do not call _createRubySystem() "
                    "from outside ChipletSystem."
                )

        if not hasattr(self._system.ruby, "network"):
            if hasattr(self, "_garnet_network"):
                panic(
                    "Garnet network was not properly assigned to "
                    "gem5 System!"
                )

        # if not hasattr(self, "_garnet_network"):
        #     fatal(
        #         "User tried to create Ruby system, but Garnet network "
        #         "has not yet been created. Do not call "
        #         "_createRubySystem() from outside ChipletSystem."
        #     )

        # if isinstance(self._garnet_network, FakeGarnetNetwork):
        #     panic(
        #         "ChipletSystem's GarnetNetwork is FakeGarnetNetwork! "
        #         "Either GarnetNetwork failed to instantiate, or "
        #         "_createRubySystem() was called before "
        #         "_createRootGarnetNetwork()!"
        #     )

        # * Create the Ruby/memory system
        try:
            # Call `create_system()` corresponding to the
            # Ruby protocol that gem5 was compiled with.
            # Note: this can be the `MULTIPLE` protocol.
            # The corresponding `create_system()` method can
            # be found in `configs/ruby/<PROTOCOL>.py`.
            # For example/reference:
            if TYPE_CHECKING:
                from ruby.MESI_Two_Level import create_system
            # Currently, MOST of the protocols (except GPU_VIPER
            # and MOESI_AMD_Base, which use the `Cluster` topology)
            # call `create_topology()` from `Ruby.py`, passing
            # in the controllers created in the `create_system()`
            # for that Ruby protocol.
            #
            # Since we have a hierarchy, and all topologies
            # inheriting from `SimpleTopology` are flat, this
            # doesn't quite work as is.
            #
            # Instead, we want to control how the controllers
            # are distributed across the hierarchy ourselves.
            # To accomplish this without breaking/changing
            # everything, we created a "dummy"/"meta" topology
            # (`topologies.ChipletMetaTopology`) which we use
            # to avoid affecting the actual topology that the
            # user intended the parent `ChipletSystem` to use.
            #
            # This topology is then used to retrieve the
            # controllers and distribute them appropriately.
            #
            # Thus, we must temporarily change `options.topology`
            # to get the Ruby protocol to use our meta topology.
            tmp = None
            if hasattr(options, "_topology"):
                tmp = options.topology
            options.topology = "ChipletMetaTopology"
            (
                self._cpu_sequencers,
                self._dir_controllers,
                self._meta_topology,
            ) = import_module(f"ruby.{buildEnv['PROTOCOL']}").create_system(
                options,
                full_system=self._is_full_system,
                system=self._system,
                dma_ports=dma_ports,
                bootmem=bootmem,
                ruby_system=self._ruby_system,
                cpus=self._system.cpu,
            )
            # `options.topology` should've initially been the same
            # as `self._topology_cls`, but just in case it wasn't:
            if tmp:
                options.topology = tmp
        except Exception as e:
            panic(
                "Could not create Ruby system for Ruby protocol "
                f"{buildEnv['PROTOCOL']}; exception thrown: {e}"
            )

    def _sortTopologyControllers(
        self,
        l2_is_private: bool,
        include_dir_ctrls_in_topology: bool = False,
        include_cache_ctrls_in_topology: bool = False,
    ):
        """
        Populates lists of Ruby controllers for this `Chiplet[System]`
        based on what `create_system()` of the Ruby protocol generated.
        Specifically, populates:
         - `self._topology_controllers`
         - `self._dir_controllers`
         - `self._cache_controllers`
        ! `_rubyProtocolCreateSystem()` needs to be called before this.
        """

        # grab all controllers
        # this includes every controller, since that's what's
        # returned from `create_system()` of the Ruby protocol
        # (see `_createRubySystem()` for more details).
        all_controllers = self.getRoot()._meta_topology.nodes

        for c in all_controllers:
            # we should already have a list of all directory controllers in
            # `_dir_controllers` after `_createRubySystem()`, but verify
            if self.isDirController(c):
                if c not in self._dir_controllers:
                    warn(
                        f"Found directory controller that was not in this "
                        f"ChipletSystem's list of directory controllers: "
                        f"{c}"
                    )
                    self._dir_controllers.append(c)
                if include_dir_ctrls_in_topology:
                    self._topology_controllers.append(c)
            else:
                # not directory controller, should be a different cache
                # controller (or perhaps a DMA controller)
                # todo: we don't really handle DMA controllers specially

                #! for now this assumes that LLC is globally shared.
                # todo: make ^ not the case

                if self.getControllerCPU(c, l2_is_private) is None:
                    self._cache_controllers.append(c)
                    if include_cache_ctrls_in_topology:
                        self._topology_controllers.append(c)

    def _manualLinkControllers(
        self,
        controllers: list[RubyController],
        ctrl_router_lat: int,
        ctrl_to_router_link_lat: int,
        ctrl_router_to_main_link_lat: int,
    ):
        """
        Manually link a list of controllers to routers.
        For each controller, this method creates, registers, and adds
        a new routers with latency `ctrl_router_lat`. Then, the new router
        is connected to the controller with a link latency of
        `ctrl_to_router_link_lat`. Finally, the new router is linked to the
        main router for this `Chiplet[System]` with a link latency of
        `ctrl_router_to_main_link_lat`.

        Args:
            controllers (list[RubyController]):
                List of controllers to link and create routers for.
            ctrl_router_lat (int):
                Latency of the new `GarnetRouter` corresponding to
                each controller.
            ctrl_to_router_link_lat (int):
                Latency of the `GarnetExtLink`s between each controller
                and its corresponding router.
            ctrl_router_to_main_link_lat (int):
                Latency of the `GarnetIntLink`s between the router
                corresponding to each controller and the main router
                for this `Chiplet[System]`. Note that two such links
                are created, since `GarnetIntLink`s are unidirectional.
        """
        for ctrl in controllers:
            # is dir ctrl linked?
            lnk = self._getExtLinkFromController(ctrl)
            if lnk is None:
                ctrl_router = self.create_and_register_and_add_router(
                    latency=ctrl_router_lat,
                )
                self._connectRubyControllerGarnet(
                    ctrl=ctrl,
                    router=ctrl_router,
                    link_lat=ctrl_to_router_link_lat,
                )

                # if you want to change the latency of this link,
                # use `_findIntLinkFromRouters(main_router, ctrl_router)`
                self._biLinkGarnetRouters(
                    self._main_router,
                    ctrl_router,
                    ctrl_router_to_main_link_lat,
                )

    def _manualLinkDirectoryControllers(self):
        self._manualLinkControllers(
            controllers=self._dir_controllers,
            ctrl_router_lat=self.default_dir_controller_router_lat,
            ctrl_to_router_link_lat=self.default_dir_controller_link_lat,
            ctrl_router_to_main_link_lat=self.default_intra_node_link_lat,
        )

    def _manualLinkCacheControllers(self):
        self._manualLinkControllers(
            controllers=self._cache_controllers,
            ctrl_router_lat=self.default_cache_controller_router_lat,
            ctrl_to_router_link_lat=self.default_cache_controller_link_lat,
            ctrl_router_to_main_link_lat=self.default_intra_node_link_lat,
        )

    def _rubyFinalSetup(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        piobus: IOXBar | None,
    ):
        if not self.isRoot():
            fatal(
                "User tried to call _rubyFinalSetup() on "
                "non-root ChipletSystem."
            )

        if not isinstance(self._garnet_network, GarnetNetwork):
            fatal(
                "_rubyFinalSetup() was called before "
                "GarnetNetwork was properly initialized. "
                "(Probably out of order with respect to "
                "`_createChipletHierarchy()`)"
            )
        # * do some port configuration from `Ruby.py`
        self._ruby_connect_system_ports(piobus)

        # todo: don't rely on `Ruby.py` for this
        # unfortunately, in order to get `dir_cntrls`, the Ruby
        # protocols also call back to `Ruby.create_directories`.
        # so can't completely avoid it without major refactoring
        options.mem_type = self._memory_cls.__name__
        Ruby.setup_memory_controllers(
            system=self._system,
            ruby=self._ruby_system,
            dir_cntrls=self._dir_controllers,
            options=options,
        )

        # * more stuff from `Ruby.py`
        # Connect the cpu sequencers and the piobus
        if piobus != None:
            for cpu_seq in self._cpu_sequencers:
                cpu_seq.connectIOPorts(piobus)

        assert isinstance(self._ruby_system.network, GarnetNetwork)
        self._ruby_system.number_of_virtual_networks = (
            self._ruby_system.network.number_of_virtual_networks
        )  # implicit line break on assign due to 76 char limit
        self._ruby_system._cpu_ports = self._cpu_sequencers
        self._ruby_system.num_of_sequencers = len(self._cpu_sequencers)

        # Create a backing copy of physical memory in case required
        if options.access_backing_store:
            self._ruby_system.access_backing_store = True
            self._ruby_system.phys_mem = SimpleMemory(
                range=self._system.mem_ranges[0], in_addr_map=False
            )

    def _createChipletHierarchy(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        piobus: IOXBar | None,
        dma_ports: list[DmaDevice],
        bootmem: AbstractMemory | None,
        l2_is_private: bool,
        include_dir_ctrls_in_topology: bool,
        include_cache_ctrls_in_topology: bool,
        node_router_topology: CSMRT,
    ):
        """
        Recursively create this chiplet hierarchy.
        This contains most of the core logic for creating
        the `ChipletSystem` and its children.

        Args:
            See `createSystem()`.
        """

        #! this code is adapted from `Ruby.py` and `Network.py`,
        #! as changes were needed in order to support hierarchies.
        is_root = self.isRoot()
        root = self.getRoot()

        # TODO NEW HIERARCHY:
        """
        # TODO | Figure out how to properly divide list of controllers
        # TODO | across multiple `ChipletSystem`s.
        # TODO | Currently, this doesn't really happen.
        # TODO | This will probably involve adjusting how
        # TODO | `_sortRubyControllers()` works.
        # TODO |
        # TODO | It's relatively easy to determine if a controller belongs
        # TODO | to a `Chiplet`, because we can just check if it
        # TODO | corresponds to one of that `Chiplet`'s cores.
        # TODO | Determining if a controller belongs to a `ChipletSystem`
        # TODO | will require some additional infrastructure.
        # TODO |
        # TODO | Right now, it's structured so that each `Chiplet[System]`
        # TODO | grabs the entire list of all controllers from the meta
        # TODO | topology / Ruby protocol, and tries to figure out if
        # TODO | each belongs to it or not.
        """

        if is_root:
            # create `RubySystem` and assign to `self._ruby_system`
            # also some filesystem config
            self._instantiateRubySystem(options)

            root_garnet_network = self._instantiateRootGarnetNetwork()

            # Ruby expects the `System` object to have a `ruby` attr,
            # and that the `ruby` property has a `network` attribute.
            self._system.ruby.network = root_garnet_network
            # note that our `self._ruby_system` should reference the same
            # object as `self._system.ruby`, but Ruby doesn't know or care
            # about our `self._ruby_system`, so avoid setting its attrs.
            # reading or using it as a `RubySystem` is fine.

            # call `create_system()` for the appropriate Ruby protocol
            # this populates the lists of sequencers, dir controllers,
            # and creates the meta topology (to get list of all ctrls)
            self._rubyProtocolCreateSystem(options, dma_ports, bootmem)

            # after this, our lists of controllers should be populated
            self._sortTopologyControllers(
                l2_is_private,
                include_dir_ctrls_in_topology,
                include_cache_ctrls_in_topology,
            )

            if not include_dir_ctrls_in_topology:
                self._manualLinkDirectoryControllers()

            if not include_cache_ctrls_in_topology:
                self._manualLinkCacheControllers()

        #! this is where (some of) the magic happens.
        """
        We recursively "create" the children (nodes). Each has its own
        topology (from `configs/topologies`). To create a Ruby system and
        Garnet network representing a hierarchical topology without
        breaking all the existing topologies and interfering with the
        cache coherence protocol, for each child/node, we use
        `makeTopology()` on a fake Garnet network to collect all the
        Garnet routers and links, all structured according to the topology
        of the child. Creating multiple `GarnetNetwork`s and trying to
        combine them does not work properly, so this method is better.
        The root `ChipletSystem` also has a topology, so we also need to
        do the same thing for it (this), and we do that first.
        """
        # instantiating FakeGarnetNetwork has no prereqs
        # so just do it in the constructor
        # self._garnet_network = FakeGarnetNetwork()
        root._root_fake_garnet_networks_dict[self] = self._garnet_network

        # register main router in main routers dict
        root._root_main_routers_dict[self] = self._main_router

        # * some topologies override network's lists of routers/links,
        # * so we expect that and prepare accordingly
        # we likely don't have many routers/links in the fake network yet,
        # but better to be safe
        self._rootAccumulateNetworkObjects(self._garnet_network)

        # gn = self._garnet_network
        # tmp_routers = [r for r in gn.routers]
        # tmp_ext_links = [l for l in gn.ext_links]
        # tmp_int_links = [l for l in gn.int_links]
        # tmp_network = FakeGarnetNetwork(
        #     tmp_routers, tmp_ext_links, tmp_int_links
        # )

        # populate `FakeGarnetNetwork` with routers and links
        # based on the specified topology for this `Chiplet[System]`
        self._createTopology(options, self._topology_controllers)

        # recombine existing routers/links with those from the topology
        # self._mergeFakeGarnetNetworks(
        #     tmp_network,
        #     self._garnet_network,
        #     add_to_dict_if_missing=True,
        #     skip_duplicates=True,
        # )
        # self._garnet_network = tmp_network
        # root._root_fake_garnet_networks_dict[self] = self._garnet_network

        # * need to instantiate and connect to children before init network
        for node in self._nodes:
            # todo: SimObject parenting to __init__(), hopefully that works
            # set `SimObject` parent hierarchy
            # node.set_parent(self, f"{node.__class__.__name__}{node.id}")

            # * create node
            if isinstance(node, ChipletSystem):
                # cascade ruby system, although it shouldn't be needed
                node._ruby_system = self._ruby_system

                # most of these args technically shouldn't be needed
                # for any child systems, since they're only used by root
                node._createChipletHierarchy(
                    options,
                    piobus,
                    dma_ports,
                    bootmem,
                    l2_is_private,
                    include_dir_ctrls_in_topology,
                    include_cache_ctrls_in_topology,
                    node_router_topology,
                )
            elif isinstance(node, Chiplet):
                node.createChiplet(options, l2_is_private)

            # * connect child nodes to main router
            # this will result in a topology where the child main routers
            # are not connected to each other, but only to the parent's
            # main router. this may or may not be desirable,
            # depending on what you are trying to model.
            if (
                node_router_topology == CSMRT.ParentOnly
                or node_router_topology == CSMRT.Pt2Pt
            ):
                # * connect each node's main router to our main router
                # i.e., connect child directly to parents
                # this is `ParentOnly`, but Pt2Pt also requires this
                # so let's reuse the code
                self._biLinkGarnetRouters(
                    self._main_router,
                    node._main_router,
                    self.default_inter_node_link_lat,
                )

                # * connect all nodes to each other if using
                # * point-to-point main router topology
                if node_router_topology == CSMRT.Pt2Pt:
                    for other in self._nodes:
                        if node != other:
                            self._biLinkGarnetRouters(
                                node._main_router,
                                other._main_router,
                                self.default_inter_node_link_lat,
                            )

            #! renumber all of the routers and links as we
            #! transfer all of them into the root lists
            # since this triggers for all `ChipletSystem`s, it should
            # work on any hierarchy, although it might be rather slow
            self._rootAccumulateNetworkObjects(node._garnet_network)

        self._rootAccumulateNetworkObjects(self._garnet_network)

        if is_root:
            #! this where the rest of the magic happens.
            """
            Once we have created all of the topologies of all children,
            and collected all routers, links, etc., we can then combine
            everything into a singular `GarnetNetwork` that aligns with
            a hierarchical topology, constructed from multiple simpler
            topologies (from `configs/topologies`), without having to
            manually define the hierarchical topology at a low level.
            """
            self._rootReAssignAllIDs()

            # replace the root's `FakeGarnetNetwork` with the real one here
            self._garnet_network = root_garnet_network

            # * do the actual combination into the real `GarnetNetwork`
            self._initializeRootGarnetNetwork(options)

            self._rubyFinalSetup(options, piobus)

            self._x86_connect_ruby_ports()

    def createSystem(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        use_legacy_ruby: bool = False,
        piobus: IOXBar | None = None,
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
        l2_is_private: bool = False,
        include_dir_ctrls_in_topology: bool = True,
        include_cache_ctrls_in_topology: bool = True,
        node_router_topology: CSMRT = CSMRT.ParentOnly,
    ):
        """
        Recursively construct this `ChipletSystem` based on the list
        of nodes and other parameters (e.g., latencies) specified
        upon creation of the object(s).
        Specifically, create the backing Garnet network(s) and other
        necessary objects.

        Args:
            options (Namespace):
                argparse options from CLI and/or other definitions
                such as `Network.define_options()`.
            use_legacy_ruby (bool):
                Whether to use (legacy) `Ruby.py` or not.
                Defaults to False.
            piobus (IOXBar, optional):
                System proxy port IO bus. Defaults to None.
                Needed only to support direct memory access (DMA).
                See Also: `AbstractBoard.get_io_bus()`.
            dma_ports (list[DmaDevice], optional):
                Direct memory access ports. Defaults to an empty list.
                Needed to support DMA.
            bootmem (AbstractMemory, optional):
                Boot memory. Defaults to None.
                Usually not needed, seemingly only for some FS configs.
            l2_is_private (bool, optional):
                If the L2 cache is private (one per CPU).
                Used for distributing controllers.
                Defaults to False.

            include_dir_ctrls_in_topology (bool, optional):
                If the directory controllers should be included directly
                in the specified topology of this `ChipletSystem`.

                However, if the specified topology is a mesh, unless the
                number of directory controllers matches the number of
                directory controllers expected by the mesh, the directory
                controllers will not be included in the topology.

                If this is `False`, directory controllers will only be
                connected to the main router of this `ChipletSystem`.
                Defaults to True.

            include_cache_ctrls_in_topology (bool, optional):
                If the cache controllers for this level of the hierarchy
                should be included directly in the specified topology
                of this `ChipletSystem`.

                However, if the specified topology is a mesh, the cache
                controllers will not be included in the topology.

                If this is `False`, cache controllers will only be
                connected to the main router of this `ChipletSystem`.
                Defaults to True.

            node_router_topology (ChipletSystemMainRouterTopology):
                See `class ChipletSystemMainRouterTopology`.
                Import name is `CSMRT`.
                Currently valid options are `ParentOnly` and `Pt2Pt`.
                Defaults to `ParentOnly`.
        """

        if self._created is True:
            warn(
                "Attempted to call createSystem() on this "
                "ChipletSystem, but it has already been created."
            )
            return

        if options is None:
            fatal("Called createSystem() with no argparse options.")

        if use_legacy_ruby:
            self._createLegacyRubyChiplet(
                options,
                piobus,
                dma_ports,
                bootmem,
            )
        else:
            self._createChipletHierarchy(
                options,
                piobus,
                dma_ports,
                bootmem,
                l2_is_private,
                include_dir_ctrls_in_topology,
                include_cache_ctrls_in_topology,
                node_router_topology,
            )

        self._created = True

    def _ruby_connect_system_ports(self, piobus: IOXBar):
        # * some port configuration lifted from `Ruby.py`
        # ? not sure why some of this stuff errors. perhaps...
        # ? ...things may have flipped since `Ruby.py` was written?
        # Create a port proxy for connecting the system port. This is
        # independent of the protocol and kept in the protocol-agnostic
        # part (i.e. here).
        system = self._system  # only for staying under 76 chars/line
        sys_port_proxy = RubyPortProxy(ruby_system=self._ruby_system)
        if piobus is not None:
            #! the line below type-errors (response port = request port)
            sys_port_proxy.pio_request_port = (  # type: ignore
                piobus.cpu_side_ports
            )  # implicit line break on assign due to 76 char limit

        # Give the system port proxy a SimObject parent without creating
        # a full-fledged controller
        system.sys_port_proxy = sys_port_proxy

        # Connect the system port for loading of binaries etc
        #! the line below type-errors (response port = request port)
        system.system_port = system.sys_port_proxy.in_ports  # type: ignore

    def _x86_connect_ruby_ports(self):
        """
        Connects ruby ports for x86 CPUs present in the system.
        """

        # todo: make this not a hack
        for i, cpu in enumerate(self._system.cpu):
            if issubclass(cpu.__class__, BaseCPU):
                # figure out if we're using x86 ISA
                archs_enabled = list(
                    filter(lambda var: buildEnv[var], ["USE_X86_ISA"])
                )
                if len(archs_enabled) == 1:
                    arch = archs_enabled[0]
                    if arch == "USE_X86_ISA":
                        # has the interrupt controller
                        # been created yet?
                        if len(cpu.interrupts) == 0:
                            cpu.createInterruptController()
                            # probably should let the user do this,
                            # but this should be done near when
                            # create_system() is called, which
                            # would then require another method
                            warn(
                                "ChipletSystem: detected x86 "
                                "ISA and created interrupt "
                                "controller automatically."
                            )

                        if len(cpu.interrupts) != 0:
                            ruby_port = self._system.ruby._cpu_ports[i]
                            ruby_port.connectCpuPorts(cpu)
                        else:
                            panic(
                                "Interrupt controller was created, "
                                "but no interrupts were found."
                            )

    def to_string(self):
        import textwrap

        parent_id = "None"
        if hasattr(self, "_parent_sys") and self._parent_sys is not None:
            parent_id = self._parent_sys.id
        return f"""
        {self.__class__.__name__} (ID: {self.id}) [
            Parent ID: {parent_id}
            Created: {self._created}
            Protocol: {self.protocol}
            Topology: {self._topology_cls.__name__}
            Connections: {self._connected_nodes}
            Nodes: [
        {textwrap.indent(
            "\n\n        ".join([n.to_string() for n in self._nodes]),
            "        "
        )}
            ]:{self.id}n
        ]:{self.id}"""
