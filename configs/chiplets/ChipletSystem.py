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
    """

    #! see BaseChipletSystem for additional important attributes

    type = "ChipletSystem"
    abstract = False
    clk_domain = Param.ClockDomain(Parent.clk_domain, "Clock domain")

    # class-local reference to `self.system.ruby`
    # (gem5 system SimObject's RubySystem)
    _ruby_system: RubySystem

    # whether createSystem() has been called (successfully)
    _created: bool

    # see `createSystem()` or `class ChipletMetaTopology`
    _meta_topology: ChipletMetaTopology

    # storage for objects for network aggregation
    _root_all_routers_dict: dict[GarnetRouter, ChipletGarnetObjectInfo]
    _root_all_ext_links_dict: dict[GarnetExtLink, ChipletGarnetObjectInfo]
    _root_all_int_links_dict: dict[GarnetIntLink, ChipletGarnetObjectInfo]

    _fake_garnet_networks_dict: dict[BaseChipletSystem, FakeGarnetNetwork]

    def __init__(
        self,
        system: System,
        full_system: bool,
        TopologyClass: Type[BaseTopology],
        MemoryClass: Type[AbstractMemory],
        inter_node_link_lat: int,
        inter_node_router_lat: int,
        nodes: Sequence[BaseChipletSystem] = [],
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
                to the topology to be used in this `ChipletSystem`.
                Only the root `ChipletSystem` needs to specify this.
            MemoryClass (Type[AbstractMemory]):
                The class corresponding to the memory configuration
                for the gem5 `System` this `ChipletSystem` is part of.
            inter_node_link_lat (int):
                The default latency (in cycles) of the links between
                nodes for this layer of abstraction.
            inter_node_router_lat (int):
                The default latency (in cycles) of the routers
                corresponding to the the links between nodes
                for this layer of abstraction.
            nodes (Sequence[BaseChipletSystem]):
                Optional list of nodes. May be passed to constructor
                or added separately using `addNode()`.
        """

        super().__init__(
            system=system,
            full_system=full_system,
            TopologyClass=TopologyClass,
            MemoryClass=MemoryClass,
            nodes=nodes,
            inter_node_link_lat=inter_node_link_lat,
            inter_node_router_lat=inter_node_router_lat,
        )

        # * set gem5 SimObject parent-child relationship
        if not self.has_parent():
            self.set_parent(
                self._system, f"{self.__class__.__name__}{self.id}"
            )

        # * set _parent_sys property for children
        # currently only adding nodes in constructor is really supported
        for node in self._nodes:
            if not hasattr(node, "_parent_sys"):
                node._parent_sys = self

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

    # * helper methods for `createSystem()`

    def _createChildren(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        l2_is_private: bool = False,
    ):
        pass

    def _createLegacyRubyChiplet(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        piobus: IOXBar | None = None,
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
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

    def _createRootGarnetNetwork(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
    ):
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
        self._garnet_network = GarnetNetwork(
            ruby_system=self._ruby_system,
            topology=self._topology_cls.__name__,
            routers=self.getRootRouters(),
            ext_links=self.getRootExtLinks(),
            int_links=self.getRootIntLinks(),
            netifs=[],  # will be populated by `init_network()`
            ignore_mesh_chk=True,
        )

        # * ignore_mesh_chk:
        # if `num_rows > 0` we will fail an assertion
        # when the C++ objects are instantiated
        # (`m_num_rows * m_num_cols == m_routers.size()`)
        # (which will not be true since we added routers)
        # so we had to add an override to prevent this
        # TODO NEW HIERARCHY: this might not be needed anymore

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

    def _createRubySystem(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
    ):
        """
        Creates the Ruby system.
        The `RubySystem` object must be instantiated first.
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

        if not hasattr(self, "_garnet_network"):
            fatal(
                "User tried to create Ruby system, but Garnet network "
                "has not yet been created. Do not call "
                "_createRubySystem() from outside ChipletSystem."
            )

        if isinstance(self._garnet_network, FakeGarnetNetwork):
            panic(
                "ChipletSystem's GarnetNetwork is FakeGarnetNetwork! "
                "Either GarnetNetwork failed to instantiate, or "
                "_createRubySystem() was called before "
                "_createRootGarnetNetwork()!"
            )

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

    def _sortRubyControllers(
        self,
        l2_is_private: bool,
        include_dir_ctrls_in_topology: bool = False,
    ):
        """
        Populates lists of Ruby controllers for this `Chiplet[System]`
        based on what `create_system()` of the Ruby protocol generated.
        Specifically, populates:
         - `self._ruby_controllers`
         - `self._dir_controllers`
         - `self._non_dir_controllers`
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
                    self._ruby_controllers.append(c)
            else:
                # not directory controller, should be a different cache
                # controller (or perhaps a DMA controller)
                # todo: we don't really handle DMA controllers specially

                #! for now this assumes that LLC is globally shared.
                # todo: make ^ not the case

                self._non_dir_controllers.append(c)
                self._ruby_controllers.append(c)

                if self.getControllerCPU(c, l2_is_private) is not None:
                    warn(
                        f"Found non-directory controller that seems to "
                        f"belong to a CPU: {c}"
                    )

    def _createChipletHierarchy(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        piobus: IOXBar | None = None,
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
        l2_is_private: bool = False,
        include_dir_ctrls_in_topology: bool = False,
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
        self._garnet_network = FakeGarnetNetwork()
        self._fake_garnet_networks_dict[self] = self._garnet_network

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

            #! this where the rest of the magic happens.
            """
            Once we have created all of the topologies of all children,
            and collected all routers, links, etc., we can then combine
            everything into a singular `GarnetNetwork` that aligns with
            a hierarchical topology, constructed from multiple simpler
            topologies (from `configs/topologies`), without having to
            manually define the hierarchical topology at a low level.
            """
            self._createRootGarnetNetwork(options)
            #! ^ also instantiates network via `init_network()`...
            #! not sure if this needs to be done later

            # Ruby expects the `System` object to have a `ruby` attr,
            # and that the `ruby` property has a `network` attribute.
            self._ruby_system.network = self._garnet_network

            self._createRubySystem(options, dma_ports, bootmem)

            # after this, our lists of controllers should be populated
            self._sortRubyControllers(
                l2_is_private,
                include_dir_ctrls_in_topology,
            )

    def createSystem(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        use_legacy_ruby: bool = False,
        piobus: IOXBar | None = None,
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
        l2_is_private: bool = False,
        include_dir_ctrls_in_topology: bool = False,
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
                in the specified topology of the root `ChipletSystem`.
                If this is `False`, directory controllers will only be
                connected to the main router of the root `ChipletSystem`.
                Defaults to False.
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
            self._createLegacyRubyChiplet(options)
        else:
            self._createChipletHierarchy(options)

        self._created = True

    def createSystemOld(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        use_legacy_ruby: bool = False,
        piobus: IOXBar | None = None,
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
        l2_is_private: bool = False,
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
                )

                self._x86_connect_ruby_ports()
        else:
            #! this branch is adapted from `Ruby.py` and `Network.py`,
            #! as changes were needed in order to support hierarchies.

            # upon review I don't think that the previous condition(s) for
            # checking root actually would've worked, so added isRoot().

            # * this block should only run for the parent `ChipletSystem`.
            # if not hasattr(self._system, "ruby"):
            if self.isRoot():
                # instantiate `RubySystem`, to be "created" next
                self._system.ruby = RubySystem()
                self._ruby_system = self._system.ruby

                # Generate pseudo filesystem
                FileSystemConfig.config_filesystem(self._system, options)

            if self._ruby_system is None:
                panic(
                    f"RubySystem is None for "
                    f"{self.__class__.__name__} {self.id}!"
                )

            # only create a `GarnetNetwork` at the root.
            # otherwise, use `FakeGarnetNetwork` to collect routers and
            # links, then construct the root `GarnetNetwork` with them.
            if self.isRoot():
                # Create the `GarnetNetwork` for the entire `ChipletSystem`
                self._garnet_network = GarnetNetwork(
                    ruby_system=self._ruby_system,
                    topology=self._topology_cls.__name__,
                    routers=[],
                    ext_links=[],
                    int_links=[],
                    netifs=[],
                    ignore_mesh_chk=True,
                )

                # ignore_mesh_chk:
                # if `num_rows > 0` we will fail an assertion
                # when the C++ objects are instantiated
                # (`m_num_rows * m_num_cols == m_routers.size()`)
                # (which will not be true since we added routers)
                # so we had to add an override to prevent this
            else:  # not root
                self._garnet_network = FakeGarnetNetwork()

            #! create Ruby system
            # * this block should only run for the parent `ChipletSystem`.
            # if self._system.ruby is not None:
            if self.isRoot():
                # Ruby expects the `System` object to have a `ruby` attr,
                # and that the `ruby` property has a `network` attribute.
                self._ruby_system.network = self._garnet_network

                # * Create the Ruby/memory system
                try:
                    # Call `create_system()` corresponding to the
                    # Ruby protocol that gem5 was compiled with.
                    # Note: this can be the `MULTIPLE`` protocol.
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
                        cpu_sequencers,
                        dir_controllers,
                        self._meta_topology,
                    ) = import_module(
                        f"ruby.{buildEnv['PROTOCOL']}"
                    ).create_system(
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

            # TODO NEW HIERARCHY: shouldn't be needed
            # ? with new network building strategy this shouldn't be needed
            # # set `SimObject` parent hierarchy
            # # make sure not to set root `GarnetNetwork` parent
            # # if we did, it would break Ruby because it expects
            # # the gem5 `System` to have a `.network` param
            # if not self._garnet_network.has_parent():
            #     print(
            #         f"setting gem5 parent for GarnetNetwork of "
            #         f"ChipletSystem ID {self.id}"
            #     )
            #     self._garnet_network.set_parent(self, "network")

            # * collect controllers for this layer of the hierarchy
            # type error ignorable because subclasses of `BaseTopology`
            # typically override constructor with this signature
            all_controllers = self.getRoot()._meta_topology.nodes

            for c in all_controllers:
                if self.isDirController(c):
                    # if directory controller, don't include in main
                    # ruby controllers list
                    self._dir_controllers.append(c)
                elif self.getControllerCPU(c, l2_is_private) is None:
                    # if it doesn't belong to a CPU, it doesn't belong
                    # to a Chiplet, so it should be ours
                    #! for now this assumes that LLC is globally shared.
                    # todo: make ^ not the case
                    self._ruby_controllers.append(c)

            # TODO NEW HIERARCHY: revise/refactor to indicate that ...
            # TODO ... it's more of generating routers/links than ...
            # TODO ... actually creating a topology
            #! create topology
            # need to populate `self._ruby_controllers` first
            self._createTopology(options, self._ruby_controllers)

            # TODO NEW HIERARCHY: maybe needed, not sure yet
            # self._defineMainRouterParams()

            # # goal of this block is to create only one ext_link for
            # # each directory controller
            linked_dcs = [dc for dc, lnk in self._dir_controller_link_map]
            for dir_ctrl in self._dir_controllers:
                if dir_ctrl not in linked_dcs:
                    num_routers = len(self._garnet_network.routers)
                    dir_router = GarnetRouter(
                        router_id=num_routers,
                        latency=self.default_inter_node_router_lat,
                        # the following are default parameters
                        # gem5 complains if I don't include these params
                        vcs_per_vnet=4,
                    )
                    self._garnet_network.routers.append(dir_router)
                    self._connectRubyControllerGarnet(
                        ctrl=dir_ctrl,
                        router=dir_router,
                        link_lat=self.default_inter_node_link_lat,
                    )
                    self._biLinkGarnetRouters(self._main_router, dir_router)
                    self._dir_controller_router_map.append(
                        (dir_ctrl, dir_router)
                    )

            # # try linking L2s to children main routers directly
            # non_private_cache_routers = [
            #     r for c, r in self._ruby_controller_router_map
            # ]
            # for r in non_private_cache_routers:
            #     for n in self._nodes:
            #         self._biLinkGarnetRouters(
            #             r, n._main_router
            #         )

        # * need to instantiate children before we connect hierarchy
        # * and init networks
        for node in self._nodes:
            # set `SimObject` parent hierarchy
            node.set_parent(self, f"{node.__class__.__name__}{node.id}")
            node._ruby_system = self._ruby_system
            if node is ChipletSystem:
                node.createSystem(options, use_legacy_ruby)
            elif isinstance(node, Chiplet):
                # not sure why, but `is` doesn't work here
                node.createChiplet(options, l2_is_private)

        if not use_legacy_ruby:

            # TODO NEW HIERARCHY: remove this
            # self._connectHierarchyGarnet()

            # * this block should only run for the parent `ChipletSystem`.
            # if self._system.ruby is not None:
            if self.isRoot():
                # TODO NEW HIERARCHY: remove this
                # self._combineHierarchyGarnetNetworks(self._garnet_network)
                pass

            if self.isRoot():
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

            # TODO NEW HIERARCHY: remove this
            # self._fixAllGarnetObjectParams()

            #! other Ruby config
            # * this block should only run for the parent `ChipletSystem`.
            # if self._system.ruby is not None:
            if self.isRoot():
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
                    dir_cntrls=dir_controllers,
                    options=options,
                )

                # * more stuff from `Ruby.py`
                # Connect the cpu sequencers and the piobus
                if piobus != None:
                    for cpu_seq in cpu_sequencers:
                        cpu_seq.connectIOPorts(piobus)

                assert isinstance(self._ruby_system.network, GarnetNetwork)
                self._ruby_system.number_of_virtual_networks = (
                    self._ruby_system.network.number_of_virtual_networks
                )  # implicit line break on assign due to 76 char limit
                self._ruby_system._cpu_ports = cpu_sequencers
                self._ruby_system.num_of_sequencers = len(cpu_sequencers)

                # Create a backing copy of physical memory in case required
                if options.access_backing_store:
                    self._ruby_system.access_backing_store = True
                    self._ruby_system.phys_mem = SimpleMemory(
                        range=self._system.mem_ranges[0], in_addr_map=False
                    )

        if self.isRoot():
            self._x86_connect_ruby_ports()

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
