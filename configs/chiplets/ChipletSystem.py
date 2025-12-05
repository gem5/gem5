from argparse import Namespace  # for type checking
from importlib import import_module
from typing import (
    TYPE_CHECKING,
    Self,
    Sequence,
    Type,
)

from chiplets.BaseChipletSystem import BaseChipletSystem
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
    from src.mem.ruby.system.RubySystem import RubySystem
    from src.mem.ruby.system.Sequencer import RubyPortProxy
    from src.mem.SimpleMemory import SimpleMemory
    from src.mem.XBar import IOXBar
    from src.sim.System import System


class ChipletSystem(BaseChipletSystem):
    """
    A `ChipletSystem` encapsulates a collection of child `Chiplet`s and/or
    `ChipletSystem`s. It acts as an abstraction of a physical processor
    package, a multi-chip module (MCM), or CCD, insofar as its role in
    containing multiple processors (heterogeneous or homogenous) across
    multiple chiplets linked by interconnects. Uses Ruby and Garnet.
    """

    #! see BaseChipletSystem for additional key properties, namely:
    #  - nodes
    #  - default_inter_node_link_lat
    #  - default_inter_node_router_lat
    #  - parent
    #  - connected_nodes

    # whether createSystem() has been called (successfully)
    _created: bool

    # class-local reference to `self.system.ruby`
    # (gem5 system SimObject's RubySystem)
    _ruby_system: RubySystem

    # see `createSystem()` or `class ChipletMetaTopology`
    _meta_topology: ChipletMetaTopology

    def __init__(
        self,
        system: System,
        full_system: bool,
        TopologyClass: Type[BaseTopology],
        MemoryClass: Type[AbstractMemory],
        inter_node_link_lat: int,
        inter_node_router_lat: int,
        nodes: list[BaseChipletSystem] = [],
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
            nodes (list[BaseChipletSystem]):
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

        for node in self.nodes:
            if not hasattr(node, "parent"):
                node.parent = self

        self._created = False
        self._ruby_system = None

    def createSystem(
        self,
        options: Namespace,  # argparse Namespace (parsed args)
        use_legacy_ruby: bool = False,
        piobus: IOXBar | None = None,
        dma_ports: list[DmaDevice] = [],
        bootmem: AbstractMemory | None = None,
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
        """

        if self._created is True:
            warn(
                "Attempted to call createSystem() on this "
                "ChipletSystem, but it has already been created."
            )
            return

        if use_legacy_ruby and options is not None:
            # at top level, run `Ruby.create_system()`
            # the way Ruby seems to be set up right now,
            # this should only run once, and not sure if
            # hierarchical topologies are plausible
            if not hasattr(self, "parent"):
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

            # * this block should only run for the parent `ChipletSystem`.
            if not hasattr(self._system, "ruby"):
                # instantiate `RubySystem`, to be "created" next
                self._system.ruby = RubySystem()
                self._ruby_system = self._system.ruby

                # Generate pseudo filesystem
                FileSystemConfig.config_filesystem(self._system, options)

            if self._ruby_system is None:
                panic("ChipletSystem's RubySystem is None!")

            # Create the `GarnetNetwork` for this level of the hierarchy
            self._garnet_network = GarnetNetwork(
                ruby_system=self._ruby_system,
                topology=self._topology_cls.__name__,
                routers=[],
                ext_links=[],
                int_links=[],
                netifs=[],
            )

            # * this block should only run for the parent `ChipletSystem`.
            if self._system.ruby is not None:
                # `Ruby.py` does this, not sure why since `RubySystem`
                # doesn't have a 'network' param
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
                    if hasattr(options, "topology"):
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
                    # as `self.TopologyClass`, but just in case it wasn't:
                    if tmp:
                        options.topology = tmp

                    protocol_controllers = self._meta_topology.nodes
                except Exception as e:
                    panic(
                        "Could not create Ruby system for Ruby protocol "
                        f"{buildEnv['PROTOCOL']}; exception thrown: {e}"
                    )

                #! instantiate topology for this layer of the hierarchy
                # type error ignorable because subclasses of `BaseTopology`
                # typically override constructor with this signature
                # todo: actually distribute controllers properly
                self.topology = self._topology_cls(
                    protocol_controllers  # type: ignore
                )

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

                # if in SE mode, register topology with (faux) filesystem
                # only some topologies implement this (as of now, only
                # `Mesh_XY` and `MeshDirCorners_XY` do.)
                if not self._is_full_system:
                    # notes: `Mesh_XY` requires
                    # `options.num_cpus` and `options.mem_size`.
                    # `MeshDirCorners_XY` requires only `options.mem_size`.
                    self.topology.registerTopology(options)

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

                # * do some port configuration from `Ruby.py`
                self._ruby_connect_system_ports(piobus)

                # todo: don't rely on `Ruby.py` for this
                # unfortunately, in order to get `dir_cntrls`, the Ruby
                # protocols also call back to `Ruby.create_directories`.
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

        for node in self.nodes:
            if node is ChipletSystem:
                node._ruby_system = self._ruby_system
                node.createSystem(options, use_legacy_ruby)
            elif node is Chiplet:
                # todo: initialize chiplet
                pass

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
