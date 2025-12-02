from argparse import Namespace  # for type checking
from typing import (
    TYPE_CHECKING,
    Self,
    Sequence,
    Type,
)

from chiplets.BaseChipletSystem import BaseChipletSystem
from ruby import Ruby
from topologies.BaseTopology import BaseTopology

from m5.defines import buildEnv
from m5.objects import *
from m5.util import (
    addToPath,
    fatal,
    panic,
    warn,
)

if TYPE_CHECKING:
    from src.cpu.BaseCPU import BaseCPU
    from src.mem.AbstractMemory import AbstractMemory
    from src.sim.System import System


class ChipletSystem(BaseChipletSystem):
    """
    A ChipletSystem encapsulates a collection of child Chiplets and/or
    ChipletSystems. It acts as an abstraction of a physical processor
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
    created: bool

    def __init__(
        self,
        system: System,
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
            TopologyClass=TopologyClass,
            MemoryClass=MemoryClass,
            nodes=nodes,
            inter_node_link_lat=inter_node_link_lat,
            inter_node_router_lat=inter_node_router_lat,
        )

        for node in self.nodes:
            if not hasattr(node, "parent"):
                node.parent = self

        self.created = False

    def createSystem(
        self,
        options: Namespace,  # argparse Namespace (parsed options/args)
        full_system: bool,
    ):
        """
        Recursively construct this ChipletSystem based on the list of
        nodes and other parameters (e.g., latencies) specified.
        Specifically, create the backing Garnet network and other
        necessary objects.
        """

        if self.created is True:
            warn(
                "Attempted to call createSystem() on this "
                "ChipletSystem, but it has already been created."
            )
            return

        for node in self.nodes:
            if node is ChipletSystem:
                node.createSystem(options, full_system)

        # at top level, run Ruby.create_system()
        # the way Ruby seems to be set up right now,
        # this should only run once, and not sure if
        # hierarchical topologies are plausible
        if not hasattr(self, "parent"):
            # configure options that Ruby.py expects
            options.network = "garnet"
            options.topology = self._topology_cls.__name__
            options.mem_type = self._memory_cls.__name__

            # create ruby system
            # todo: may need to call MULTIPLE.py create_system()
            Ruby.create_system(
                options=options, full_system=full_system, system=self.system
            )

            # todo: make this not a hack
            # connect ruby ports if x86
            for i, cpu in enumerate(self.system.cpu):
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
                                ruby_port = self.system.ruby._cpu_ports[i]
                                ruby_port.connectCpuPorts(cpu)
                            else:
                                panic(
                                    "Interrupt controller was created, "
                                    "but no interrupts were found."
                                )

        self.created = True
