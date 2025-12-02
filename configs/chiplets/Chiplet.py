from typing import (
    TYPE_CHECKING,
    Self,
    Sequence,
    Type,
)

from chiplets.BaseChipletSystem import BaseChipletSystem
from chiplets.ChipletSystem import ChipletSystem
from topologies.BaseTopology import BaseTopology

from m5.defines import buildEnv
from m5.objects import *

if TYPE_CHECKING:
    from src.mem.AbstractMemory import AbstractMemory
    from src.sim.SubSystem import SubSystem
    from src.sim.System import System


class Chiplet(BaseChipletSystem):
    """
    A Chiplet denotes a node at the innermost level of a ChipletSystem.
    A Chiplet does not have any child ChipletSystems; all of its children
    are processor objects (cores) -- in other words, "leaf" nodes.
    """

    def __init__(
        self,
        system: System,
        TopologyClass: Type[BaseTopology],
        MemoryClass: Type[AbstractMemory],
        inter_node_link_lat: int,
        inter_node_router_lat: int,
        nodes: list[BaseChipletSystem] = [],
    ):
        BaseChipletSystem.__init__(
            self,
            system=system,
            TopologyClass=TopologyClass,
            MemoryClass=MemoryClass,
            nodes=nodes,
            inter_node_link_lat=inter_node_link_lat,
            inter_node_router_lat=inter_node_router_lat,
        )
