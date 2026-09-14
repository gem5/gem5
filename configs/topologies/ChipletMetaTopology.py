from typing import TYPE_CHECKING

from topologies.BaseTopology import BaseTopology

from m5.util import fatal

if TYPE_CHECKING:
    from chiplets.ChipletSystem import ChipletSystem


class ChipletMetaTopology(BaseTopology):
    """
    This is a dummy/meta/internal topology used by `ChipletSystem`
    to enable hierarchical topologies while maintaining compatibility
    with existing topologies and Ruby protocols.
    """

    description = "ChipletMetaTopology"

    # use same constructor format as `SimpleTopology` for compat with
    # existing Ruby protocols, which mostly call `create_topology()`
    # from `Ruby.py`, which assumes this constructor format.
    def __init__(self, controllers):
        self.nodes = controllers

    def makeTopology(self, options, network, IntLink, ExtLink, Router):
        fatal("makeTopology() should not be called on ChipletMetaTopology!")
