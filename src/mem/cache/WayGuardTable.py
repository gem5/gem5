from m5.params import *
from m5.SimObject import SimObject


class WayGuardTable(SimObject):
    type = "WayGuardTable"
    cxx_class = "gem5::WayGuardTable"
    cxx_header = "mem/cache/dawg.hh"

    num_sets = Param.Unsigned(1, "# of cache sets")
    num_ways = Param.Unsigned(1, "# of ways per set")
    initial_masks = VectorParam.String([], "domain:mask, ...]")
