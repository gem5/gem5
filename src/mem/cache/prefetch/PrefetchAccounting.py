from m5.SimObject import SimObject, PyBindMethod
from m5.params import *


class PrefetchAccounting(SimObject):
    type = "PrefetchAccounting"
    cxx_header = "mem/cache/prefetch/accounting.hh"
    cxx_class = "gem5::PrefetchAccounting"
    cxx_exports = [PyBindMethod("beginPhase")]
    trace = Param.Bool(False, "Print per-record accounting events (debugging only)")
