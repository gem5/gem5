from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *

class BasePrefetcher(SimObject):
    type = 'BasePrefetcher'
    abstract = True
    size = Param.Int(100,
         "Number of entries in the hardware prefetch queue")
    cross_pages = Param.Bool(False,
         "Allow prefetches to cross virtual page boundaries")
    serial_squash = Param.Bool(False,
         "Squash prefetches with a later time on a subsequent miss")
    degree = Param.Int(1,
         "Degree of the prefetch depth")
    latency = Param.Latency('10t',
         "Latency of the prefetcher")
    use_master_id = Param.Bool(True,
         "Use the master id to separate calculations of prefetches")
    data_accesses_only = Param.Bool(False,
         "Only prefetch on data not on instruction accesses")
    sys = Param.System(Parent.any, "System this device belongs to")

class GHBPrefetcher(BasePrefetcher):
    type = 'GHBPrefetcher'
    cxx_class = 'GHBPrefetcher'

class StridePrefetcher(BasePrefetcher):
    type = 'StridePrefetcher'
    cxx_class = 'StridePrefetcher'

class TaggedPrefetcher(BasePrefetcher):
    type = 'TaggedPrefetcher'
    cxx_class = 'TaggedPrefetcher'




