"""Common TPC algorithm, controller and transport interfaces."""
from m5.objects.Prefetcher import BasePrefetcher, QueuedPrefetcher
from m5.params import *
from m5.SimObject import PyBindMethod


class TPCComponent(BasePrefetcher):
    type = "TPCComponent"
    abstract = True
    cxx_class = "gem5::prefetch::TPCComponent"
    cxx_header = "mem/cache/prefetch/framework.hh"
    observe_levels = VectorParam.Int([1], "Demand/fill observation layers")
    prefetch_on_access = True
    prefetch_on_pf_hit = False


class FrameworkQueue(QueuedPrefetcher):
    type = "FrameworkQueue"
    cxx_class = "gem5::prefetch::FrameworkQueue"
    cxx_header = "mem/cache/prefetch/framework.hh"
    prefetch_on_access = True
    prefetch_on_pf_hit = False


class TPCController(BasePrefetcher):
    type = "TPCController"
    abstract = True
    cxx_class = "gem5::prefetch::TPCController"
    cxx_header = "mem/cache/prefetch/framework.hh"
    cxx_exports = [PyBindMethod("bindCache")]
    components = VectorParam.TPCComponent([], "One component per enabled source")
    routes = VectorParam.FrameworkQueue([], "L1D, L2, L3 transport queues")
    prefetch_on_access = True
    prefetch_on_pf_hit = False

    def bindHierarchy(self, caches):
        self._hierarchy = list(caches)

    def regProbeListeners(self):
        for level, cache in enumerate(self._hierarchy, 1):
            self.getCCObject().bindCache(level, cache.getCCObject())
        super().regProbeListeners()

