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
    cxx_exports = [PyBindMethod("bindCache"), PyBindMethod("bindInstructionCache")]
    components = VectorParam.TPCComponent([], "One component per enabled source")
    routes = VectorParam.FrameworkQueue([], "L1D, L2, L3 transport queues")
    prefetch_on_access = True
    prefetch_on_pf_hit = False

    def bindHierarchy(self, caches, instruction_cache=None):
        self._hierarchy = list(caches)
        self._instruction_cache = instruction_cache

    def regProbeListeners(self):
        if self._instruction_cache is not None:
            self.getCCObject().bindInstructionCache(self._instruction_cache.getCCObject())
        for level, cache in enumerate(self._hierarchy, 1):
            self.getCCObject().bindCache(level, cache.getCCObject())
        super().regProbeListeners()



class C1Prefetcher(TPCComponent):
    type = "C1Prefetcher"
    cxx_class = "gem5::prefetch::C1Prefetcher"
    cxx_header = "mem/cache/prefetch/c1.hh"
    accounting_source = 3
    region_lines = Param.Unsigned(16, "Cache lines per region")
    rm_entries = Param.Unsigned(16, "LRU region monitor entries")
    im_entries = Param.Unsigned(16, "Non-replacing instruction monitor entries")
    dense_lines = Param.Unsigned(6, "Dense region has strictly more distinct lines")
    training_regions = Param.Unsigned(4, "Evicted region samples before classification")


class DivisionOfLaborController(TPCController):
    type = "DivisionOfLaborController"
    cxx_class = "gem5::prefetch::DivisionOfLaborController"
    cxx_header = "mem/cache/prefetch/c1.hh"
    mode = Param.String("TPC", "Enabled combination: C, T, TP, TPC")
