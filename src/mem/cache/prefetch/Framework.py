"""Common TPC algorithm, controller and transport interfaces."""
from m5.objects.Prefetcher import BasePrefetcher, QueuedPrefetcher
from m5.params import *
from m5.SimObject import PyBindMethod, SimObject


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


class T2Prefetcher(TPCComponent):
    type = "T2Prefetcher"
    cxx_class = "gem5::prefetch::T2Prefetcher"
    cxx_header = "mem/cache/prefetch/t2.hh"
    accounting_source = 1
    sit_entries = Param.Unsigned(32, "Finite stride identifier table")
    nlpct_entries = Param.Unsigned(16, "Table II capacity; prose instead says 20")
    min_segment_size = Param.Unsigned(16, "Unspecified Figure 3 filter threshold")
    confirm_deltas = Param.Unsigned(16, "Consecutive equal delta samples")
    reject_changes = Param.Unsigned(4, "Consecutive changing delta samples")
    provisional_deltas = Param.Unsigned(4, "Early observation prefetch threshold")
    timing_window = Param.Unsigned(16, "Arithmetic mean window in samples")
    margin_cycles = Param.Unsigned(0, "AMAT margin in component cycles")
    max_distance = Param.Unsigned(64, "Maximum base lookahead iterations")
    max_pending = Param.Unsigned(8192, "Bounded dynamic-load join capacity")
    provisional_per_cycle = Param.Unsigned(1, "Provisional scheduling budget")
    provisional_queue_limit = Param.Unsigned(8, "Suppress early traffic at this L1 route occupancy")
    trace = Param.Bool(False, "Log recognition and candidate events")


class TPCCPUAdapter(SimObject):
    type = "TPCCPUAdapter"
    cxx_class = "gem5::prefetch::TPCCPUAdapter"
    cxx_header = "mem/cache/prefetch/tpc_cpu_adapter.hh"
    cpu = Param.BaseCPU("O3 CPU providing typed probes")
    controller = Param.TPCController("Shared TPC event/request owner")
    branch_predictor = Param.BranchPredictor("Fetch-stage RAS context")
    max_inflight = Param.Unsigned(8192, "Fail instead of dropping live fetch context")
