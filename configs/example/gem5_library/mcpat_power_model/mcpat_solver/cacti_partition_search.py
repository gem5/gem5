"""CACTI candidate enumeration, filtering and joint data/tag ranking.

Preserves Ucache.cc traversal, tie-breaking and the default ED² objective.
RAM, paired caches, fully associative arrays and CAMs share this path.
Normal, fast and sequential access retain their native delay combinations.
Optional ArrayST timing optimization wraps the same baseline search."""

import math
from collections import (
    OrderedDict,
    namedtuple,
)

from .cacti_component import (
    CactiMat,
    CactiUCA,
)
from .cacti_dynamic_params import CactiDynamicParameter

# Ucache.cc const.h: maximum for Ndwl/Ndbl, Nspd, and Ndcm/Ndsam_lev_1/
# Ndsam_lev_2 respectively.
MAXDATAN = 512
MAXDATASPD = 256
MAX_COL_MUX = 256

# Phase 39 finding, measured not assumed (see PROGRESS.md): real CACTI's
# per-candidate wr loop (Ucache.cc:calc_time_mt_wrapper, wt_min=Global(0) to
# wt_max=Low_swing(5)) does NOT feed into calculate_time/DynamicParameter at
# all -- every array's H-tree wires are built from g_ip->wt, a single FIXED
# value set once per array by McPAT (processor.cc/core.cc/etc, always Global
# or Global_30 depending on the Embedded flag, never Low_swing in any live
# McPAT code path -- confirmed by grepping every wt/force_wiretype
# assignment site in McPAT-proper). The wr loop only produces 6 numerically
# IDENTICAL, differently-labeled copies of each candidate; confirmed via a
# real, temporary C++ probe in Ucache.cc against a live BTB run: 76,735/
# 76,735 clean (non-key-collision) (array, geometry) groups showed 0
# divergence in access_time/dynamic/leakage/area across all 6 wr labels,
# both data and tag sides. So "modelling Low_swing" does not mean porting
# Wire::low_swing_model() physics (unreachable for any array's real wt
# value) -- it means reproducing the SAME non-causal 6-way label
# enumeration real CACTI performs, so filter_data_side/filter_tag_side's
# tie-break rules (which now determine the "winning" label from
# enumeration order and comparator direction alone, exactly as in the
# real search) land on the same label real CACTI does.
#
# WIRE_TYPE_ORDINALS: the 6 cacti_interface.h Wire_type enum positions
# (Global=0, Global_5=1, Global_10=2, Global_20=3, Global_30=4,
# Low_swing=5), used only for this label enumeration -- NOT overhead
# values. The actual computation input is cfg.wt_overhead (this port's
# %-delay-overhead wire-model selector, 0/5/10/20/30 -- see
# machine_model.py's _WT_ORDINAL_TO_OVERHEAD for the ordinal<->overhead
# mapping used elsewhere), which is a FIXED given input (matching real
# CACTI's g_ip->wt) and is never varied by this search.
WIRE_TYPE_ORDINALS = tuple(range(6))

# McPAT's own Layer-1/2 baseline (processor.cc:756-768), before any Layer-3
# override. Passed explicitly (not hardcoded downstream) so a future Layer-3
# wrapper can override them per relax-and-repick iteration.
DEFAULT_ED = 2
DEFAULT_DELAY_WT = 100.0
DEFAULT_AREA_WT = 0.0
DEFAULT_DYNAMIC_POWER_WT = 100.0
DEFAULT_LEAKAGE_POWER_WT = 0.0
DEFAULT_CYCLE_TIME_WT = 0.0
DEFAULT_DELAY_DEV = 10000.0
DEFAULT_AREA_DEV = 10000.0
DEFAULT_DYNAMIC_POWER_DEV = 10000.0
DEFAULT_LEAKAGE_POWER_DEV = 10000.0
DEFAULT_CYCLE_TIME_DEV = 10000.0

BIGNUM = 1e100


def _pow2_upto(maxv):
    vals = []
    v = 1
    while v <= maxv:
        vals.append(v)
        v *= 2
    return vals


_NDWL_VALS = _pow2_upto(MAXDATAN)
_NDBL_VALS = _pow2_upto(MAXDATAN)
_NDCM_VALS = _pow2_upto(MAX_COL_MUX)
_NDSAM_VALS = _pow2_upto(MAX_COL_MUX)


Candidate = namedtuple(
    "Candidate",
    "Nspd Ndwl Ndbl Ndcm Ndsam1 Ndsam2 wt_overhead wt_ordinal uca "
    "access_time cycle_time dynamic leakage area",
)


class RunningMin:
    """Port of min_values_t. Only ever lowered (update_min_values), never
    reset -- a population minimum accumulated across every candidate seen,
    exactly mirroring the real struct's semantics."""

    __slots__ = ("min_delay", "min_dyn", "min_leakage", "min_area", "min_cyc")

    def __init__(self):
        self.min_delay = math.inf
        self.min_dyn = math.inf
        self.min_leakage = math.inf
        self.min_area = math.inf
        self.min_cyc = math.inf

    def update(self, c):
        self.min_delay = min(self.min_delay, c.access_time)
        self.min_dyn = min(self.min_dyn, c.dynamic)
        self.min_leakage = min(self.min_leakage, c.leakage)
        self.min_area = min(self.min_area, c.area)
        self.min_cyc = min(self.min_cyc, c.cycle_time)

    def copy(self):
        c = RunningMin()
        c.min_delay = self.min_delay
        c.min_dyn = self.min_dyn
        c.min_leakage = self.min_leakage
        c.min_area = self.min_area
        c.min_cyc = self.min_cyc
        return c


# enumerate_side() is a pure function of (cfg, sys_params, nspd_min), so
# memoize it: search_partition_layer3() re-runs an identical enumeration up
# to 9x, and back-to-back searches on one config are common. Bounded at 2 --
# each entry pins every candidate's CactiUCA graph (~300 MB for a large table).
_ENUM_CACHE = OrderedDict()
_ENUM_CACHE_MAXSIZE = 2


def _enum_cache_key(cfg, sys_params, nspd_min):
    # Value-based so two equivalent config objects hit the same entry.
    return (
        tuple(sorted(vars(cfg).items())),
        tuple(sorted(sys_params._tp.items())),
        tuple(sorted(sys_params._wp.items())),
        nspd_min,
    )


def clear_enumerate_cache():
    """Drop every memoized candidate list (tests use this to force a cold run)."""
    _ENUM_CACHE.clear()


def enumerate_side(cfg, sys_params, nspd_min, name_hint="search"):
    """Memoizing wrapper around _enumerate_side_uncached. Hands back a private
    list + RunningMin copy each call; callers only ever read the shared
    Candidate/CactiUCA objects. name_hint is not part of the cache key."""
    key = _enum_cache_key(cfg, sys_params, nspd_min)
    if key in _ENUM_CACHE:
        _ENUM_CACHE.move_to_end(key)
        candidates, running_min, skipped = _ENUM_CACHE[key]
    else:
        candidates, running_min, skipped = _enumerate_side_uncached(
            cfg, sys_params, nspd_min, name_hint
        )
        _ENUM_CACHE[key] = (candidates, running_min, skipped)
        if len(_ENUM_CACHE) > _ENUM_CACHE_MAXSIZE:
            _ENUM_CACHE.popitem(last=False)
    return list(candidates), running_min.copy(), skipped


def _enumerate_side_uncached(cfg, sys_params, nspd_min, name_hint="search"):
    """Layer 1 (Ucache.cc:calc_time_mt_wrapper) for one side -- cfg.is_tag
    selects tag vs data. Single-threaded: this port never parallelizes:
    NTHREADS only changes how the C++ splits `iter` across pthreads, not
    which (Nspd, wr, Ndwl, Ndbl, Ndcm, Ndsam1, Ndsam2) combinations get
    tried, so the candidate SET this produces is identical regardless of
    thread count -- see the native reference for the one place this can
    matter (exact floating-point cost ties, addressed by the sort below,
    not by simulating thread interleaving).

    Each geometry is computed ONCE, using cfg.wt_overhead unchanged (the
    array's real, fixed wire config -- see module docstring, Phase 39:
    real CACTI's wr loop is non-causal, so varying wt_overhead per
    candidate here would be simulating a difference real CACTI itself
    never produces). The result is then replicated into 6 Candidate
    entries (wt_ordinal 0..5, WIRE_TYPE_ORDINALS) that share identical
    computed stats but differ in label and enumeration position -- this
    reproduces real CACTI's redundant 6x wr loop closely enough for
    filter_data_side/filter_tag_side's tie-break rules (order- and
    comparator-direction-dependent) to land on the same label real CACTI
    does, without recomputing the same geometry 6 times.

    Returns (candidates, running_min) where candidates is sorted by
    mem_array::lt's real key (Nspd, Ndwl, Ndbl, Ndcm, Ndsam1, Ndsam2) --
    note wt_ordinal is NOT part of that key, exactly matching the real
    comparator (cacti_interface.cc:51-64), so ties across wire-type labels
    at identical geometry are broken by Python's stable sort preserving
    enumeration order (wt_ordinal ascending), the same behavior real CACTI
    gets from a stable std::list::sort at nthreads=1.
    """
    candidates = []
    running_min = RunningMin()
    skipped = 0

    # For FA/CAM arrays (assoc==0) the CactiDynamicParameter guard only
    # accepts Ndwl=Ndcm=Ndsam_lev_1=Ndsam_lev_2=1, so restrict the sweep to
    # those -- identical candidate set, ~10^4x fewer rejected geometries.
    if cfg.assoc == 0:
        ndwl_vals = (1,)
        ndcm_vals = (1,)
        ndsam1_vals = (1,)
        ndsam2_vals = (1,)
    else:
        ndwl_vals = _NDWL_VALS
        ndcm_vals = _NDCM_VALS
        ndsam1_vals = _NDSAM_VALS
        ndsam2_vals = _NDSAM_VALS

    nspd = nspd_min
    while nspd <= MAXDATASPD:
        for Ndwl in ndwl_vals:
            for Ndbl in _NDBL_VALS:
                for Ndcm in ndcm_vals:
                    for Ndsam1 in ndsam1_vals:
                        for Ndsam2 in ndsam2_vals:
                            dp = CactiDynamicParameter(
                                cfg,
                                Nspd=nspd,
                                Ndwl=Ndwl,
                                Ndbl=Ndbl,
                                Ndcm=Ndcm,
                                Ndsam_lev_1=Ndsam1,
                                Ndsam_lev_2=Ndsam2,
                            )
                            if not dp.valid:
                                continue
                            # A handful of dp.valid==True geometries hit
                            # ZeroDivisionError/etc. deeper in the delay
                            # chain -- edge cases exhaustive enumeration
                            # reaches that no prior (hand-picked, single-
                            # partition) validation anchor ever did. Real
                            # CACTI's own DynamicParameter constructor
                            # would mark these invalid before delay is
                            # ever computed; this port's is_valid check is
                            # evidently not yet 1:1 complete for every
                            # corner. Skip and count rather than crash the
                            # whole search -- see the native reference for
                            # the measured skip rate, not asserted here.
                            try:
                                mat = CactiMat(name_hint, sys_params, dp)
                                # mat_area_w/h default to 0.0 (CactiMat's
                                # constructor signature) -- this project's
                                # ENERGY pipeline always took real mat area
                                # as a given input (CLAUDE.md's standing
                                # scope rule), since no search existed to
                                # derive it. This search IS that derivation.
                                # mat.compute_area()  computes the
                                # real geometry but -- BY DESIGN, see its own
                                # docstring -- stores it only under
                                # derived_area_w/h, never mat.area_w/h itself
                                # (those remain the "given input" every other
                                # caller, e.g. machine_model.py, already
                                # supplies correctly). CactiBank/CactiUCA's
                                # OWN outer geometry (self.area_w/h) already
                                # picks up derived_area_w/h via their
                                # hasattr(mat, 'derived_area_w') checks --
                                # that's how bug 2 below got "area" right.
                                # But CactiBank's ENERGY-computing H-trees
                                # (self.htree_in_add/in_data/out_data) are
                                # built from mat_w=mat.area_w/mat_h=mat.area_h
                                # directly, which this call site never fills
                                # in -- so every multi-mat candidate's inter-
                                # mat H-tree dynamic energy was silently
                                # computed off a 0x0 mat -- the Phase 35
                                # dynamic-energy divergence, root-caused and
                                # fixed in Phase 36 (PROGRESS.md) via a real
                                # C++ probe: mat.power matched bit-exact,
                                # all three bank H-trees diverged 19x-29x.
                                # Single-mat candidates were unaffected
                                # (CactiBank's inter-mat H-tree geometry term
                                # is provably zero at
                                # num_mats_h_dir==num_mats_v_dir==1 regardless
                                # of mat_w/h), which is exactly why 34 phases
                                # of winner-only validation never saw this.
                                # Fix: propagate compute_area()'s return
                                # into mat.area_w/h before CactiBank/CactiUCA
                                # are built, so they see the real geometry
                                # through the SAME "given input" path
                                # production already uses -- not a new path.
                                mat.area_w, mat.area_h = mat.compute_area()
                                uca = CactiUCA(mat, dp, cfg.nbanks)
                                uca.compute_delays(0.0)
                                area = uca.derived_area_w * uca.derived_area_h
                                # Same rationale as the except clause below:
                                # a geometry real CACTI's own is_valid would
                                # exclude, that this port's is_valid doesn't
                                # yet catch. A non-positive area/delay/energy
                                # is never a legitimate CACTI result -- it
                                # would corrupt every RunningMin division
                                # downstream (ZeroDivisionError or a bogus
                                # negative-cost winner) if kept.
                                if not (
                                    area > 0
                                    and uca.access_time > 0
                                    and uca.cycle_time > 0
                                    and uca.read > 0
                                    and uca.leakage > 0
                                ):
                                    skipped += 1
                                    continue
                            except (
                                ZeroDivisionError,
                                ValueError,
                                OverflowError,
                                AssertionError,
                            ):
                                skipped += 1
                                continue
                            for wt_ordinal in WIRE_TYPE_ORDINALS:
                                cand = Candidate(
                                    Nspd=nspd,
                                    Ndwl=Ndwl,
                                    Ndbl=Ndbl,
                                    Ndcm=Ndcm,
                                    Ndsam1=Ndsam1,
                                    Ndsam2=Ndsam2,
                                    wt_overhead=cfg.wt_overhead,
                                    wt_ordinal=wt_ordinal,
                                    uca=uca,
                                    access_time=uca.access_time,
                                    cycle_time=uca.cycle_time,
                                    dynamic=uca.read,
                                    leakage=uca.leakage,
                                    area=area,
                                )
                                running_min.update(cand)
                                candidates.append(cand)
        nspd *= 2

    candidates.sort(
        key=lambda c: (c.Nspd, c.Ndwl, c.Ndbl, c.Ndcm, c.Ndsam1, c.Ndsam2)
    )
    return candidates, running_min, skipped


def filter_data_side(candidates, running_min):
    """Ucache.cc:filter_data_arr -- drops a candidate only when BOTH its
    access_time AND dynamic power are >50% worse than the (per-side, i.e.
    this side's own RunningMin, not the later cross-product min) minimum.
    Order-preserving; may keep many candidates (dominance prune, not a
    single-winner pick -- contrast filter_tag_side below)."""
    kept = []
    for c in candidates:
        delay_bad = (
            c.access_time - running_min.min_delay
        ) / running_min.min_delay > 0.5
        dyn_bad = (c.dynamic - running_min.min_dyn) / running_min.min_dyn > 0.5
        if delay_bad and dyn_bad:
            continue
        kept.append(c)
    return kept


def _check_mem_org(
    c,
    running_min,
    delay_dev,
    dynamic_power_dev,
    leakage_power_dev,
    cycle_time_dev,
    area_dev,
):
    if (
        c.access_time - running_min.min_delay
    ) * 100 / running_min.min_delay > delay_dev:
        return False
    if (
        c.dynamic - running_min.min_dyn
    ) / running_min.min_dyn * 100 > dynamic_power_dev:
        return False
    if (
        c.leakage - running_min.min_leakage
    ) / running_min.min_leakage * 100 > leakage_power_dev:
        return False
    if (
        c.cycle_time - running_min.min_cyc
    ) / running_min.min_cyc * 100 > cycle_time_dev:
        return False
    if (c.area - running_min.min_area) / running_min.min_area * 100 > area_dev:
        return False
    return True


def filter_tag_side(
    candidates,
    running_min,
    delay_wt=DEFAULT_DELAY_WT,
    dynamic_power_wt=DEFAULT_DYNAMIC_POWER_WT,
    leakage_power_wt=DEFAULT_LEAKAGE_POWER_WT,
    cycle_time_wt=DEFAULT_CYCLE_TIME_WT,
    area_wt=DEFAULT_AREA_WT,
    delay_dev=DEFAULT_DELAY_DEV,
    dynamic_power_dev=DEFAULT_DYNAMIC_POWER_DEV,
    leakage_power_dev=DEFAULT_LEAKAGE_POWER_DEV,
    cycle_time_dev=DEFAULT_CYCLE_TIME_DEV,
    area_dev=DEFAULT_AREA_DEV,
):
    """Ucache.cc:filter_tag_arr -- NOT a filter in the usual sense: it
    reduces the tag-side candidate list to a SINGLE winner, chosen by a
    weighted-sum cost (using the SAME weights as find_optimal_uca's ed==0
    branch, applied here unconditionally regardless of the data-side `ed`
    mode -- a real, must-replicate-not-fix CACTI asymmetry: the tag array
    is always weighted-cost-picked even when the final data+tag combo is
    ranked by ED/ED2P). Traverses in REVERSE (mirrors list::back()/
    pop_back()) with a strict `<` improvement test, so ties favor the
    LAST candidate in `candidates`' order -- i.e. this must be called on
    the SAME sorted-by-mem_array::lt list enumerate_side returns, not a
    re-ordered one, to match real CACTI's tie-breaking."""
    best = None
    best_cost = BIGNUM
    for c in reversed(candidates):
        if _check_mem_org(
            c,
            running_min,
            delay_dev,
            dynamic_power_dev,
            leakage_power_dev,
            cycle_time_dev,
            area_dev,
        ):
            cost = (
                delay_wt * (c.access_time / running_min.min_delay)
                + cycle_time_wt * (c.cycle_time / running_min.min_cyc)
                + dynamic_power_wt * (c.dynamic / running_min.min_dyn)
                + leakage_power_wt * (c.leakage / running_min.min_leakage)
                + area_wt * (c.area / running_min.min_area)
            )
        else:
            cost = BIGNUM
        if cost < best_cost:
            best_cost = cost
            best = c
    return best


Combined = namedtuple(
    "Combined", "tag data access_time cycle_time dynamic leakage area"
)


def _combine(tag, data, fast_access=False, is_seq_acc=False):
    """cacti_interface.cc:uca_org_t::find_delay/find_energy/find_area/
    find_cyc. tag=None means the pure_ram/fully_assoc/pure_cam case (data
    only) -- this is a validated, common branch (see search_partition()'s
    docstring), reached whenever the caller passes tag_cfg=None.

    The three tag+data delay branches are tested in find_delay's own order
    (fast_access, then is_seq_acc, then normal) -- only find_delay differs
    between access modes; find_energy/find_area/find_cyc are mode-agnostic.
    is_seq_acc (McPAT's shared L2/L3/Directorycache, sharedcache.cc:127) adds
    the tag and data access times outright, because the data array is only
    read after the tag lookup resolves the way."""
    if tag is None:
        access_time = data.access_time
        cycle_time = data.cycle_time
        dynamic = data.dynamic
        leakage = data.leakage
        area = data.area
    elif fast_access:
        access_time = max(tag.access_time, data.access_time)
        cycle_time = max(tag.cycle_time, data.cycle_time)
        dynamic = data.dynamic + tag.dynamic
        leakage = data.leakage + tag.leakage
        area = max(tag.uca.derived_area_h, data.uca.derived_area_h) * (
            tag.uca.derived_area_w + data.uca.derived_area_w
        )
    elif is_seq_acc:
        access_time = tag.access_time + data.access_time
        cycle_time = max(tag.cycle_time, data.cycle_time)
        dynamic = data.dynamic + tag.dynamic
        leakage = data.leakage + tag.leakage
        area = max(tag.uca.derived_area_h, data.uca.derived_area_h) * (
            tag.uca.derived_area_w + data.uca.derived_area_w
        )
    else:
        sense_mux_decoder = max(
            data.uca.delay_array_to_sa_mux_lev_1_decoder,
            data.uca.delay_array_to_sa_mux_lev_2_decoder,
        )
        access_time = (
            max(
                tag.access_time + sense_mux_decoder,
                data.uca.delay_before_subarray_output_driver,
            )
            + data.uca.delay_from_subarray_out_drv_to_out
        )
        cycle_time = max(tag.cycle_time, data.cycle_time)
        dynamic = data.dynamic + tag.dynamic
        leakage = data.leakage + tag.leakage
        area = max(tag.uca.derived_area_h, data.uca.derived_area_h) * (
            tag.uca.derived_area_w + data.uca.derived_area_w
        )
    return Combined(
        tag=tag,
        data=data,
        access_time=access_time,
        cycle_time=cycle_time,
        dynamic=dynamic,
        leakage=leakage,
        area=area,
    )


def rank_candidates(
    tag_winner,
    data_candidates,
    ed=DEFAULT_ED,
    delay_wt=DEFAULT_DELAY_WT,
    dynamic_power_wt=DEFAULT_DYNAMIC_POWER_WT,
    leakage_power_wt=DEFAULT_LEAKAGE_POWER_WT,
    cycle_time_wt=DEFAULT_CYCLE_TIME_WT,
    area_wt=DEFAULT_AREA_WT,
    delay_dev=DEFAULT_DELAY_DEV,
    dynamic_power_dev=DEFAULT_DYNAMIC_POWER_DEV,
    leakage_power_dev=DEFAULT_LEAKAGE_POWER_DEV,
    cycle_time_dev=DEFAULT_CYCLE_TIME_DEV,
    area_dev=DEFAULT_AREA_DEV,
    fast_access=False,
    is_seq_acc=False,
):
    """Ucache.cc:solve()'s cross-product loop + find_optimal_uca. tag_winner
    is filter_tag_side's single winner (or None for a pure_ram-shaped
    array, out of this phase's scope but not special-cased away). Builds
    the SECOND, cross-product-level RunningMin (`cache_min` in the C++) --
    a real second pass, not reusable from the per-side mins, since it's
    computed over the COMBINED tag+data metrics.

    Forward traversal with strict `<`, matching find_optimal_uca's
    `for niter = ulist.begin()...` -- ties favor the FIRST combined
    candidate in `data_candidates`' order (which must be
    filter_data_side's output, itself order-preserving over
    enumerate_side's sorted list)."""
    combined = [
        _combine(tag_winner, d, fast_access=fast_access, is_seq_acc=is_seq_acc)
        for d in data_candidates
    ]
    cache_min = RunningMin()
    for c in combined:
        cache_min.min_delay = min(cache_min.min_delay, c.access_time)
        cache_min.min_dyn = min(cache_min.min_dyn, c.dynamic)
        cache_min.min_leakage = min(cache_min.min_leakage, c.leakage)
        cache_min.min_area = min(cache_min.min_area, c.area)
        cache_min.min_cyc = min(cache_min.min_cyc, c.cycle_time)

    if not combined:
        raise ValueError("no valid cache organizations found")

    best = None
    best_cost = BIGNUM
    for c in combined:
        if ed == 1:
            cost = (c.access_time / cache_min.min_delay) * (
                c.dynamic / cache_min.min_dyn
            )
        elif ed == 2:
            cost = ((c.access_time / cache_min.min_delay) ** 2) * (
                c.dynamic / cache_min.min_dyn
            )
        else:
            if not _check_mem_org_combined(
                c,
                cache_min,
                delay_dev,
                dynamic_power_dev,
                leakage_power_dev,
                cycle_time_dev,
                area_dev,
            ):
                continue
            cost = (
                delay_wt * (c.access_time / cache_min.min_delay)
                + cycle_time_wt * (c.cycle_time / cache_min.min_cyc)
                + dynamic_power_wt * (c.dynamic / cache_min.min_dyn)
                + leakage_power_wt * (c.leakage / cache_min.min_leakage)
                + area_wt * (c.area / cache_min.min_area)
            )
        if cost < best_cost:
            best_cost = cost
            best = c

    if best is None:
        raise ValueError("no cache organizations met optimization criteria")
    return best, cache_min


def _check_mem_org_combined(
    c,
    running_min,
    delay_dev,
    dynamic_power_dev,
    leakage_power_dev,
    cycle_time_dev,
    area_dev,
):
    if (
        c.access_time - running_min.min_delay
    ) * 100 / running_min.min_delay > delay_dev:
        return False
    if (
        c.dynamic - running_min.min_dyn
    ) / running_min.min_dyn * 100 > dynamic_power_dev:
        return False
    if (
        c.leakage - running_min.min_leakage
    ) / running_min.min_leakage * 100 > leakage_power_dev:
        return False
    if (
        c.cycle_time - running_min.min_cyc
    ) / running_min.min_cyc * 100 > cycle_time_dev:
        return False
    if (c.area - running_min.min_area) / running_min.min_area * 100 > area_dev:
        return False
    return True


def search_partition(
    data_cfg, sys_params, tag_cfg=None, ed=DEFAULT_ED, **cost_kwargs
):
    """Select an organization using native per-side filtering and joint ranking.

    A missing tag_cfg selects a single RAM/FA/CAM array. Returns the combined
    winner, tag winner, surviving data candidates and data-side minima."""
    out_w = data_cfg.out_w
    block_sz = data_cfg.block_sz
    if getattr(data_cfg, "pure_cam", False) or data_cfg.assoc == 0:
        data_nspd_min = 1.0
    else:
        data_nspd_min = out_w / (block_sz * 8.0)

    data_candidates, data_running_min, data_skipped = enumerate_side(
        data_cfg, sys_params, data_nspd_min, name_hint="search_data"
    )
    data_candidates = filter_data_side(data_candidates, data_running_min)

    tag_winner = None
    tag_skipped = 0
    if tag_cfg is not None:
        tag_candidates, tag_running_min, tag_skipped = enumerate_side(
            tag_cfg, sys_params, 0.125, name_hint="search_tag"
        )
        # filter_tag_side shares the same weight/dev knobs as rank_candidates
        # (real CACTI: both read one shared l_ip object).
        tag_kwargs = {
            k: v
            for k, v in cost_kwargs.items()
            if k not in ("fast_access", "is_seq_acc")
        }
        tag_winner = filter_tag_side(
            tag_candidates, tag_running_min, **tag_kwargs
        )

    # find_delay's branch (cacti_interface.cc:69-101) is a property of the
    # ARRAY, read straight off g_ip in the C++ -- so read it off data_cfg here
    # rather than making every caller re-pass it as a cost knob (a caller may
    # still override either flag explicitly through cost_kwargs).
    combine_kwargs = dict(
        fast_access=data_cfg.fast_access, is_seq_acc=data_cfg.is_seq_acc
    )
    combine_kwargs.update(
        {
            k: cost_kwargs.pop(k)
            for k in list(cost_kwargs)
            if k in ("fast_access", "is_seq_acc")
        }
    )
    winner, cache_min = rank_candidates(
        tag_winner, data_candidates, ed=ed, **combine_kwargs, **cost_kwargs
    )
    return SearchResult(
        winner=winner,
        tag_winner=tag_winner,
        data_candidates=data_candidates,
        data_running_min=data_running_min,
        data_skipped=data_skipped,
        tag_skipped=tag_skipped,
    )


SearchResult = namedtuple(
    "SearchResult",
    "winner tag_winner data_candidates data_running_min data_skipped tag_skipped",
)


def _area_efficiency_pct(candidate):
    """Ucache.cc:331 (ptr_array->area_efficiency = uca->area_all_dataramcells
    * 100 / uca->area.get_area()) + uca.cc:101 (area_all_dataramcells =
    subarray.get_total_cell_area() * dp.num_subarrays * nbanks) +
    subarray.cc:98 (get_total_cell_area). Used ONLY by
    search_partition_layer3()'s FA/CAM escape-hatch admission test
    (array.cc:149, `area_efficiency < 20.0 && assoc==0`) -- no other caller
    in this port needs it, so it is not a CactiSubarray method, just a
    read of already-computed private geometry off a Candidate's built
    CactiUCA/CactiMat/CactiSubarray chain (same pattern _combine already
    uses for tag.uca.derived_area_h etc.).

    pure_cam's non-FA CAM branch (subarray.cc:114) is intentionally not
    ported -- pure_cam is guarded out everywhere else in this project
    (never set true anywhere in McPAT, per cacti_dynamic_params.py's own
    note) and CactiSubarray has no pure_cam-only geometry fields to read.

    The `* nbanks` multiply is ported per the C++ formula but UNTESTED:
    both real ground-truth points this function has ever been validated
    against (see search_partition_layer3()'s docstring) have nbanks==1,
    so a real nbanks>1 bug here would not have been caught."""
    mat = candidate.uca.mat
    sub = mat.subarray
    # num_subarrays/dp live on CactiMat (set_dynamic_parameters is only
    # ever called there, cacti_component.py:1713), NOT on CactiSubarray
    # itself -- confirmed by grep, not assumed (CactiSubarray.__init__
    # never calls it).
    if sub._is_fa:
        cell_area = (
            sub._cam_cell_h
            * (sub._rows + 1)
            * (
                sub._cam_cell_w * sub._num_cols_fa_cam
                + sub._cell_w * sub._num_cols_fa_ram
            )
        )
    else:
        cell_area = sub._cell_w * sub._cell_h * sub._rows * sub._cols
    area_all_dataramcells = cell_area * mat._num_subarrays * mat.dp.cfg.nbanks
    return area_all_dataramcells * 100.0 / candidate.area


_L3_FIXED_WEIGHTS = dict(
    delay_wt=100.0,
    cycle_time_wt=1000.0,
    area_wt=10.0,
    dynamic_power_wt=10.0,
    leakage_power_wt=10.0,
)
_L3_FIXED_DEVS = dict(
    delay_dev=1_000_000.0,
    area_dev=1_000_000.0,
    dynamic_power_dev=1_000_000.0,
    leakage_power_dev=1_000_000.0,
)
_L3_AREA_EFFICIENCY_THRESHOLD = 20.0
_L3_OPTIMIZATION_END = 20
_L3_CYCLE_TIME_DEV_START = 100

Layer3Result = namedtuple(
    "Layer3Result",
    "winner engaged iters candidates satisfied throughput_ok latency_ok",
)


def search_partition_layer3(
    data_cfg, sys_params, throughput, latency, tag_cfg=None
):
    """McPAT ArrayST timing-target relaxation around the baseline ED² search.

    Targets are seconds. Preserve the descending cycle-deviation loop,
    fully-associative area-efficiency escape and native warning flags.
    Unsatisfied targets still return a usable selected organization.
    The outer optimize_array_partition applies the native timing gate."""
    base = search_partition(data_cfg, sys_params, tag_cfg=tag_cfg)
    throughput_overflow = (base.winner.cycle_time - throughput) > 1e-10
    latency_overflow = (base.winner.access_time - latency) > 1e-10

    if not (throughput_overflow or latency_overflow):
        return Layer3Result(
            winner=base.winner,
            engaged=False,
            iters=0,
            candidates=[],
            satisfied=True,
            throughput_ok=True,
            latency_ok=True,
        )

    is_fa_cam = data_cfg.assoc == 0
    candidates = []
    # The most recent iteration's result, whether or not it was admitted --
    # real McPAT's `local_result` is simply overwritten by every
    # compute_base_power() call, so this tracks the same object the C++
    # falls back on when candidate_solutions ends up empty.
    last_result = None
    iters = 0
    cycle_time_dev = _L3_CYCLE_TIME_DEV_START
    while (
        throughput_overflow or latency_overflow
    ) and cycle_time_dev > _L3_OPTIMIZATION_END:
        result = search_partition(
            data_cfg,
            sys_params,
            tag_cfg=tag_cfg,
            ed=0,
            cycle_time_dev=float(cycle_time_dev),
            **_L3_FIXED_WEIGHTS,
            **_L3_FIXED_DEVS,
        )
        iters += 1
        cycle_time_dev -= 10
        last_result = result
        w = result.winner

        meets_throughput = (w.cycle_time - throughput) <= 1e-10
        meets_latency = (w.access_time - latency) <= 1e-10
        satisfies_both = meets_throughput and meets_latency
        # area_efficiency is a DATA-side-only quantity (mem_array::
        # area_efficiency lives on data_array2, array.cc:149 reads it
        # unconditionally regardless of tag_cfg) -- read off the winning
        # data candidate specifically, not the combined result.
        escape = (
            is_fa_cam
            and _area_efficiency_pct(w.data) < _L3_AREA_EFFICIENCY_THRESHOLD
        )

        if satisfies_both or escape:
            candidates.append(result)
            if satisfies_both:
                throughput_overflow = False
                latency_overflow = False
            # else: admitted via the escape hatch, flags stay untouched --
            # the loop keeps relaxing even though a candidate was recorded,
            # replicating real McPAT's control flow exactly.
        else:
            # A rejected candidate can still clear an individual flag --
            # a later, worse iteration can end the loop early this way.
            if meets_throughput:
                throughput_overflow = False
            if meets_latency:
                latency_overflow = False

    if not candidates:
        # Loop exhausted with nothing admitted (array.cc:229, candidate_
        # solutions empty -> the `local_result = *min_dynamic_energy_iter`
        # assignment never runs). Real McPAT then keeps whatever the LAST
        # loop iteration's compute_base_power() left in local_result: the
        # else-branch only cleans it up while `l_ip.cycle_time_dev >
        # optimization_end`, and the decrement to 20 happens BEFORE that
        # test, so the final (cycle_time_dev=30) result is never cleaned and
        # survives as the answer. Returning base.winner here instead was
        # wrong twice over -- it is not what McPAT keeps, and McPAT has
        # already explicitly cleanup()'d the ed=2 base before the loop
        # (array.cc:135-136). Measured against real ./mcpat on the 1MiB L2
        # at 180nm/HP, which reaches this branch: assoc=8 blocks the FA/CAM
        # area_efficiency escape and the latency target is unsatisfiable by
        # any geometry, so nothing is ever admitted.
        #
        # `satisfied` uses the SAME expression as the non-empty return below,
        # because real McPAT's "cannot satisfy throughput/latency" warnings
        # (array.cc:180-187) are emitted from the post-loop flag values and
        # sit BEFORE the candidate_solutions min-scan -- they do not care
        # whether anything was admitted. Hardcoding False here would be wrong
        # in a reachable state: the else-branch can clear throughput_overflow
        # on one iteration and latency_overflow on a later one WITHOUT ever
        # admitting a candidate (satisfies_both is required to append, and it
        # is false whenever only one flag's condition holds), so the loop can
        # exit with both flags cleared and `candidates` still empty. The
        # measured 180nm L2 does not reach that state -- it exits with
        # throughput_overflow False but latency_overflow True, so `satisfied`
        # is False there either way.
        return Layer3Result(
            winner=last_result.winner,
            engaged=True,
            iters=iters,
            candidates=[],
            satisfied=(not throughput_overflow and not latency_overflow),
            throughput_ok=not throughput_overflow,
            latency_ok=not latency_overflow,
        )

    # Real McPAT only replaces the running winner on a strictly lower
    # value, scanning candidates in cycle_time_dev-descending order -- a
    # tie keeps the highest-dev candidate. Python's min() ties the same way.
    winner_result = min(candidates, key=lambda r: r.winner.dynamic)
    return Layer3Result(
        winner=winner_result.winner,
        engaged=True,
        iters=iters,
        candidates=candidates,
        satisfied=(not throughput_overflow and not latency_overflow),
        throughput_ok=not throughput_overflow,
        latency_ok=not latency_overflow,
    )


def layer3_gate(opt_for_clk, opt_local, capacity, assoc):
    """array.cc:113 -- whether ArrayST::optimize_array may run Layer 3 at
    all (small arrays are skipped: "over opt small array lead to
    sub-optimal solutions"). `capacity` is the array.cc:63-clamped
    cache_sz."""
    if not (opt_for_clk and opt_local):
        return False
    return (capacity > 2048 and assoc != 0) or (capacity > 256 and assoc == 0)


def optimize_array_partition(
    data_cfg,
    sys_params,
    tag_cfg=None,
    *,
    opt_for_clk=False,
    opt_local=False,
    throughput=None,
    latency=None,
):
    """ArrayST::optimize_array's search: search_partition_layer3() at the
    array's own throughput/latency targets (seconds, McPAT's
    N_cycles/clockRate) when layer3_gate() fires, else the plain Layer-1/2
    search_partition(). Both results expose `.winner`. Raises ValueError
    when the gate fires but a target is missing."""
    if not layer3_gate(
        opt_for_clk, opt_local, data_cfg.capacity, data_cfg.assoc
    ):
        return search_partition(data_cfg, sys_params, tag_cfg=tag_cfg)
    if throughput is None or latency is None:
        raise ValueError(
            "Layer-3 gate fired (opt_for_clk and opt_local on, "
            f"capacity={data_cfg.capacity}, assoc={data_cfg.assoc}) but "
            "throughput/latency targets are missing"
        )
    return search_partition_layer3(
        data_cfg, sys_params, throughput, latency, tag_cfg=tag_cfg
    )
