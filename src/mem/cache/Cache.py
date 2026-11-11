# Copyright (c) 2012-2013, 2015, 2018, 2023-2024 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2005-2007 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from m5.objects.ClockedObject import ClockedObject
from m5.objects.Compressors import BaseCacheCompressor
from m5.objects.Prefetcher import BasePrefetcher
from m5.objects.ReplacementPolicies import *
from m5.objects.Tags import *
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


# Enum for cache clusivity, currently mostly inclusive or mostly
# exclusive.
class Clusivity(Enum):
    vals = ["mostly_incl", "mostly_excl"]


class WriteAllocator(SimObject):
    type = "WriteAllocator"
    cxx_header = "mem/cache/cache.hh"
    cxx_class = "gem5::WriteAllocator"

    # Control the limits for when the cache introduces extra delays to
    # allow whole-line write coalescing, and eventually switches to a
    # write-no-allocate policy.
    coalesce_limit = Param.Unsigned(
        2, "Consecutive lines written before delaying for coalescing"
    )
    no_allocate_limit = Param.Unsigned(
        12, "Consecutive lines written before skipping allocation"
    )

    delay_threshold = Param.Unsigned(
        8,
        "Number of delay quanta imposed on an "
        "MSHR with write requests to allow for "
        "write coalescing",
    )

    block_size = Param.Int(Parent.cache_line_size, "block size in bytes")


class BaseCache(ClockedObject):
    type = "BaseCache"
    abstract = True
    cxx_header = "mem/cache/base.hh"
    cxx_class = "gem5::BaseCache"

    size = Param.MemorySize("Capacity")
    assoc = Param.Unsigned("Associativity")
    drt = Param.Tick(
        0, "Data retention time (DRT)"
    )  # default 0 - no retention

    top_mru = Param.Unsigned(0, "Number of MRU blocks to refresh in daemon")

    refresh_dirty_daemon = Param.Bool(
        0, "Refresh dirty blocks using daemon process"
    )

    refresh_set_scan_cycles = Param.Cycles(
        1, "Cycles for the refresh daemon to scan one set's tags"
    )

    refresh_banks = Param.Unsigned(
        1,
        "Data-array banks (bank = set mod banks); a refresh occupies only "
        "its bank and banks refresh in parallel",
    )

    refresh_subarrays = Param.Unsigned(
        1,
        "Subarrays per bank refreshed in parallel: one bank operation "
        "(refresh_line_cycles) covers this many consecutive rows of the "
        "bank, one per subarray (refresh_scan=line or refresh_all_even "
        "only)",
    )

    refresh_subbank_block = Param.Bool(
        False,
        "Refresh-all even spacing with subarrays: an operation blocks only "
        "the subarrays (sub-banks) whose row it refreshes (holds a valid "
        "line after the window decision), for refresh_line_cycles; a hit "
        "waits only for its own sub-bank. Off: the whole bank for every "
        "operation",
    )

    reuse_stats = Param.Bool(
        False,
        "Stats only: time since each line's last access at a demand hit, "
        "lines leaving with no hit, and misses to lines the SRC dropped "
        "that an SRAM of the same geometry would still hold (shadow LRU "
        "tags). Works with drt 0 too; changes no timing",
    )

    reuse_idle_gaps = Param.Bool(
        False,
        "Stats only: per-line idle gaps. At every access that finds a line "
        "(demand hit, writeback from above, CleanEvict) the time since its "
        "previous access (a fill starts the clock), and at every replacement "
        "eviction the time since its last access (reuse.idleGapAtAccess, "
        "reuse.idleGapAtEviction). Changes no timing",
    )

    reuse_fill_stats = Param.Bool(
        False,
        "Stats only (group reuse.fill): per line the tick of its demand "
        "fill; the time from that fill to its first demand hit and to its "
        "first use (demand hit, writeback or CleanEvict from above), and how "
        "many demand-filled lines leave by replacement with no hit or no "
        "use. Changes no timing",
    )

    data_read_ports = Param.Unsigned(
        0,
        "Data-array reads (load hits, writeback reads) each bank serves "
        "per cycle (bank = set mod refresh_banks), in arrival order; 0 "
        "with data_write_ports 0 means unlimited, as stock gem5. A "
        "refresh blocks its bank's reads and writes for "
        "refresh_line_cycles; a forced one pre-empts the waiting accesses",
    )

    data_write_ports = Param.Unsigned(
        0,
        "Data-array writes (store hits, fills) each bank serves per "
        "cycle; 0 with data_read_ports 0 means unlimited",
    )

    refresh_line_cycles = Param.Cycles(
        0,
        "Data-array cycles of one full-line refresh (read + write); "
        "0 means 2 x data_latency",
    )

    refresh_line_budget_cycles = Param.Cycles(
        0,
        "Cycles one refresh or writeback read is budgeted for in T_sweep "
        "and T_guard (the bank is occupied for refresh_line_cycles); 0 "
        "means refresh_line_cycles",
    )

    refresh_scan = Param.String(
        "set",
        "Refresh scanner: set (one scanner visits a whole set per step) "
        "or line (one scanner per bank, one line per step, a single "
        "pending operation per bank, as the RTL controller does); line "
        "needs active_invalidate and top_mru = 0",
    )

    refresh_mode = Param.String(
        "policy",
        "policy (the daemon's keep and refresh rules), or refresh-all: "
        "every period of DRT - refresh_all_window_cycles each bank's sweep "
        "refreshes its rows in order, back to back from the period start "
        "(all_strict) or in idle bank cycles with a forced tail that ends "
        "within the window (all_opp); banks sweep in parallel (numSets a "
        "multiple of refresh_banks); needs no daemon and no inline refresh",
    )

    refresh_all_window_cycles = Param.Cycles(
        4096,
        "Refresh-all: the window W a sweep completes in; the period is "
        "DRT - W",
    )

    refresh_all_even = Param.Bool(
        False,
        "Refresh-all strict: one row of each bank every P / rows cycles "
        "(row interval P), starting on the next cycle ahead of the waiting "
        "accesses, instead of back-to-back sweeps",
    )

    refresh_all_free = Param.Bool(
        False,
        "Refresh-all, test only: refreshes take no bank time (the sweep "
        "still recharges every row on its schedule)",
    )

    window_mode = Param.String(
        "off",
        "Adaptive-window drops on top of refresh-all strict with even "
        "spacing (same sweep and bank occupancy): off, fixed, tcp, amc, "
        "wb (Refrint WB(n,m): per-line countdowns, see wb_n / wb_m) or "
        "chang (time-based dead-line predictor of Chang et al., HPCA 2013: "
        "per-set indicators, see chang_time) or tail (per-line ages and a "
        "global window W from a reuse-age histogram, see tail_*). "
        "The cache is split into window_groups groups (set mod groups); a "
        "group decides at a frame (one sweep) at least N frames after its "
        "last decision, and its unused lines are dropped then",
    )

    window_n_fixed = Param.Unsigned(
        8,
        "Window mode fixed: N; above window_group_counter_max + 1 no "
        "group is ever due (nothing drops)",
    )

    window_tcp_n_init = Param.Unsigned(1, "Window mode tcp: initial N")

    window_amc_n_init = Param.Unsigned(8, "Window mode amc: initial N")

    window_ssthresh_init = Param.Unsigned(
        8, "Window mode tcp: initial ssthresh"
    )

    window_n_max = Param.Unsigned(64, "Window modes tcp and amc: largest N")

    window_up_shift = Param.Unsigned(
        7,
        "Window mode tcp: N grows at a frame end when the retention misses "
        "exceed the demand accesses >> window_up_shift",
    )

    window_down_shift = Param.Unsigned(
        9,
        "Window mode tcp: a shrink check is calm when the retention misses "
        "are below the demand accesses >> window_down_shift",
    )

    access_trace = Param.Bool(
        False,
        "Text trace of the accesses from above, fills and evictions, from "
        "the first stats reset on, to <cache>.access_trace.txt.gz (stats "
        "only)",
    )

    window_n_min = Param.Unsigned(
        1,
        "Window mode tcp / amc: smallest N (shrinks stop there); the tcp "
        "initial N must be >= it",
    )

    window_frame_log = Param.Bool(
        False,
        "Window modes (wb too): write <cache>.window_frames.csv, one row "
        "per frame (N, retention misses, demand accesses, drops, count "
        "write backs); stats only",
    )

    window_min_accesses = Param.Unsigned(
        512,
        "Window mode tcp: a shrink check is judged only with at least this "
        "many demand accesses since the last reset",
    )

    window_streak_len = Param.Unsigned(
        3,
        "Window mode tcp: calm decrements of N before shrinks halve it",
    )

    window_calm_len = Param.Unsigned(
        2, "Window mode tcp: calm checks in a row that shrink N"
    )

    window_amc_interval_cycles = Param.Cycles(
        1000000, "Window mode amc: decision interval"
    )

    window_amc_target_frac = Param.Float(
        0.5,
        "Window mode amc: target retention misses as a fraction of the "
        "interval's demand misses that are not retention misses",
    )

    window_amc_hi = Param.Float(
        1.5, "Window mode amc: N doubles when misses > hi x target"
    )

    window_amc_lo = Param.Float(
        0.5, "Window mode amc: N halves when misses < lo x target"
    )

    window_groups = Param.Unsigned(
        64,
        "Window modes: groups (set mod groups); at most "
        "max(1, groups >> floor(log2 N)) due groups decide per frame",
    )

    window_group_counter_max = Param.Unsigned(
        127, "Window modes: saturation of the per-group frame counters"
    )

    window_fill_u_always = Param.Bool(
        False,
        "Window modes: a fill sets U = 1 (instead of U = decide of its "
        "group)",
    )

    window_cleanevict_sets_u = Param.Bool(
        False,
        "Window modes: a CleanEvict from above that finds a valid line "
        "uses it (U = 1, D = 0)",
    )

    tail_drop_wb_cached = Param.Bool(
        False,
        "Retention drops (window modes) with writeback_clean: a clean drop "
        "whose line is still cached above still sends its WritebackClean "
        "with data, BLOCK_CACHED set (the level below stores it and keeps "
        "this cache listed); replacement evictions unchanged",
    )

    tail_keep_cached = Param.Bool(
        False,
        "window_mode=tail: at its slot a due line still cached above is "
        "refreshed instead of dropped (retried at its next slot; its age "
        "keeps counting)",
    )

    window_drop_notify = Param.Bool(
        False,
        "Window modes: a clean drop sends its CleanEvict at the drop "
        "(write buffer, cached-above check, as an SRAM eviction) instead "
        "of when its expired tag is replaced. Needed when a cache below "
        "can evict the line on its own (an L3): a late notice reaches an "
        "L3 that has already evicted the line",
    )

    window_probation_frames = Param.Unsigned(
        0,
        "Window mode: a line filled by a demand miss is on probation, "
        "exempt from its group's decisions, and is dropped if still unused "
        "at its M-th sweep slot after the fill (clean: invalidated; dirty: "
        "written back or deferred as other drops). Any use ends probation. "
        "Misses on lines probation dropped are counted apart and do not "
        "feed the controller. 0 = off; at most 3 (2-bit counter)",
    )

    wb_n = Param.Int(
        -1,
        "window_mode=wb (Refrint WB(n,m)): idle frames a dirty line is "
        "kept; at its slot with the count at 0 it is written back (it stays "
        "valid and clean, count = wb_m). -1 = infinity",
    )

    wb_m = Param.Int(
        -1,
        "window_mode=wb: idle frames a clean line is kept; at its slot with "
        "the count at 0 it is invalidated (expired tag as the window "
        "drops). Every use reloads the count (wb_n if dirty, wb_m if "
        "clean) at the line's next slot. -1 = infinity",
    )

    chang_time = Param.Unsigned(
        256,
        "window_mode=chang: frames (sweeps) per global tick. At a tick a "
        "LIVE line latches r = I of its set (I <= 5; I = 6 keeps it LIVE) "
        "and a counting line decrements r; r = 0 is DEAD, dropped at its "
        "slot. A use makes the line LIVE. A retention miss raises the "
        "set's I (to 6, absorbing: no more drops), an expired tag replaced "
        "by another address lowers it",
    )

    tail_eps_shift = Param.Unsigned(
        9,
        "window_mode=tail: W covers the reuse ages whose tail holds more "
        "than 2^-tail_eps_shift of the ended intervals: at an epoch, for "
        "b = 10 down to 1, tail += reuses[b]; the first b with tail << "
        "tail_eps_shift > total gives K = 2^b - 1 (b = 10: inf), else K = 0",
    )

    tail_epoch = Param.Unsigned(
        64,
        "window_mode=tail: frames (sweeps) per controller epoch. At an "
        "epoch W follows K (see tail_damp), then every ended[] and reuses[] "
        "counter is halved",
    )

    tail_epoch_events = Param.Unsigned(
        0,
        "window_mode=tail (ablation, user 2026-11-10): 0 = an epoch every "
        "tail_epoch frames; c > 0 = an epoch at the first frame end where "
        "the interval ends counted since the last epoch reach c x "
        "2^tail_eps_shift (c / eps; tail_epoch unused); tail_fastup "
        "unchanged (every other frame end)",
    )
    tail_fastup_events = Param.Unsigned(
        0,
        "window_mode=tail (ablation, user 2026-11-10): 1 = an event-"
        "triggered fast-up: count the reuses at ages > W since W last "
        "changed; the moment count x 2^tail_eps_shift > the live "
        "interval-end count, recompute K from the live counters and raise "
        "W to K if larger (no halving); frame-end fast-up unchanged",
    )
    tail_budget_stop = Param.Unsigned(
        0,
        "window_mode=tail (ablation, user 2026-11-10): 1 = count the "
        "retention misses since the last epoch end; once count x "
        "2^tail_eps_shift >= the live interval-end count (the controller "
        "histogram total), W = inf until the next epoch end, where the "
        "normal epoch update resumes; 0 = off",
    )

    tail_w0 = Param.Unsigned(
        7,
        "window_mode=tail: initial W (2^k - 1 <= 511). At its slot a valid "
        "line's age (its set's slots since its last use) grows by 1; age "
        "<= W refreshes it, else it is dropped (clean: invalidated, dirty: "
        "written back, both keep an expired tag)",
    )

    tail_damp = Param.Unsigned(
        1,
        "window_mode=tail: 1 = W grows to K at once but shrinks at most "
        "one rung per epoch (W = max(K, 511 if W = inf else (W - 1) / 2)); "
        "0 = W = K",
    )

    tail_fixed = Param.String(
        "-1",
        "window_mode=tail: -1 = the controller sets W; inf = W never drops "
        "(refresh-all behaviour); n >= 0 = W fixed at n (controller off, "
        "the histograms are still kept)",
    )

    tail_kmin = Param.Unsigned(
        0,
        "window_mode=tail (ablation): W never goes below 2^tail_kmin - 1: "
        "after every W update W = max(W, 2^tail_kmin - 1), w0 clamped the "
        "same way (0 = no floor; ignored with tail_fixed)",
    )

    tail_kmax = Param.Unsigned(
        10,
        "window_mode=tail (ablation): a K whose bin is above tail_kmax is "
        "infinity, so W takes only the values 2^k - 1 for k <= tail_kmax, "
        "or inf (damping from inf goes to the largest of those, at most "
        "511); 10 = no cap (ignored with tail_fixed)",
    )

    tail_budget = Param.String(
        "eps",
        "window_mode=tail (user, 2026-11-10): the right-hand side of the K "
        "rule. eps = the reuse tail above K at most 2^-tail_eps_shift x "
        "all interval ends (sum of ended[]); lru_rel = at most "
        "tail_budget_delta x the interval ends that were not reuses "
        "(sum of ended[] - reuses[]); used at epochs and fast-up checks; "
        "bucket (user, 2026-11-10) = the eps K rule plus a token bucket on "
        "the drops (tail_bucket_max)",
    )

    tail_bucket_max = Param.Unsigned(
        4096,
        "window_mode=tail, tail_budget=bucket: TOK_MAX. A token per "
        "2^tail_eps_shift interval ends (the ended[] events; prescaler), "
        "saturating at tail_bucket_max; a drop at the sweep needs a token "
        "and spends it, else the line is refreshed and kept; an expired tag "
        "that ends without a reuse (replaced, refilled by a writeback, "
        "invalidated, cleared) refunds one (saturating); a retention miss "
        "refunds nothing. Starts empty, never reset",
    )

    tail_bucket_start = Param.String(
        "empty",
        "window_mode=tail, tail_budget=bucket: the token count at the "
        "start (the checkpoint restore): empty = 0 (default); full = "
        "tail_bucket_max (user, 2026-11-10: a warmup-free check); or a "
        "token count <= tail_bucket_max (user, 2026-11-11: INIT 6144 = "
        "the L2's lines)",
    )

    tail_budget_delta = Param.Float(
        0.01,
        "window_mode=tail, tail_budget=lru_rel: delta (fraction, kept in "
        "parts per million)",
    )

    tail_fastup = Param.Bool(
        False,
        "window_mode=tail (ablation): at every frame end that is not an "
        "epoch, K from the current ended[] / reuses[] with the epoch rule; "
        "W = K if K > W (no halving there; epochs unchanged)",
    )

    tail_halve = Param.Bool(
        True,
        "window_mode=tail (ablation): False = ended[] and reuses[] are "
        "never halved (cumulative from the start of the run)",
    )

    tail_victim = Param.String(
        "expired",
        "window_mode=tail victim in a set holding expired tags: expired = "
        "an expired tag of the address, an empty way, the expired tag with "
        "the oldest last use, (else the tags' LRU); lru = an expired tag of "
        "the address, an empty way, else the way (valid line or expired "
        "tag) whose last use (fill, demand hit, writeback, CleanEvict) is "
        "oldest",
    )

    tail_log_hist = Param.Bool(
        False,
        "window_mode=tail (stats only): <cache>.tail_hist.csv, one row per "
        "epoch before the halving: tick, frame, W before, K, W after, "
        "total, ended[0..10], reuses[0..10] (with tail_fixed: K is "
        "computed and logged, never applied)",
    )

    tail_log_frame = Param.Bool(
        False,
        "window_mode=tail (stats only): <cache>.tail_frame.csv, one row per "
        "frame end (W in effect during the frame): tick, frame, W, demand "
        "retention misses, accesses matching an expired tag, expired tags "
        "present, their mean age (slots), valid lines, drops, expired tags "
        "replaced by another address, demand accesses (all per frame "
        "except the present counts)",
    )

    refresh_opp_age = Param.Float(
        0.8,
        "Age, as a fraction of DRT, from which the daemon refreshes a "
        "block while the data array is idle",
    )

    retention_margin = Param.Float(
        0.02, "Safety margin of the retention guarantee, fraction of DRT"
    )

    retention_guard_sweeps = Param.Float(
        1.0,
        "Worst-case daemon sweeps in T_guard: T_guard = guard_sweeps x "
        "T_sweep + line + margin x DRT",
    )

    cold_dirty = Param.String(
        "refresh",
        "Dirty lines with active_invalidate: refresh (keep them all), mru "
        "(write back and invalidate dirty lines outside the top_mru once "
        "inside T_guard), touched (the same for dirty lines not hit since "
        "their last full charge)",
    )

    keep_touched = Param.Bool(
        False,
        "With active_invalidate: a line hit since its last full charge "
        "(a per-line reference bit, set by a hit, cleared by a fill or "
        "refresh) is kept like a top_mru line, clean or dirty; with "
        "cold_dirty, a dirty line is cold if neither top_mru nor touched",
    )

    touched_clear_age = Param.Float(
        0.0,
        "With keep_touched: the daemon also clears a line's reference bit "
        "once its age reaches this fraction of DRT, so the bit means 'hit "
        "since then' (0: cleared only by a fill or refresh)",
    )

    active_invalidate = Param.Bool(
        False,
        "The daemon invalidates every clean line it does not keep once "
        "its remaining retention is below T_guard, so no valid line can "
        "expire; needs refresh_dirty_daemon and no inline refresh",
    )

    silent_clean_drop = Param.Bool(
        False,
        "With active_invalidate: a clean line dropped at its deadline is "
        "invalidated without a CleanEvict and takes no write-buffer slot "
        "(as the RTL controller does)",
    )

    quantized_age = Param.Bool(
        False,
        "The daemon compares line ages in epochs of 2^age_epoch_log2 "
        "cycles, read from an age_bits-bit stamp written at each full "
        "charge (as the RTL controller does), instead of exact ticks",
    )

    age_epoch_log2 = Param.Unsigned(
        10, "With quantized_age: log2 of the epoch length in cycles"
    )

    age_bits = Param.Unsigned(
        5, "With quantized_age: bits of the per-line epoch stamp"
    )

    deadline_epochs = Param.Int(
        -1,
        "With quantized_age: deadline age F in epochs; -1 derives "
        "F = floor((DRT - T_guard) / epoch)",
    )

    opp_age_epochs = Param.Int(
        -1,
        "With quantized_age: age in epochs from which idle-time refreshes "
        "start; -1 derives floor(refresh_opp_age x DRT / epoch)",
    )

    touched_clear_epochs = Param.Int(
        -1,
        "With quantized_age: age in epochs at which the reference bit is "
        "cleared; -1 derives round(touched_clear_age x DRT / epoch)",
    )

    inline_refresh = Param.Bool(
        True,
        "A store hit to a block with less than T_guard of retention left "
        "recharges the line (read, merge, write the full line)",
    )

    inline_refresh_on_read = Param.Bool(
        False, "Read hits near expiry also recharge the line"
    )

    inline_refresh_cycles = Param.Cycles(
        0, "Extra hit latency of an inline refresh; 0 means data_latency"
    )

    drt_strict = Param.Bool(
        False,
        "Panic when a dirty block expires (retention invariant violation) "
        "instead of counting it",
    )

    tag_latency = Param.Cycles("Tag lookup latency")
    data_latency = Param.Cycles("Data access latency")
    response_latency = Param.Cycles("Latency for the return path on a miss")

    warmup_percentage = Param.Percent(
        0, "Percentage of tags to be touched to warm up the cache"
    )

    max_miss_count = Param.Counter(
        0, "Number of misses to handle before calling exit"
    )

    mshrs = Param.Unsigned("Number of MSHRs (max outstanding requests)")
    demand_mshr_reserve = Param.Unsigned(1, "MSHRs reserved for demand access")
    tgts_per_mshr = Param.Unsigned("Max number of accesses per MSHR")
    write_buffers = Param.Unsigned(8, "Number of write buffers")

    is_read_only = Param.Bool(False, "Is this cache read only (e.g. inst)")

    prefetcher = Param.BasePrefetcher(NULL, "Prefetcher attached to cache")

    tags = Param.BaseTags(BaseSetAssoc(), "Tag store")
    replacement_policy = Param.BaseReplacementPolicy(
        LRURP(), "Replacement policy"
    )
    partitioning_manager = Param.PartitionManager(
        NULL, "Cache partitioning manager"
    )

    compressor = Param.BaseCacheCompressor(NULL, "Cache compressor.")
    replace_expansions = Param.Bool(
        True,
        "Apply replacement policy to "
        "decide which blocks should be evicted on a data expansion",
    )
    # When a block passes from uncompressed to compressed, it may become
    # co-allocatable with another existing entry of the same superblock,
    # so try move the block to co-allocate it
    move_contractions = Param.Bool(
        True, "Try to co-allocate blocks that contract"
    )

    sequential_access = Param.Bool(
        False, "Whether to access tags and data sequentially"
    )

    cpu_side = ResponsePort("Upstream port closer to the CPU and/or device")
    mem_side = RequestPort("Downstream port closer to memory")

    addr_ranges = VectorParam.AddrRange(
        [AllMemory], "Address range for the CPU-side port (to allow striping)"
    )

    system = Param.System(Parent.any, "System we belong to")

    # Determine if this cache sends out writebacks for clean lines, or
    # simply clean evicts. If this cache does not have a downstream cache,
    # the cache should not writeback clean lines not to waste memory
    # bandwidth. If this cache has a downstream cache whose clusivity is
    # mostly exclusive (i.e., victim cache), this shoule be set to True.
    # If not, there will never be any spills from read-only caches (e.g.,
    # L1I cache, MMU cache of ARM) to the downstream cache.
    # In case of the downstream cache is mostly inclusive, this should be
    # set to False.
    writeback_clean = Param.Bool(False, "Writeback clean lines")

    # Control whether this cache should be mostly inclusive or mostly
    # exclusive with respect to upstream caches. The behaviour on a
    # fill is determined accordingly. For a mostly inclusive cache,
    # blocks are allocated on all fill operations. Thus, L1 caches
    # should be set as mostly inclusive even if they have no upstream
    # caches. In the case of a mostly exclusive cache, fills are not
    # allocating unless they came directly from a non-caching source,
    # e.g. a table walker. Additionally, on a hit from an upstream
    # cache a line is dropped for a mostly exclusive cache.
    clusivity = Param.Clusivity("mostly_incl", "Clusivity with upstream cache")

    # The write allocator enables optimizations for streaming write
    # accesses by first coalescing writes and then avoiding allocation
    # in the current cache. Typically, this would be enabled in the
    # data cache.
    write_allocator = Param.WriteAllocator(NULL, "Write allocator")


class Cache(BaseCache):
    type = "Cache"
    cxx_header = "mem/cache/cache.hh"
    cxx_class = "gem5::Cache"


class NoncoherentCache(BaseCache):
    type = "NoncoherentCache"
    cxx_header = "mem/cache/noncoherent_cache.hh"
    cxx_class = "gem5::NoncoherentCache"

    # This is typically a last level cache and any clean
    # writebacks would be unnecessary traffic to the main memory.
    writeback_clean = False
