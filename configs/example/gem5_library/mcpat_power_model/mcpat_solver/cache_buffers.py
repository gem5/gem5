"""Pure cache-buffer geometry, shared by presets and live composition.

Mirrors core.cc and sharedcache.cc capacities, ports and data/tag widths.
Present buffers retain ArrayST's 64-byte minimum; configured policy omits
absent buffers. Search and energy calculations belong to the caller."""

import collections
import math

from .array_spec import CactiArraySpec

# McPAT EXTRA_TAG_BITS (core.cc / sharedcache.cc): the fixed pad added to
# phy_addr_width for every miss/fill/prefetch/WBB buffer's tag width.
_EXTRA_TAG_BITS = 5


def _miss_line_sz(capacity, line_bytes, phy_addr_width, shared):
    """MissBuffer / MissB packed data-word width, in bytes.

    core.cc:116/770 (L1):     data = phy_addr_w + ceil(log2(cap/line)) + line*8
    sharedcache.cc:155 (LLC):  data = phy_addr_w + ceil(log2(cap/line)) + line
      -- the LLC path adds the line size in BYTES, not bits (confirmed
      against real McPAT: L2 MissB gets block_sz=10, not 14).
    line_sz = ceil(data / 8).
    """
    idx_bits = int(math.ceil(math.log2(capacity / line_bytes)))
    payload = line_bytes if shared else line_bytes * 8
    return int(math.ceil((phy_addr_width + idx_bits + payload) / 8.0))


def _clamp64(cache_sz):
    # array.cc:63 -- if (l_ip.cache_sz < 64) l_ip.cache_sz = 64;
    return cache_sz if cache_sz >= 64 else 64


def build_cache_buffers(
    *,
    level,
    prefix,
    capacity,
    line_bytes,
    assoc,
    phy_addr_width,
    entry_counts,
    cache_policy,
    fetch_or_memory_ports=1,
    device_ty="core",
    wire_is_mat_type=0,
    wire_os_mat_type=None,
    wt_overhead=30,
    partitions=None,
    buffer_policy="mcpat",
):
    """Build the miss/fill/prefetch/write-back buffer Arrays for one cache.

    level     -- "l1i" | "l1d" | "shared"
    prefix    -- trace-name prefix ("icache", "dcache", "L2")
    capacity  -- parent cache data-array capacity, bytes
    line_bytes-- parent cache block width, bytes (NOT the XML output_width)
    assoc     -- parent cache associativity; accepted for call-site symmetry
                 with the cache config, unused here (buffers are always FA)
    entry_counts -- {"mshr", "fill", "prefetch", "wbb"} entry counts
    cache_policy -- "write_back" | "write_through" (gates the l1d WBB)
    wire_is_mat_type / wire_os_mat_type / wt_overhead -- default to McPAT's
                 embedded core setting (the frozen A9 anchors); a live caller
                 passes machine_model.mcpat_wire_kwargs(machine_params,
                 shared=level == "shared")
    partitions  -- None for geometry-only Arrays, else
                 {buffer_key: dict(Nspd=, Ndwl=, Ndbl=, Ndcm=, Ndsam_lev_1=,
                  Ndsam_lev_2=, mat_area_w=, mat_area_h=)} forwarded to CactiArraySpec

    Returns an OrderedDict keyed by trace name. Keys, in order:
      l1i:    <p>MissBuffer, <p>FillBuffer, <p>prefetchBuffer
      l1d:    those three, plus <p>WBB iff cache_policy == "write_back"
      shared: <p>MissB, <p>FillB, <p>PrefetchB, <p>WBB (always)
    """
    if level not in ("l1i", "l1d", "shared"):
        raise ValueError(level)
    if cache_policy not in ("write_back", "write_through"):
        raise ValueError(cache_policy)

    if buffer_policy not in ("configured", "mcpat"):
        raise ValueError("buffer_policy must be configured or mcpat")
    if any(value < 0 for value in entry_counts.values()):
        raise ValueError("buffer entry counts must be nonnegative")
    shared = level == "shared"
    buf_tag_w = phy_addr_width + _EXTRA_TAG_BITS
    # sharedcache.cc hardcodes num_rw_ports = num_search_ports = 1 for every
    # LLC buffer (it ignores any ports argument); L1 uses the fetch/memory
    # port count.
    ports = 1 if shared else fetch_or_memory_ports
    miss_ls = _miss_line_sz(capacity, line_bytes, phy_addr_width, shared)

    def _arr(key, line_sz, entries):
        if buffer_policy == "configured" and entries == 0:
            return None
        cache_sz = _clamp64(entries * line_sz)
        # sharedcache.cc drives its buffer ArrayST with out_w = line_sz*8/2;
        # core.cc uses the full line_sz*8.
        out_w = line_sz * 8 // 2 if shared else line_sz * 8
        geom = dict(
            capacity=cache_sz,
            block_sz=line_sz,
            assoc=0,
            nbanks=1,
            out_w=out_w,
            specific_tag=True,
            tag_w=buf_tag_w,
            wire_is_mat_type=wire_is_mat_type,
            wire_os_mat_type=wire_os_mat_type,
            wt_overhead=wt_overhead,
            num_rw_ports=ports,
            num_rd_ports=0,
            num_wr_ports=0,
            num_search_ports=ports,
            device_ty=device_ty,
        )
        if partitions is None:
            return CactiArraySpec(key.lower(), **geom)
        return CactiArraySpec(key.lower(), **geom, **partitions[key])

    out = collections.OrderedDict()
    if shared:
        # sharedcache.cc SharedCache: MissB / FillB / PrefetchB / WBB; the
        # LLC path has no write-through variant, so WBB is unconditional.
        out[f"{prefix}MissB"] = _arr(
            f"{prefix}MissB", miss_ls, entry_counts["mshr"]
        )
        out[f"{prefix}FillB"] = _arr(
            f"{prefix}FillB", line_bytes, entry_counts["fill"]
        )
        out[f"{prefix}PrefetchB"] = _arr(
            f"{prefix}PrefetchB", line_bytes, entry_counts["prefetch"]
        )
        out[f"{prefix}WBB"] = _arr(
            f"{prefix}WBB", line_bytes, entry_counts["wbb"]
        )
    else:
        # core.cc InstFetchU (l1i) / LoadStoreU (l1d): MissBuffer /
        # FillBuffer / prefetchBuffer; l1d adds WBB only for a write-back
        # cache, l1i (read-only) never builds one.
        out[f"{prefix}MissBuffer"] = _arr(
            f"{prefix}MissBuffer", miss_ls, entry_counts["mshr"]
        )
        out[f"{prefix}FillBuffer"] = _arr(
            f"{prefix}FillBuffer", line_bytes, entry_counts["fill"]
        )
        out[f"{prefix}prefetchBuffer"] = _arr(
            f"{prefix}prefetchBuffer", line_bytes, entry_counts["prefetch"]
        )
        if level == "l1d" and cache_policy == "write_back":
            out[f"{prefix}WBB"] = _arr(
                f"{prefix}WBB", line_bytes, entry_counts["wbb"]
            )
    return collections.OrderedDict(
        (k, v) for k, v in out.items() if v is not None
    )
