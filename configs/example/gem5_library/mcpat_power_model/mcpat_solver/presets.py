# SPDX-License-Identifier: BSD-3-Clause
"""Named legacy organizations; factories isolate each composition."""

import math

from .array_spec import CactiArraySpec
from .cache import Cache
from .cache_buffers import build_cache_buffers

ICACHE = CactiArraySpec(
    "icache",
    capacity=32768,
    block_sz=8,
    assoc=8,
    nbanks=1,
    out_w=64,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=2,
    Ndsam_lev_1=8,
    Ndsam_lev_2=1,
)
ICACHE_TAG = CactiArraySpec(
    "icache_tag",
    capacity=32768,
    block_sz=8,
    assoc=8,
    nbanks=1,
    out_w=64,
    is_tag=True,
    specific_tag=True,
    tag_w=25,
    wt_overhead=30,
    Nspd=2,
    Ndwl=4,
    Ndbl=4,
    Ndcm=2,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=119.5593526,
    mat_area_h=89.2540367,
)
DCACHE = CactiArraySpec(
    "dcache",
    capacity=32768,
    block_sz=8,
    assoc=4,
    nbanks=1,
    out_w=64,
    wire_is_mat_type=0,
    wt_overhead=30,
    Nspd=1,
    Ndwl=4,
    Ndbl=2,
    Ndcm=1,
    Ndsam_lev_1=4,
    Ndsam_lev_2=1,
    mat_area_w=69.03602079,
    mat_area_h=615.6022962,
)
DCACHE_TAG = CactiArraySpec(
    "dcache_tag",
    capacity=32768,
    block_sz=8,
    assoc=4,
    nbanks=1,
    out_w=64,
    is_tag=True,
    specific_tag=True,
    tag_w=24,
    wt_overhead=30,
    Nspd=4,
    Ndwl=16,
    Ndbl=2,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=25.45463971,
    mat_area_h=160.5134443,
)
# DCACHE/DCACHE_TAG are 4-way; gem5-pm's L1D is 8-way (unvalidated at assoc=8).
# is_seq_acc: McPAT builds the shared cache with access_mode=1
# (sharedcache.cc:127); it selects find_delay's tag+data branch in a search.
L2 = CactiArraySpec(
    "l2cache",
    capacity=1048576,
    block_sz=32,
    assoc=8,
    nbanks=8,
    out_w=128,
    wire_is_mat_type=0,
    wire_os_mat_type=1,
    wt_overhead=30,
    data_assoc=1,
    is_seq_acc=True,
    Nspd=2,
    Ndwl=2,
    Ndbl=8,
    Ndcm=8,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=260.29333680520614,
    mat_area_h=325.21335047811169,
    device_ty="llc",
)
L2_TAG = CactiArraySpec(
    "l2cache_tag",
    capacity=1048576,
    block_sz=32,
    assoc=8,
    nbanks=8,
    out_w=128,
    is_tag=True,
    specific_tag=True,
    tag_w=20,
    data_assoc=1,
    is_seq_acc=True,
    wire_is_mat_type=0,
    wire_os_mat_type=1,
    wt_overhead=30,
    Nspd=2,
    Ndwl=4,
    Ndbl=2,
    Ndcm=2,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=87.3264042837425,
    mat_area_h=162.6056880976396,
    device_ty="llc",
)
# Matches BTB_config="4096,4,2,2,1,1" in ARM_A9_2GHz_withIOC.xml; the BTB
# activation energy is data + BTB_TAG summed.
BTB = CactiArraySpec(
    "btb",
    capacity=4096,
    block_sz=4,
    assoc=2,
    nbanks=2,
    out_w=32,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=1,
    num_wr_ports=1,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=2,
    Ndsam_lev_1=2,
    Ndsam_lev_2=1,
    mat_area_w=67.76609775,
    mat_area_h=275.7502962,
)
# BTB tag array; partition from the same sweep trace entry as BTB.
BTB_TAG = CactiArraySpec(
    "btb_tag",
    capacity=4096,
    block_sz=4,
    assoc=2,
    nbanks=2,
    out_w=32,
    is_tag=True,
    specific_tag=True,
    tag_w=37,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=1,
    num_wr_ports=1,
    Nspd=2,
    Ndwl=2,
    Ndbl=2,
    Ndcm=4,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=171.72160156333615,
    mat_area_h=146.72294626675924,
)
# One-thread ITLB/DTLB shape; callers widen tag_w per thread (anchor_tlbs).
TLB = CactiArraySpec(
    "tlb",
    capacity=192,
    block_sz=3,
    assoc=0,
    nbanks=1,
    out_w=24,
    specific_tag=True,
    tag_w=25,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    num_search_ports=1,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
)


# OOO-only structures (predictors, register files, rename, issue queues, LSQ,
# IFB, ROB) at the same 40nm/LOP/core corner; none has a tag array.
GLOBAL_PRED = CactiArraySpec(
    "globalpred",
    capacity=4096,
    block_sz=1,
    assoc=1,
    nbanks=1,
    out_w=8,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    Nspd=32,
    Ndwl=2,
    Ndbl=2,
    Ndcm=64,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=201.8402592195769,
    mat_area_h=117.96158846667088,
    core_ooo=True,
)
L1_PRED = CactiArraySpec(
    "l1pred",
    capacity=64,
    block_sz=2,
    assoc=1,
    nbanks=1,
    out_w=16,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    Nspd=1,
    Ndwl=2,
    Ndbl=1,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=15.842308715094722,
    mat_area_h=32.97957405000947,
    core_ooo=True,
)
L2_PRED = CactiArraySpec(
    "l2pred",
    capacity=64,
    block_sz=1,
    assoc=1,
    nbanks=1,
    out_w=8,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=2,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=9.865401806622765,
    mat_area_h=60.385588466670875,
    core_ooo=True,
)
PRED_CHOOSER = CactiArraySpec(
    "predchooser",
    capacity=4096,
    block_sz=1,
    assoc=1,
    nbanks=1,
    out_w=8,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    Nspd=32,
    Ndwl=2,
    Ndbl=2,
    Ndcm=64,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=201.8402592195769,
    mat_area_h=117.96158846667088,
    core_ooo=True,
)
RAS = CactiArraySpec(
    "ras",
    capacity=64,
    block_sz=4,
    assoc=1,
    nbanks=1,
    out_w=32,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    Nspd=1,
    Ndwl=2,
    Ndbl=1,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=25.934931180361126,
    mat_area_h=24.127148100018935,
    core_ooo=True,
)

INT_REGFILE = CactiArraySpec(
    "intregfile",
    capacity=256,
    block_sz=4,
    assoc=1,
    nbanks=1,
    out_w=32,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=14,
    num_wr_ports=7,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=2,
    mat_area_w=202.91126351874584,
    mat_area_h=420.723221500284,
    core_ooo=True,
)
FP_REGFILE = CactiArraySpec(
    "fpregfile",
    capacity=256,
    block_sz=4,
    assoc=1,
    nbanks=1,
    out_w=32,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=8,
    num_wr_ports=4,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=1,
    Ndsam_lev_1=2,
    Ndsam_lev_2=1,
    mat_area_w=129.42351643711336,
    mat_area_h=260.46433290017046,
    core_ooo=True,
)

# FRAT is FA/CAM (rename_scheme=1); the free list is plain RAM (core.cc never
# branches its construction on rm_ty).
INT_FRAT = CactiArraySpec(
    "intfrat",
    capacity=64,
    block_sz=1,
    assoc=0,
    nbanks=1,
    out_w=8,
    specific_tag=True,
    tag_w=5,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=2,
    num_wr_ports=2,
    num_search_ports=4,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=104.98505043428723,
    mat_area_h=116.43607340026512,
    core_ooo=True,
)
FP_FRAT = CactiArraySpec(
    "fpfrat",
    capacity=64,
    block_sz=1,
    assoc=0,
    nbanks=1,
    out_w=8,
    specific_tag=True,
    tag_w=5,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=1,
    num_wr_ports=1,
    num_search_ports=2,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=68.09989963443716,
    mat_area_h=79.13718480015149,
    core_ooo=True,
)
INT_FREELIST = CactiArraySpec(
    "intfreelist",
    capacity=64,
    block_sz=1,
    assoc=1,
    nbanks=1,
    out_w=8,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=2,
    num_wr_ports=5,
    Nspd=2,
    Ndwl=2,
    Ndbl=2,
    Ndcm=4,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=42.77379164535922,
    mat_area_h=83.95676540001261,
    core_ooo=True,
)
FP_FREELIST = CactiArraySpec(
    "fpfreelist",
    capacity=64,
    block_sz=1,
    assoc=1,
    nbanks=1,
    out_w=8,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=1,
    num_rd_ports=1,
    num_wr_ports=4,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=2,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=24.06331141287517,
    mat_area_h=117.91517693334175,
    core_ooo=True,
)

INT_ISSUE_QUEUE = CactiArraySpec(
    "intissuequeue",
    capacity=64,
    block_sz=2,
    assoc=0,
    nbanks=1,
    out_w=16,
    specific_tag=True,
    tag_w=6,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=7,
    num_wr_ports=7,
    num_search_ports=7,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=272.0114575873688,
    mat_area_h=173.17622020079534,
    core_ooo=True,
)
FP_ISSUE_QUEUE = CactiArraySpec(
    "fpissuequeue",
    capacity=75,
    block_sz=5,
    assoc=0,
    nbanks=1,
    out_w=40,
    specific_tag=True,
    tag_w=12,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    num_search_ports=1,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=103.02735654374119,
    mat_area_h=39.56971953358161,
    core_ooo=True,
)

LOAD_STORE_QUEUE = CactiArraySpec(
    "loadstorequeue",
    capacity=64,
    block_sz=4,
    assoc=0,
    nbanks=1,
    out_w=32,
    specific_tag=True,
    tag_w=44,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    num_search_ports=1,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=136.55712017096678,
    mat_area_h=52.80460813369523,
    core_ooo=True,
)
LOAD_QUEUE = CactiArraySpec(
    "loadqueue",
    capacity=64,
    block_sz=4,
    assoc=0,
    nbanks=1,
    out_w=32,
    specific_tag=True,
    tag_w=44,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=1,
    num_wr_ports=1,
    num_search_ports=1,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=136.55712017096678,
    mat_area_h=52.80460813369523,
    core_ooo=True,
)

INST_BUFFER = CactiArraySpec(
    "instbuffer",
    capacity=896,
    block_sz=28,
    assoc=1,
    nbanks=1,
    out_w=224,
    wire_is_mat_type=0,
    wt_overhead=30,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=2,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=118.98008159333435,
    mat_area_h=56.190036700132566,
    core_ooo=True,
)

INST_FETCH_QUEUE = CactiArraySpec(
    "instfetchqueue",
    capacity=80,
    block_sz=4,
    assoc=0,
    nbanks=1,
    out_w=32,
    specific_tag=True,
    tag_w=8,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=7,
    num_wr_ports=7,
    num_search_ports=7,
    Nspd=1,
    Ndwl=1,
    Ndbl=4,
    Ndcm=1,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=416.75554147913624,
    mat_area_h=189.79636700132556,
)

# The A9 XML has no ROB but gem5's O3CPU does, so this anchor grafts
# ROB_size=40 (mcpat/sweep_traces/ARM_A9_2GHz_gem5_ooo_rob40.jsonl).
ROB = CactiArraySpec(
    "reorderbuffer",
    capacity=240,
    block_sz=6,
    assoc=1,
    nbanks=1,
    out_w=48,
    wire_is_mat_type=0,
    wt_overhead=30,
    num_rw_ports=0,
    num_rd_ports=7,
    num_wr_ports=7,
    Nspd=1,
    Ndwl=2,
    Ndbl=2,
    Ndcm=2,
    Ndsam_lev_1=1,
    Ndsam_lev_2=1,
    mat_area_w=196.89863196991575,
    mat_area_h=224.45627523352482,
    core_ooo=True,
)

# Cache buffers, built by cache_buffers.build_cache_buffers with the frozen
# anchor partitions from ARM_A9_2GHz_gem5_ooo.jsonl; the read-only icache has
# no WBB.

# Shared buffer partition; only the mat area (miss vs line, L1 vs L2) varies.
_BUF_PART = dict(Nspd=1, Ndwl=1, Ndbl=4, Ndcm=1, Ndsam_lev_1=1, Ndsam_lev_2=1)
_BUF_MA_L1_MISS = dict(
    mat_area_w=172.06280530874218, mat_area_h=58.95162780071959
)
_BUF_MA_L1_LINE = dict(
    mat_area_w=127.46641735972442, mat_area_h=46.28985060049234
)
_BUF_MA_L2_MISS = dict(
    mat_area_w=142.40150690321278, mat_area_h=56.490128585837645
)
_BUF_MA_L2_LINE = dict(
    mat_area_w=307.9321476878023, mat_area_h=112.4289838450662
)

# Entry counts from .ARM_A9_2GHz_gem5.xml buffer_sizes; line_bytes is the
# block width, not the XML output_width.
_ICACHE_BUFFERS = build_cache_buffers(
    level="l1i",
    prefix="icache",
    capacity=32768,
    line_bytes=8,
    assoc=8,
    phy_addr_width=32,
    entry_counts={"mshr": 4, "fill": 4, "prefetch": 4, "wbb": 0},
    cache_policy="write_back",
    fetch_or_memory_ports=1,
    partitions={
        "icacheMissBuffer": {**_BUF_PART, **_BUF_MA_L1_MISS},
        "icacheFillBuffer": {**_BUF_PART, **_BUF_MA_L1_LINE},
        "icacheprefetchBuffer": {**_BUF_PART, **_BUF_MA_L1_LINE},
    },
)
_DCACHE_BUFFERS = build_cache_buffers(
    level="l1d",
    prefix="dcache",
    capacity=32768,
    line_bytes=8,
    assoc=4,
    phy_addr_width=32,
    entry_counts={"mshr": 4, "fill": 4, "prefetch": 4, "wbb": 4},
    cache_policy="write_back",
    fetch_or_memory_ports=1,
    partitions={
        "dcacheMissBuffer": {**_BUF_PART, **_BUF_MA_L1_MISS},
        "dcacheFillBuffer": {**_BUF_PART, **_BUF_MA_L1_LINE},
        "dcacheprefetchBuffer": {**_BUF_PART, **_BUF_MA_L1_LINE},
        "dcacheWBB": {**_BUF_PART, **_BUF_MA_L1_LINE},
    },
)
_L2_BUFFERS = build_cache_buffers(
    level="shared",
    prefix="L2",
    capacity=1048576,
    line_bytes=32,
    assoc=8,
    phy_addr_width=32,
    entry_counts={"mshr": 16, "fill": 16, "prefetch": 16, "wbb": 16},
    cache_policy="write_back",
    wire_os_mat_type=1,
    device_ty="llc",
    partitions={
        "L2MissB": {**_BUF_PART, **_BUF_MA_L2_MISS},
        "L2FillB": {**_BUF_PART, **_BUF_MA_L2_LINE},
        "L2PrefetchB": {**_BUF_PART, **_BUF_MA_L2_LINE},
        "L2WBB": {**_BUF_PART, **_BUF_MA_L2_LINE},
    },
)

# Named because verify_new_arrays.py imports them.
ICACHE_MISS_BUFFER = _ICACHE_BUFFERS["icacheMissBuffer"]
ICACHE_FILL_BUFFER = _ICACHE_BUFFERS["icacheFillBuffer"]
ICACHE_PREFETCH_BUFFER = _ICACHE_BUFFERS["icacheprefetchBuffer"]
DCACHE_MISS_BUFFER = _DCACHE_BUFFERS["dcacheMissBuffer"]
DCACHE_FILL_BUFFER = _DCACHE_BUFFERS["dcacheFillBuffer"]
DCACHE_PREFETCH_BUFFER = _DCACHE_BUFFERS["dcacheprefetchBuffer"]
DCACHE_WBB = _DCACHE_BUFFERS["dcacheWBB"]
L2_MISS_B = _L2_BUFFERS["L2MissB"]
L2_FILL_B = _L2_BUFFERS["L2FillB"]
L2_PREFETCH_B = _L2_BUFFERS["L2PrefetchB"]
L2_WBB = _L2_BUFFERS["L2WBB"]

ICACHE_CACHE = Cache(
    "Instruction Cache",
    ICACHE,
    ICACHE_TAG,
    buffers=dict(_ICACHE_BUFFERS),
)
DCACHE_CACHE = Cache(
    "Data Cache",
    DCACHE,
    DCACHE_TAG,
    buffers=dict(_DCACHE_BUFFERS),
)
L2_CACHE = Cache(
    "L2",
    L2,
    L2_TAG,
    buffers=dict(_L2_BUFFERS),
)


def array_preset(name):
    array = globals()[name]
    if not isinstance(array, CactiArraySpec):
        raise ValueError(f"unknown array preset {name}")
    return array.replace()


def cache_preset(name):
    cache = globals()[name]
    if not isinstance(cache, Cache):
        raise ValueError(f"unknown cache preset {name}")
    return Cache(
        cache.bucket_name,
        cache.data_array.replace(),
        cache.tag_array.replace() if cache.tag_array else None,
        {k: v.replace() for k, v in cache.buffers.items()},
    )


def anchor_tlbs(core_params):
    """MachineModel itlb/dtlb kwargs from the anchor TLB, with tag_w widened
    by ceil(log2(num_threads)) (core.cc:959/985)."""
    extra = math.ceil(math.log2(max(core_params["num_threads"], 1)))
    tlb = TLB.replace(tag_w=TLB._cfg_kwargs["tag_w"] + extra)
    return {"itlb": tlb, "dtlb": tlb}
