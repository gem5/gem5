# SPDX-License-Identifier: BSD-3-Clause
"""Resolved McPAT component composition; no gem5 dependencies."""

import math
from copy import deepcopy

from .array_search import _ArraySearch
from .array_spec import CactiArraySpec
from .cache import Cache
from .cache_buffers import build_cache_buffers
from .cacti_memory import CactiMemory
from .defaults import _is_ooo
from .machine_model import (
    CoreArrays,
    CoreLogic,
    MachineModel,
)
from .presets import (
    INST_BUFFER,
    anchor_tlbs,
)
from .technology import mcpat_wire_kwargs

_EXTRA_TAG_BITS = 5
_PORT_KEYS = (
    "num_rw_ports",
    "num_rd_ports",
    "num_wr_ports",
    "num_search_ports",
)

_CACHE_CONFIG_KEYS = (
    "icache_config",
    "icache_buffer_sizes",
    "dcache_config",
    "dcache_buffer_sizes",
    "L2_config",
    "L2_buffer_sizes",
    "BTB_config",
    "itlb_entries",
    "dtlb_entries",
)


def _derive_buffer_entry_counts(cache_simobj):
    """McPAT buffer_sizes[] from a live gem5 BaseCache: fill mirrors mshrs
    (gem5 has no analogue), prefetch is the QueuedPrefetcher queue depth or 0
    (array.cc:63 clamps a 0-entry buffer up to 64 B)."""
    mshrs = int(cache_simobj.mshrs)
    pf = getattr(cache_simobj, "prefetcher", None)
    # The gem5 "NULL" SimObject is falsy.
    pf_entries = (
        int(pf.queue_size) if (pf and hasattr(pf, "queue_size")) else 0
    )
    return {
        "mshr": mshrs,
        "fill": mshrs,
        "prefetch": pf_entries,
        "wbb": int(cache_simobj.write_buffers),
    }


def _buffer_entry_counts(cache_simobj, buffer_sizes):
    """McPAT `buffer_sizes` (miss, fill, prefetch, wbb) when given, else
    derived from `cache_simobj`."""
    if buffer_sizes is None:
        return _derive_buffer_entry_counts(cache_simobj)
    miss, fill, prefetch, wbb = buffer_sizes
    return {"mshr": miss, "fill": fill, "prefetch": prefetch, "wbb": wbb}


def _ports(cfg):
    """(rw, rd, wr, search) port counts of a CactiArraySpec `_cfg_kwargs`."""
    return tuple(cfg[key] for key in _PORT_KEYS)


def _thread_tag_bits(num_threads):
    return math.ceil(math.log2(num_threads)) + _EXTRA_TAG_BITS


def _cache_tag_width(phy_addr_width, capacity, line, assoc):
    """core.cc:69-81, 742-753 / sharedcache.cc cache tag width."""
    index_bits = math.ceil(math.log2(capacity / line / assoc))
    return (
        phy_addr_width
        - index_bits
        - math.ceil(math.log2(line))
        + _EXTRA_TAG_BITS
    )


def _search_geometry_buffers(
    geom_buffers, search, timing=None, timing_by_name=None
):
    """Search a partition for each geometry-only buffer from
    `build_cache_buffers(..., partitions=None)`; returns
    `{trace_name: CactiArraySpec}`. `timing` is the parent cache's Layer-3
    targets; `timing_by_name` overrides it per buffer."""
    timing_by_name = timing_by_name or {}
    out = {}
    for key, geom in geom_buffers.items():
        ck = geom._cfg_kwargs
        out[key] = search.array(
            key,
            ck["capacity"],
            ck["block_sz"],
            (ck["num_rw_ports"], 0, 0, ck["num_search_ports"]),
            ck["tag_w"],
            ck["out_w"],
            assoc=0,
            specific_tag=True,
            device_ty=geom._device_ty,
            wire_is_mat_type=ck["wire_is_mat_type"],
            wire_os_mat_type=ck["wire_os_mat_type"],
            **(timing_by_name[key] if key in timing_by_name else timing or {}),
        )
    return out


def _cache_shape(search, cache_simobj, line, config):
    """(capacity, line, assoc, nbanks, timing): from McPAT's cache `config`
    tuple (capacity, line, assoc, banks, throughput, latency, ...) when
    given, else from the live `cache_simobj` at `line` bytes."""
    if config is None:
        return (
            int(cache_simobj.size.value),
            line,
            int(cache_simobj.assoc),
            1,
            {},
        )
    return (*config[:4], search.cycles(config[4], config[5]))


def _live_l1_cache(
    bucket_name,
    prefix,
    cache_simobj,
    cache_line_size,
    num_rw_ports,
    search,
    config=None,
    buffer_sizes=None,
):
    """Icache/dcache (data, tag, buffers) Cache. `num_rw_ports` is McPAT's
    number_instruction_fetch_ports / memory_ports, shared by the buffers
    (core.cc:83-190, 751-861). A `config` (icache/dcache_config) replaces
    the SimObject's shape, banks, Layer-3 cycles and write policy ([7] == 1:
    write-back); `buffer_sizes` replaces its buffer entry counts."""
    phy_addr_width = search.config["phy_addr_width"]
    capacity, line, assoc, nbanks, timing = _cache_shape(
        search, cache_simobj, cache_line_size, config
    )
    # gem5's classic BaseCache has no write-through mode.
    write_back = config is None or config[7] == 1
    data_array, tag_array = search.cache_pair(
        prefix,
        capacity,
        line,
        assoc,
        line * 8,
        _cache_tag_width(phy_addr_width, capacity, line, assoc),
        (num_rw_ports, 0, 0, 0),
        nbanks,
        **timing,
    )
    geom_buffers = build_cache_buffers(
        level="l1i" if prefix == "icache" else "l1d",
        prefix=prefix,
        capacity=capacity,
        line_bytes=line,
        assoc=assoc,
        phy_addr_width=phy_addr_width,
        entry_counts=_buffer_entry_counts(cache_simobj, buffer_sizes),
        cache_policy="write_back" if write_back else "write_through",
        fetch_or_memory_ports=num_rw_ports,
        partitions=None,
        buffer_policy=search.buffer_policy,
        **mcpat_wire_kwargs(search.config),
    )
    buffers = _search_geometry_buffers(geom_buffers, search, timing)
    return Cache(bucket_name, data_array, tag_array=tag_array, buffers=buffers)


def _live_shared_cache(
    bucket_name,
    prefix,
    cache_simobj,
    search,
    cache_line_size,
    config=None,
    buffer_sizes=None,
):
    """Shared L2/L3 (data, tag, buffers) Cache (mcpat/sharedcache.cc):
    sequential access, out_w = line * 8 / 2 (sharedcache.cc:100-135), wire
    setting per `mcpat_wire_kwargs(shared=True)` (sharedcache.cc:75-86). With
    a `config` (L2_config) the SimObject may be None if `buffer_sizes` is
    given; the WBB's latency reads L2_config[4] (sharedcache.cc:395-396)."""
    phy_addr_width = search.config["phy_addr_width"]
    capacity, line, assoc, nbanks, timing = _cache_shape(
        search, cache_simobj, cache_line_size, config
    )
    wbb_timing = (
        {}
        if config is None
        else {f"{prefix}WBB": search.cycles(config[4], config[4])}
    )
    wire = mcpat_wire_kwargs(search.config, shared=True)
    # sharedcache.cc uses the same outside wire for cache and buffers.
    buffer_wire = dict(wire)
    data_array, tag_array = search.cache_pair(
        prefix,
        capacity,
        line,
        assoc,
        line * 8 // 2,
        _cache_tag_width(phy_addr_width, capacity, line, assoc),
        (1, 0, 0, 0),
        nbanks,
        device_ty="llc",
        data_assoc=1,
        wire_os_mat_type=wire["wire_os_mat_type"],
        is_seq_acc=True,
        **timing,
    )
    geom_buffers = build_cache_buffers(
        level="shared",
        prefix=prefix,
        capacity=capacity,
        line_bytes=line,
        assoc=assoc,
        phy_addr_width=phy_addr_width,
        entry_counts=_buffer_entry_counts(cache_simobj, buffer_sizes),
        cache_policy="write_back",
        device_ty="llc",
        partitions=None,
        buffer_policy=search.buffer_policy,
        **buffer_wire,
    )
    buffers = _search_geometry_buffers(
        geom_buffers, search, timing, wbb_timing
    )
    return Cache(bucket_name, data_array, tag_array=tag_array, buffers=buffers)


def _live_renaming_issue_rob_arrays(core_params, search):
    """The 10 O3 structures whose core.cc line_sz/tag_w/cache_sz/ports depend
    on live core widths: FRATs, free lists, issue queues, ROB, InstBuffer and
    the register files. Widths come from the core0 config the logic
    components use."""
    cfg = search.config
    core = cfg["core0"]
    virt_addr_width = cfg["virt_addr_width"]
    line_sz = math.ceil(cfg["data_width"] / 32.0) * 4
    phy_i, phy_f = core["phy_ireg_width"], core["phy_freg_width"]
    arch_i, arch_f = core["arch_ireg_width"], core["arch_freg_width"]
    decode_w, issue_w = core["decode_width"], core["issue_width"]
    fp_w = core["fp_issue_width"]  # core.cc:4355-4356
    peak_w = core["peak_issue_width"]  # core.cc:4351/4353
    commit_w = core["commit_width"]
    inst_len = core["inst_len"]
    threads = core["num_threads"]
    cam = core_params["rename_scheme"] == 1
    # core.cc:4464-4472: a RAM-based RAT caps at 4 global checkpoints, a
    # CAM-based one has at least 1.
    checkpoints = (
        max(core["chkpt_depth"], 1) if cam else min(core["chkpt_depth"], 4)
    )
    arrays = {}

    def add(
        key, name, block_sz, entries, ports, tag_w=0, cycles=1, out_w=None
    ):
        arrays[key] = search.array(
            name,
            block_sz * entries,
            block_sz,
            ports,
            tag_w,
            out_w,
            **search.cycles(cycles),
        )

    for key, name, arch_w, phy_w, arch_regs, phys_regs, width in (
        (
            "int_frat",
            "Int FrontRAT",
            arch_i,
            phy_i,
            core_params["archi_regs_irf_size"],
            core_params["phys_regs_irf_size"],
            decode_w,
        ),
        (
            "fp_frat",
            "FP FrontRAT",
            arch_f,
            phy_f,
            core_params["archi_regs_frf_size"],
            core_params["phys_regs_frf_size"],
            fp_w,
        ),
    ):
        if cam:  # core.cc:1419-1480: one entry per physical register
            add(
                key,
                name,
                math.ceil((arch_w + checkpoints) / 8.0),
                phys_regs,
                (1, width, width, 2 * width),
                tag_w=arch_w + math.ceil(math.log2(threads)),
                out_w=math.ceil(arch_w / 8.0) * 8,
            )
        else:  # core.cc:1366-1417
            add(
                key,
                name,
                math.ceil(phy_w * (1 + checkpoints) / 8.0),
                arch_regs * threads,
                (1, 2 * width, width, 0),
                out_w=math.ceil(phy_w / 8.0) * 8,
            )

    # Free lists: core.cc:1545-1593; entries == physical registers
    # (core.cc:4450).
    add(
        "int_freelist",
        "Int Free List",
        math.ceil(phy_i / 8.0),
        core_params["phys_regs_irf_size"],
        (1, decode_w, decode_w - 1 + commit_w, 0),
    )
    add(
        "fp_freelist",
        "FP Free List",
        math.ceil(phy_f / 8.0),
        core_params["phys_regs_frf_size"],
        (1, fp_w, fp_w - 1 + commit_w, 0),
    )
    # Issue queues, PhysicalRegFile scheduler: core.cc:530-613.
    add(
        "int_issue_queue",
        "InstIssueQueue",
        int(math.ceil((inst_len + 2 * (phy_i - arch_i)) / 2.0) / 8.0),
        core_params["inst_window_size"],
        (0, peak_w, peak_w, peak_w),
        tag_w=phy_i,
        cycles=2,
    )
    add(
        "fp_issue_queue",
        "FPIssueQueue",
        math.ceil((inst_len + 2 * (phy_f - arch_f)) / 8.0),
        core_params["fp_inst_window_size"],
        (0, fp_w, fp_w, fp_w),
        tag_w=2 * phy_f,
    )
    # ROB, PhysicalRegFile scheduler: core.cc:615-702; a CAM-based RAT
    # stores max(phy_i, phy_f) register bits, a RAM-based one the sum.
    rob_regs = max(phy_i, phy_f) if cam else phy_i + phy_f
    add(
        "rob",
        "ReorderBuffer",
        math.ceil(
            (math.ceil(5 + math.log2(threads)) + virt_addr_width + rob_regs)
            / 8.0
        ),
        core_params["rob_size"],
        (0, peak_w, peak_w, 0),
    )
    # InstBuffer: core.cc:195-219; its size and fetch ports have no gem5
    # source and stay at the anchor values.
    add(
        "inst_buffer",
        "InstBuffer",
        math.ceil(inst_len * peak_w / 8.0),
        threads * core["inst_buffer_size"],
        (core["num_ifetch_ports"], 0, 0, 0),
    )
    # Register files: core.cc:1034-1081. Only the ports depend on core
    # width, but they steer the search.
    add(
        "int_regfile",
        "IntRegFile",
        line_sz,
        core_params["phys_regs_irf_size"],
        (1, 2 * peak_w, peak_w, 0),
    )
    add(
        "fp_regfile",
        "FPRegFile",
        line_sz,
        core_params["phys_regs_frf_size"],
        (1, 2 * issue_w, issue_w, 0),
    )
    return arrays


def _live_tlb_arrays(
    core_params,
    search,
    itlb_entries=None,
    dtlb_entries=None,
    icache_config=None,
    dcache_config=None,
):
    """{"itlb", "dtlb"} CactiArraySpecs for the entry counts given
    (core.cc:952-1000, MemManU): fully associative, line_sz =
    ceil((phy_addr_width - log2(page)) / 8). ITLB ports follow
    number_instruction_fetch_ports and the icache_config cycles, DTLB ports
    memory_ports and the dcache_config cycles (no config: no Layer-3
    targets). io.cc's error_checking makes read = write = search = ports."""
    cfg = search.config
    page_bits = math.floor(math.log2(cfg["vm_pg_size"]))
    line_sz = math.ceil((cfg["phy_addr_width"] - page_bits) / 8.0)
    tag_w = (
        cfg["virt_addr_width"]
        - page_bits
        + _thread_tag_bits(core_params["num_threads"])
    )
    arrays = {}
    for key, name, entries, ports, config in (
        (
            "itlb",
            "ITLB",
            itlb_entries,
            core_params["num_ifetch_ports"],
            icache_config,
        ),
        (
            "dtlb",
            "DTLB",
            dtlb_entries,
            core_params["mem_ports"],
            dcache_config,
        ),
    ):
        if entries is not None:
            timing = {} if config is None else search.cycles(*config[4:6])
            arrays[key] = search.array(
                name,
                entries * line_sz,
                line_sz,
                (0, ports, ports, ports),
                tag_w,
                **timing,
            )
    return arrays


def _core_arrays_with_live_geometry(
    default_core_arrays, core_params, predictor_params, search
):
    """CoreArrays with each structure whose McPAT cache_sz is entries *
    bytes-per-entry rebuilt at live entry counts (the predictors core.cc:349-
    435, the Inorder register files core.cc:4385-4386) or, for the
    renaming/issue/ROB/LSQ families, from live widths."""
    ooo = search.core_ooo
    config = search.config
    l1_bits, l2_bits = predictor_params["local_bits"]
    # (CoreArrays kwarg, anchor key, entries, bytes per entry)
    rows = [
        (
            "global_pred",
            "GlobalPred",
            predictor_params["global_entries"],
            math.ceil(predictor_params["global_ctr_bits"] / 8.0),
        ),
        (
            "pred_chooser",
            "ChooserPred",
            predictor_params["choice_entries"],
            math.ceil(predictor_params["choice_ctr_bits"] / 8.0),
        ),
        (
            "l1_pred",
            "L1LocalPred",
            predictor_params["l1_entries"],
            math.ceil(l1_bits / 8.0),
        ),
        (
            "l2_pred",
            "L2LocalPred",
            predictor_params["l2_entries"],
            math.ceil(l2_bits / 8.0),
        ),
        (
            "ras",
            "RAS",
            predictor_params["ras_entries"],
            math.ceil(config["virt_addr_width"] / 8.0),
        ),
    ]
    port_overrides = None
    if not ooo:
        # O3 register files come from the renaming family below. The
        # Inorder FpRegFile ports track issue_width (core.cc:1078-1079).
        issue_w = core_params["issue_width"]
        port_overrides = {"num_rd_ports": 2 * issue_w, "num_wr_ports": issue_w}
        rows += [
            (
                "int_regfile",
                "IntRegFile",
                core_params["archi_regs_irf_size"],
                None,
            ),
            (
                "fp_regfile",
                "FpRegFile",
                core_params["archi_regs_frf_size"],
                None,
            ),
        ]
    overrides = {}
    for kwarg, anchor_key, entries, block_sz in rows:
        if (
            core_params["prediction_width"] == 0
            and anchor_key in CoreArrays.PREDICTOR_KEYS
        ):
            continue
        overrides[kwarg] = search.like(
            default_core_arrays._arrays[anchor_key],
            entries,
            block_sz,
            (
                {
                    "num_rd_ports": 2 * core_params["peak_issue_width"],
                    "num_wr_ports": core_params["peak_issue_width"],
                }
                if anchor_key == "IntRegFile"
                else port_overrides if anchor_key == "FpRegFile" else None
            ),
            **search.cycles(1),
        )

    if ooo:
        overrides.update(_live_renaming_issue_rob_arrays(core_params, search))
    else:
        # core.cc:194-218: one fetch-width bundle per buffer entry.
        overrides["inst_buffer"] = search.like(
            INST_BUFFER,
            core_params["num_threads"] * core_params["inst_buffer_size"],
            math.ceil(
                core_params["inst_len"] * core_params["peak_issue_width"] / 8
            ),
            {"num_rw_ports": core_params["num_ifetch_ports"]},
            name="InstBuffer",
            **search.cycles(1),
        )

    # LoadStoreQueue for both core types, LoadQueue for OOO only
    # (core.cc:714-937): tag = opcode + virtual address + thread bits.
    line_sz = math.ceil(config["machine_bits"] / 32.0) * 4
    tag_w = (
        core_params["opcode_width"]
        + config["virt_addr_width"]
        + _thread_tag_bits(core_params["num_threads"])
    )
    mem_ports = core_params["mem_ports"]
    queues = [
        ("load_store_queue", "LoadStoreQueue", core_params["store_buffer_sz"])
    ]
    if ooo and core_params["load_buffer_sz"] > 0:
        queues.append(
            ("load_queue", "LoadQueue", core_params["load_buffer_sz"])
        )
    for key, name, entries in queues:
        overrides[key] = search.array(
            name,
            entries * line_sz,
            line_sz,
            (0, mem_ports, mem_ports, mem_ports),
            tag_w,
            **search.cycles(1),
        )
    arrays = CoreArrays(**overrides)
    if core_params["prediction_width"] == 0:
        arrays._arrays = {
            name: array
            for name, array in arrays._arrays.items()
            if name not in CoreArrays.PREDICTOR_KEYS
        }
    return arrays


def build_machine_model_from_params(
    machine_params,
    core_params,
    predictor_params,
    cpu_type,
    l1icache=None,
    l1dcache=None,
    l2cache=None,
    cache_line_size=None,
    l2_device_type=0,
    cache_configs=None,
    opt_for_clk=False,
    opt_local=False,
    tech_node=None,
    buffer_policy="mcpat",
    tag_widths=None,
):
    """MachineModel from resolved machine/core/predictor dicts (the values
    `build_machine_model_from_config` derives from a board and overrides), so
    it needs no gem5 board. `cache_configs`, `opt_for_clk` and `opt_local` are
    as there; `tech_node` is only a consistency check against the dicts. The
    model's `timing_report` lists every searched array's Layer-3 outcome."""
    if tech_node is not None and machine_params["tech_node"] != tech_node:
        raise ValueError(
            f"tech_node {tech_node} != machine tech_node "
            f"{machine_params['tech_node']}"
        )
    timing_log = []
    cfg = dict.fromkeys(_CACHE_CONFIG_KEYS)
    if cache_configs is not None:
        unknown = set(cache_configs) - set(_CACHE_CONFIG_KEYS)
        if unknown:
            raise ValueError(f"unknown cache_configs keys: {sorted(unknown)}")
        cfg.update(cache_configs)

    # CoreLogic defaults to 1/1/1 units; has_fpu/has_mul follow core.cc's
    # num_fpus>0/num_muls>0.
    core_logic = CoreLogic(
        num_alus=core_params["alus_per_core"],
        num_fpus=core_params["fpu_per_core"],
        num_muls=core_params["muls_per_core"],
        has_alu=True,
        has_fpu=core_params["fpu_per_core"] > 0,
        has_mul=core_params["muls_per_core"] > 0,
    )
    default_model = MachineModel(
        machine_params,
        core_params,
        l2_device_type=l2_device_type,
        core_logic=core_logic,
        **anchor_tlbs(core_params),
    )
    ooo = _is_ooo(cpu_type)
    search = _ArraySearch(
        default_model._cacti_params,
        ooo,
        opt_for_clk,
        opt_local,
        timing_log,
        buffer_policy=buffer_policy,
        tag_widths=tag_widths,
    )
    core_arrays = _core_arrays_with_live_geometry(
        default_model._core_arrays, core_params, predictor_params, search
    )

    icache_cache = default_model._icache_cache
    if l1icache is not None or cfg["icache_config"] is not None:
        icache_cache = _live_l1_cache(
            "Instruction Cache",
            "icache",
            l1icache,
            cache_line_size,
            core_params["num_ifetch_ports"],
            search,
            cfg["icache_config"],
            cfg["icache_buffer_sizes"],
        )
    dcache_cache = default_model._dcache_cache
    if l1dcache is not None or cfg["dcache_config"] is not None:
        dcache_cache = _live_l1_cache(
            "Data Cache",
            "dcache",
            l1dcache,
            cache_line_size,
            core_params["mem_ports"],
            search,
            cfg["dcache_config"],
            cfg["dcache_buffer_sizes"],
        )

    l2_cache = default_model._l2_cache
    if l2cache is not None or cfg["L2_config"] is not None:
        l2_cache = _live_shared_cache(
            "L2",
            "L2",
            l2cache,
            _ArraySearch(
                default_model._cacti_params_l2,
                ooo,
                opt_for_clk,
                True,
                timing_log,
                buffer_policy=buffer_policy,
                tag_widths=tag_widths,
            ),
            cache_line_size,
            cfg["L2_config"],
            cfg["L2_buffer_sizes"],
        )
    else:
        # Anchor L2 shape searched at the live node; buffers stay the anchor's.
        l2_search = _ArraySearch(
            default_model._cacti_params_l2, ooo, timing_log=timing_log
        )
        data_array, tag_array = l2_search.anchor_cache_pair(
            "L2",
            l2_cache,
            mcpat_wire_kwargs(machine_params, shared=True)["wire_os_mat_type"],
        )
        l2_cache = Cache(
            "L2", data_array, tag_array=tag_array, buffers=l2_cache.buffers
        )

    if core_params["prediction_width"] == 0:
        btb_pair = (
            default_model._btb_cache.data_array,
            default_model._btb_cache.tag_array,
        )
    elif cfg["BTB_config"] is None:
        btb_pair = _ArraySearch(
            default_model._cacti_params, ooo, timing_log=timing_log
        ).anchor_cache_pair("Branch Target Buffer", default_model._btb_cache)
    else:
        # core.cc:236-270: one rw port plus prediction_width read and write.
        capacity, line, assoc, nbanks, throughput, latency = cfg["BTB_config"][
            :6
        ]
        prediction_w = core_params["prediction_width"]
        btb_pair = search.cache_pair(
            "Branch Target Buffer",
            capacity,
            line,
            assoc,
            line * 8,
            search.config["virt_addr_width"]
            + _thread_tag_bits(core_params["num_threads"]),
            (1, prediction_w, prediction_w, 0),
            nbanks,
            **search.cycles(throughput, latency),
        )
    btb_cache = Cache(
        "Branch Target Buffer", btb_pair[0], tag_array=btb_pair[1]
    )

    tlbs = anchor_tlbs(core_params)
    tlbs.update(
        _live_tlb_arrays(
            core_params,
            search,
            cfg["itlb_entries"],
            cfg["dtlb_entries"],
            cfg["icache_config"],
            cfg["dcache_config"],
        )
    )

    model = MachineModel(
        machine_params,
        core_params,
        l2_device_type=l2_device_type,
        core_logic=core_logic,
        core_arrays=core_arrays,
        icache_cache=icache_cache,
        dcache_cache=dcache_cache,
        l2_cache=l2_cache,
        btb_cache=btb_cache,
        **tlbs,
    )
    live_cache_inputs = {}
    for role, obj, buffers, cache in (
        ("icache", l1icache, cfg["icache_buffer_sizes"], icache_cache),
        ("dcache", l1dcache, cfg["dcache_buffer_sizes"], dcache_cache),
        ("L2", l2cache, cfg["L2_buffer_sizes"], l2_cache),
    ):
        live_cache_inputs[role] = dict(
            geometry=cache.data_array._cfg_kwargs,
            buffer_counts=(
                _buffer_entry_counts(obj, buffers)
                if obj is not None or buffers is not None
                else None
            ),
            buffer_count_source=(
                "configured"
                if buffers is not None
                else (
                    "SimObject; fill mirrors MSHRs"
                    if obj is not None
                    else "named preset"
                )
            ),
        )
    model.resolved_config = deepcopy(
        dict(
            predictor=predictor_params,
            caches=live_cache_inputs,
            cache_configs=cfg,
            buffer_policy=buffer_policy,
            tag_widths=tag_widths,
            l2_device_type=l2_device_type,
            opt_for_clk=opt_for_clk,
            opt_local=opt_local,
            core_proxies=(
                {
                    "pipeline_depth": {
                        "value": core_params["pipeline_depth"],
                        "source": "ARM A9 physical pipeline proxy; TimingSimpleCPU has no physical stages",
                    }
                }
                if cpu_type == "timing"
                else {}
            ),
        )
    )
    model.timing_report = timing_log
    for row in timing_log:
        if row["checked"] and not row["satisfied"]:
            missed = [
                k for k in ("throughput", "latency") if not row[k + "_ok"]
            ]
            print(
                f"timing: {row['name']} cannot satisfy {' and '.join(missed)}"
                f" (cycle {row['cycle_time']:.3e} s, access "
                f"{row['access_time']:.3e} s; targets "
                f"{row['throughput']:.3e}/{row['latency']:.3e} s)"
                + ("" if row["assoc"] else " [CAM: McPAT does not warn]")
            )
    return model
