# SPDX-License-Identifier: BSD-3-Clause
"""CPU, predictor, buffer and cache inputs resolved from SimObjects."""

import math

from .gem5_inputs import cpu_clock_mhz
from .mcpat_solver.defaults import (
    _BASE_CORE_PARAMS_INORDER,
    _BASE_MACHINE_PARAMS,
    _base_core_params,
    _is_ooo,
)

# Names -> McPAT enums: technology.cc {0:HP, 1:LSTP, 2:LOP} and XML_Parse.cc
# {0:aggressive, 1:conservative}.
_DEVICE_MAP = {"hp": 0, "lstp": 1, "lop": 2}
_IC_MAP = {"aggressive": 0, "conservative": 1}

# mcpat/cacti/const.h EXTRA_TAG_BITS: extra bits in every cache tag width.
_EXTRA_TAG_BITS = 5

_PORT_KEYS = (
    "num_rw_ports",
    "num_rd_ports",
    "num_wr_ports",
    "num_search_ports",
)

# build_machine_model_from_config's `cache_configs` keys (McPAT XML names).
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


def _positive(name, value):
    if value <= 0:
        raise ValueError(f"gem5 param {name} must be positive, got {value!r}")
    return value


def _check_device_type_at_node(name, device_type, tech_node):
    """ITRS ships LSTP/LOP device data only up to 90 nm."""
    if tech_node > 90 and device_type in ("lstp", "lop"):
        raise ValueError(
            f"{name}={device_type!r} is invalid at tech_node={tech_node} "
            "(> 90 nm): CACTI/McPAT ship ITRS device data for LSTP/LOP "
            "only up to 90 nm. Use 'hp', or a node <= 90 nm."
        )


def derive_machine_params(
    board, tech_node=None, device_type=None, interconnect_type=None
):
    """Machine-wide params from `_BASE_MACHINE_PARAMS`, with `num_cores` read
    off the board. `tech_node` (nm), `device_type` ("hp"/"lstp"/"lop") and
    `interconnect_type` ("aggressive"/"conservative") override the defaults;
    `None` keeps them."""
    params = dict(_BASE_MACHINE_PARAMS)
    if tech_node is not None:
        params["tech_node"] = int(tech_node)
    _check_device_type_at_node("device_type", device_type, params["tech_node"])
    if params["tech_node"] > 90 and device_type is None:
        # ITRS has only HP device data above 90 nm (technology.cc:229-232).
        params["device_type"] = 0
    elif device_type is not None:
        params["device_type"] = _DEVICE_MAP[device_type]
    if interconnect_type is not None:
        params["interconnect_type"] = _IC_MAP[interconnect_type]
    params["num_cores"] = _positive(
        "num_cores", board.get_processor().get_num_cores()
    )
    # One shared L2 in this driver's cache hierarchy.
    params["num_l2s"] = 1
    params["private_l2"] = 0
    return params


# McPAT core key -> the gem5 opClasses its functional units are counted by.
_FP_OPCLASSES = {
    "FloatAdd",
    "FloatCmp",
    "FloatCvt",
    "FloatMult",
    "FloatMultAcc",
    "FloatMisc",
    "FloatDiv",
    "FloatSqrt",
}
_FU_OPCLASSES = {
    "alus_per_core": {"IntAlu"},
    "muls_per_core": {"IntMult", "IntDiv"},
    "fpu_per_core": _FP_OPCLASSES,
}

# McPAT core key -> BaseO3CPU / BaseMinorCPU param.
_O3_PARAMS = {
    "fetch_width": "fetchWidth",
    "decode_width": "decodeWidth",
    "issue_width": "issueWidth",
    "commit_width": "commitWidth",
    "rob_size": "numROBEntries",
    "phys_regs_irf_size": "numPhysIntRegs",
    "phys_regs_frf_size": "numPhysFloatRegs",
    "load_buffer_sz": "LQEntries",
    "store_buffer_sz": "SQEntries",
}
_MINOR_PARAMS = {
    "decode_width": "decodeInputWidth",
    "issue_width": "executeIssueLimit",
    "commit_width": "executeCommitLimit",
}


def _count_fu_o3(fu_pool, op_classes):
    """Sum FUDesc.count over the fuPool.FUList entries whose opList has any of
    `op_classes` (McPAT's parallel ALU/MUL/FPU_per_core)."""
    return sum(
        int(fu.count)
        for fu in fu_pool.FUList
        if any(str(op.opClass) in op_classes for op in fu.opList)
    )


def _count_fu_minor(fu_pool, op_classes):
    """Number of Minor funcUnits (one discrete unit each) with any of
    `op_classes`."""
    return sum(
        1
        for fu in fu_pool.funcUnits
        if {str(oc.opClass) for oc in fu.opClasses.opClasses} & op_classes
    )


def _iq_window_sizes(c):
    """(inst_window_size, fp_inst_window_size) of a BaseO3CPU. gem5 places
    each instruction in one IQ gated by FU capability (inst_queue.cc:198-206,
    696-740), so the FP window is the FP-capable IQs' entries and the int
    window the rest; without such a split both are the total."""
    entries = [int(iq.numEntries) for iq in c.instQueues]
    fp_capable = [
        _count_fu_o3(iq.fuPool, _FP_OPCLASSES) > 0 for iq in c.instQueues
    ]
    total = _positive("instQueues numEntries", sum(entries))
    int_w = sum(n for n, fp in zip(entries, fp_capable) if not fp)
    if int_w == 0 or int_w == total:
        return total, total
    return int_w, total - int_w


def derive_core_params(board, core, cpu_type):
    base = dict(_base_core_params(cpu_type))
    if cpu_type == "timing":
        # TimingSimpleCPU has no physical pipeline. Use the named ARM A9
        # proxy underlying its other inaccessible structural parameters.
        # Keep the historical 1,1 preset unchanged for source regression.
        base["pipeline_depth"] = _BASE_CORE_PARAMS_INORDER["pipeline_depth"]
    c = core.core

    # opcode_width=16 is Penryn.xml's x86 value.
    isa = core.get_isa()
    is_x86 = str(getattr(isa, "value", isa)).lower() == "x86"
    base["isX86"] = int(is_x86)
    if is_x86:
        base["opcode_width"] = 16

    # Clock.value is the period in seconds; it is readable before the global
    # tick frequency is fixed, unlike getValue().
    base["clock_rate"] = _positive("clock", cpu_clock_mhz(board, c))
    base["num_threads"] = _positive("numThreads", int(c.numThreads))
    if not hasattr(getattr(c, "branchPred", None), "conditionalBranchPred"):
        base["prediction_width"] = 0

    if _is_ooo(cpu_type):
        for key, attr in _O3_PARAMS.items():
            base[key] = _positive(attr, int(getattr(c, attr)))
        base["inst_window_size"], base["fp_inst_window_size"] = (
            _iq_window_sizes(c)
        )
        for key, ops in _FU_OPCLASSES.items():
            base[key] = sum(
                _count_fu_o3(iq.fuPool, ops) for iq in c.instQueues
            )
    elif cpu_type == "minor":
        # Minor has no ROB / unified register file / IQ: those stay anchor.
        for key, attr in _MINOR_PARAMS.items():
            base[key] = _positive(attr, int(getattr(c, attr)))
        for key, ops in _FU_OPCLASSES.items():
            base[key] = _count_fu_minor(c.executeFuncUnits, ops)
    # TimingSimpleCPU has no width or FU-pool params: anchor values.
    # fp_issue_width has no gem5 counterpart (core.cc:4355-4356).
    base["peak_issue_width"] = base["issue_width"]
    _positive("alus_per_core", base["alus_per_core"])
    return base


def derive_predictor_params(core):
    """TournamentBP + RAS params from `core.core.branchPred`."""
    bp = core.core.branchPred
    if not hasattr(bp, "conditionalBranchPred"):
        return dict(
            enabled=False,
            global_entries=0,
            global_ctr_bits=0,
            choice_entries=0,
            choice_ctr_bits=0,
            l1_entries=0,
            l2_entries=0,
            ras_entries=0,
            local_bits=(0, 0),
        )
    cond = bp.conditionalBranchPred
    params = {
        key: _positive(attr, int(getattr(cond, attr)))
        for key, attr in _PREDICTOR_PARAMS.items()
    }
    params["ras_entries"] = _positive("ras.numEntries", int(bp.ras.numEntries))
    # McPAT local_predictor_size: (L1 history bits, L2 counter bits);
    # tournament.cc: localHistoryBits = ceilLog2(localPredictorSize).
    params["local_bits"] = (
        math.ceil(math.log2(params["l2_entries"])),
        _positive("localCtrBits", int(cond.localCtrBits)),
    )
    params["enabled"] = True
    return params


# McPAT sizes L1Pred and L2Pred with one local_predictor_entries; gem5 sizes
# them separately (localHistoryTableSize / localPredictorSize).
_PREDICTOR_PARAMS = {
    "global_entries": "globalPredictorSize",
    "global_ctr_bits": "globalCtrBits",
    "choice_entries": "choicePredictorSize",
    "choice_ctr_bits": "choiceCtrBits",
    "l1_entries": "localHistoryTableSize",
    "l2_entries": "localPredictorSize",
}


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
