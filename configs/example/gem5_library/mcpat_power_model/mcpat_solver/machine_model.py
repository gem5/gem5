"""Builder layer over the CACTI/McPAT port: config -> one-time activation
energies and an online per-bucket leakage lookup."""

from .array_spec import CactiArraySpec
from .cache import Cache
from .cacti_component import (
    CactiArrayST,
    CactiMat,
    CactiUCA,
)
from .cacti_dynamic_params import (
    CactiArrayConfig,
    CactiDynamicParameter,
)
from .cacti_params import CactiParams
from .config_input_params import McPATMachineConfig
from .mcpat_coefficients import (
    _array_unit,
    _logic_unit,
    _unit,
)
from .mcpat_logic_components import (
    DepConflictChecker,
    FuncUnit,
    InstDecoder,
    Interconnect,
    Pipeline,
    SelectionLogic,
    UndiffCore,
)
from .presets import *
from .presets import cache_preset
from .technology import (
    build_cacti_params,
    clamp_to_cacti_decade,
    fresh_machine_config,
    mcpat_wire_kwargs,
)

LEAKAGE_BUCKETS = ("Core", "Instruction Cache", "Data Cache", "L2")


def _interpolate_decade(fn, bucket, temp_k):
    """Linearly interpolate `fn(bucket, decade)` between the two 10K decades
    bracketing `temp_k` (clamped to [300, 400]K); exact at a decade."""
    t = max(300.0, min(400.0, float(temp_k)))
    lo = int(t // 10) * 10
    hi = lo if t == lo else lo + 10
    if lo == hi:
        return fn(bucket, float(lo))
    lo_v = fn(bucket, float(lo))
    hi_v = fn(bucket, float(hi))
    frac = (t - lo) / (hi - lo)
    return lo_v + frac * (hi_v - lo_v)


def _interpolated_leakage(machine_model, bucket, temp_k, units=None):
    """(subthreshold, gate) leakage in W of `bucket` at `temp_k`, each
    linearly interpolated between the solver's 10K decades. `units` limits
    the sum to those unit names (absent names contribute 0)."""
    if units is None:
        sub_at, gate_at = (
            machine_model.leakage_at,
            machine_model.gate_leakage_at,
        )
    else:

        def unit_sum(field):
            def at_decade(b, decade):
                by_unit = machine_model.leakage_by_unit_at(b, decade)
                return sum(by_unit[n][field] for n in by_unit if n in units)

            return at_decade

        sub_at, gate_at = unit_sum("leakage"), unit_sum("gate_leakage")
    return (
        _interpolate_decade(sub_at, bucket, temp_k),
        _interpolate_decade(gate_at, bucket, temp_k),
    )


def interpolated_static_power(machine_model, bucket, temp_k, units=None):
    """Subthreshold + gate leakage (W) at `temp_k`, both interpolated between
    the solver's 10K decades; `units` restricts it to those unit names."""
    sub, gate = _interpolated_leakage(machine_model, bucket, temp_k, units)
    return sub + gate


class CoreLogic:
    """Pipeline + FuncUnit + SelectionLogic + UndiffCore, plus Int/FpDCL
    (DepConflictChecker) only for an OOO core with phy_ireg_width available,
    as real McPAT never builds DCL for an in-order core."""

    def __init__(
        self,
        num_alus=1,
        num_fpus=1,
        num_muls=1,
        has_alu=True,
        has_fpu=True,
        has_mul=True,
    ):
        self._kwargs = dict(
            num_alus=num_alus,
            num_fpus=num_fpus,
            num_muls=num_muls,
            has_alu=has_alu,
            has_fpu=has_fpu,
            has_mul=has_mul,
        )

    def build(self, cacti_params):
        core0 = cacti_params._machine_config._config_params["core0"]
        pipeline = Pipeline("Pipeline", cacti_params)
        pipeline.calc_dyn_energies()
        func_unit = FuncUnit(cacti_params, **self._kwargs)
        func_unit.calc_dyn_energies()
        func_unit.calc_leakage()
        func_unit.calc_fu_height()
        sel_issue_width = core0["peak_issue_width"]
        if core0["machine_type"] == 1 and core0["num_threads"] > 1:
            sel_issue_width *= core0["num_threads"]
        sel_logic = SelectionLogic(
            "InstSel", cacti_params, issue_width=sel_issue_width
        )
        sel_logic.calc_dyn_energies()
        undiff_core = UndiffCore("UndiffCore", cacti_params)
        undiff_core.calc_leakage()

        components = {
            "pipeline": pipeline,
            "func_unit": func_unit,
            "sel_logic": sel_logic,
            "undiff_core": undiff_core,
        }

        if core0["machine_type"] == 0 and "phy_ireg_width" in core0:
            decode_width = core0["decode_width"]
            components["int_dcl"] = DepConflictChecker(
                "Int_DCL", cacti_params, core0["phy_ireg_width"], decode_width
            )
            components["fp_dcl"] = DepConflictChecker(
                "Fp_DCL", cacti_params, core0["phy_freg_width"], decode_width
            )

        return components

    def leakage_by_unit(self, cacti_params):
        """ "leakage" is each component's longer_channel_leakage when the XML
        enables it; gate_leakage has no reduced form in McPAT
        (cacti_interface.h powerComponents), so it is always plain."""
        c = self.build(cacti_params)
        fu = c["func_unit"]
        units = {
            "Pipeline": _logic_unit(cacti_params, c["pipeline"]),
            **{
                name: _unit(
                    cacti_params,
                    fu._leakage[k],
                    fu._longer_channel_leakage[k],
                    fu._gate_leakage[k],
                )
                for name, k in (
                    ("IntAlu", "ALU"),
                    ("FpAlu", "FPU"),
                    ("ComplexAlu", "MUL"),
                )
            },
            "SelLogic": _logic_unit(cacti_params, c["sel_logic"]),
            "UndiffCore": _logic_unit(cacti_params, c["undiff_core"]),
        }
        if "int_dcl" in c:
            units["IntDCL"] = _logic_unit(cacti_params, c["int_dcl"])
            units["FpDCL"] = _logic_unit(cacti_params, c["fp_dcl"])
        return units

    def activation_energies(self, cacti_params):
        c = self.build(cacti_params)
        energies = {
            "IntAlu": c["func_unit"]._dyn_energies["ALU"],
            "FpAlu": c["func_unit"]._dyn_energies["FPU"],
            "ComplexAlu": c["func_unit"]._dyn_energies["MUL"],
            "Pipeline": c["pipeline"]._dyn_energies["pipe_reg_power"],
            "SelLogic": c["sel_logic"]._dyn_energies["InstSel"],
        }
        # FuncUnit base power (logic.cc:548-583), nonzero only for an OOO
        # non-embedded core; watts, not joules (add per unit time).
        for key, fu in (
            ("IntAluBase", "ALU"),
            ("FpAluBase", "FPU"),
            ("ComplexAluBase", "MUL"),
        ):
            if c["func_unit"]._base_power[fu] != 0:
                energies[key] = c["func_unit"]._base_power[fu]
        if "int_dcl" in c:
            # Validated in main.py only at the Alpha21364 (180nm/HP) anchor.
            energies["IntDCL"] = c["int_dcl"]._dyn_energy
            energies["FpDCL"] = c["fp_dcl"]._dyn_energy
        return energies


class CoreArrays:
    PREDICTOR_KEYS = frozenset(
        {"GlobalPred", "L1LocalPred", "L2LocalPred", "ChooserPred", "RAS"}
    )
    """Core-device arrays (predictor tables, register files, rename, issue
    queues, LSQ, IFB, ROB) in the "Core" leakage bucket. Each defaults to the
    module-level anchor (ARM_A9_2GHz_gem5_ooo.jsonl; ROB from the rob40 graft);
    pass a CactiArraySpec to override. Dict keys are the names gem5-pm's power
    models read (e.g. "IntFRAT", "L1LocalPred", "FpRegFile")."""

    def __init__(
        self,
        global_pred=None,
        l1_pred=None,
        l2_pred=None,
        pred_chooser=None,
        ras=None,
        int_regfile=None,
        fp_regfile=None,
        int_frat=None,
        fp_frat=None,
        int_freelist=None,
        fp_freelist=None,
        int_issue_queue=None,
        fp_issue_queue=None,
        load_store_queue=None,
        load_queue=None,
        inst_buffer=None,
        inst_fetch_queue=None,
        rob=None,
    ):
        self._arrays = {
            "GlobalPred": global_pred or GLOBAL_PRED,
            "L1LocalPred": l1_pred or L1_PRED,
            "L2LocalPred": l2_pred or L2_PRED,
            "ChooserPred": pred_chooser or PRED_CHOOSER,
            "RAS": ras or RAS,
            "IntRegFile": int_regfile or INT_REGFILE,
            "FpRegFile": fp_regfile or FP_REGFILE,
            "IntFRAT": int_frat or INT_FRAT,
            "FpFRAT": fp_frat or FP_FRAT,
            "IntFreeList": int_freelist or INT_FREELIST,
            "FpFreeList": fp_freelist or FP_FREELIST,
            "InstIssueQueue": int_issue_queue or INT_ISSUE_QUEUE,
            "FPIssueQueue": fp_issue_queue or FP_ISSUE_QUEUE,
            "LoadStoreQueue": load_store_queue or LOAD_STORE_QUEUE,
            "LoadQueue": load_queue or LOAD_QUEUE,
            "InstBuffer": inst_buffer or INST_BUFFER,
            "InstFetchQueue": inst_fetch_queue or INST_FETCH_QUEUE,
            "ReorderBuffer": rob or ROB,
        }

    # McPAT sums these into Core only for an OOO core (LoadStoreQueue: both).
    _OOO_ONLY_KEYS = frozenset(
        {
            "IntFRAT",
            "FpFRAT",
            "IntFreeList",
            "FpFreeList",
            "InstIssueQueue",
            "FPIssueQueue",
            "LoadQueue",
            "ReorderBuffer",
        }
    )

    # InstFetchQueue is built only for Inorder&&multithreaded (core.cc:474).
    _INORDER_MT_ONLY_KEYS = frozenset({"InstFetchQueue"})

    def build(self, cacti_params, core_ooo=None):
        """{name: CactiArrayST}; a Cache slot (the four predictors at
        num_hthreads>1, core.cc:330-343) is passed through unbuilt and the
        callers below branch on isinstance(v, Cache)."""
        return {
            name: (
                arr
                if isinstance(arr, Cache)
                else arr.build(cacti_params, core_ooo=core_ooo)
            )
            for name, arr in self._arrays.items()
            if cacti_params._machine_config._config_params["core0"][
                "prediction_width"
            ]
            > 0
            or name not in self.PREDICTOR_KEYS
        }

    def leakage_by_unit(self, cacti_params, core_ooo=None):
        """Per-unit leakage, excluding the OOO-only keys for an inorder core
        and InstFetchQueue unless the core is multithreaded inorder (both read
        from cacti_params' core0), since McPAT never sums them there."""
        core0 = cacti_params._machine_config._config_params["core0"]
        is_ooo = core0["machine_type"] == 0
        is_mt_inorder = core0["machine_type"] == 1 and core0["num_threads"] > 1
        built = self.build(cacti_params, core_ooo=core_ooo)
        if not is_ooo:
            built = {
                name: a
                for name, a in built.items()
                if name not in self._OOO_ONLY_KEYS
            }
        if not is_mt_inorder:
            built = {
                name: a
                for name, a in built.items()
                if name not in self._INORDER_MT_ONLY_KEYS
            }
        units = {}
        for name, a in built.items():
            if isinstance(a, Cache):
                # Sum generically over whatever units the Cache reports.
                cache_units = a.leakage_by_unit(
                    cacti_params, core_ooo=core_ooo
                )
                units[name] = {
                    "leakage": sum(u["leakage"] for u in cache_units.values()),
                    "gate_leakage": sum(
                        u["gate_leakage"] for u in cache_units.values()
                    ),
                }
            else:
                units[name] = _array_unit(cacti_params, a)
        # core.cc:2013/2023: only the RAS's leakage scales by num_hthreads
        # (pppm_lkg_multhread, core.cc:4480); dynamic is untouched.
        n_threads = core0["num_threads"]
        if n_threads > 1 and "RAS" in units:
            units["RAS"] = {
                "leakage": units["RAS"]["leakage"] * n_threads,
                "gate_leakage": units["RAS"]["gate_leakage"] * n_threads,
            }
        return units

    def activation_energies(self, cacti_params):
        energies = {}
        for name, a in self.build(cacti_params).items():
            if isinstance(a, Cache):
                # Data+tag combined like BTB; buffer entries are dropped
                # (a predictor Cache has none).
                energies[name] = a.data_tag_energies(cacti_params)
            else:
                e = {"Read": a.read, "Write": a.write}
                if hasattr(a, "search"):
                    e["Search"] = a.search
                energies[name] = e
        if (
            cacti_params._machine_config._config_params["core0"][
                "prediction_width"
            ]
            == 0
        ):
            energies.update(
                {
                    name: {"Read": 0.0, "Write": 0.0}
                    for name in self.PREDICTOR_KEYS
                }
            )
        return energies


def _instruction_decoders(cacti_params):
    """The IDInst/IDOp/IDMisc InstDecoders (logic.cc:1097-1104 set_pppm)."""
    widths = _opcode_decoder_widths(cacti_params)
    is_x86 = bool(
        cacti_params._machine_config._config_params["core0"]["isX86"]
    )
    return {
        name: InstDecoder(name, cacti_params, width, x86=is_x86)
        for name, width in zip(("IDInst", "IDOp", "IDMisc"), widths)
    }


_CDB_OVERHEAD = 1.1  # basic_components.h: const double cdb_overhead = 1.1;


def _bypass_interconnects(cacti_params, core_arrays_built, func_unit):
    """Port of EXECU's bypass network (core.cc:1183-1300), McPAT's "Results
    Broadcast Bus": the Interconnect (data_width, wire length) for each of
    Int/Fp/MulBypass and *TagBypass, per branch (Inorder / OOO+PhysicalRegFile
    / OOO+ReservationStation). The ReservationStation branch has no numeric
    anchor. OOO configs without "phy_ireg_width" are skipped.

    Inorder register-file heights come from the OOO-sized anchor arrays, not
    archi_regs_*. McPAT's Inorder Mul paths use fp_regfile_height
    (core.cc:1197-1206); replicated verbatim.

    Returns the Interconnect objects so the dynamic and leakage callers share
    one geometry computation.
    """
    core0 = cacti_params._machine_config._config_params["core0"]
    sqrt_cdb = _CDB_OVERHEAD**0.5

    irf_h = core_arrays_built["IntRegFile"].area_h
    frf_h = core_arrays_built["FpRegFile"].area_h
    lsq_h = core_arrays_built["LoadStoreQueue"].area_h

    alu_h = func_unit._fu_height["ALU"]
    mul_h = func_unit._fu_height["MUL"]
    fpu_h = func_unit._fu_height["FPU"]
    has_mul = func_unit._has_mul
    has_fpu = func_unit._has_fpu

    int_data_width = cacti_params._machine_config._config_params["data_width"]

    def path(name, data_width, length_um):
        return Interconnect(name, cacti_params, data_width, length_um)

    energies = {}
    if core0["machine_type"] == 1:  # Inorder (core.cc:1183-1220)
        int_regfile_height = irf_h * sqrt_cdb
        fp_regfile_height = frf_h * sqrt_cdb
        lsq_height = lsq_h * sqrt_cdb
        # Iw_height is the InstFetchQueue's (core.cc:507), built only for
        # Inorder&&multithreaded (core.cc:474); else 0.
        iw_height = (
            core_arrays_built["InstFetchQueue"].area_h
            if core0["num_threads"] > 1
            else 0.0
        )
        per_thread_state = 8
        fp_mul_data_width = int(round(int_data_width * 1.5))

        energies["IntBypass"] = path(
            "Int Bypass Data",
            int_data_width,
            int_regfile_height + alu_h + lsq_height,
        )
        energies["IntTagBypass"] = path(
            "Int Bypass tag",
            per_thread_state,
            int_regfile_height + alu_h + lsq_height + iw_height,
        )
        if has_mul:
            mul_len = fp_regfile_height + alu_h + mul_h + lsq_height
            energies["MulBypass"] = path(
                "Mul Bypass Data", fp_mul_data_width, mul_len
            )
            energies["MulTagBypass"] = path(
                "Mul Bypass tag", per_thread_state, mul_len + iw_height
            )
        if has_fpu:
            energies["FpBypass"] = path(
                "FP Bypass Data", fp_mul_data_width, fp_regfile_height + fpu_h
            )
            energies["FpTagBypass"] = path(
                "FP Bypass tag",
                per_thread_state,
                fp_regfile_height + fpu_h + lsq_height + iw_height,
            )
    elif "phy_ireg_width" in core0:  # OOO (core.cc:1223-1294)
        phy_ireg_width = core0["phy_ireg_width"]
        phy_freg_width = core0["phy_freg_width"]
        fp_data_width = int_data_width  # core.cc:4378: fp_data_width == int_data_width always
        is_rs = (
            core0["inst_window_scheme"] == 1
        )  # 0=PhysicalRegFile, 1=ReservationStation

        rf_factor = core0["num_threads"] if is_rs else 1
        int_regfile_height = irf_h * rf_factor * sqrt_cdb
        fp_regfile_height = frf_h * rf_factor * sqrt_cdb

        lsq_len = lsq_h
        if core0["load_buffer_sz"] > 0:
            lsq_len += core_arrays_built["LoadQueue"].area_h
        lsq_height = lsq_len * sqrt_cdb

        iw_height = core_arrays_built["InstIssueQueue"].area_h
        fp_iw_height = core_arrays_built["FPIssueQueue"].area_h
        rob_height = (
            core_arrays_built["ReorderBuffer"].area_h
            if core0["rob_size"] > 0
            else 0.0
        )

        if not is_rs:  # PhysicalRegFile (core.cc:1223-1262)
            energies["IntBypass"] = path(
                "Int Bypass Data",
                int_data_width,
                int_regfile_height + alu_h + lsq_height,
            )
            energies["IntTagBypass"] = path(
                "Int Bypass tag",
                phy_ireg_width,
                int_regfile_height
                + alu_h
                + lsq_height
                + iw_height
                + rob_height,
            )
            if has_mul:
                mul_len = int_regfile_height + alu_h + mul_h + lsq_height
                energies["MulBypass"] = path(
                    "Mul Bypass Data", int_data_width, mul_len
                )
                energies["MulTagBypass"] = path(
                    "Mul Bypass tag",
                    phy_ireg_width,
                    mul_len + iw_height + rob_height,
                )
            if has_fpu:
                energies["FpBypass"] = path(
                    "FP Bypass Data", fp_data_width, fp_regfile_height + fpu_h
                )
                energies["FpTagBypass"] = path(
                    "FP Bypass tag",
                    phy_freg_width,
                    fp_regfile_height
                    + fpu_h
                    + lsq_height
                    + fp_iw_height
                    + rob_height,
                )
        else:  # ReservationStation (core.cc:1265-1294) -- source-verified only, no anchor
            int_len = (
                int_regfile_height
                + alu_h
                + lsq_height
                + iw_height
                + rob_height
            )
            energies["IntBypass"] = path(
                "Int Bypass Data", int_data_width, int_len
            )
            energies["IntTagBypass"] = path(
                "Int Bypass tag", phy_ireg_width, int_len
            )
            if has_mul:
                mul_len = (
                    int_regfile_height
                    + alu_h
                    + mul_h
                    + lsq_height
                    + iw_height
                    + rob_height
                )
                energies["MulBypass"] = path(
                    "Mul Bypass Data", int_data_width, mul_len
                )
                energies["MulTagBypass"] = path(
                    "Mul Bypass tag", phy_ireg_width, mul_len
                )
            if has_fpu:
                fp_len = (
                    fp_regfile_height
                    + fpu_h
                    + lsq_height
                    + fp_iw_height
                    + rob_height
                )
                energies["FpBypass"] = path(
                    "FP Bypass Data", fp_data_width, fp_len
                )
                energies["FpTagBypass"] = path(
                    "FP Bypass tag", phy_freg_width, fp_len
                )
    return energies


# McPAT weights bypass leakage/gate_leakage by operand count (2 Int/Mul, 3 FP;
# core.cc:3777-3787), but not per-access dynamic energy (core.cc:3800-3818).
_BYPASS_LEAKAGE_WEIGHT = {
    "IntBypass": 2,
    "IntTagBypass": 2,
    "MulBypass": 2,
    "MulTagBypass": 2,
    "FpBypass": 3,
    "FpTagBypass": 3,
}


def _bypass_leakage_by_unit(cacti_params, core_arrays_built, func_unit):
    """Bypass leakage/gate_leakage weighted by _BYPASS_LEAKAGE_WEIGHT; the
    per-instance longer-channel reduction commutes with McPAT's
    weight-then-reduce order (same scalar for every path)."""
    interconnects = _bypass_interconnects(
        cacti_params, core_arrays_built, func_unit
    )
    return {
        name: {
            key: _BYPASS_LEAKAGE_WEIGHT[name] * v
            for key, v in _array_unit(cacti_params, ic).items()
        }
        for name, ic in interconnects.items()
    }


def _opcode_decoder_widths(cacti_params):
    """Widths of core.cc:278-291's three InstDecoders. ID_inst always uses
    opcode_length (never micro_opcode_length, even for x86; x86 only
    multiplies dynamic energy), and leakage scales with 2**width."""
    core0 = cacti_params._machine_config._config_params["core0"]
    return core0["opcode_width"], core0["arch_ireg_width"], 8


def _apply_mcpat_ooo_core_attribution(units):
    """In place: re-attribute an OOO core's Core-bucket leakage the way
    real McPAT sums it into "Core:" (a McPAT quirk, replicated):

      * Int/Fp DCL leakage is never added to any OOO total -- RENAMEU's
        runtime/TDP leaves are FRAT/free-list + local_result (+
        idcl->power_t, which is only ever set on the in-order branch,
        core.cc:2706-2711/2764-2765) -- so IntDCL/FpDCL become 0.
      * instruction_selection's leakage is added in full to BOTH the int
        and the fp instruction window (core.cc:3106-3107), i.e. counted
        twice -- folded here into InstIssueQueue and FPIssueQueue, and
        the standalone SelLogic unit becomes 0.

    The key set is kept (zeros, not deletions); the unit sum is unchanged."""
    zero = {"leakage": 0.0, "gate_leakage": 0.0}
    sel = units["SelLogic"]
    for key in ("InstIssueQueue", "FPIssueQueue"):
        units[key] = {
            "leakage": units[key]["leakage"] + sel["leakage"],
            "gate_leakage": units[key]["gate_leakage"] + sel["gate_leakage"],
        }
    units["SelLogic"] = dict(zero)
    units["IntDCL"] = dict(zero)
    units["FpDCL"] = dict(zero)


class MachineModel:
    """Built once from a McPAT-style config. activation_energies() is a
    one-time snapshot (dynamic energy is temperature independent);
    leakage_at() is queried per bucket and memoized per temperature decade."""

    def __init__(
        self,
        machine_params,
        core_params,
        l2_device_type=0,
        core_logic=None,
        core_arrays=None,
        icache_cache=None,
        dcache_cache=None,
        l2_cache=None,
        btb_cache=None,
        *,
        itlb,
        dtlb,
        mcpat_ooo_core_attribution=True,
    ):
        self._machine_params = dict(machine_params)
        self._core_params = dict(core_params)
        self._l2_device_type = l2_device_type
        self._core_logic = core_logic or CoreLogic()
        self._core_arrays = core_arrays or CoreArrays()
        self._icache_cache = icache_cache or cache_preset("ICACHE_CACHE")
        self._dcache_cache = dcache_cache or cache_preset("DCACHE_CACHE")
        self._l2_cache = l2_cache or cache_preset("L2_CACHE")
        self._btb_cache = btb_cache or Cache(
            "Branch Target Buffer", BTB, tag_array=BTB_TAG
        )
        # core.cc:952-1000: ITLB/DTLB are separate arrays (DTLB ports follow
        # memory_ports).
        self._itlb = itlb
        self._dtlb = dtlb
        # See _apply_mcpat_ooo_core_attribution; False keeps the plain sum.
        self._mcpat_ooo_core_attribution = mcpat_ooo_core_attribution

        self._cacti_params = build_cacti_params(
            self._machine_params, self._core_params
        )
        l2_machine_params = dict(self._machine_params)
        l2_machine_params["device_type"] = l2_device_type
        self._cacti_params_l2 = build_cacti_params(
            l2_machine_params, self._core_params
        )

        # longer_channel_device_reduction() is keyed on the core's core_ty.
        core0 = self._cacti_params._machine_config._config_params["core0"]
        self._core_ooo = core0["machine_type"] == 0

        self._leakage_by_unit_cache = {}

    def cpu_activation_energies(self):
        """The init_act_energies() keys this port builds: CoreLogic,
        CoreArrays, the bypass network, decoders, BTB and TLBs. Callers layer
        it onto their own dict."""
        core_logic_built = self._core_logic.build(self._cacti_params)
        core_arrays_built = self._core_arrays.build(self._cacti_params)

        energies = self._core_logic.activation_energies(self._cacti_params)
        energies.update(
            self._core_arrays.activation_energies(self._cacti_params)
        )
        energies.update(
            {
                name: ic.dynamic
                for name, ic in _bypass_interconnects(
                    self._cacti_params,
                    core_arrays_built,
                    core_logic_built["func_unit"],
                ).items()
            }
        )

        # The O3 power model reads "IntInstWindow"/"FpInstWindow" as aliases of
        # the issue queues; a multithreaded inorder core has one unified window
        # (SchedulerU, core.cc:474-518).
        core0 = self._cacti_params._machine_config._config_params["core0"]
        if core0["machine_type"] == 1 and core0["num_threads"] > 1:
            energies["IntInstWindow"] = energies["InstFetchQueue"]
        else:
            energies["IntInstWindow"] = energies["InstIssueQueue"]
            energies["FpInstWindow"] = energies["FPIssueQueue"]

        itlb = self._itlb.build(self._cacti_params)
        dtlb = self._dtlb.build(self._cacti_params)

        energies.update(
            {
                **{
                    name: d._power.read.dynamic
                    for name, d in _instruction_decoders(
                        self._cacti_params
                    ).items()
                },
                "BTB": (
                    self._btb_cache.data_tag_energies(self._cacti_params)
                    if core0["prediction_width"] > 0
                    else {
                        "Read": 0.0,
                        "Write": 0.0,
                        "TagRead": 0.0,
                        "TagWrite": 0.0,
                    }
                ),
                "ITLB": {
                    "Read": itlb.read,
                    "Write": itlb.write,
                    "Search": itlb.search,
                },
                "DTLB": {
                    "Read": dtlb.read,
                    "Write": dtlb.write,
                    "Search": dtlb.search,
                },
            }
        )
        # A CAM RAT (rename_scheme=1) pays a search per lookup, a RAM RAT a
        # read (core.cc:2626-2637).
        lookup_key = "Search" if core0["rename_scheme"] == 1 else "Read"
        for frat in ("IntFRAT", "FpFRAT"):
            energies[frat]["Lookup"] = energies[frat][lookup_key]
        return energies

    def cache_activation_energies(self):
        """Per-bucket Cache.activation_energies(), including each cache's
        buffers under their trace names (e.g. "icacheMissBuffer")."""
        return {
            "Instruction Cache": self._icache_cache.activation_energies(
                self._cacti_params
            ),
            "Data Cache": self._dcache_cache.activation_energies(
                self._cacti_params
            ),
            "L2": self._l2_cache.activation_energies(self._cacti_params_l2),
        }

    def activation_energies(self):
        energies = self.cpu_activation_energies()
        energies.update(self.cache_activation_energies())
        return energies

    @property
    def machine_params(self):
        return dict(self._machine_params)

    def leakage_by_unit_at(self, bucket, temp_k):
        """{unit_name: {"leakage":, "gate_leakage":}} for every unit in
        `bucket`: CoreLogic, CoreArrays, bypass, BTB/TLBs and decoders for
        "Core", a Cache's data/tag/buffers otherwise. leakage_at() and
        gate_leakage_at() sum this dict. Returns a fresh copy per call."""
        if bucket not in LEAKAGE_BUCKETS:
            raise ValueError(
                f"Unknown leakage bucket {bucket!r}; expected one of {LEAKAGE_BUCKETS}"
            )

        decade = clamp_to_cacti_decade(temp_k)
        key = (bucket, decade)
        if key in self._leakage_by_unit_cache:
            return {
                name: dict(v)
                for name, v in self._leakage_by_unit_cache[key].items()
            }

        if bucket == "L2":
            l2_machine_params = dict(self._machine_params)
            l2_machine_params["device_type"] = self._l2_device_type
            cacti_params = build_cacti_params(
                l2_machine_params, self._core_params, decade
            )
            units = self._l2_cache.leakage_by_unit(
                cacti_params, core_ooo=self._core_ooo
            )
        else:
            cacti_params = build_cacti_params(
                self._machine_params, self._core_params, decade
            )
            if bucket == "Core":
                # core_arrays_built only supplies the bypass wire lengths
                # (area_h), which do not depend on core_ooo.
                core_logic_built = self._core_logic.build(cacti_params)
                core_arrays_built = self._core_arrays.build(cacti_params)
                units = self._core_logic.leakage_by_unit(cacti_params)
                units.update(
                    self._core_arrays.leakage_by_unit(
                        cacti_params, core_ooo=self._core_ooo
                    )
                )
                units.update(
                    _bypass_leakage_by_unit(
                        cacti_params,
                        core_arrays_built,
                        core_logic_built["func_unit"],
                    )
                )
                if self._core_params["prediction_width"] > 0:
                    btb = self._btb_cache.leakage_by_unit(
                        cacti_params, core_ooo=self._core_ooo
                    )
                    units["BTB"] = {
                        key: btb["Data"][key] + btb["Tag"][key]
                        for key in ("leakage", "gate_leakage")
                    }
                for name, tlb in (("ITLB", self._itlb), ("DTLB", self._dtlb)):
                    units[name] = _array_unit(
                        cacti_params,
                        tlb.build(cacti_params, core_ooo=self._core_ooo),
                    )
                for name, decoder in _instruction_decoders(
                    cacti_params
                ).items():
                    units[name] = _logic_unit(cacti_params, decoder)
                if self._mcpat_ooo_core_attribution and self._core_ooo:
                    _apply_mcpat_ooo_core_attribution(units)
            elif bucket == "Instruction Cache":
                units = self._icache_cache.leakage_by_unit(
                    cacti_params, core_ooo=self._core_ooo
                )
            else:  # "Data Cache"
                units = self._dcache_cache.leakage_by_unit(
                    cacti_params, core_ooo=self._core_ooo
                )

        self._leakage_by_unit_cache[key] = units
        return {name: dict(v) for name, v in units.items()}

    def leakage_at(self, bucket, temp_k):
        return sum(
            u["leakage"]
            for u in self.leakage_by_unit_at(bucket, temp_k).values()
        )

    def gate_leakage_at(self, bucket, temp_k):
        return sum(
            u["gate_leakage"]
            for u in self.leakage_by_unit_at(bucket, temp_k).values()
        )
