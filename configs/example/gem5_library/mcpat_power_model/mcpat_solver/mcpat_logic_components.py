from math import (
    ceil,
    log,
    log2,
)

from .cacti_circuit import CactiCircuit
from .cacti_component import (
    CactiComponent,
    CactiDecoder,
    CactiPredec,
    CactiPredecBlk,
    CactiPredecBlkDrv,
)
from .cacti_params import CactiParams


class DFFCell(CactiComponent):
    def __init__(
        self,
        name,
        cacti_params: CactiParams,
        is_dram,
        WdecNANDn,
        WdecNANDp,
        cell_load,
    ):
        super().__init__(name, cacti_params)
        self._is_dram = is_dram
        self._WdecNANDn = WdecNANDn
        self._WdecNANDp = WdecNANDp
        self._cell_load = cell_load
        self._dyn_energies = {
            "e_switch": 0,
            "e_keep_0": 0,
            "e_keep_1": 0,
            "e_clk": 0,
        }
        # Only e_switch carries leakage/gate_leakage in logic.cc:288-292 (the
        # NAND2/NAND3 gates making up the DFF) -- e_keep_0/e_keep_1/e_clk are
        # never assigned a leakage term in compute_DFF_cell, so they stay 0.
        self._e_switch_leakage = 0
        self._e_switch_gate_leakage = 0

    def compute_dff_cell(self):
        c1 = self.fpfp_node_cap(2, 1)
        c2 = self.fpfp_node_cap(2, 3)
        c3 = self.fpfp_node_cap(3, 2)
        c4 = self.fpfp_node_cap(2, 2)

        clock_cap = 2 * self.gate_C(self._WdecNANDn + self._WdecNANDp, 0)

        self._dyn_energies["e_switch"] = (
            0.5 * (c1 + c1 + c1 + c4 + c2 + c3 + 2 * self._cell_load)
        ) * self._tp["Vdd"] ** 2

        self._dyn_energies["e_keep_0"] = c2 * self._tp["Vdd"] ** 2
        self._dyn_energies["e_keep_1"] = c3 * self._tp["Vdd"] ** 2
        self._dyn_energies["e_clk"] = clock_cap * self._tp["Vdd"] ** 2

        Vdd = self._tp["Vdd"]
        self._e_switch_leakage = (
            self.cmos_Isub_leakage(self._WdecNANDn, self._WdecNANDp, 2, "NAND")
            * 5
            + self.cmos_Isub_leakage(
                self._WdecNANDn, self._WdecNANDn, 3, "NAND"
            )
        ) * Vdd
        self._e_switch_gate_leakage = (
            self.cmos_Ig_leakage(self._WdecNANDn, self._WdecNANDp, 2, "NAND")
            * 5
            + self.cmos_Ig_leakage(self._WdecNANDn, self._WdecNANDn, 3, "NAND")
        ) * Vdd

    def fpfp_node_cap(self, fan_in, fan_out):
        c_total = self.drain_C(
            self._WdecNANDn, "NCH", 2, 1, self._tp["cell_h_def"]
        ) + fan_in * self.drain_C(
            self._WdecNANDp, "PCH", 1, 1, self._tp["cell_h_def"]
        )
        c_total += fan_out * self.gate_C(self._WdecNANDn + self._WdecNANDp, 0)
        return c_total


class SelectionLogic(CactiComponent):
    def __init__(self, name, cacti_params: CactiParams, issue_width=None):
        super().__init__(name, cacti_params)
        self._dyn_energies = {}
        self._win_entries = self._machine_config._config_params["core0"][
            "inst_window_size"
        ]
        if issue_width is None:
            issue_width = self._machine_config._config_params["core0"][
                "peak_issue_width"
            ]
        self._issue_width = issue_width

    def calc_dyn_energies(self):
        WSelOrn = 12.5 * self._node_um
        WSelOrprequ = 50 * self._node_um
        WSelPn = 12.5 * self._node_um
        WSelPp = 18.75 * self._node_um
        WSelEnn = 6.25 * self._node_um
        WSelEnp = 12.5 * self._node_um

        # Local, not self._win_entries: logic.cc's win_entries is mutated the
        # same way, but re-running calc_dyn_energies() must not compound it.
        win_entries = self._win_entries
        num_arbiters = 1
        while win_entries > 4:
            win_entries = int(ceil(win_entries / 4.0))
            num_arbiters += win_entries

        Cor = 4 * self.drain_C(
            WSelOrn, "NCH", 1, 1, self._tp["cell_h_def"]
        ) + self.drain_C(WSelOrprequ, "PCH", 1, 1, self._tp["cell_h_def"])
        Cpencode = (
            self.drain_C(WSelPn, "NCH", 1, 1, self._tp["cell_h_def"])
            + self.drain_C(WSelPp, "PCH", 1, 1, self._tp["cell_h_def"])
            + 2 * self.drain_C(WSelPn, "NCH", 1, 1, self._tp["cell_h_def"])
            + self.drain_C(WSelPp, "PCH", 2, 1, self._tp["cell_h_def"])
            + 3 * self.drain_C(WSelPn, "NCH", 1, 1, self._tp["cell_h_def"])
            + self.drain_C(WSelPp, "PCH", 3, 1, self._tp["cell_h_def"])
            + 4 * self.drain_C(WSelPn, "NCH", 1, 1, self._tp["cell_h_def"])
            + self.drain_C(WSelPp, "PCH", 4, 1, self._tp["cell_h_def"])
            + 2 * 4 * self.gate_C(WSelEnn + WSelEnp, 20.0)
            + 4 * self.drain_C(WSelEnn, "NCH", 1, 1, self._tp["cell_h_def"])
            + 2
            * 4
            * self.drain_C(WSelEnp, "PCH", 1, 1, self._tp["cell_h_def"])
            + (2 * 4 + 2 * 3 + 2 * 2 + 2) * self.gate_C(WSelPn + WSelPp, 10.0)
        )

        Ctotal = self._issue_width * num_arbiters * (Cor + Cpencode)
        self._dyn_energies["InstSel"] = (
            Ctotal * 2 * self._tp["Vdd"] ** 2 * self._tp["sckt_coeff"]
        )

        Vdd = self._tp["Vdd"]
        self._power.read.leakage = (
            self._issue_width
            * num_arbiters
            * (
                self.cmos_Isub_leakage(WSelPn, WSelPp, 2, "NOR")
                + self.cmos_Isub_leakage(WSelPn, WSelPp, 3, "NOR")
                + self.cmos_Isub_leakage(WSelPn, WSelPp, 4, "NOR")
                + self.cmos_Isub_leakage(WSelEnn, WSelEnp, 2, "NOR") * 4
                + self.cmos_Isub_leakage(WSelEnn, WSelEnp, 1, "INV") * 2 * 3
            )
            * Vdd
        )
        self._power.read.gate_leakage = (
            self._issue_width
            * num_arbiters
            * (
                self.cmos_Ig_leakage(WSelPn, WSelPp, 2, "NOR")
                + self.cmos_Ig_leakage(WSelPn, WSelPp, 3, "NOR")
                + self.cmos_Ig_leakage(WSelPn, WSelPp, 4, "NOR")
                + self.cmos_Ig_leakage(WSelEnn, WSelEnp, 2, "NOR") * 4
                + self.cmos_Ig_leakage(WSelEnn, WSelEnp, 1, "INV") * 2 * 3
            )
            * Vdd
        )
        # logic.cc:63-64 (selection_logic constructor, right after
        # selection_power()) -- Core_device unconditionally (core.cc:516-520).
        self.longer_channel_leakage = (
            self._power.read.leakage * self._long_channel_reduction()
        )


class Pipeline(CactiComponent):
    def __init__(self, name, cacti_params: CactiParams):
        super().__init__(name, cacti_params)
        self._dyn_energies = {}
        self._core_type = self._machine_config._config_params["core0"][
            "machine_type"
        ]
        if self._machine_config._config_params["core0"]["isX86"]:
            self._opcode_width = self._machine_config._config_params["core0"][
                "micro_opcode_width"
            ]
        else:
            self._opcode_width = self._machine_config._config_params["core0"][
                "opcode_width"
            ]
        self._pc_width = self._machine_config._config_params["virt_addr_width"]
        self._fetch_width = self._machine_config._config_params["core0"][
            "fetch_width"
        ]
        self._decode_width = self._machine_config._config_params["core0"][
            "decode_width"
        ]
        self._issue_width = self._machine_config._config_params["core0"][
            "issue_width"
        ]
        self._commit_width = self._machine_config._config_params["core0"][
            "commit_width"
        ]
        self._num_threads = self._machine_config._config_params["core0"][
            "num_threads"
        ]
        self._inst_len = self._machine_config._config_params["core0"][
            "inst_len"
        ]

        """ This is the data widths for int/fp. McPAT assumes they're the
          same. """

        self._data_width = self._machine_config._config_params["data_width"]

        """ Width of Integer/Fp architectural RFs """
        self._arch_irf_width = self._machine_config._config_params["core0"][
            "archi_regs_irf_size"
        ]
        self._arch_irf_width = self._machine_config._config_params["core0"][
            "arch_ireg_width"
        ]
        self._arch_frf_width = self._machine_config._config_params["core0"][
            "arch_freg_width"
        ]
        if self._core_type == 0:
            self._phys_irf_width = self._machine_config._config_params[
                "core0"
            ]["phy_ireg_width"]
            self._phys_frf_width = self._machine_config._config_params[
                "core0"
            ]["phy_freg_width"]
        self._per_thread_states = 8
        if not self._machine_config._config_params["embedded"]:
            WNANDn = 25 * self._node_um
            WNANDp = 37.5 * self._node_um
        else:
            WNANDn = self._tp["min_w_nmos"]
            WNANDp = cacti_params.get_pmos_to_nmos_sz_ratio() * WNANDn

        self._load_per_pipeline_stage = 2 * self.gate_C(WNANDn + WNANDp, 0)
        self._pipereg_dff = DFFCell(
            "PipelineRegDFF",
            cacti_params,
            False,
            WNANDn,
            WNANDp,
            self._load_per_pipeline_stage,
        )

    def compute_stage_vector(self):
        if self._core_type:
            # McPAT considers VA width == PC Width
            num_piperegs = self._pc_width * 2 * self._num_threads
            num_piperegs += (
                self._fetch_width
                * (self._inst_len + self._pc_width)
                * self._num_threads
            )
            if self._num_threads > 1:
                num_piperegs += self._num_threads * self._per_thread_states
            num_piperegs += (
                self._decode_width
                * (
                    self._inst_len
                    + self._pc_width
                    + 2**self._opcode_width
                    + 2 * self._data_width
                )
                * self._num_threads
            )
            num_piperegs += self._issue_width * (
                3 * self._arch_irf_width
                + 2**self._opcode_width
                + 8 * 2 * self._data_width
            )
            num_piperegs += self._issue_width * (
                2 * self._data_width
                + 2**self._opcode_width
                + 8 * 2 * self._data_width
            )
            num_stages = 6
        else:
            num_piperegs = self._pc_width * 2 * self._num_threads
            num_piperegs += (
                self._fetch_width
                * (self._inst_len + self._pc_width)
                * self._num_threads
            )
            num_piperegs += (
                self._decode_width
                * (self._inst_len + self._pc_width)
                * self._num_threads
            )
            num_piperegs += self._decode_width * (
                self._inst_len + self._pc_width
            )
            num_piperegs += (
                self._issue_width
                * (self._inst_len + self._pc_width + 3 * self._phys_irf_width)
                * self._num_threads
            )
            num_piperegs += self._issue_width * (
                self._inst_len + 3 * self._phys_irf_width
            )
            num_piperegs += self._issue_width * (
                3 * self._phys_irf_width
                + self._pc_width
                + 2**self._opcode_width
            )

            """
                  McPAT adds below twice b/c there are 2 execute stages
                  and the ops are not distinguished (thus the 2x)
              """
            num_piperegs += 2 * (
                self._issue_width
                * (2 * self._data_width + 2**self._opcode_width)
            )

            num_piperegs += self._issue_width * (
                self._data_width + self._pc_width + 2**self._opcode_width
            )
            num_piperegs += self._issue_width * (
                self._data_width + self._phys_irf_width
            )
            num_piperegs += (
                self._commit_width
                * (self._data_width + self._pc_width + self._phys_irf_width)
                * self._num_threads
            )

            num_stages = 12
        num_piperegs *= 1.5
        pipeline_stages = self._machine_config.comma_sep_str_to_list(
            self._machine_config._config_params["core0"]["pipeline_depth"]
        )[0]
        per_stage_vec = num_piperegs / num_stages
        if self._core_type:
            if pipeline_stages > 6:
                num_piperegs = per_stage_vec * pipeline_stages
                # return per_stage_vec * pipeline_stages
        else:
            if pipeline_stages > 12:
                num_piperegs = per_stage_vec * pipeline_stages
                # return per_stage_vec * pipeline_stages
        return num_piperegs

    def calc_dyn_energies(self):
        num_piperegs = self.compute_stage_vector()
        self._pipereg_dff.compute_dff_cell()
        clk_power_pipereg = (
            num_piperegs * self._pipereg_dff._dyn_energies["e_clk"]
        )
        pipe_reg_power = (
            num_piperegs
            * (
                self._pipereg_dff._dyn_energies["e_switch"]
                + self._pipereg_dff._dyn_energies["e_keep_0"]
                + self._pipereg_dff._dyn_energies["e_keep_1"]
            )
            / 3
            + clk_power_pipereg
        )

        self._dyn_energies["pipe_reg_power"] = pipe_reg_power
        self._dyn_energies["pipe_reg_power"] *= self._tp["sckt_coeff"]

        # logic.cc:348-352 -- leakage/gate_leakage are NOT scaled by sckt_coeff
        # (only readOp/writeOp/searchOp.dynamic are, at logic.cc:362-365).
        self._power.read.leakage = (
            num_piperegs * self._pipereg_dff._e_switch_leakage
        )
        self._power.read.gate_leakage = (
            num_piperegs * self._pipereg_dff._e_switch_gate_leakage
        )
        # logic.cc:353-354 (Pipeline::compute(), right after the leakage sum
        # above) -- device_ty defaults to Core_device (logic.h:155), the only
        # value core.cc:1859 ever passes.
        self.longer_channel_leakage = (
            self._power.read.leakage * self._long_channel_reduction()
        )


class DepConflictChecker(CactiComponent):
    def __init__(
        self, name, cacti_params: CactiParams, compare_bits, decode_width
    ):
        super().__init__(name, cacti_params)
        self._compare_bits = compare_bits + 32
        self._decode_width = decode_width
        self._dyn_energy = self.calc_dyn_energies()

    def calc_dyn_energies(self):
        num_comparators = 3 * (self._decode_width**2 - self._decode_width)
        C_total = num_comparators * self.compare_cap()

        Wcompn = 25 * self._node_um
        self._power.read.leakage = (
            num_comparators
            * self._compare_bits
            * 2
            * self.simplified_nmos_leakage(Wcompn, False)
        )
        self._power.read.gate_leakage = (
            num_comparators
            * self._compare_bits
            * 2
            * self.cmos_Ig_leakage(Wcompn, 0, 2, "NMOS")
        )
        # logic.cc:179-180 (conflict_check_power(), right after the leakage
        # assignment above) -- Core_device unconditionally (core.cc:1595-1596).
        self.longer_channel_leakage = (
            self._power.read.leakage * self._long_channel_reduction()
        )

        return C_total * self._tp["Vdd"] ** 2 * self._tp["sckt_coeff"]

    def compare_cap(self):
        Wcompn = 25 * self._node_um
        Wevalinvp = 25 * self._node_um
        Wevalinvn = 100 * self._node_um
        Wcomppreequ = 50 * self._node_um
        WNORn = 6.75 * self._node_um
        WNORp = 38.125 * self._node_um

        WNORp *= self._compare_bits / 2.0
        c2 = (
            self._compare_bits
            * (
                self.drain_C(Wcompn, "NCH", 1, 1, self._tp["cell_h_def"])
                + self.drain_C(Wcompn, "NCH", 2, 1, self._tp["cell_h_def"])
            )
            + self.drain_C(Wevalinvp, "PCH", 1, 1, self._tp["cell_h_def"])
            + self.drain_C(Wevalinvn, "NCH", 1, 1, self._tp["cell_h_def"])
        )

        c1 = (
            self._compare_bits
            * (
                self.drain_C(Wcompn, "NCH", 1, 1, self._tp["cell_h_def"])
                + self.drain_C(Wcompn, "NCH", 2, 1, self._tp["cell_h_def"])
                + self.drain_C(
                    Wcomppreequ, "NCH", 1, 1, self._tp["cell_h_def"]
                )
            )
            + self.gate_C(WNORn + WNORp, 10.0)
            + self.drain_C(WNORp, "NCH", 2, 1, self._tp["cell_h_def"])
            + self._compare_bits
            * self.drain_C(WNORn, "NCH", 2, 1, self._tp["cell_h_def"])
        )

        return c1 + c2


class UndiffCore(CactiComponent):
    """Port of UndiffCore::UndiffCore (logic.cc:808-867), leakage/gate_leakage
    only. Dynamic is structurally 0 in the source (power.readOp.dynamic is
    only ever *= scktRatio'd there, never assigned a nonzero value); area
    and the power_gated leakage variant are out of scope, matching this
    project's standing decisions (area: SS1; power_gated_leakage/
    power_gated_with_long_channel_leakage: still unported, mirrored by
    every other logic component in this file). longer_channel_leakage
    itself is now ported (Phase 45, PROGRESS.md) -- see
    self.longer_channel_leakage below.

    Unconditionally constructed once per core (core.cc:1854, core_ty/embedded-
    gated internally, not by any XML flag) and its leakage/gate_leakage feed
    the real per-core power total (core.cc:3966/4089) -- unlike every other
    class in this file, McPAT never skips building this one.
    """

    def __init__(self, name, cacti_params: CactiParams, opt_clockrate=True):
        super().__init__(name, cacti_params)
        self._core_type = self._machine_config._config_params["core0"][
            "machine_type"
        ]
        self._embedded = self._machine_config._config_params["embedded"]
        self._pipeline_stage = self._machine_config.comma_sep_str_to_list(
            self._machine_config._config_params["core0"]["pipeline_depth"]
        )[0]
        self._num_hthreads = self._machine_config._config_params["core0"][
            "num_threads"
        ]
        # sys.opt_clockrate (XML_Parse.cc:1467 default true; only consulted on
        # the embedded branch below) -- not one of McPATMachineConfig's defined
        # fields, so taken as a constructor param like InstDecoder's x86.
        self._opt_clockrate = opt_clockrate
        self._power.read.leakage = 0
        self._power.read.gate_leakage = 0

    def calc_leakage(self):
        p_to_n = self.pmos_to_nmos_sz_ratio()

        if not self._embedded:
            if self._core_type == 0:  # OOO
                undiff_core = max(3.57 * log(self._pipeline_stage) - 1.2643, 0)
            else:  # Inorder
                undiff_core = max(-2.19 * log(self._pipeline_stage) + 6.55, 0)
            undiff_core *= 1 + log2(self._num_hthreads) * 0.0716
        else:
            undiff_core_coe = 0.05 if self._opt_clockrate else 0
            undiff_core = (
                0.4109 * self._pipeline_stage - 0.776
            ) * undiff_core_coe
            undiff_core *= 1 + log2(self._num_hthreads) * 0.0426

        undiff_core *= self._tp["logic_scaling_co_eff"] * 1e6  # mm^2 -> um^2
        core_tx_density = self._tp["core_tx_density"]
        self._undiff_core = undiff_core

        min_w = self._tp["min_w_nmos"]
        Vdd = self._tp["Vdd"]
        self._power.read.leakage = (
            undiff_core
            * core_tx_density
            * self.cmos_Isub_leakage(5 * min_w, 5 * min_w * p_to_n, 1, "INV")
            * Vdd
        )
        self._power.read.gate_leakage = (
            undiff_core
            * core_tx_density
            * self.cmos_Ig_leakage(5 * min_w, 5 * min_w * p_to_n, 1, "INV")
            * Vdd
        )
        # logic.cc:869-870 (UndiffCore constructor, right after the leakage
        # assignment above) -- Core_device unconditionally.
        self.longer_channel_leakage = (
            self._power.read.leakage * self._long_channel_reduction()
        )


class FuncUnit(CactiComponent):
    def __init__(
        self,
        cacti_params: CactiParams,
        has_alu=True,
        has_fpu=True,
        has_mul=True,
        num_alus=1,
        num_fpus=1,
        num_muls=1,
    ):
        super().__init__("FuncUnits", cacti_params)
        self._embedded = self._machine_config._config_params["embedded"]
        self._core_type = self._machine_config._config_params["core0"][
            "machine_type"
        ]
        self._has_alu = has_alu
        self._has_fpu = has_fpu
        self._has_mul = has_mul
        self._num_alus = num_alus
        self._num_fpus = num_fpus
        self._num_muls = num_muls
        self._dyn_energies = {"ALU": 0, "FPU": 0, "MUL": 0}
        self._base_power = {"ALU": 0, "FPU": 0, "MUL": 0}
        self._leakage = {"ALU": 0, "FPU": 0, "MUL": 0}
        self._gate_leakage = {"ALU": 0, "FPU": 0, "MUL": 0}
        self._longer_channel_leakage = {"ALU": 0, "FPU": 0, "MUL": 0}
        self._fu_height = {"ALU": 0.0, "FPU": 0.0, "MUL": 0.0}

    def calc_dyn_energies(self):
        # Local, not self._dyn_energies: a type with has_X=False must stay 0
        # even on a second call, rather than re-scaling a stale value left
        # over from an earlier call (same class of bug as SelectionLogic's
        # win_entries mutation).
        raw = {"ALU": 0, "FPU": 0, "MUL": 0}
        const = 1.15 / 1e9 / 4 / 1.3 / 1.3
        if self._embedded:
            const *= 0.5
            if self._has_mul:
                raw["MUL"] = 2 * const / 3
            if self._has_fpu:
                raw["FPU"] = const
            if self._has_alu:
                raw["ALU"] = const / 3
        else:
            if self._has_fpu:
                raw["FPU"] = const * 3
            if self._has_mul:
                raw["MUL"] = const * 2
            if self._has_alu:
                raw["ALU"] = const

        per_access_energy = (
            lambda c: c
            * (self._tp["Vdd"]) ** 2
            * (self._node / 90.0)
            * self._tp["sckt_coeff"]
        )
        self._dyn_energies = {c: per_access_energy(v) for c, v in raw.items()}

        # logic.cc:548-583 base_energy (W): an OOO, non-embedded core's FUs
        # draw a constant 89e-3 x {ALU 1, MUL 2, FPU 3} W (Wattch average)
        # scaled by Vdd^2/1.2^2, NOT multiplied by num_fu. The embedded
        # branch sets it to 0 and so does an Inorder core. computeEnergy's
        # runtime branch (logic.cc:684) adds base_energy*executionTime to
        # per_access_energy*accesses before the sckt_co_eff scale, so the
        # runtime-dynamic contribution is base_energy*sckt_co_eff W for
        # every second simulated, independent of the access count.
        # (FU_duty_cycle only scales the is_tdp peak branch.)
        base_raw = {"ALU": 0, "FPU": 0, "MUL": 0}
        if (not self._embedded) and self._core_type != 1:
            if self._has_fpu:
                base_raw["FPU"] = 89e-3 * 3
            if self._has_alu:
                base_raw["ALU"] = 89e-3
            if self._has_mul:
                base_raw["MUL"] = 89e-3 * 2
        Vdd = self._tp["Vdd"]
        self._base_power = {
            c: v * (Vdd * Vdd / 1.2 / 1.2) * self._tp["sckt_coeff"]
            for c, v in base_raw.items()
        }

    def calc_leakage(self):
        # Port of logic.cc:487-612 (FunctionalUnit constructor)'s area_t /
        # leakage / gate_leakage terms, per fu_type, differentiated by
        # embedded vs non-embedded. area_t itself is out of scope (energy
        # only) except as an intermediate factor of leakage.
        p2n = self.pmos_to_nmos_sz_ratio()
        min_w_nmos = self._tp["min_w_nmos"]
        Vdd = self._tp["Vdd"]
        logic_scaling_co_eff = self._tp["logic_scaling_co_eff"]
        core_tx_density = self._tp["core_tx_density"]

        def leak_pair(w_n, w_p):
            leak = (
                core_tx_density
                * self.cmos_Isub_leakage(w_n, w_p, 1, "INV")
                * Vdd
                / 2
            )
            gleak = (
                core_tx_density
                * self.cmos_Ig_leakage(w_n, w_p, 1, "INV")
                * Vdd
                / 2
            )
            return leak, gleak

        if self._has_fpu:
            fpu_w_n = 5 * min_w_nmos
            fpu_w_p = 5 * min_w_nmos * p2n
            # logic.cc:508-511/560-562 -- the node>90nm scaling override only
            # applies to FPU's area_t, not ALU/MUL (those always use
            # logic_scaling_co_eff, unconditionally).
            base = 4.47e6 if self._embedded else 8.47e6
            if self._node > 90:
                area_t = base * logic_scaling_co_eff
            else:
                area_t = base * (self._node**2 / 90.0**2)
            leak, gleak = leak_pair(fpu_w_n, fpu_w_p)
            self._leakage["FPU"] = area_t * leak * self._num_fpus
            self._gate_leakage["FPU"] = area_t * gleak * self._num_fpus
            self._longer_channel_leakage["FPU"] = (
                self._leakage["FPU"] * self._long_channel_reduction()
            )

        if self._has_alu:
            alu_w_n = 20 * min_w_nmos
            alu_w_p = 20 * min_w_nmos * p2n
            area_t = (
                280 * 260 * (1 if self._embedded else 2) * logic_scaling_co_eff
            )
            leak, gleak = leak_pair(alu_w_n, alu_w_p)
            self._leakage["ALU"] = area_t * leak * self._num_alus
            self._gate_leakage["ALU"] = area_t * gleak * self._num_alus
            self._longer_channel_leakage["ALU"] = (
                self._leakage["ALU"] * self._long_channel_reduction()
            )

        if self._has_mul:
            mul_w_n = 20 * min_w_nmos
            mul_w_p = 20 * min_w_nmos * p2n
            area_t = (
                280
                * 260
                * 3
                * (1 if self._embedded else 2)
                * logic_scaling_co_eff
            )
            leak, gleak = leak_pair(mul_w_n, mul_w_p)
            self._leakage["MUL"] = area_t * leak * self._num_muls
            self._gate_leakage["MUL"] = area_t * gleak * self._num_muls
            self._longer_channel_leakage["MUL"] = (
                self._leakage["MUL"] * self._long_channel_reduction()
            )

    def calc_fu_height(self):
        # Port of FunctionalUnit::FU_height (logic.cc:518,530,543,567,578,590):
        # a closed-form floorplan-height estimate ("from Sun's data") feeding
        # EXECU's bypass-network wire length (core.cc:1183-1300) -- NOT a
        # CACTI area quantity, just a fixed per-fu_type constant times
        # F_sz_um (self._node_um). ALU/MUL use the SAME constant in the
        # embedded and non-embedded branches (6222 / 9334); only FPU's
        # constant differs (18667 embedded / 38667 non-embedded).
        F_sz_um = self._node_um
        height = {"ALU": 0.0, "FPU": 0.0, "MUL": 0.0}
        if self._has_fpu:
            height["FPU"] = (
                (18667 if self._embedded else 38667) * self._num_fpus * F_sz_um
            )
        if self._has_alu:
            height["ALU"] = 6222 * self._num_alus * F_sz_um
        if self._has_mul:
            height["MUL"] = 9334 * self._num_muls * F_sz_um
        self._fu_height = height


class InstDecoder(CactiComponent):
    # opcode_length_ mirrors logic.cc's three real McPAT call sites (core.cc:278-291):
    # ID_inst uses coredynp.opcode_length, ID_operand uses coredynp.arch_ireg_width,
    # ID_misc uses the literal 8 -- none of them is micro_opcode_width, so the
    # caller must pass the width explicitly rather than this class guessing it.
    def __init__(
        self, name, cacti_params: CactiParams, opcode_length, x86=False
    ):
        super().__init__(name, cacti_params)
        self._cell_w = self._tp["cell_h_def"]
        self._cell_h = self._tp["cell_h_def"]
        self._x86 = x86

        # num_decoder_segments MUST be derived from the UNCLAMPED opcode_length
        # (logic.cc:1005-1007 computes ceil(opcode_length/18.0) before clamping
        # opcode_length itself to 18 for num_decoded_signals) -- doing the clamp
        # first (as this class used to) always yields segments==1.
        self._decoder_segments = int(ceil(opcode_length / 18.0))
        self._opcode_length = 18 if opcode_length > 18 else opcode_length
        self._num_dec_signals = int(2 ** (self._opcode_length))

        pmos_nmos_sizing = cacti_params.get_pmos_to_nmos_sz_ratio()
        load_nmos = self._tp["max_w_nmos"] / 2
        load_pmos = self._tp["max_w_nmos"] * pmos_nmos_sizing
        self._C_driver_load = 1024 * self.gate_C(load_nmos + load_pmos, 0)
        self._R_driver_load = 3000 * (self._node_um * self._wp["R_per_micron"])
        self._decoder = CactiDecoder(
            "Decoder",
            cacti_params,
            self._num_dec_signals,
            False,
            self._C_driver_load,
            self._R_driver_load,
            False,
            self._cell_h,
            self._cell_w,
        )
        self._predec1 = CactiPredecBlk(
            "PredecBlk1",
            cacti_params,
            self._num_dec_signals,
            self._decoder,
            0,
            0,
            1,
            True,
        )
        self._predec2 = CactiPredecBlk(
            "PredecBlk2",
            cacti_params,
            self._num_dec_signals,
            self._decoder,
            0,
            0,
            1,
            False,
        )
        self._predec_drv1 = CactiPredecBlkDrv(
            "PredecDrv1", cacti_params, 0, self._predec1
        )
        self._predec_drv2 = CactiPredecBlkDrv(
            "PredecDrv2", cacti_params, 0, self._predec2
        )
        self._predec = CactiPredec(
            "Predec", cacti_params, self._predec_drv1, self._predec_drv2
        )

        # logic.cc:1097-1104 (inst_decoder_delay_power): pre_dec and final_dec's
        # power get folded in via set_pppm/powerComponents::operator*, which
        # scales {dynamic, leakage, gate_leakage, short_circuit} independently
        # (io.cc:818-830) -- gate_leakage does NOT always follow leakage's scale
        # (pre_dec: gate_leakage scales like dynamic; final_dec: it scales like
        # leakage). Segments is the UNCLAMPED-derived value above.
        squencer_passes = 2 if self._x86 else 1
        segs = self._decoder_segments
        signals = self._num_dec_signals

        dynamic = self._predec._power.read.dynamic * (
            squencer_passes * segs
        ) + self._decoder._power.read.dynamic * (squencer_passes * segs)
        leakage = (
            self._predec._power.read.leakage * segs
            + self._decoder._power.read.leakage * (segs * signals)
        )
        gate_leakage = self._predec._power.read.gate_leakage * (
            squencer_passes * segs
        ) + self._decoder._power.read.gate_leakage * (segs * signals)

        self._power.read.dynamic = dynamic * self._tp["sckt_coeff"]
        self._power.read.leakage = leakage
        self._power.read.gate_leakage = gate_leakage
        # logic.cc:1074-1075 (inst_decoder constructor, right after the
        # leakage assignment above) -- Core_device unconditionally
        # (core.cc:278-291's three inst_decoder call sites).
        self.longer_channel_leakage = (
            self._power.read.leakage * self._long_channel_reduction()
        )


class Interconnect(CactiCircuit):
    """McPAT bypass interconnect coefficients, before operand weighting.

    wire_os_mat_type selects geometry; native Wire(Global) selects the
    delay-optimal repeater configuration. Dynamic energy alone receives
    sckt_co_eff. Machine composition weights static Int/Mul paths by two
    and FP paths by three. The native opt_local width/space-doubling timing
    loop is not modeled here; baseline width/space scaling is one."""

    def __init__(self, name, cacti_params: CactiParams, data_width, length_um):
        super().__init__(cacti_params)
        self._name = name
        self.data_width = data_width
        self.length_um = length_um

        embedded = self._machine_config._config_params["embedded"]
        wire_type_idx = 0 if embedded else 2
        wire_geom = self._wire_layer_geom(wire_type_idx)
        wire = self._wire_model(*wire_geom, overhead=0)

        sckt_coeff = self._tp["sckt_coeff"]
        self.dynamic = (
            wire["dynamic_per_um"] * length_um * data_width * sckt_coeff
        )
        self.leakage = wire["leak_per_um"] * length_um * data_width
        self.gate_leakage = wire["gleak_per_um"] * length_um * data_width
        # interconnect.cc:158-166/219-226 -- Core_device unconditionally
        # (core.cc's bypass-network construction, every call site).
        self.longer_channel_leakage = (
            self.leakage * self._long_channel_reduction()
        )
