# SPDX-License-Identifier: BSD-3-Clause
from dataclasses import dataclass
from math import (
    ceil,
    log,
    log2,
    sqrt,
)

from .cacti_circuit import CactiCircuit
from .cacti_params import CactiParams
from .cacti_primitives import (
    CactiComponent,
    ComponentPower,
    PowerStats,
)

VBITSENSEMIN = 0.08

# CAM matchline-stage threshold-crossing points (const.h:79-84), used only by
# CactiMat.compute_cam_delay's horowitz() chain -- distinct from the
# VTHCOMPINV/VTHEVALINV constants the (unported) regular tag comparator path
# would use.
VTHFA1 = 0.452
VTHFA2 = 0.304
VTHFA3 = 0.420
VTHFA4 = 0.413
VTHFA5 = 0.405
VTHFA6 = 0.452


class CactiDecoder(CactiComponent):
    def __init__(
        self,
        name,
        cacti_params,
        num_dec_signals,
        flag_way_select,
        C_ld_dec_out,
        R_wire_dec_out,
        is_wl_tr,
        cell_h,
        cell_w,
    ):
        super().__init__(name, cacti_params)

        self._num_dec_signals = num_dec_signals
        self._flag_way_select = flag_way_select
        self._C_ld_dec_out = C_ld_dec_out
        self._R_wire_dec_out = R_wire_dec_out
        self._is_wl_tr = is_wl_tr
        self._cell_h = cell_h
        self._cell_w = cell_w

        self._exist = False
        self._num_in_signals = 0
        self._num_gates_min = 2
        self._num_gates = 0
        self.delay = 0.0
        self.area_w = 0.0

        MAX_NUMBER_GATES_STAGE = 20
        self._w_dec_n = [0.0] * MAX_NUMBER_GATES_STAGE
        self._w_dec_p = [0.0] * MAX_NUMBER_GATES_STAGE

        # CACTI's _log2 is a bit-shift floor-log2 on an integer. Use bit_length
        # to reproduce it exactly and avoid float rounding (e.g. log2(1024)==9.999..).
        num_addr_bits_dec = (
            (int(self._num_dec_signals).bit_length() - 1)
            if self._num_dec_signals > 0
            else 0
        )

        if num_addr_bits_dec < 4:
            if self._flag_way_select:
                self._exist = True
                self._num_in_signals = 2
        else:
            self._exist = True
            self._num_in_signals = 3 if self._flag_way_select else 2

        assert self._cell_h > 0
        assert self._cell_w > 0
        self._area_h = self._tp["h_dec"] * self._cell_h

        self.compute_widths()
        self.compute_power()

    def compute_widths(self):
        if not self._exist:
            return

        p_to_n_sz_ratio = self.pmos_to_nmos_sz_ratio()
        gnand2 = (2 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio)
        gnand3 = (3 + p_to_n_sz_ratio) / (1 + p_to_n_sz_ratio)

        if self._num_in_signals == 2 or self._is_fa:
            self._w_dec_n[0] = 2 * self._tp["min_w_nmos"]
            self._w_dec_p[0] = p_to_n_sz_ratio * self._tp["min_w_nmos"]
            F = gnand2
        else:
            self._w_dec_n[0] = 3 * self._tp["min_w_nmos"]
            self._w_dec_p[0] = p_to_n_sz_ratio * self._tp["min_w_nmos"]
            F = gnand3

        gate_C_n = self.gate_C(
            self._w_dec_n[0], 0, is_dram=self._is_dram, is_wl_tr=self._is_wl_tr
        )
        gate_C_p = self.gate_C(
            self._w_dec_p[0], 0, is_dram=self._is_dram, is_wl_tr=self._is_wl_tr
        )
        F *= self._C_ld_dec_out / (gate_C_n + gate_C_p)

        self._num_gates = self.logical_effort(
            self._num_gates_min,
            gnand2 if (self._num_in_signals == 2 or self._is_fa) else gnand3,
            F,
            self._w_dec_n,
            self._w_dec_p,
            self._C_ld_dec_out,
            p_to_n_sz_ratio,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
            max_w_nmos=self._tp["max_w_nmos_dec"],
        )

    def dynamic_power(self):
        if not self._exist:
            return

        Vdd = self._tp["Vdd"]
        # Wordline transistors are driven to a boosted rail in CACTI (g_tp.vpp for
        # DRAM, g_tp.sram_cell.Vdd for SRAM). This single-Vdd port does not model a
        # separate cell / boosted supply, so the wordline swing uses Vdd.
        Vpp = Vdd

        # Stage 0 NAND
        c_load = self.gate_C(
            self._w_dec_n[1] + self._w_dec_p[1],
            0.0,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        c_intrinsic = self.drain_C(
            self._w_dec_p[0],
            "PCH",
            1,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        ) * self._num_in_signals + self.drain_C(
            self._w_dec_n[0],
            "NCH",
            self._num_in_signals,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        self._power.read.dynamic += (c_load + c_intrinsic) * Vdd * Vdd

        # Inverter chain. decoder.cc uses area.h (== h_dec * cell.h) as the
        # drain fold dimension for EVERY stage, not cell_h_def, so use self._area_h.
        for i in range(1, self._num_gates - 1):
            c_load = self.gate_C(
                self._w_dec_p[i + 1] + self._w_dec_n[i + 1],
                0.0,
                is_dram=self._is_dram,
                is_wl_tr=self._is_wl_tr,
            )
            c_intrinsic = self.drain_C(
                self._w_dec_p[i],
                "PCH",
                1,
                1,
                self._area_h,
                is_dram=self._is_dram,
                is_wl_tr=self._is_wl_tr,
            ) + self.drain_C(
                self._w_dec_n[i],
                "NCH",
                1,
                1,
                self._area_h,
                is_dram=self._is_dram,
                is_wl_tr=self._is_wl_tr,
            )
            self._power.read.dynamic += (c_load + c_intrinsic) * Vdd * Vdd

        # Final stage
        i = self._num_gates - 1
        c_intrinsic = self.drain_C(
            self._w_dec_p[i],
            "PCH",
            1,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        ) + self.drain_C(
            self._w_dec_n[i],
            "NCH",
            1,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        self._power.read.dynamic += (
            c_intrinsic * Vdd * Vdd + self._C_ld_dec_out * Vpp * Vpp
        )

    def static_power(self):
        if not self._exist:
            return

        cumulative_curr = 0.0
        cumulative_curr_Ig = 0.0

        if self._num_in_signals == 2:
            cumulative_curr = self.cmos_Isub_leakage(
                self._w_dec_n[0],
                self._w_dec_p[0],
                2,
                "NAND",
                is_dram=self._is_dram,
            )
            cumulative_curr_Ig = self.cmos_Ig_leakage(
                self._w_dec_n[0],
                self._w_dec_p[0],
                2,
                "NAND",
                is_dram=self._is_dram,
            )
        elif self._num_in_signals == 3:
            cumulative_curr = self.cmos_Isub_leakage(
                self._w_dec_n[0],
                self._w_dec_p[0],
                3,
                "NAND",
                is_dram=self._is_dram,
            )
            cumulative_curr_Ig = self.cmos_Ig_leakage(
                self._w_dec_n[0],
                self._w_dec_p[0],
                3,
                "NAND",
                is_dram=self._is_dram,
            )

        for i in range(1, self._num_gates):
            cumulative_curr += self.cmos_Isub_leakage(
                self._w_dec_n[i],
                self._w_dec_p[i],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            # decoder.cc uses '=' (not '+=') for the gate leakage here, so only the
            # LAST inverter's gate leakage is kept. This is a CACTI bug, replicated
            # verbatim (the subthreshold current above correctly accumulates).
            cumulative_curr_Ig = self.cmos_Ig_leakage(
                self._w_dec_n[i],
                self._w_dec_p[i],
                1,
                "INV",
                is_dram=self._is_dram,
            )

        self._power.read.leakage = cumulative_curr * self._tp["Vdd"]
        self._power.read.gate_leakage = cumulative_curr_Ig * self._tp["Vdd"]

    def compute_delays(self, inrisetime):
        # Port of Decoder::compute_delays (decoder.cc:233-301). Not called
        # from anywhere yet (Phase 28 -- timing/area additive, not wired
        # into CactiMat/the energy pipeline; see PROGRESS.md).
        if not self._exist:
            return 0.0

        # Stage 0 NAND
        rd = self.tr_R_on(
            self._w_dec_n[0],
            "NCH",
            self._num_in_signals,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        c_load = self.gate_C(
            self._w_dec_n[1] + self._w_dec_p[1],
            0.0,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        c_intrinsic = self.drain_C(
            self._w_dec_p[0],
            "PCH",
            1,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        ) * self._num_in_signals + self.drain_C(
            self._w_dec_n[0],
            "NCH",
            self._num_in_signals,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        tf = rd * (c_intrinsic + c_load)
        this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
        self.delay += this_delay
        inrisetime = this_delay / (1.0 - 0.5)

        # Inverter chain (area.h drain fold dimension, matching dynamic_power).
        for i in range(1, self._num_gates - 1):
            rd = self.tr_R_on(
                self._w_dec_n[i],
                "NCH",
                1,
                is_dram=self._is_dram,
                is_wl_tr=self._is_wl_tr,
            )
            c_load = self.gate_C(
                self._w_dec_p[i + 1] + self._w_dec_n[i + 1],
                0.0,
                is_dram=self._is_dram,
                is_wl_tr=self._is_wl_tr,
            )
            c_intrinsic = self.drain_C(
                self._w_dec_p[i],
                "PCH",
                1,
                1,
                self._area_h,
                is_dram=self._is_dram,
                is_wl_tr=self._is_wl_tr,
            ) + self.drain_C(
                self._w_dec_n[i],
                "NCH",
                1,
                1,
                self._area_h,
                is_dram=self._is_dram,
                is_wl_tr=self._is_wl_tr,
            )
            tf = rd * (c_intrinsic + c_load)
            this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
            self.delay += this_delay
            inrisetime = this_delay / (1.0 - 0.5)

        # Final stage driving the external load (+ distributed wire-R term).
        i = self._num_gates - 1
        c_load = self._C_ld_dec_out
        rd = self.tr_R_on(
            self._w_dec_n[i],
            "NCH",
            1,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        c_intrinsic = self.drain_C(
            self._w_dec_p[i],
            "PCH",
            1,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        ) + self.drain_C(
            self._w_dec_n[i],
            "NCH",
            1,
            1,
            self._area_h,
            is_dram=self._is_dram,
            is_wl_tr=self._is_wl_tr,
        )
        tf = rd * (c_intrinsic + c_load) + self._R_wire_dec_out * c_load / 2
        this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
        self.delay += this_delay
        ret_val = this_delay / (1.0 - 0.5)
        return ret_val

    def compute_area(self):
        # Port of Decoder::compute_area (decoder.cc:164-203). power_gating
        # (Sleep_tx) is never enabled by this project's configs -- out of
        # scope, matching the energy side (no power-gating anywhere in this
        # port).
        if not self._exist:
            return

        if self._num_in_signals == 2:
            cumulative_area = self.compute_gate_area(
                "NAND", 2, self._w_dec_p[0], self._w_dec_n[0], self._area_h
            )
        elif self._num_in_signals == 3:
            cumulative_area = self.compute_gate_area(
                "NAND", 3, self._w_dec_p[0], self._w_dec_n[0], self._area_h
            )
        else:
            cumulative_area = 0.0

        for i in range(1, self._num_gates):
            cumulative_area += self.compute_gate_area(
                "INV", 1, self._w_dec_p[i], self._w_dec_n[i], self._area_h
            )

        self.area_w = cumulative_area / self._area_h


class CactiPredecBlk(CactiComponent):
    def __init__(
        self,
        name,
        cacti_params,
        num_dec_signals,
        dec,  # dec: CactiDecoder
        C_wire_predec_blk_out,
        R_wire_predec_blk_out,
        num_dec_per_predec,
        is_blk1,
    ):
        super().__init__(name, cacti_params)

        self.dec = dec
        self.exist = False
        self.is_blk1 = is_blk1

        self.power_nand2_path = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self.power_nand3_path = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self.power_L2 = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )

        num_addr_bits_dec = (
            int(num_dec_signals.bit_length() - 1) if num_dec_signals > 0 else 0
        )
        blk1_num_input_addr_bits = (num_addr_bits_dec + 1) // 2
        blk2_num_input_addr_bits = num_addr_bits_dec - blk1_num_input_addr_bits

        self.w_L1_nand2_n = [0.0] * 20
        self.w_L1_nand2_p = [0.0] * 20
        self.w_L1_nand3_n = [0.0] * 20
        self.w_L1_nand3_p = [0.0] * 20
        self.w_L2_n = [0.0] * 20
        self.w_L2_p = [0.0] * 20

        self.number_inputs_L1_gate = 0
        self.flag_two_unique_paths = False
        self.flag_L2_gate = 0
        self.number_gates_L1_nand2_path = 0
        self.number_gates_L1_nand3_path = 0
        self.number_gates_L2 = 0
        self.num_L1_active_nand2_path = 0
        self.num_L1_active_nand3_path = 0
        self.num_L1_nand2 = 0
        self.num_L1_nand3 = 0
        self.num_L2 = 0
        self.branch_effort_nand2_gate_output = 1
        self.branch_effort_nand3_gate_output = 1
        self.min_number_gates = 2
        self.C_ld_predec_blk_out = 0.0
        self.R_wire_predec_blk_out = 0.0
        self.number_input_addr_bits = 0
        self.delay_nand2_path = 0.0
        self.delay_nand3_path = 0.0
        self.delay = 0.0
        self.area = 0.0

        if self.is_blk1:
            if num_addr_bits_dec <= 0:
                return
            elif num_addr_bits_dec < 4:
                self.exist = True
                self.number_input_addr_bits = num_addr_bits_dec
                self.R_wire_predec_blk_out = self.dec._R_wire_dec_out
                self.C_ld_predec_blk_out = self.dec._C_ld_dec_out
            else:
                self.exist = True
                self.number_input_addr_bits = blk1_num_input_addr_bits
                branch_effort_predec_out = 1 << blk2_num_input_addr_bits
                C_ld_dec_gate = num_dec_per_predec * self.gate_C(
                    self.dec._w_dec_n[0] + self.dec._w_dec_p[0],
                    0,
                    is_dram=self._is_dram,
                    is_wl_tr=False,
                )
                self.R_wire_predec_blk_out = R_wire_predec_blk_out
                self.C_ld_predec_blk_out = (
                    branch_effort_predec_out * C_ld_dec_gate
                    + C_wire_predec_blk_out
                )
        else:
            if num_addr_bits_dec >= 4:
                self.exist = True
                self.number_input_addr_bits = blk2_num_input_addr_bits
                branch_effort_predec_out = 1 << blk1_num_input_addr_bits
                C_ld_dec_gate = num_dec_per_predec * self.gate_C(
                    self.dec._w_dec_n[0] + self.dec._w_dec_p[0],
                    0,
                    is_dram=self._is_dram,
                    is_wl_tr=False,
                )
                self.R_wire_predec_blk_out = R_wire_predec_blk_out
                self.C_ld_predec_blk_out = (
                    branch_effort_predec_out * C_ld_dec_gate
                    + C_wire_predec_blk_out
                )

        self.compute_widths()
        self.compute_power()

    def compute_widths(self):
        # Faithful port of PredecBlk::compute_widths (decoder.cc): sizes the L2
        # gate chain and the L1 NAND2 / NAND3 gate chains with logical_effort.
        if not self.exist:
            return

        p_to_n = self.pmos_to_nmos_sz_ratio()
        gnand2 = (2 + p_to_n) / (1 + p_to_n)
        gnand3 = (3 + p_to_n) / (1 + p_to_n)
        min_w = self._tp["min_w_nmos"]
        max_w = self._tp["max_w_nmos"]

        # number_input_addr_bits -> path structure + branch efforts + active counts
        # num_L1_active_* weight the DYNAMIC energy (active gates per access);
        # num_L1_nand2/nand3/num_L2 are the TOTAL gate counts weighting LEAKAGE
        # (PredecBlk::compute_area switch, decoder.cc).
        n = self.number_input_addr_bits
        if n == 1:
            self.number_inputs_L1_gate = 2
            self.num_L1_active_nand2_path = 1
            self.num_L1_nand2 = 2
        elif n == 2:
            self.number_inputs_L1_gate = 2
            self.num_L1_active_nand2_path = 1
            self.num_L1_nand2 = 4
        elif n == 3:
            self.number_inputs_L1_gate = 3
            self.num_L1_active_nand3_path = 1
            self.num_L1_nand3 = 8
        elif n == 4:
            self.number_inputs_L1_gate = 2
            self.flag_L2_gate = 2
            self.branch_effort_nand2_gate_output = 4
            self.num_L1_active_nand2_path = 2
            self.num_L1_nand2 = 8
            self.num_L2 = 16
        elif n == 5:
            self.flag_two_unique_paths = True
            self.flag_L2_gate = 2
            self.branch_effort_nand2_gate_output = 8
            self.branch_effort_nand3_gate_output = 4
            self.num_L1_active_nand2_path = 1
            self.num_L1_active_nand3_path = 1
            self.num_L1_nand2 = 4
            self.num_L1_nand3 = 8
            self.num_L2 = 32
        elif n == 6:
            self.number_inputs_L1_gate = 3
            self.flag_L2_gate = 2
            self.branch_effort_nand3_gate_output = 8
            self.num_L1_active_nand3_path = 2
            self.num_L1_nand3 = 16
            self.num_L2 = 64
        elif n == 7:
            self.flag_two_unique_paths = True
            self.flag_L2_gate = 3
            self.branch_effort_nand2_gate_output = 32
            self.branch_effort_nand3_gate_output = 16
            self.num_L1_active_nand2_path = 2
            self.num_L1_active_nand3_path = 1
            self.num_L1_nand2 = 8
            self.num_L1_nand3 = 8
            self.num_L2 = 128
        elif n == 8:
            self.flag_two_unique_paths = True
            self.flag_L2_gate = 3
            self.branch_effort_nand2_gate_output = 64
            self.branch_effort_nand3_gate_output = 32
            self.num_L1_active_nand2_path = 2
            self.num_L1_active_nand3_path = 2
            self.num_L1_nand2 = 4
            self.num_L1_nand3 = 16
            self.num_L2 = 256
        elif n == 9:
            self.number_inputs_L1_gate = 3
            self.flag_L2_gate = 3
            self.branch_effort_nand3_gate_output = 64
            self.num_L1_active_nand3_path = 3
            self.num_L1_nand3 = 24
            self.num_L2 = 512

        if self.flag_L2_gate:
            if self.flag_L2_gate == 2:
                self.w_L2_n[0] = 2 * min_w
                F = gnand2
            else:
                self.w_L2_n[0] = 3 * min_w
                F = gnand3
            self.w_L2_p[0] = p_to_n * min_w
            F *= self.C_ld_predec_blk_out / (
                self.gate_C(self.w_L2_n[0], 0, is_dram=self._is_dram)
                + self.gate_C(self.w_L2_p[0], 0, is_dram=self._is_dram)
            )
            self.number_gates_L2 = self.logical_effort(
                self.min_number_gates,
                gnand2 if self.flag_L2_gate == 2 else gnand3,
                F,
                self.w_L2_n,
                self.w_L2_p,
                self.C_ld_predec_blk_out,
                p_to_n,
                is_dram=self._is_dram,
                is_wl_tr=False,
                max_w_nmos=max_w,
            )

            if self.flag_two_unique_paths or self.number_inputs_L1_gate == 2:
                c_load_nand2_path = self.branch_effort_nand2_gate_output * (
                    self.gate_C(self.w_L2_n[0], 0, is_dram=self._is_dram)
                    + self.gate_C(self.w_L2_p[0], 0, is_dram=self._is_dram)
                )
                self.w_L1_nand2_n[0] = 2 * min_w
                self.w_L1_nand2_p[0] = p_to_n * min_w
                F = (
                    gnand2
                    * c_load_nand2_path
                    / (
                        self.gate_C(
                            self.w_L1_nand2_n[0], 0, is_dram=self._is_dram
                        )
                        + self.gate_C(
                            self.w_L1_nand2_p[0], 0, is_dram=self._is_dram
                        )
                    )
                )
                self.number_gates_L1_nand2_path = self.logical_effort(
                    self.min_number_gates,
                    gnand2,
                    F,
                    self.w_L1_nand2_n,
                    self.w_L1_nand2_p,
                    c_load_nand2_path,
                    p_to_n,
                    is_dram=self._is_dram,
                    is_wl_tr=False,
                    max_w_nmos=max_w,
                )

            if self.flag_two_unique_paths or self.number_inputs_L1_gate == 3:
                c_load_nand3_path = self.branch_effort_nand3_gate_output * (
                    self.gate_C(self.w_L2_n[0], 0, is_dram=self._is_dram)
                    + self.gate_C(self.w_L2_p[0], 0, is_dram=self._is_dram)
                )
                self.w_L1_nand3_n[0] = 3 * min_w
                self.w_L1_nand3_p[0] = p_to_n * min_w
                F = (
                    gnand3
                    * c_load_nand3_path
                    / (
                        self.gate_C(
                            self.w_L1_nand3_n[0], 0, is_dram=self._is_dram
                        )
                        + self.gate_C(
                            self.w_L1_nand3_p[0], 0, is_dram=self._is_dram
                        )
                    )
                )
                self.number_gates_L1_nand3_path = self.logical_effort(
                    self.min_number_gates,
                    gnand3,
                    F,
                    self.w_L1_nand3_n,
                    self.w_L1_nand3_p,
                    c_load_nand3_path,
                    p_to_n,
                    is_dram=self._is_dram,
                    is_wl_tr=False,
                    max_w_nmos=max_w,
                )
        else:
            if self.number_inputs_L1_gate == 2:
                self.w_L1_nand2_n[0] = 2 * min_w
                self.w_L1_nand2_p[0] = p_to_n * min_w
                F = (
                    gnand2
                    * self.C_ld_predec_blk_out
                    / (
                        self.gate_C(
                            self.w_L1_nand2_n[0], 0, is_dram=self._is_dram
                        )
                        + self.gate_C(
                            self.w_L1_nand2_p[0], 0, is_dram=self._is_dram
                        )
                    )
                )
                self.number_gates_L1_nand2_path = self.logical_effort(
                    self.min_number_gates,
                    gnand2,
                    F,
                    self.w_L1_nand2_n,
                    self.w_L1_nand2_p,
                    self.C_ld_predec_blk_out,
                    p_to_n,
                    is_dram=self._is_dram,
                    is_wl_tr=False,
                    max_w_nmos=max_w,
                )
            elif self.number_inputs_L1_gate == 3:
                self.w_L1_nand3_n[0] = 3 * min_w
                self.w_L1_nand3_p[0] = p_to_n * min_w
                F = (
                    gnand3
                    * self.C_ld_predec_blk_out
                    / (
                        self.gate_C(
                            self.w_L1_nand3_n[0], 0, is_dram=self._is_dram
                        )
                        + self.gate_C(
                            self.w_L1_nand3_p[0], 0, is_dram=self._is_dram
                        )
                    )
                )
                self.number_gates_L1_nand3_path = self.logical_effort(
                    self.min_number_gates,
                    gnand3,
                    F,
                    self.w_L1_nand3_n,
                    self.w_L1_nand3_p,
                    self.C_ld_predec_blk_out,
                    p_to_n,
                    is_dram=self._is_dram,
                    is_wl_tr=False,
                    max_w_nmos=max_w,
                )

    def dynamic_power(self):
        # Faithful port of PredecBlk::compute_delays energy terms (no timing).
        if not self.exist:
            return
        Vdd = self._tp["Vdd"]
        cell_h_def = self._tp["cell_h_def"]

        # ---- L1 NAND2 path ----
        if self.flag_two_unique_paths or self.number_inputs_L1_gate == 2:
            c_load = self.gate_C(
                self.w_L1_nand2_n[1] + self.w_L1_nand2_p[1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = 2 * self.drain_C(
                self.w_L1_nand2_p[0],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.w_L1_nand2_n[0],
                "NCH",
                2,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand2_path.read.dynamic += (
                (c_load + c_intrinsic) * Vdd * Vdd
            )
            for i in range(1, self.number_gates_L1_nand2_path - 1):
                c_load = self.gate_C(
                    self.w_L1_nand2_n[i + 1] + self.w_L1_nand2_p[i + 1],
                    0.0,
                    is_dram=self._is_dram,
                )
                c_intrinsic = self.drain_C(
                    self.w_L1_nand2_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand2_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                self.power_nand2_path.read.dynamic += (
                    (c_intrinsic + c_load) * Vdd * Vdd
                )
            i = self.number_gates_L1_nand2_path - 1
            if self.flag_L2_gate:
                c_load = self.branch_effort_nand2_gate_output * (
                    self.gate_C(self.w_L2_n[0], 0, is_dram=self._is_dram)
                    + self.gate_C(self.w_L2_p[0], 0, is_dram=self._is_dram)
                )
            else:
                c_load = self.C_ld_predec_blk_out
            c_intrinsic = self.drain_C(
                self.w_L1_nand2_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.w_L1_nand2_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand2_path.read.dynamic += (
                (c_intrinsic + c_load) * Vdd * Vdd
            )

        # ---- L1 NAND3 path ----
        if self.flag_two_unique_paths or self.number_inputs_L1_gate == 3:
            c_load = self.gate_C(
                self.w_L1_nand3_n[1] + self.w_L1_nand3_p[1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = 3 * self.drain_C(
                self.w_L1_nand3_p[0],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.w_L1_nand3_n[0],
                "NCH",
                3,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand3_path.read.dynamic += (
                (c_intrinsic + c_load) * Vdd * Vdd
            )
            for i in range(1, self.number_gates_L1_nand3_path - 1):
                c_load = self.gate_C(
                    self.w_L1_nand3_n[i + 1] + self.w_L1_nand3_p[i + 1],
                    0.0,
                    is_dram=self._is_dram,
                )
                c_intrinsic = self.drain_C(
                    self.w_L1_nand3_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand3_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                self.power_nand3_path.read.dynamic += (
                    (c_intrinsic + c_load) * Vdd * Vdd
                )
            i = self.number_gates_L1_nand3_path - 1
            if self.flag_L2_gate:
                c_load = self.branch_effort_nand3_gate_output * (
                    self.gate_C(self.w_L2_n[0], 0, is_dram=self._is_dram)
                    + self.gate_C(self.w_L2_p[0], 0, is_dram=self._is_dram)
                )
            else:
                c_load = self.C_ld_predec_blk_out
            c_intrinsic = self.drain_C(
                self.w_L1_nand3_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.w_L1_nand3_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand3_path.read.dynamic += (
                (c_intrinsic + c_load) * Vdd * Vdd
            )

        # ---- L2 path ----
        if self.flag_L2_gate:
            c_load = self.gate_C(
                self.w_L2_n[1] + self.w_L2_p[1], 0.0, is_dram=self._is_dram
            )
            if self.flag_L2_gate == 2:
                c_intrinsic = 2 * self.drain_C(
                    self.w_L2_p[0],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L2_n[0],
                    "NCH",
                    2,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
            else:
                c_intrinsic = 3 * self.drain_C(
                    self.w_L2_p[0],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L2_n[0],
                    "NCH",
                    3,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
            self.power_L2.read.dynamic += (c_intrinsic + c_load) * Vdd * Vdd
            for i in range(1, self.number_gates_L2 - 1):
                c_load = self.gate_C(
                    self.w_L2_n[i + 1] + self.w_L2_p[i + 1],
                    0.0,
                    is_dram=self._is_dram,
                )
                c_intrinsic = self.drain_C(
                    self.w_L2_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L2_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                self.power_L2.read.dynamic += (
                    (c_intrinsic + c_load) * Vdd * Vdd
                )
            i = self.number_gates_L2 - 1
            c_load = self.C_ld_predec_blk_out
            c_intrinsic = self.drain_C(
                self.w_L2_p[i], "PCH", 1, 1, cell_h_def, is_dram=self._is_dram
            ) + self.drain_C(
                self.w_L2_n[i], "NCH", 1, 1, cell_h_def, is_dram=self._is_dram
            )
            self.power_L2.read.dynamic += (c_intrinsic + c_load) * Vdd * Vdd

    def static_power(self):
        # Faithful port of PredecBlk::leakage_feedback (per-gate, NOT multiplied
        # by the num_L1_nand counts -- Predec applies num_L1_active weighting).
        if not self.exist:
            return
        Vdd = self._tp["Vdd"]

        # CACTI bug (decoder.cc:960-969), replicated verbatim: the nand2-path
        # seed gate (i=0) leakage is ALWAYS computed, even when the block has no
        # nand2 path (harmless there -- num_L1_nand2 is 0 so it's multiplied
        # away). The nand3-path seed gate is only computed when
        # number_inputs_L1_gate==3; for the "two unique paths" cases (n=5,7,8)
        # number_inputs_L1_gate is left at its default 0, so the seed nand3
        # gate's leakage is silently dropped and the sum loop below starts at
        # i=1, missing gate 0 entirely.
        leak2 = self.cmos_Isub_leakage(
            self.w_L1_nand2_n[0],
            self.w_L1_nand2_p[0],
            2,
            "NAND",
            is_dram=self._is_dram,
        )
        gleak2 = self.cmos_Ig_leakage(
            self.w_L1_nand2_n[0],
            self.w_L1_nand2_p[0],
            2,
            "NAND",
            is_dram=self._is_dram,
        )
        if self.number_inputs_L1_gate == 3:
            leak3 = self.cmos_Isub_leakage(
                self.w_L1_nand3_n[0],
                self.w_L1_nand3_p[0],
                3,
                "NAND",
                is_dram=self._is_dram,
            )
            gleak3 = self.cmos_Ig_leakage(
                self.w_L1_nand3_n[0],
                self.w_L1_nand3_p[0],
                3,
                "NAND",
                is_dram=self._is_dram,
            )
        else:
            leak3 = gleak3 = 0.0

        for i in range(1, self.number_gates_L1_nand2_path):
            leak2 += self.cmos_Isub_leakage(
                self.w_L1_nand2_n[i],
                self.w_L1_nand2_p[i],
                2,
                "NAND",
                is_dram=self._is_dram,
            )
            gleak2 += self.cmos_Ig_leakage(
                self.w_L1_nand2_n[i],
                self.w_L1_nand2_p[i],
                2,
                "NAND",
                is_dram=self._is_dram,
            )
        self.power_nand2_path.read.leakage = leak2 * self.num_L1_nand2 * Vdd
        self.power_nand2_path.read.gate_leakage = (
            gleak2 * self.num_L1_nand2 * Vdd
        )

        for i in range(1, self.number_gates_L1_nand3_path):
            leak3 += self.cmos_Isub_leakage(
                self.w_L1_nand3_n[i],
                self.w_L1_nand3_p[i],
                3,
                "NAND",
                is_dram=self._is_dram,
            )
            gleak3 += self.cmos_Ig_leakage(
                self.w_L1_nand3_n[i],
                self.w_L1_nand3_p[i],
                3,
                "NAND",
                is_dram=self._is_dram,
            )
        self.power_nand3_path.read.leakage = leak3 * self.num_L1_nand3 * Vdd
        self.power_nand3_path.read.gate_leakage = (
            gleak3 * self.num_L1_nand3 * Vdd
        )

        if self.flag_L2_gate == 2:
            leak = self.cmos_Isub_leakage(
                self.w_L2_n[0],
                self.w_L2_p[0],
                2,
                "NAND",
                is_dram=self._is_dram,
            )
            gleak = self.cmos_Ig_leakage(
                self.w_L2_n[0],
                self.w_L2_p[0],
                2,
                "NAND",
                is_dram=self._is_dram,
            )
        elif self.flag_L2_gate == 3:
            leak = self.cmos_Isub_leakage(
                self.w_L2_n[0],
                self.w_L2_p[0],
                3,
                "NAND",
                is_dram=self._is_dram,
            )
            gleak = self.cmos_Ig_leakage(
                self.w_L2_n[0],
                self.w_L2_p[0],
                3,
                "NAND",
                is_dram=self._is_dram,
            )
        else:
            leak = gleak = 0.0
        for i in range(1, self.number_gates_L2):
            leak += self.cmos_Isub_leakage(
                self.w_L2_n[i], self.w_L2_p[i], 2, "INV", is_dram=self._is_dram
            )
            gleak += self.cmos_Ig_leakage(
                self.w_L2_n[i], self.w_L2_p[i], 2, "INV", is_dram=self._is_dram
            )
        self.power_L2.read.leakage = leak * self.num_L2 * Vdd
        self.power_L2.read.gate_leakage = gleak * self.num_L2 * Vdd

    def compute_delays(self, inrisetime):
        # Faithful port of PredecBlk::compute_delays (decoder.cc:759-949),
        # timing terms only (energy already in dynamic_power()). Not called
        # from anywhere yet -- Phase 28, additive only (see PROGRESS.md).
        #
        # inrisetime: (inrise_nand2_path, inrise_nand3_path) tuple.
        # Returns:    (outrise_nand2_path, outrise_nand3_path) tuple.
        #
        # NOTE the real asymmetry in the L2 stage (replicated verbatim, not a
        # simplification error): the FIRST L2 gate only advances the ONE
        # rise-time matching flag_L2_gate's own NAND2/NAND3 type (there is
        # physically only one first L2 gate, structurally fed by whichever L1
        # path connects to it) -- but every L2 stage AFTER that advances BOTH
        # inrise_nand2_path and inrise_nand3_path unconditionally (decoder.cc
        # models both as independent notional signals through the same
        # physical L2 chain sizing from that point on).
        ret_first = 0.0
        ret_second = 0.0
        if not self.exist:
            self.delay = max(ret_first, ret_second)
            return (ret_first, ret_second)

        cell_h_def = self._tp["cell_h_def"]
        inrise_nand2, inrise_nand3 = inrisetime

        if self.flag_two_unique_paths or self.number_inputs_L1_gate == 2:
            rd = self.tr_R_on(
                self.w_L1_nand2_n[0], "NCH", 2, is_dram=self._is_dram
            )
            c_load = self.gate_C(
                self.w_L1_nand2_n[1] + self.w_L1_nand2_p[1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = 2 * self.drain_C(
                self.w_L1_nand2_p[0],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.w_L1_nand2_n[0],
                "NCH",
                2,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            tf = rd * (c_intrinsic + c_load)
            this_delay = self.horowitz(inrise_nand2, tf, 0.5, 0.5, 1)
            self.delay_nand2_path += this_delay
            inrise_nand2 = this_delay / (1.0 - 0.5)

            for i in range(1, self.number_gates_L1_nand2_path - 1):
                rd = self.tr_R_on(
                    self.w_L1_nand2_n[i], "NCH", 1, is_dram=self._is_dram
                )
                c_load = self.gate_C(
                    self.w_L1_nand2_n[i + 1] + self.w_L1_nand2_p[i + 1],
                    0.0,
                    is_dram=self._is_dram,
                )
                c_intrinsic = self.drain_C(
                    self.w_L1_nand2_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand2_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = rd * (c_intrinsic + c_load)
                this_delay = self.horowitz(inrise_nand2, tf, 0.5, 0.5, 1)
                self.delay_nand2_path += this_delay
                inrise_nand2 = this_delay / (1.0 - 0.5)

            i = self.number_gates_L1_nand2_path - 1
            rd = self.tr_R_on(
                self.w_L1_nand2_n[i], "NCH", 1, is_dram=self._is_dram
            )
            if self.flag_L2_gate:
                c_load = self.branch_effort_nand2_gate_output * (
                    self.gate_C(self.w_L2_n[0], 0, is_dram=self._is_dram)
                    + self.gate_C(self.w_L2_p[0], 0, is_dram=self._is_dram)
                )
                c_intrinsic = self.drain_C(
                    self.w_L1_nand2_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand2_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = rd * (c_intrinsic + c_load)
                this_delay = self.horowitz(inrise_nand2, tf, 0.5, 0.5, 1)
                self.delay_nand2_path += this_delay
                inrise_nand2 = this_delay / (1.0 - 0.5)
            else:
                c_load = self.C_ld_predec_blk_out
                c_intrinsic = self.drain_C(
                    self.w_L1_nand2_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand2_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = (
                    rd * (c_intrinsic + c_load)
                    + self.R_wire_predec_blk_out * c_load / 2
                )
                this_delay = self.horowitz(inrise_nand2, tf, 0.5, 0.5, 1)
                self.delay_nand2_path += this_delay
                ret_first = this_delay / (1.0 - 0.5)

        if self.flag_two_unique_paths or self.number_inputs_L1_gate == 3:
            rd = self.tr_R_on(
                self.w_L1_nand3_n[0], "NCH", 3, is_dram=self._is_dram
            )
            c_load = self.gate_C(
                self.w_L1_nand3_n[1] + self.w_L1_nand3_p[1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = 3 * self.drain_C(
                self.w_L1_nand3_p[0],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.w_L1_nand3_n[0],
                "NCH",
                3,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            tf = rd * (c_intrinsic + c_load)
            this_delay = self.horowitz(inrise_nand3, tf, 0.5, 0.5, 1)
            self.delay_nand3_path += this_delay
            inrise_nand3 = this_delay / (1.0 - 0.5)

            for i in range(1, self.number_gates_L1_nand3_path - 1):
                rd = self.tr_R_on(
                    self.w_L1_nand3_n[i], "NCH", 1, is_dram=self._is_dram
                )
                c_load = self.gate_C(
                    self.w_L1_nand3_n[i + 1] + self.w_L1_nand3_p[i + 1],
                    0.0,
                    is_dram=self._is_dram,
                )
                c_intrinsic = self.drain_C(
                    self.w_L1_nand3_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand3_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = rd * (c_intrinsic + c_load)
                this_delay = self.horowitz(inrise_nand3, tf, 0.5, 0.5, 1)
                self.delay_nand3_path += this_delay
                inrise_nand3 = this_delay / (1.0 - 0.5)

            i = self.number_gates_L1_nand3_path - 1
            rd = self.tr_R_on(
                self.w_L1_nand3_n[i], "NCH", 1, is_dram=self._is_dram
            )
            if self.flag_L2_gate:
                c_load = self.branch_effort_nand3_gate_output * (
                    self.gate_C(self.w_L2_n[0], 0, is_dram=self._is_dram)
                    + self.gate_C(self.w_L2_p[0], 0, is_dram=self._is_dram)
                )
                c_intrinsic = self.drain_C(
                    self.w_L1_nand3_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand3_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = rd * (c_intrinsic + c_load)
                this_delay = self.horowitz(inrise_nand3, tf, 0.5, 0.5, 1)
                self.delay_nand3_path += this_delay
                inrise_nand3 = this_delay / (1.0 - 0.5)
            else:
                c_load = self.C_ld_predec_blk_out
                c_intrinsic = self.drain_C(
                    self.w_L1_nand3_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L1_nand3_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = (
                    rd * (c_intrinsic + c_load)
                    + self.R_wire_predec_blk_out * c_load / 2
                )
                this_delay = self.horowitz(inrise_nand3, tf, 0.5, 0.5, 1)
                self.delay_nand3_path += this_delay
                ret_second = this_delay / (1.0 - 0.5)

        if self.flag_L2_gate:
            if self.flag_L2_gate == 2:
                rd = self.tr_R_on(
                    self.w_L2_n[0], "NCH", 2, is_dram=self._is_dram
                )
                c_load = self.gate_C(
                    self.w_L2_n[1] + self.w_L2_p[1], 0.0, is_dram=self._is_dram
                )
                c_intrinsic = 2 * self.drain_C(
                    self.w_L2_p[0],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L2_n[0],
                    "NCH",
                    2,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = rd * (c_intrinsic + c_load)
                this_delay = self.horowitz(inrise_nand2, tf, 0.5, 0.5, 1)
                self.delay_nand2_path += this_delay
                inrise_nand2 = this_delay / (1.0 - 0.5)
            else:  # flag_L2_gate == 3
                rd = self.tr_R_on(
                    self.w_L2_n[0], "NCH", 3, is_dram=self._is_dram
                )
                c_load = self.gate_C(
                    self.w_L2_n[1] + self.w_L2_p[1], 0.0, is_dram=self._is_dram
                )
                c_intrinsic = 3 * self.drain_C(
                    self.w_L2_p[0],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L2_n[0],
                    "NCH",
                    3,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = rd * (c_intrinsic + c_load)
                this_delay = self.horowitz(inrise_nand3, tf, 0.5, 0.5, 1)
                self.delay_nand3_path += this_delay
                inrise_nand3 = this_delay / (1.0 - 0.5)

            for i in range(1, self.number_gates_L2 - 1):
                rd = self.tr_R_on(
                    self.w_L2_n[i], "NCH", 1, is_dram=self._is_dram
                )
                c_load = self.gate_C(
                    self.w_L2_n[i + 1] + self.w_L2_p[i + 1],
                    0.0,
                    is_dram=self._is_dram,
                )
                c_intrinsic = self.drain_C(
                    self.w_L2_p[i],
                    "PCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                ) + self.drain_C(
                    self.w_L2_n[i],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                tf = rd * (c_intrinsic + c_load)
                this_delay = self.horowitz(inrise_nand2, tf, 0.5, 0.5, 1)
                self.delay_nand2_path += this_delay
                inrise_nand2 = this_delay / (1.0 - 0.5)
                this_delay = self.horowitz(inrise_nand3, tf, 0.5, 0.5, 1)
                self.delay_nand3_path += this_delay
                inrise_nand3 = this_delay / (1.0 - 0.5)

            i = self.number_gates_L2 - 1
            c_load = self.C_ld_predec_blk_out
            rd = self.tr_R_on(self.w_L2_n[i], "NCH", 1, is_dram=self._is_dram)
            c_intrinsic = self.drain_C(
                self.w_L2_p[i], "PCH", 1, 1, cell_h_def, is_dram=self._is_dram
            ) + self.drain_C(
                self.w_L2_n[i], "NCH", 1, 1, cell_h_def, is_dram=self._is_dram
            )
            tf = (
                rd * (c_intrinsic + c_load)
                + self.R_wire_predec_blk_out * c_load / 2
            )
            this_delay = self.horowitz(inrise_nand2, tf, 0.5, 0.5, 1)
            self.delay_nand2_path += this_delay
            ret_first = this_delay / (1.0 - 0.5)
            this_delay = self.horowitz(inrise_nand3, tf, 0.5, 0.5, 1)
            self.delay_nand3_path += this_delay
            ret_second = this_delay / (1.0 - 0.5)

        # NOTE this compares the two OUTRISETIMES (ret_first/ret_second), not
        # the accumulated delay_nand2_path/delay_nand3_path -- matches
        # decoder.cc:947 exactly, an odd-looking but real CACTI choice,
        # replicated verbatim rather than "fixed" to compare delays instead.
        self.delay = max(ret_first, ret_second)
        return (ret_first, ret_second)

    def compute_area(self):
        # Port of PredecBlk::compute_area (decoder.cc:606-755), area terms
        # only -- leakage/gate_leakage are already ported in static_power().
        # Reuses self.num_L1_nand2/num_L1_nand3/num_L2 (already set by
        # compute_widths(), which folds C++'s separate compute_area() switch
        # -- same n==1..9 dispatch, same magic numbers -- into its own table
        # already; see compute_widths()'s own comment above).
        if not self.exist:
            return
        cell_h_def = self._tp["cell_h_def"]

        # Real bug found and replicated (Phase 30, decoder.cc:620-629): the
        # stage-0 NAND3 gate's own area is added UNCONDITIONALLY for the
        # nand2 path (line 617) but only when `number_inputs_L1_gate == 3`
        # for the nand3 path -- an asymmetry, not a "both zero when unused"
        # equivalence. For flag_two_unique_paths blocks (n=5/7/8),
        # number_inputs_L1_gate stays 0 (never set by compute_widths' n=5/
        # 7/8 cases) even though w_L1_nand3_n/p[0] ARE genuinely sized
        # nonzero there (both paths exist) -- so real CACTI silently drops
        # the nand3 stage-0 gate's area (but NOT its later INV buffer
        # stages' area, i>=1 below, which are unconditional) in exactly
        # that case. A prior version of this method assumed "path doesn't
        # exist -> zero widths -> compute_gate_area returns 0 anyway" made
        # this guard a no-op; that assumption is false here (found via a
        # live PREDECBLK_AREA_PROBE on a real McPAT dcache config with
        # number_input_addr_bits=5, off by a real, confirmed 3.25 sq-um
        # before this fix -- see the native reference).
        tot_area_L1_nand2 = self.compute_gate_area(
            "NAND", 2, self.w_L1_nand2_p[0], self.w_L1_nand2_n[0], cell_h_def
        )
        for i in range(1, self.number_gates_L1_nand2_path):
            tot_area_L1_nand2 += self.compute_gate_area(
                "INV",
                1,
                self.w_L1_nand2_p[i],
                self.w_L1_nand2_n[i],
                cell_h_def,
            )
        tot_area_L1_nand2 *= self.num_L1_nand2

        tot_area_L1_nand3 = (
            self.compute_gate_area(
                "NAND",
                3,
                self.w_L1_nand3_p[0],
                self.w_L1_nand3_n[0],
                cell_h_def,
            )
            if self.number_inputs_L1_gate == 3
            else 0.0
        )
        for i in range(1, self.number_gates_L1_nand3_path):
            tot_area_L1_nand3 += self.compute_gate_area(
                "INV",
                1,
                self.w_L1_nand3_p[i],
                self.w_L1_nand3_n[i],
                cell_h_def,
            )
        tot_area_L1_nand3 *= self.num_L1_nand3

        cumulative_area_L2 = 0.0
        if self.flag_L2_gate == 2:
            cumulative_area_L2 = self.compute_gate_area(
                "NAND", 2, self.w_L2_p[0], self.w_L2_n[0], cell_h_def
            )
        elif self.flag_L2_gate == 3:
            cumulative_area_L2 = self.compute_gate_area(
                "NAND", 3, self.w_L2_p[0], self.w_L2_n[0], cell_h_def
            )
        for i in range(1, self.number_gates_L2):
            cumulative_area_L2 += self.compute_gate_area(
                "INV", 1, self.w_L2_p[i], self.w_L2_n[i], cell_h_def
            )
        cumulative_area_L2 *= self.num_L2

        self.area = tot_area_L1_nand2 + tot_area_L1_nand3 + cumulative_area_L2


class CactiPredecBlkDrv(CactiComponent):
    # Faithful port of PredecBlkDrv (decoder.cc). way_select > 1 (the
    # way-select driver, Mat's way_sel_drv1, mat.cc:258) is ported as of
    # Phase 30 -- see __init__'s way_select>1 branch and compute_widths'
    # `self.way_select == 0` guard/extra OR clause below, ported from
    # decoder.cc:1119-1133/1162/1220/1238.
    def __init__(self, name, cacti_params, way_select, blk):
        super().__init__(name, cacti_params)

        self.blk = blk
        self.dec = blk.dec
        self.way_select = int(way_select) if way_select else 0
        self.flag_driver_exists = 0
        self.number_gates_nand2_path = 0
        self.number_gates_nand3_path = 0
        self.min_number_gates = 2
        self.num_buffers_driving_1_nand2_load = 0
        self.num_buffers_driving_2_nand2_load = 0
        self.num_buffers_driving_4_nand2_load = 0
        self.num_buffers_driving_2_nand3_load = 0
        self.num_buffers_driving_8_nand3_load = 0
        self.c_load_nand2_path_out = 0.0
        self.c_load_nand3_path_out = 0.0
        self.delay_nand2_path = 0.0
        self.delay_nand3_path = 0.0
        self.area = 0.0
        self.power_nand2_path = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self.power_nand3_path = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )

        self.width_nand2_path_n = [0.0] * 20
        self.width_nand2_path_p = [0.0] * 20
        self.width_nand3_path_n = [0.0] * 20
        self.width_nand3_path_p = [0.0] * 20

        self.number_input_addr_bits = blk.number_input_addr_bits
        if self.way_select > 1:
            self.flag_driver_exists = 1
            self.number_input_addr_bits = self.way_select
            if self.dec._num_in_signals == 2:
                self.c_load_nand2_path_out = self.gate_C(
                    self.dec._w_dec_n[0] + self.dec._w_dec_p[0],
                    0,
                    is_dram=self._is_dram,
                )
                self.num_buffers_driving_2_nand2_load = (
                    self.number_input_addr_bits
                )
            elif self.dec._num_in_signals == 3:
                self.c_load_nand3_path_out = self.gate_C(
                    self.dec._w_dec_n[0] + self.dec._w_dec_p[0],
                    0,
                    is_dram=self._is_dram,
                )
                self.num_buffers_driving_2_nand3_load = (
                    self.number_input_addr_bits
                )
        elif self.way_select == 0 and blk.exist:
            self.flag_driver_exists = 1

        self.compute_widths()
        self.compute_power()

    def num_addr_bits_nand2_path(self):
        return (
            self.num_buffers_driving_1_nand2_load
            + self.num_buffers_driving_2_nand2_load
            + self.num_buffers_driving_4_nand2_load
        )

    def num_addr_bits_nand3_path(self):
        return (
            self.num_buffers_driving_2_nand3_load
            + self.num_buffers_driving_8_nand3_load
        )

    def compute_widths(self):
        if not self.flag_driver_exists:
            return
        p_to_n = self.pmos_to_nmos_sz_ratio()
        min_w = self._tp["min_w_nmos"]
        max_w = self._tp["max_w_nmos"]
        C_nand2_gate_blk = self.gate_C(
            self.blk.w_L1_nand2_n[0] + self.blk.w_L1_nand2_p[0],
            0,
            is_dram=self._is_dram,
        )
        C_nand3_gate_blk = self.gate_C(
            self.blk.w_L1_nand3_n[0] + self.blk.w_L1_nand3_p[0],
            0,
            is_dram=self._is_dram,
        )

        # C++'s guard here (decoder.cc:1162) is `way_select == 0` -- when
        # way_select > 1, c_load_nand2/3_path_out and num_buffers_driving_*
        # were already set in __init__'s way_select>1 branch and must not
        # be overwritten by blk-derived values here.
        if self.way_select == 0:
            n = self.blk.number_input_addr_bits
            if n == 1:
                self.num_buffers_driving_2_nand2_load = 1
                self.c_load_nand2_path_out = 2 * C_nand2_gate_blk
            elif n == 2:
                self.num_buffers_driving_4_nand2_load = 2
                self.c_load_nand2_path_out = 4 * C_nand2_gate_blk
            elif n == 3:
                self.num_buffers_driving_8_nand3_load = 3
                self.c_load_nand3_path_out = 8 * C_nand3_gate_blk
            elif n == 4:
                self.num_buffers_driving_4_nand2_load = 4
                self.c_load_nand2_path_out = 4 * C_nand2_gate_blk
            elif n == 5:
                self.num_buffers_driving_4_nand2_load = 2
                self.num_buffers_driving_8_nand3_load = 3
                self.c_load_nand2_path_out = 4 * C_nand2_gate_blk
                self.c_load_nand3_path_out = 8 * C_nand3_gate_blk
            elif n == 6:
                self.num_buffers_driving_8_nand3_load = 6
                self.c_load_nand3_path_out = 8 * C_nand3_gate_blk
            elif n == 7:
                self.num_buffers_driving_4_nand2_load = 4
                self.num_buffers_driving_8_nand3_load = 3
                self.c_load_nand2_path_out = 4 * C_nand2_gate_blk
                self.c_load_nand3_path_out = 8 * C_nand3_gate_blk
            elif n == 8:
                self.num_buffers_driving_4_nand2_load = 2
                self.num_buffers_driving_8_nand3_load = 6
                self.c_load_nand2_path_out = 4 * C_nand2_gate_blk
                self.c_load_nand3_path_out = 8 * C_nand3_gate_blk
            elif n == 9:
                self.num_buffers_driving_8_nand3_load = 9
                self.c_load_nand3_path_out = 8 * C_nand3_gate_blk

        if (
            self.blk.flag_two_unique_paths
            or self.blk.number_inputs_L1_gate == 2
            or self.number_input_addr_bits == 0
            or (self.way_select and self.dec._num_in_signals == 2)
        ):
            self.width_nand2_path_n[0] = min_w
            self.width_nand2_path_p[0] = p_to_n * self.width_nand2_path_n[0]
            F = self.c_load_nand2_path_out / self.gate_C(
                self.width_nand2_path_n[0] + self.width_nand2_path_p[0],
                0,
                is_dram=self._is_dram,
            )
            self.number_gates_nand2_path = self.logical_effort(
                self.min_number_gates,
                1,
                F,
                self.width_nand2_path_n,
                self.width_nand2_path_p,
                self.c_load_nand2_path_out,
                p_to_n,
                is_dram=self._is_dram,
                is_wl_tr=False,
                max_w_nmos=max_w,
            )

        if (
            self.blk.flag_two_unique_paths
            or self.blk.number_inputs_L1_gate == 3
            or (self.way_select and self.dec._num_in_signals == 3)
        ):
            self.width_nand3_path_n[0] = min_w
            self.width_nand3_path_p[0] = p_to_n * self.width_nand3_path_n[0]
            F = self.c_load_nand3_path_out / self.gate_C(
                self.width_nand3_path_n[0] + self.width_nand3_path_p[0],
                0,
                is_dram=self._is_dram,
            )
            self.number_gates_nand3_path = self.logical_effort(
                self.min_number_gates,
                1,
                F,
                self.width_nand3_path_n,
                self.width_nand3_path_p,
                self.c_load_nand3_path_out,
                p_to_n,
                is_dram=self._is_dram,
                is_wl_tr=False,
                max_w_nmos=max_w,
            )

    def dynamic_power(self):
        # Port of PredecBlkDrv::compute_delays energy (drivers switch at 0.5*Vdd^2).
        if not self.flag_driver_exists:
            return
        Vdd = self._tp["Vdd"]
        cell_h_def = self._tp["cell_h_def"]

        for i in range(0, self.number_gates_nand2_path - 1):
            c_gate_load = self.gate_C(
                self.width_nand2_path_p[i + 1]
                + self.width_nand2_path_n[i + 1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = self.drain_C(
                self.width_nand2_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand2_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand2_path.read.dynamic += (
                (c_gate_load + c_intrinsic) * 0.5 * Vdd * Vdd
            )
        if self.number_gates_nand2_path != 0:
            i = self.number_gates_nand2_path - 1
            c_intrinsic = self.drain_C(
                self.width_nand2_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand2_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand2_path.read.dynamic += (
                (c_intrinsic + self.c_load_nand2_path_out) * 0.5 * Vdd * Vdd
            )

        for i in range(0, self.number_gates_nand3_path - 1):
            c_gate_load = self.gate_C(
                self.width_nand3_path_p[i + 1]
                + self.width_nand3_path_n[i + 1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = self.drain_C(
                self.width_nand3_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand3_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand3_path.read.dynamic += (
                (c_gate_load + c_intrinsic) * 0.5 * Vdd * Vdd
            )
        if self.number_gates_nand3_path != 0:
            i = self.number_gates_nand3_path - 1
            c_intrinsic = self.drain_C(
                self.width_nand3_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand3_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            self.power_nand3_path.read.dynamic += (
                (c_intrinsic + self.c_load_nand3_path_out) * 0.5 * Vdd * Vdd
            )

    def static_power(self):
        # Port of PredecBlkDrv::compute_area leakage (per-gate leakage x num_buffers).
        if not self.flag_driver_exists:
            return
        Vdd = self._tp["Vdd"]
        leak2 = gleak2 = 0.0
        for i in range(0, self.number_gates_nand2_path):
            leak2 += self.cmos_Isub_leakage(
                self.width_nand2_path_n[i],
                self.width_nand2_path_p[i],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            gleak2 += self.cmos_Ig_leakage(
                self.width_nand2_path_n[i],
                self.width_nand2_path_p[i],
                1,
                "INV",
                is_dram=self._is_dram,
            )
        nbuf2 = self.num_addr_bits_nand2_path()
        self.power_nand2_path.read.leakage = leak2 * nbuf2 * Vdd
        self.power_nand2_path.read.gate_leakage = gleak2 * nbuf2 * Vdd

        leak3 = gleak3 = 0.0
        for i in range(0, self.number_gates_nand3_path):
            leak3 += self.cmos_Isub_leakage(
                self.width_nand3_path_n[i],
                self.width_nand3_path_p[i],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            gleak3 += self.cmos_Ig_leakage(
                self.width_nand3_path_n[i],
                self.width_nand3_path_p[i],
                1,
                "INV",
                is_dram=self._is_dram,
            )
        nbuf3 = self.num_addr_bits_nand3_path()
        self.power_nand3_path.read.leakage = leak3 * nbuf3 * Vdd
        self.power_nand3_path.read.gate_leakage = gleak3 * nbuf3 * Vdd

    def compute_delays(self, inrisetime_nand2_path, inrisetime_nand3_path):
        # Port of PredecBlkDrv::compute_delays (decoder.cc:1307-1378), timing
        # terms only. Not called from anywhere yet (Phase 28, additive only).
        #
        # NOTE: decoder.cc's r_load_nand{2,3}_path_out fields are declared
        # but never assigned anywhere in the real C++ (always 0.0) -- the
        # wire-R Elmore term (`+ r_load_*_path_out*c_load/2`) is a permanent
        # dead-code no-op there. Omitted here as a deliberate replication of
        # that behavior (project rule: replicate CACTI bugs/dead-code, don't
        # silently "fix" them), not an oversight.
        ret_first = 0.0
        ret_second = 0.0
        if not self.flag_driver_exists:
            return (ret_first, ret_second)

        cell_h_def = self._tp["cell_h_def"]

        for i in range(0, self.number_gates_nand2_path - 1):
            rd = self.tr_R_on(
                self.width_nand2_path_n[i], "NCH", 1, is_dram=self._is_dram
            )
            c_gate_load = self.gate_C(
                self.width_nand2_path_p[i + 1]
                + self.width_nand2_path_n[i + 1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = self.drain_C(
                self.width_nand2_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand2_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            tf = rd * (c_intrinsic + c_gate_load)
            this_delay = self.horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, 1)
            self.delay_nand2_path += this_delay
            inrisetime_nand2_path = this_delay / (1.0 - 0.5)

        if self.number_gates_nand2_path != 0:
            i = self.number_gates_nand2_path - 1
            rd = self.tr_R_on(
                self.width_nand2_path_n[i], "NCH", 1, is_dram=self._is_dram
            )
            c_intrinsic = self.drain_C(
                self.width_nand2_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand2_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            c_load = self.c_load_nand2_path_out
            tf = rd * (c_intrinsic + c_load)
            this_delay = self.horowitz(inrisetime_nand2_path, tf, 0.5, 0.5, 1)
            self.delay_nand2_path += this_delay
            ret_first = this_delay / (1.0 - 0.5)

        for i in range(0, self.number_gates_nand3_path - 1):
            rd = self.tr_R_on(
                self.width_nand3_path_n[i], "NCH", 1, is_dram=self._is_dram
            )
            c_gate_load = self.gate_C(
                self.width_nand3_path_p[i + 1]
                + self.width_nand3_path_n[i + 1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = self.drain_C(
                self.width_nand3_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand3_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            tf = rd * (c_intrinsic + c_gate_load)
            this_delay = self.horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, 1)
            self.delay_nand3_path += this_delay
            inrisetime_nand3_path = this_delay / (1.0 - 0.5)

        if self.number_gates_nand3_path != 0:
            i = self.number_gates_nand3_path - 1
            rd = self.tr_R_on(
                self.width_nand3_path_n[i], "NCH", 1, is_dram=self._is_dram
            )
            c_intrinsic = self.drain_C(
                self.width_nand3_path_p[i],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            ) + self.drain_C(
                self.width_nand3_path_n[i],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            c_load = self.c_load_nand3_path_out
            tf = rd * (c_intrinsic + c_load)
            this_delay = self.horowitz(inrisetime_nand3_path, tf, 0.5, 0.5, 1)
            self.delay_nand3_path += this_delay
            ret_second = this_delay / (1.0 - 0.5)

        return (ret_first, ret_second)

    def compute_area(self):
        # Port of PredecBlkDrv::compute_area (decoder.cc:1258-1303), area
        # terms only -- leakage/gate_leakage already ported in static_power().
        if not self.flag_driver_exists:
            return
        cell_h_def = self._tp["cell_h_def"]

        area_nand2_path = 0.0
        for i in range(0, self.number_gates_nand2_path):
            area_nand2_path += self.compute_gate_area(
                "INV",
                1,
                self.width_nand2_path_p[i],
                self.width_nand2_path_n[i],
                cell_h_def,
            )
        area_nand2_path *= self.num_addr_bits_nand2_path()

        area_nand3_path = 0.0
        for i in range(0, self.number_gates_nand3_path):
            area_nand3_path += self.compute_gate_area(
                "INV",
                1,
                self.width_nand3_path_p[i],
                self.width_nand3_path_n[i],
                cell_h_def,
            )
        area_nand3_path *= self.num_addr_bits_nand3_path()

        self.area = area_nand2_path + area_nand3_path


class CactiPredec(CactiComponent):
    def __init__(self, name, cacti_params, drv1, drv2):
        super().__init__(name, cacti_params)

        self.drv1 = drv1
        self.drv2 = drv2
        self.blk1 = drv1.blk
        self.blk2 = drv2.blk

        self.driver_power = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self.block_power = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self.delay = 0.0

        self.compute_power()

    def dynamic_power(self):
        # Predec::compute_delays: drivers weighted by num_addr_bits (one buffer per
        # address bit driven), blocks weighted by num_L1_active. NOTE CACTI applies
        # blk1's num_L1_active to blk2 as well -- replicated verbatim.
        self.driver_power.read.dynamic = (
            self.drv1.num_addr_bits_nand2_path()
            * self.drv1.power_nand2_path.read.dynamic
            + self.drv1.num_addr_bits_nand3_path()
            * self.drv1.power_nand3_path.read.dynamic
            + self.drv2.num_addr_bits_nand2_path()
            * self.drv2.power_nand2_path.read.dynamic
            + self.drv2.num_addr_bits_nand3_path()
            * self.drv2.power_nand3_path.read.dynamic
        )
        self.block_power.read.dynamic = (
            self.blk1.power_nand2_path.read.dynamic
            * self.blk1.num_L1_active_nand2_path
            + self.blk1.power_nand3_path.read.dynamic
            * self.blk1.num_L1_active_nand3_path
            + self.blk1.power_L2.read.dynamic
            + self.blk2.power_nand2_path.read.dynamic
            * self.blk1.num_L1_active_nand2_path
            + self.blk2.power_nand3_path.read.dynamic
            * self.blk1.num_L1_active_nand3_path
            + self.blk2.power_L2.read.dynamic
        )
        self._power.read.dynamic = (
            self.driver_power.read.dynamic + self.block_power.read.dynamic
        )

    def static_power(self):
        self.driver_power.read.leakage = (
            self.drv1.power_nand2_path.read.leakage
            + self.drv1.power_nand3_path.read.leakage
            + self.drv2.power_nand2_path.read.leakage
            + self.drv2.power_nand3_path.read.leakage
        )
        self.block_power.read.leakage = (
            self.blk1.power_nand2_path.read.leakage
            + self.blk1.power_nand3_path.read.leakage
            + self.blk1.power_L2.read.leakage
            + self.blk2.power_nand2_path.read.leakage
            + self.blk2.power_nand3_path.read.leakage
            + self.blk2.power_L2.read.leakage
        )
        self._power.read.leakage = (
            self.driver_power.read.leakage + self.block_power.read.leakage
        )

        # Port of Predec::Predec's gate_leakage block (decoder.cc:1420-1428).
        # Previously stranded as dead code after the `return` in
        # _get_max_delay_before_decoder below (a misplacement, not a missing
        # formula -- the leaf-level power_nand2_path/power_nand3_path/power_L2
        # .gate_leakage values were always computed correctly by
        # CactiPredecBlk/CactiPredecBlkDrv's own static_power(); only this
        # aggregation was unreachable). This left r_predec/b_mux_predec/
        # sa_mux_lev_1_predec/sa_mux_lev_2_predec's gate_leakage at 0.0 for
        # every array in the project -- see the native reference.
        self.driver_power.read.gate_leakage = (
            self.drv1.power_nand2_path.read.gate_leakage
            + self.drv1.power_nand3_path.read.gate_leakage
            + self.drv2.power_nand2_path.read.gate_leakage
            + self.drv2.power_nand3_path.read.gate_leakage
        )
        self.block_power.read.gate_leakage = (
            self.blk1.power_nand2_path.read.gate_leakage
            + self.blk1.power_nand3_path.read.gate_leakage
            + self.blk1.power_L2.read.gate_leakage
            + self.blk2.power_nand2_path.read.gate_leakage
            + self.blk2.power_nand3_path.read.gate_leakage
            + self.blk2.power_L2.read.gate_leakage
        )
        self._power.read.gate_leakage = (
            self.driver_power.read.gate_leakage
            + self.block_power.read.gate_leakage
        )

    def compute_delays(self, inrisetime):
        # Port of Predec::compute_delays (decoder.cc:1469-1496), timing only
        # (energy already in dynamic_power()). Not called from anywhere yet
        # (Phase 28, additive only -- see PROGRESS.md). No compute_area() on
        # this class -- matches real CACTI: area rollup across
        # blk1/blk2/drv1/drv2 happens at the Mat level .
        tmp_pair1 = self.drv1.compute_delays(inrisetime, inrisetime)
        tmp_pair1 = self.blk1.compute_delays(tmp_pair1)
        tmp_pair2 = self.drv2.compute_delays(inrisetime, inrisetime)
        tmp_pair2 = self.blk2.compute_delays(tmp_pair2)
        max_delay, outrisetime = self._get_max_delay_before_decoder(
            tmp_pair1, tmp_pair2
        )
        self.delay = max_delay
        return outrisetime

    def _get_max_delay_before_decoder(self, input_pair1, input_pair2):
        # Port of Predec::get_max_delay_before_decoder (decoder.cc:1531-1562):
        # picks the candidate with the largest CUMULATIVE delay among the 4
        # drv+blk nand2/nand3 chains, and returns that winner's outrisetime
        # (a strict '<' comparison -- ties keep the earlier candidate,
        # matching the C++ exactly).
        delay = self.drv1.delay_nand2_path + self.blk1.delay_nand2_path
        ret_first = delay
        ret_second = input_pair1[0]

        delay = self.drv1.delay_nand3_path + self.blk1.delay_nand3_path
        if ret_first < delay:
            ret_first = delay
            ret_second = input_pair1[1]

        delay = self.drv2.delay_nand2_path + self.blk2.delay_nand2_path
        if ret_first < delay:
            ret_first = delay
            ret_second = input_pair2[0]

        delay = self.drv2.delay_nand3_path + self.blk2.delay_nand3_path
        if ret_first < delay:
            ret_first = delay
            ret_second = input_pair2[1]

        return (ret_first, ret_second)


class CactiDriver(CactiComponent):
    """Port of CACTI's Driver: a logical-effort-sized inverter chain driving a
    gate + wire load. Used here for the bitline precharge/equalization driver.
    Both the energy (dynamic_power/static_power) and timing (compute_delay,
    Phase 29) terms of Driver::compute_delay are ported."""

    def __init__(
        self, name, cacti_params, c_gate_load, c_wire_load, r_wire_load
    ):
        super().__init__(name, cacti_params)

        self._c_gate_load = c_gate_load
        self._c_wire_load = c_wire_load
        self._r_wire_load = r_wire_load
        self._num_gates = 0
        self._min_number_gates = 2
        self.delay = 0.0

        MAX_NUMBER_GATES_STAGE = 20
        self._width_n = [0.0] * MAX_NUMBER_GATES_STAGE
        self._width_p = [0.0] * MAX_NUMBER_GATES_STAGE

        self.compute_widths()
        self.compute_power()

    def compute_widths(self):
        p_to_n_sz_ratio = self.pmos_to_nmos_sz_ratio()
        c_load = self._c_gate_load + self._c_wire_load
        self._width_n[0] = self._tp["min_w_nmos"]
        self._width_p[0] = p_to_n_sz_ratio * self._tp["min_w_nmos"]

        F = c_load / self.gate_C(
            self._width_n[0] + self._width_p[0], 0, is_dram=self._is_dram
        )
        self._num_gates = self.logical_effort(
            self._min_number_gates,
            1,
            F,
            self._width_n,
            self._width_p,
            c_load,
            p_to_n_sz_ratio,
            is_dram=self._is_dram,
            is_wl_tr=False,
            max_w_nmos=self._tp["max_w_nmos"],
        )

    def dynamic_power(self):
        Vdd = self._tp["Vdd"]

        for i in range(0, self._num_gates - 1):
            c_load = self.gate_C(
                self._width_n[i + 1] + self._width_p[i + 1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = self.drain_C(
                self._width_p[i],
                "PCH",
                1,
                1,
                self._tp["cell_h_def"],
                is_dram=self._is_dram,
            ) + self.drain_C(
                self._width_n[i],
                "NCH",
                1,
                1,
                self._tp["cell_h_def"],
                is_dram=self._is_dram,
            )
            self._power.read.dynamic += (c_intrinsic + c_load) * Vdd * Vdd

        i = self._num_gates - 1
        c_load = self._c_gate_load + self._c_wire_load
        c_intrinsic = self.drain_C(
            self._width_p[i],
            "PCH",
            1,
            1,
            self._tp["cell_h_def"],
            is_dram=self._is_dram,
        ) + self.drain_C(
            self._width_n[i],
            "NCH",
            1,
            1,
            self._tp["cell_h_def"],
            is_dram=self._is_dram,
        )
        self._power.read.dynamic += (c_intrinsic + c_load) * Vdd * Vdd

    def static_power(self):
        Vdd = self._tp["Vdd"]
        for i in range(0, self._num_gates):
            self._power.read.leakage += (
                self.cmos_Isub_leakage(
                    self._width_n[i],
                    self._width_p[i],
                    1,
                    "INV",
                    is_dram=self._is_dram,
                )
                * Vdd
            )
            self._power.read.gate_leakage += (
                self.cmos_Ig_leakage(
                    self._width_n[i],
                    self._width_p[i],
                    1,
                    "INV",
                    is_dram=self._is_dram,
                )
                * Vdd
            )

    def compute_delay(self, inrisetime):
        # Port of Driver::compute_delay (decoder.cc:1660-1696), timing terms
        # only (energy already in dynamic_power()/static_power()). Phase 29,
        # additive only -- not called from __init__ (unlike compute_widths/
        # compute_power, which already run eagerly there). Matches the C++,
        # where compute_delay is invoked explicitly at each call site -- and
        # deliberately preserves the already-documented, real CACTI bug that
        # cam_bl_precharge_eq_drv.compute_delay() is never called anywhere in
        # real CACTI (see this class's construction site in CactiMat): adding
        # an eager call here would silently "fix" that bug for every
        # CactiDriver instance, not just bl_precharge_eq_drv.
        #
        # self.delay is NOT reset here -- it accumulates via += across every
        # stage, matching the C++ member field exactly (real CACTI never
        # resets it either; each Driver is constructed fresh per candidate).
        this_delay = 0.0
        for i in range(0, self._num_gates - 1):
            rd = self.tr_R_on(
                self._width_n[i], "NCH", 1, is_dram=self._is_dram
            )
            c_load = self.gate_C(
                self._width_n[i + 1] + self._width_p[i + 1],
                0.0,
                is_dram=self._is_dram,
            )
            c_intrinsic = self.drain_C(
                self._width_p[i],
                "PCH",
                1,
                1,
                self._tp["cell_h_def"],
                is_dram=self._is_dram,
            ) + self.drain_C(
                self._width_n[i],
                "NCH",
                1,
                1,
                self._tp["cell_h_def"],
                is_dram=self._is_dram,
            )
            tf = rd * (c_intrinsic + c_load)
            this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
            self.delay += this_delay
            inrisetime = this_delay / (1.0 - 0.5)

        i = self._num_gates - 1
        c_load = self._c_gate_load + self._c_wire_load
        rd = self.tr_R_on(self._width_n[i], "NCH", 1, is_dram=self._is_dram)
        c_intrinsic = self.drain_C(
            self._width_p[i],
            "PCH",
            1,
            1,
            self._tp["cell_h_def"],
            is_dram=self._is_dram,
        ) + self.drain_C(
            self._width_n[i],
            "NCH",
            1,
            1,
            self._tp["cell_h_def"],
            is_dram=self._is_dram,
        )
        tf = rd * (c_intrinsic + c_load) + self._r_wire_load * (
            self._c_wire_load / 2 + self._c_gate_load
        )
        this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
        self.delay += this_delay

        return this_delay / (1.0 - 0.5)
