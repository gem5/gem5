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


@dataclass
class PowerStats:
    dynamic: float
    leakage: float
    gate_leakage: float


@dataclass
class ComponentPower:
    read: PowerStats
    write: PowerStats
    search: PowerStats


class CactiComponent(CactiCircuit):
    def __init__(
        self,
        name,
        cacti_params: CactiParams,
        num_rw_ports=1,
        num_r_ports=0,
        num_w_ports=0,
        num_se_r_ports=0,
        num_sr_ports=0,
        is_fa=False,
        pure_cam=False,
    ):
        super().__init__(cacti_params)
        self._name = name
        self._fopt = 4
        self._power = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self._is_dram = False
        self._is_fa = is_fa
        self._is_pure_cam = pure_cam
        self._has_ecc = False
        self._is_cam = self._is_fa or self._is_pure_cam
        # Port counts widen the memory cell (init_cell_area): each extra port
        # adds wordline/bitline wire tracks, growing cell_h/cell_w and hence the
        # C_wl / C_bl metal caps. Decoders keep the default single port (they are
        # handed the array's cell dimensions explicitly).
        self._num_rw_ports = num_rw_ports
        self._num_r_ports = num_r_ports
        self._num_w_ports = num_w_ports
        self._num_se_r_ports = num_se_r_ports
        # Search ports (g_ip->num_search_ports, SCHP) -- only FA/CAM cells widen
        # for these (parameter.cc:343-351).
        self._num_sr_ports = num_sr_ports

        self.init_cell_area()
        self.init_v_b_sense()

    def logical_effort(
        self,
        num_gates_min,
        g,
        F,
        w_n,
        w_p,
        C_load,
        p_to_n_sz_ratio,
        is_dram,
        is_wl_tr,
        max_w_nmos,
    ):

        # Initial gate count estimation based on optimal fanout (fopt).
        # NOTE: CACTI's Component::logical_effort uses C-style truncation
        # ( (int)(log(F)/log(fopt)) ), NOT ceil(). Using ceil() here would
        # over-count driver stages and inflate decoder dynamic/leakage energy.
        #
        # F<=0 is a real, reachable input (the router configuration,
        # cacti_noc.CactiRouter's sa_mux_lev_1_dec preset: C_ld_dec_out==0
        # when Ndsam_lev_1==1 with number_way_select_signals_mat!=0 --
        # flag_way_select forces exist=True regardless). C's
        # `(int)(log(F)/log(fopt))` casts an out-of-range double to a
        # 32-bit int, which is undefined behavior. F==0 (the only case
        # actually reachable by the router preset above) was verified
        # directly against real CACTI: `(int)(log(0.0)/log(4.0))` reliably
        # produces INT32_MIN. F<0 is untested (not reachable by any known
        # caller) and its exact UB value is unverified, but immaterial
        # either way: the very next line's MAX(num_gates, num_gates_min)
        # clamps any sufficiently-negative garbage value back up to
        # num_gates_min regardless of magnitude. Replicated as the measured
        # F==0 constant rather than raising ValueError.
        num_gates = -(2**31) if F <= 0 else int(log(F) / log(self._fopt))

        # Check if num_gates is odd. If so, add 1 to make it even (maintain polarity)
        num_gates += 1 if (num_gates % 2) else 0
        num_gates = max(num_gates, num_gates_min)

        # Recalculate the effective fanout of each stage
        f = F ** (1.0 / num_gates)
        i = num_gates - 1
        # f==0.0 (F==0) makes this 0.0/C_load in C++ terms; Python raises
        # ZeroDivisionError for float division instead of C's silent
        # IEEE754 inf/nan, so replicate that explicitly.
        if f == 0.0:
            if C_load == 0.0:
                C_in = float("nan")
            else:
                C_in = float("inf") if C_load > 0.0 else float("-inf")
        else:
            C_in = C_load / f

        # Size the final driver stage
        w_n[i] = (
            (1.0 / (1.0 + p_to_n_sz_ratio))
            * C_in
            / self.gate_C(1, 0, is_dram=is_dram, is_wl_tr=is_wl_tr)
        )
        # C's `MAX(a,b)` macro is `(a>b)?a:b` -- when a is NaN (reachable
        # via the C_in==nan case just above), `a>b` is false so the macro
        # falls through to `b`. Python's builtin max(nan, b) instead KEEPS
        # the first (NaN) argument (verified directly: max(float('nan'), 5)
        # == nan) and would poison every downstream energy term for this
        # decoder. The explicit ternary below reproduces the C macro's
        # semantics exactly for both the normal (finite) case and this
        # NaN-recovery case.
        w_n[i] = (
            w_n[i]
            if w_n[i] > self._tp["min_w_nmos"]
            else self._tp["min_w_nmos"]
        )
        w_p[i] = p_to_n_sz_ratio * w_n[i]

        # If the required width exceeds the maximum allowed layout width, we must cap it
        # and recalculate the driver chain backwards from the new max load.
        if w_n[i] > max_w_nmos:
            C_ld = self.gate_C(
                (1 + p_to_n_sz_ratio) * max_w_nmos,
                0,
                is_dram=is_dram,
                is_wl_tr=is_wl_tr,
            )
            F = (
                g
                * C_ld
                / self.gate_C(
                    w_n[0] + w_p[0], 0, is_dram=is_dram, is_wl_tr=is_wl_tr
                )
            )

            num_gates = int(log(F) / log(self._fopt)) + 1
            num_gates += 1 if (num_gates % 2) else 0
            num_gates = max(num_gates, num_gates_min)

            f = F ** (1.0 / (num_gates - 1))
            i = num_gates - 1
            w_n[i] = max_w_nmos
            w_p[i] = p_to_n_sz_ratio * w_n[i]

        # Propagate the sizing backwards through the intermediate inverter stages
        for j in range(num_gates - 2, 0, -1):
            w_n[j] = max(w_n[j + 1] / f, self._tp["min_w_nmos"])
            w_p[j] = p_to_n_sz_ratio * w_n[j]

        # Ensure we do not overflow the allocated sizing arrays
        # (MAX_NUMBER_GATES_STAGE is implicitly the length of the arrays in the Python port)
        assert num_gates <= len(
            w_n
        ), "num_gates exceeds MAX_NUMBER_GATES_STAGE constraint"

        return num_gates

    def compute_diffusion_width(self, num_stacked_in, num_folded_tr):
        # Port of Component::compute_diffusion_width (component.cc:60-76).
        # NOTE: C++'s local `spacing_poly_to_poly` here (w_poly_contact +
        # 2*spacing_poly_to_contact, a poly-to-contact pitch) is a DIFFERENT
        # quantity from the tech param self._tp['spacing_poly_to_poly']
        # (a poly-to-poly gate pitch) despite sharing a name in the C++ source
        # -- named `contact_pitch` here to avoid that collision.
        w_poly = self._node_um
        contact_pitch = (
            self._tp["w_poly_contact"]
            + 2 * self._tp["spacing_poly_to_contact"]
        )
        total_diff_w = (
            2 * contact_pitch
            + num_stacked_in * w_poly
            + (num_stacked_in - 1) * self._tp["spacing_poly_to_poly"]
        )
        if num_folded_tr > 1:
            total_diff_w += (
                (num_folded_tr - 2) * 2 * contact_pitch
                + (num_folded_tr - 1) * num_stacked_in * w_poly
                + (num_folded_tr - 1)
                * (num_stacked_in - 1)
                * self._tp["spacing_poly_to_poly"]
            )
        return total_diff_w

    def compute_gate_area(self, gate_type, num_inputs, w_pmos, w_nmos, h_gate):
        # Port of Component::compute_gate_area (component.cc:80-146).
        if w_pmos <= 0.0 or w_nmos <= 0.0:
            return 0.0

        h_tr_region = h_gate - 2 * self._tp["hpowerrail"]
        ratio_p_to_n = w_pmos / (w_pmos + w_nmos)
        if ratio_p_to_n >= 1 or ratio_p_to_n <= 0:
            return 0.0

        w_folded_pmos = (
            h_tr_region - self._tp["min_gap_p_to_n_diff"]
        ) * ratio_p_to_n
        w_folded_nmos = (h_tr_region - self._tp["min_gap_p_to_n_diff"]) * (
            1 - ratio_p_to_n
        )
        assert w_folded_pmos > 0

        num_folded_pmos = int(ceil(w_pmos / w_folded_pmos))
        num_folded_nmos = int(ceil(w_nmos / w_folded_nmos))

        if gate_type == "INV":
            total_ndiff_w = self.compute_diffusion_width(1, num_folded_nmos)
            total_pdiff_w = self.compute_diffusion_width(1, num_folded_pmos)
        elif gate_type == "NOR":
            total_ndiff_w = self.compute_diffusion_width(
                1, num_inputs * num_folded_nmos
            )
            total_pdiff_w = self.compute_diffusion_width(
                num_inputs, num_folded_pmos
            )
        elif gate_type == "NAND":
            total_ndiff_w = self.compute_diffusion_width(
                num_inputs, num_folded_nmos
            )
            total_pdiff_w = self.compute_diffusion_width(
                1, num_inputs * num_folded_pmos
            )
        else:
            raise ValueError(f"Unknown gate type: {gate_type}")

        gate_w = max(total_ndiff_w, total_pdiff_w)
        if w_folded_nmos > w_nmos:
            # Gate fits in less than h_gate -- shrink height to match.
            gate_h = (
                w_nmos
                + w_pmos
                + self._tp["min_gap_p_to_n_diff"]
                + 2 * self._tp["hpowerrail"]
            )
        else:
            gate_h = h_gate
        return gate_w * gate_h

    def compute_tr_width_after_folding(
        self, input_width, threshold_folding_width
    ):
        # Port of Component::compute_tr_width_after_folding (component.cc:150-166).
        # NOTE: despite the name, this returns the width of a folded-transistor
        # CELL (diffusion width), not the width of a single device -- same
        # "cell width is orthogonal to device width" caveat as the C++ comment.
        if input_width <= 0:
            return 0.0
        num_folded_tr = int(ceil(input_width / threshold_folding_width))
        spacing_poly_to_poly = (
            self._tp["w_poly_contact"]
            + 2 * self._tp["spacing_poly_to_contact"]
        )
        width_poly = self._node_um
        total_diff_width = (
            num_folded_tr * width_poly
            + (num_folded_tr + 1) * spacing_poly_to_poly
        )
        return total_diff_width

    def height_sense_amplifier(self, pitch_sense_amp):
        # Port of Component::height_sense_amplifier (component.cc:170-184).
        h_pmos_tr = (
            self.compute_tr_width_after_folding(
                self._tp["w_sense_p"], pitch_sense_amp
            )
            * 2
            + self.compute_tr_width_after_folding(
                self._tp["w_iso"], pitch_sense_amp
            )
            + 2 * self._tp["min_gap_bet_same_type_diffs"]
        )
        h_nmos_tr = (
            self.compute_tr_width_after_folding(
                self._tp["w_sense_n"], pitch_sense_amp
            )
            * 2
            + self.compute_tr_width_after_folding(
                self._tp["w_sense_en"], pitch_sense_amp
            )
            + 2 * self._tp["min_gap_bet_same_type_diffs"]
        )
        return h_pmos_tr + h_nmos_tr + self._tp["min_gap_p_to_n_diff"]

    def set_dynamic_parameters(self, dp):
        # Take the array geometry from a CactiDynamicParameter (port of
        # DynamicParameter, parameter.cc). This replaces the earlier approximate
        # partition guess: the 5 partition integers come from a one-time CACTI
        # run and everything downstream is derived exactly as CACTI does.
        self.dp = dp
        self._num_subarrays = dp.num_subarrays
        self._num_mats = dp.num_mats
        self._associativity = dp.cfg.assoc

        # Partition factors chosen by CACTI's optimizer.
        self._Ndbl = dp.Ndbl
        self._Ndwl = dp.Ndwl
        self._Ndcm = dp.Ndcm
        self._Ndsam_lev_1 = dp.Ndsam_lev_1
        self._Ndsam_lev_2 = dp.Ndsam_lev_2

        # Subarray dimensions.
        self._subarray_rows = dp.num_r_subarray
        self._subarray_cols = dp.num_c_subarray

        # Mat partitions / floorplan.
        self._num_subarrays_per_mat = dp.num_subarrays_per_mat
        self._num_mats_h_dir = dp.num_mats_h_dir
        self._num_mats_v_dir = dp.num_mats_v_dir
        self._num_subarrays_per_row = dp.num_subarrays_per_row

        # Multiplexing degrees. deg_senseamp_muxing_non_associativity is the
        # number of sa-mux-level-1 decode signals (Ndsam1/assoc for a normal
        # SRAM), NOT Ndsam1*Ndsam2.
        self._deg_bl_muxing = dp.deg_bl_muxing
        self._deg_senseamp_muxing_non_associativity = (
            dp.deg_senseamp_muxing_non_associativity
        )

        # Way-select signals (0 for non-FA arrays).
        self._number_way_select_signals_mat = dp.number_way_select_signals_mat

        # Data-in / data-out bits per mat and active-mat count in the data-out
        # (horizontal) direction.
        self._num_di_b_mat = dp.num_di_b_mat
        self._num_do_b_mat = dp.num_do_b_mat
        self._num_act_mats_hor_dir = dp.num_act_mats_hor_dir
        # Search-out bits per mat (parameter.cc:468/537, fully_assoc only;
        # never set for a non-FA DP).
        self._num_so_b_mat = getattr(dp, "num_so_b_mat", 0)

    def init_v_b_sense(self):
        if self._is_dram:
            # TODO: Implement DRAM version
            raise NotImplementedError
        else:
            self._V_b_sense = (
                0.05 * self._tp["Vdd"]
                if (0.05 * self._tp["Vdd"] > VBITSENSEMIN)
                else VBITSENSEMIN
            )

    def init_cell_area(self):
        if self._is_cam:
            self._cam_cell_h = (
                self._tp["cam_b_h"]
                + 2
                * self._wp["pitch"]
                * (
                    self._num_rw_ports
                    - 1
                    + self._num_r_ports
                    + self._num_w_ports
                )
                + 2 * self._wp["pitch"] * (self._num_sr_ports - 1)
                + self._wp["pitch"] * self._num_se_r_ports
            )
            self._cam_cell_w = (
                self._tp["cam_b_w"]
                + 2
                * self._wp["pitch"]
                * (
                    self._num_rw_ports
                    - 1
                    + self._num_r_ports
                    + self._num_w_ports
                )
                + 2 * self._wp["pitch"] * (self._num_sr_ports - 1)
                + self._wp["pitch"] * self._num_se_r_ports
            )
            self._cell_h = (
                self._tp["sram_b_h"]
                + 2
                * self._wp["pitch"]
                * (
                    self._num_w_ports
                    + self._num_rw_ports
                    - 1
                    + self._num_r_ports
                )
                + 2 * self._wp["pitch"] * (self._num_sr_ports - 1)
            )
            self._cell_w = (
                self._tp["sram_b_w"]
                + 2
                * self._wp["pitch"]
                * (
                    self._num_rw_ports
                    - 1
                    + (self._num_r_ports - self._num_se_r_ports)
                    + self._num_w_ports
                )
                + self._wp["pitch"] * self._num_se_r_ports
                + 2 * self._wp["pitch"] * (self._num_sr_ports - 1)
            )
        else:
            self._cell_h = self._tp["sram_b_h"] + 2 * self._wp["pitch"] * (
                self._num_w_ports + self._num_rw_ports - 1 + self._num_r_ports
            )
            self._cell_w = (
                self._tp["sram_b_w"]
                + 2
                * self._wp["pitch"]
                * (
                    self._num_rw_ports
                    - 1
                    + (self._num_r_ports - self._num_se_r_ports)
                    + self._num_w_ports
                )
                + self._wp["pitch"] * self._num_se_r_ports
            )

    def compute_power(self):
        self.dynamic_power()
        self.static_power()

    def dynamic_power(self):
        raise NotImplementedError

    def static_power(self):
        raise NotImplementedError


def _htree_log2_floor(n):
    # CACTI's _log2 is a bit-shift floor-log2 on an integer.
    return int(n).bit_length() - 1 if n > 0 else 0
