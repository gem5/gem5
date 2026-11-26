# SPDX-License-Identifier: BSD-3-Clause
from dataclasses import dataclass
from math import (
    ceil,
    log,
    log2,
    sqrt,
)

from .cacti_circuit import CactiCircuit
from .cacti_decoders import (
    CactiDecoder,
    CactiDriver,
    CactiPredec,
    CactiPredecBlk,
    CactiPredecBlkDrv,
)
from .cacti_params import CactiParams
from .cacti_primitives import (
    CactiComponent,
    ComponentPower,
    PowerStats,
    _htree_log2_floor,
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


class CactiSubarray(CactiComponent):
    NUM_BITS_PER_ECC_B = 8.0

    def __init__(
        self,
        name,
        cacti_params: CactiParams,
        rows,
        cols,
        add_ecc=False,
        num_rw_ports=1,
        num_r_ports=0,
        num_w_ports=0,
        num_se_r_ports=0,
        num_sr_ports=0,
        is_fa=False,
        pure_cam=False,
        num_cols_fa_cam=0,
        num_cols_fa_ram=0,
        cell_h_override=None,
        cell_w_override=None,
    ):
        super().__init__(
            name,
            cacti_params,
            num_rw_ports=num_rw_ports,
            num_r_ports=num_r_ports,
            num_w_ports=num_w_ports,
            num_se_r_ports=num_se_r_ports,
            num_sr_ports=num_sr_ports,
            is_fa=is_fa,
            pure_cam=pure_cam,
        )
        # Opt-in cell-dimension override (the router configuration,
        # cacti_noc.py / CactiMat's own identical override). Real CACTI's
        # Subarray(dp, is_fa) (mat.cc:47) reads dp.cell directly, the SAME
        # field Mat itself uses -- there is only ONE cell-dimension source
        # of truth in the C++. This port instead gives Subarray its own
        # independent init_cell_area() (correct for every array whose cell
        # dims are the standard local-pitch formula, which is every one
        # except the router's), so CactiMat must forward any override it
        # received down into its Subarray construction too -- confirmed
        # necessary by measurement: without this, subarray.C_bl (and
        # therefore bitline/output-driver energy) used the STANDARD
        # (local-pitch) cell dims even when the Mat itself used the
        # wire-outside-mat-pitch override, a real, silent two-cell-
        # dimensions bug for exactly this preset. Both default to None
        # (no-op), so every existing caller is unchanged.
        if cell_h_override is not None:
            self._cell_h = cell_h_override
        if cell_w_override is not None:
            self._cell_w = cell_w_override
        self._rows = rows
        self._cols = cols
        self._add_ecc = add_ecc
        # Raw (pre subarray-ECC) FA column splits: dp.tag_num_c_subarray (CAM/tag
        # part) and dp.data_num_c_subarray (RAM/data part). subarray.cc:67-68
        # applies ECC to EACH part independently (ceil(part/8)), separate from
        # the ECC already folded into DP's num_do_b_mat/num_di_b_mat H-tree widths.
        self._num_cols_fa_cam = num_cols_fa_cam
        self._num_cols_fa_ram = num_cols_fa_ram
        self.calc_dimensions()
        self.compute_area()
        self.compute_capacitance()

    def calc_dimensions(self):
        if not (self._is_fa or self._is_pure_cam):
            # ECC overhead (subarray.cc:51): one check bit per 8 data bits.
            # McPAT enables ECC for every array; standalone CACTI configs with
            # "Add ECC false" pass add_ecc=False and keep num_cols unchanged.
            if self._add_ecc:
                self._cols += int(ceil(self._cols / self.NUM_BITS_PER_ECC_B))
        else:
            if self._is_fa:
                if self._add_ecc:
                    self._num_cols_fa_cam += int(
                        ceil(self._num_cols_fa_cam / self.NUM_BITS_PER_ECC_B)
                    )
                    self._num_cols_fa_ram += int(
                        ceil(self._num_cols_fa_ram / self.NUM_BITS_PER_ECC_B)
                    )
                self._cols = self._num_cols_fa_cam + self._num_cols_fa_ram
            else:
                if self._add_ecc:
                    self._num_cols_fa_cam += int(
                        ceil(self._num_cols_fa_cam / self.NUM_BITS_PER_ECC_B)
                    )
                self._num_cols_fa_ram = 0
                self._cols = self._num_cols_fa_cam

    def compute_area(self):
        # Port of Subarray::area.w/.h (subarray.cc:49-83). SRAM/CAM cell
        # dimensions only: ram_cell_tech_type only ever carries HP/LSTP/LOP
        # process-corner values in this project (never lp_dram/comm_dram --
        # confirmed via every sweep_traces record), matching
        # CactiComponent's own self._is_dram=False / init_cell_area()'s
        # SRAM-only _cell_w/_cell_h, and compute_capacitance()'s explicit
        # `raise NotImplementedError` for the DRAM case above. So the real
        # ram_num_cells_wl_stitching selector (sram=16 / lp_dram=64 /
        # comm_dram=256) is deliberately not ported -- only the sram=16
        # branch is reachable, same "kept permanently unimplemented, not a
        # silent gap" style as _is_pure_cam elsewhere in this file.
        SRAM_NUM_CELLS_WL_STITCHING = 16
        if not (self._is_fa or self._is_pure_cam):
            self.area_h = self._cell_h * self._rows
            # CACTI's ceil() acts on an already-integer division (both
            # operands are ints in C++), so it's really floor, not a true
            # ceiling -- Python's `//` replicates this exactly.
            self.area_w = (
                self._cell_w * self._cols
                + (self._cols // SRAM_NUM_CELLS_WL_STITCHING)
                * self._tp["ram_wl_stitching_overhead"]
            )
        else:
            # subarray.cc:78: "+1" is a dummy row -- blank space in the SRAM
            # half is filled with dummy cells, so subarray height is decided
            # by the CAM array alone, one row taller than num_rows.
            self.area_h = self._cam_cell_h * (self._rows + 1)
            self.area_w = (
                self._cam_cell_w * self._num_cols_fa_cam
                + self._cell_w * self._num_cols_fa_ram
                + (
                    (self._num_cols_fa_cam + self._num_cols_fa_ram)
                    // SRAM_NUM_CELLS_WL_STITCHING
                )
                * self._tp["ram_wl_stitching_overhead"]
                + 16 * self._wp["pitch"]  # NAND gate connecting the two halves
                + 128
                * self._wp["pitch"]  # matchline-to-wordline-driver overhead
            )

    def compute_capacitance(self):
        c_w_metal = self._cell_w * self._wp["C_per_micron"]
        r_w_metal = self._cell_w * self._wp["R_per_micron"]
        C_b_metal = self._cell_h * self._wp["C_per_micron"]
        if self._is_dram:
            raise NotImplementedError
        else:
            if not (self._is_fa or self._is_pure_cam):
                # subarray.cc: C_wl = (gate_C_pass(cell_a_w,..)*2 + c_w_metal)*num_cols.
                # There are two access transistors per 6T cell on the wordline,
                # plus the wordline metal wire cap (c_w_metal) per cell.
                self._C_wl = (
                    self.gate_C(
                        self._tp["sram_cell_a_w"],
                        (self._tp["sram_b_w"] - 2 * self._tp["sram_cell_a_w"])
                        / 2.0,
                    )
                    * 2
                    + c_w_metal
                ) * self._cols
                # subarray.cc:147 divides the row drain cap by 2 (shared bitline
                # contact between vertically adjacent cells) and uses the SRAM
                # cell device (is_cell=True).
                C_b_row_drain_C = (
                    self.drain_C(
                        self._tp["sram_cell_a_w"],
                        "NCH",
                        1,
                        0,
                        self._cell_w,
                        is_cell=True,
                    )
                    / 2.0
                )
                self._C_bl = self._rows * (C_b_row_drain_C + C_b_metal)
            else:
                c_w_metal = self._cam_cell_w * self._wp["C_per_micron"]
                r_w_metal = self._cam_cell_w * self._wp["R_per_micron"]
                C_wl_cam = (
                    (
                        self.gate_C(
                            self._tp["cam_cell_a_w"],
                            (
                                self._tp["cam_b_w"]
                                - 2 * self._tp["cam_cell_a_w"]
                            )
                            / 2.0,
                        )
                    )
                    * 2
                    + c_w_metal
                ) * self._num_cols_fa_cam
                R_wl_cam = r_w_metal * self._num_cols_fa_cam
                if not self._is_pure_cam:
                    c_w_metal = self._cell_w * self._wp["C_per_micron"]
                    r_w_metal = self._cell_w * self._wp["R_per_micron"]
                    C_wl_ram = (
                        (
                            self.gate_C(
                                self._tp["sram_cell_a_w"],
                                (
                                    self._tp["sram_b_w"]
                                    - 2 * self._tp["sram_cell_a_w"]
                                )
                                / 2,
                            )
                        )
                        * 2
                        + c_w_metal
                    ) * self._num_cols_fa_ram
                    R_wl_ram = r_w_metal * self._num_cols_fa_ram
                else:
                    C_wl_ram = R_wl_ram = 0
                # Exposed as attributes (subarray.h keeps these distinct from
                # the combined C_wl/R_wl): mat.cc's ml_to_ram_wl_drv loads the
                # RAM-only wordline wire, not the CAM+RAM combined C_wl.
                self._C_wl_ram = C_wl_ram
                self._R_wl_ram = R_wl_ram
                self._C_wl = (
                    C_wl_cam
                    + C_wl_ram
                    + (
                        (16 + 128)
                        * self._wp["pitch"]
                        * self._wp["C_per_micron"]
                    )
                )
                self._R_wl = (
                    R_wl_cam
                    + R_wl_ram
                    + (
                        (16 + 128)
                        * self._wp["pitch"]
                        * self._wp["R_per_micron"]
                    )
                )
                C_b_metal = self._cam_cell_h * self._wp["C_per_micron"]
                C_b_row_drain_C = (
                    self.drain_C(
                        self._tp["cam_cell_a_w"],
                        "NCH",
                        1,
                        0,
                        self._cam_cell_w,
                        is_cell=True,
                    )
                    / 2
                )
                self._C_bl_cam = (self._rows + 1) * (
                    C_b_row_drain_C + C_b_metal
                )
                C_b_row_drain_C = (
                    self.drain_C(
                        self._tp["sram_cell_a_w"],
                        "NCH",
                        1,
                        0,
                        self._cell_w,
                        is_cell=True,
                    )
                    / 2
                )
                self._C_bl = (self._rows + 1) * (C_b_row_drain_C + C_b_metal)


class CactiMat(CactiComponent):
    def __init__(
        self,
        name,
        cacti_params,
        dyn_p,
        mat_area_w=0.0,
        mat_area_h=0.0,
        cell_h_override=None,
        cell_w_override=None,
        v_b_sense_override=None,
    ):
        cfg = dyn_p.cfg
        super().__init__(
            name,
            cacti_params,
            num_rw_ports=cfg.num_rw_ports,
            num_r_ports=cfg.num_rd_ports,
            num_w_ports=cfg.num_wr_ports,
            num_se_r_ports=cfg.num_se_rd_ports,
            num_sr_ports=cfg.num_search_ports,
            is_fa=dyn_p.fully_assoc,
            pure_cam=getattr(dyn_p, "pure_cam", False),
        )

        # Opt-in cell-dimension override (the router configuration,
        # cacti_noc.py / Router::buffer_stats). init_cell_area() (just run
        # by super().__init__() above) always widens cell.h/cell.w by
        # g_tp.wire_LOCAL.pitch per port -- correct for every normal
        # DynamicParameter ctor (parameter.cc:348-360), which is what every
        # other caller of this class relies on. router.cc's OWN
        # DynamicParameter construction (buffer_stats(), router.cc:190-201)
        # is a real, documented exception: it widens by
        # g_tp.wire_OUTSIDE_mat.pitch instead (the wire_os_mat_type-selected
        # tier, = inside_mat/semi-global for the embedded NoC, not local) --
        # a genuine CACTI quirk specific to the router's VC-buffer Mat, not
        # a general parameter.cc formula. Both params default to None
        # (no-op) so every existing caller/behavior is unchanged; only
        # cacti_noc.CactiRouter passes them, pre-computed with the correct
        # (wire_os_mat_type) pitch.
        if cell_h_override is not None:
            self._cell_h = cell_h_override
        if cell_w_override is not None:
            self._cell_w = cell_w_override

        # Opt-in V_b_sense override (Task 5, same call site). init_v_b_sense()
        # (also just run by super().__init__()) always derives
        # self._V_b_sense as max(0.05*Vdd, VBITSENSEMIN) -- correct for
        # every normal DynamicParameter. router.cc's buffer_stats() sets
        # dyn_p.V_b_sense = Vdd (router.cc:174, "// FIXME check power
        # calc.") -- a real, documented exception (full peri Vdd, not the
        # usual 5%-of-Vdd bitline sense margin) that this port's
        # independently-recomputed self._V_b_sense cannot see otherwise.
        if v_b_sense_override is not None:
            self._V_b_sense = v_b_sense_override

        # 1. Take the array geometry (subarray dims, mats, muxing, active-mat
        #    count, ...) from the CactiDynamicParameter (port of parameter.cc).
        self.set_dynamic_parameters(dyn_p)

        # Physical mat dimensions (mat.cc:445-446, Mat::compute_area -- NOT ported,
        # out of scope per the energy-only decision; area computation would need
        # every predecoder/decoder/driver sub-component's own area). Needed only
        # for multi-mat/multi-bank H-tree geometry (CactiBank/CactiUCA); taken as a
        # given input (like the 5 partition integers) from a one-time CACTI run,
        # rather than derived. Single-mat single-bank arrays never construct a
        # nonzero H-tree, so this is unused (and may be left at its 0.0 default).
        self.area_w = mat_area_w
        self.area_h = mat_area_h

        # In-mat routing wires (bit/sa-mux decoder output, predecode output,
        # precharge driver) load g_tp.wire_inside_mat, which CACTI populates from
        # g_ip->wire_is_mat_type.
        def _layer_for(mat_type):
            if mat_type == 0:
                return "local"  # local (2.5F)
            elif mat_type == 2:
                return "global"  # global (8F)
            else:
                return "inside_mat"  # semi-global (4F)

        layer = _layer_for(dyn_p.cfg.wire_is_mat_type)
        if layer == "local":
            self._C_per_micron_mat = self._wp["C_per_micron"]
            self._R_per_micron_mat = self._wp["R_per_micron"]
        elif layer == "global":
            self._C_per_micron_mat = self._wp["C_per_micron_global"]
            self._R_per_micron_mat = self._wp["R_per_micron_global"]
        else:
            self._C_per_micron_mat = self._wp["C_per_micron_inside_mat"]
            self._R_per_micron_mat = self._wp["R_per_micron_inside_mat"]
        # g_tp.wire_inside_mat.pitch (mat.cc:371/382/386/390/398/404), same
        # wire_is_mat_type-selected tier as _C/R_per_micron_mat above --
        # Phase 30's row-predecode-output/mux-decode-out/addr-datain wire
        # height/width terms.
        self._pitch_mat = self._wp[
            {
                "local": "pitch",
                "global": "pitch_global",
                "inside_mat": "pitch_inside_mat",
            }[layer]
        ]

        # Subarray output wire (mat.cc:267, new Wire(Global, ..., inside_mat)).
        # Its repeater sizing/per-length energy come from CACTI's Wire::global,
        # a value that is lazily computed ONCE per array by the first Wire built
        # during that array's own optimizer search -- via Wire's default
        # constructor, whose wire_placement default is outside_mat (wire.h:52/60).
        # That default reads g_tp.wire_outside_mat, which CACTI populates from
        # g_ip->wire_os_mat_type -- NOT wire_is_mat_type, even though the actual
        # subarray_out_wire instance itself is later built with placement
        # inside_mat. wire_os_mat_type defaults to wire_is_mat_type (so this is
        # invisible whenever the two agree, e.g. every array validated before
        # L2), but L2/L3 set wire_os_mat_type=1 (semi-global) while
        # wire_is_mat_type stays 0 (local) -- keep this quirk, don't fix it.
        out_layer = _layer_for(dyn_p.cfg.wire_os_mat_type)
        pitch_key = {
            "local": "pitch",
            "global": "pitch_global",
            "inside_mat": "pitch_inside_mat",
        }[out_layer]
        self._out_wire_pitch_um = self._wp[pitch_key]
        self._out_wire_aspect = self._wp["aspect_ratio_" + out_layer]
        self._out_wire_horiz = self._wp["horiz_dielec_" + out_layer]
        self._out_wire_vert = self._wp["vert_dielec_" + out_layer]
        self._out_wire_ild = self._wp["ild_" + out_layer]
        # fringe_cap is 0.115 fF/um at every layer/node; the semi-global layer
        # does not store it separately.
        if out_layer == "inside_mat":
            self._out_wire_fringe = 0.115e-15
        else:
            self._out_wire_fringe = self._wp["fringe_" + out_layer]

        # Bitline precharge/equalization driver wire load (mat.cc:301-303) is ALSO
        # keyed by wire_os_mat_type / g_tp.wire_outside_mat, not wire_is_mat_type --
        # the same quirk as the subarray output wire above, just for a simple
        # C_per_um/R_per_um pair instead of full geometry. Invisible whenever
        # wire_is_mat_type == wire_os_mat_type (every array validated before
        # L2/L2Directory); found via their read/write dynamic residual (~1.25e-15 J).
        if out_layer == "local":
            self._C_per_micron_out = self._wp["C_per_micron"]
            self._R_per_micron_out = self._wp["R_per_micron"]
        elif out_layer == "global":
            self._C_per_micron_out = self._wp["C_per_micron_global"]
            self._R_per_micron_out = self._wp["R_per_micron_global"]
        else:
            self._C_per_micron_out = self._wp["C_per_micron_inside_mat"]
            self._R_per_micron_out = self._wp["R_per_micron_inside_mat"]

        # Adjust layout assumptions for fully associative / CAM arrays using natively calculated vars
        if self._is_fa or self._is_pure_cam:
            if self._num_subarrays_per_mat > 2:
                self._num_subarrays_per_row = self._num_subarrays_per_mat // 2
            else:
                self._num_subarrays_per_row = self._num_subarrays_per_mat

        # 2. Instantiate Core Subarray cleanly - passing ONLY its specific partition sizes
        self.subarray = CactiSubarray(
            f"{name}_subarray",
            cacti_params,
            rows=self._subarray_rows,
            cols=self._subarray_cols,
            add_ecc=dyn_p.add_ecc,
            num_rw_ports=cfg.num_rw_ports,
            num_r_ports=cfg.num_rd_ports,
            num_w_ports=cfg.num_wr_ports,
            num_se_r_ports=cfg.num_se_rd_ports,
            num_sr_ports=cfg.num_search_ports,
            is_fa=dyn_p.fully_assoc,
            pure_cam=getattr(dyn_p, "pure_cam", False),
            num_cols_fa_cam=getattr(dyn_p, "tag_num_c_subarray", 0),
            num_cols_fa_ram=getattr(dyn_p, "data_num_c_subarray", 0),
            cell_h_override=cell_h_override,
            cell_w_override=cell_w_override,
        )

        num_row_dec_signals = self.subarray._rows
        if self._is_fa or self._is_pure_cam:
            # mat.cc:223-224: the row decoder must also select which of the
            # num_subarrays_per_mat subarrays is active (FA/CAM route this
            # through the row address too, unlike a normal cache mat).
            num_row_dec_signals += _htree_log2_floor(
                self._num_subarrays_per_mat
            )

        active_cell_w = self._cam_cell_w if self._is_cam else self._cell_w
        active_cell_h = self._cam_cell_h if self._is_cam else self._cell_h

        # 3. Wire Loads for Decoders driving the Subarray. CACTI derives these
        # from subarray.num_cols, i.e. the POST-ECC column count (mat.cc:96-147),
        # not the pre-ECC dp.num_c_subarray.
        sub_cols = self.subarray._cols
        if not self._is_fa and not self._is_pure_cam:
            number_sa_subarray = sub_cols // self._deg_bl_muxing
            R_wire_wl_drv_out = (
                sub_cols * self._cell_w * self._wp["R_per_micron"]
            )
        elif self._is_fa and not self._is_pure_cam:
            # Note: For FA/CAM, we read the specific internal splits calculated by the Subarray itself
            number_sa_subarray = (
                self.subarray._num_cols_fa_cam + self.subarray._num_cols_fa_ram
            ) // self._deg_bl_muxing
            R_wire_wl_drv_out = (
                self.subarray._num_cols_fa_cam * self._cam_cell_w
                + self.subarray._num_cols_fa_ram * self._cell_w
            ) * self._wp["R_per_micron"]
        else:
            number_sa_subarray = (
                getattr(self.subarray, "_num_cols_fa_cam", sub_cols)
                // self._deg_bl_muxing
            )
            R_wire_wl_drv_out = (
                getattr(self.subarray, "_num_cols_fa_cam", sub_cols)
                * self._cam_cell_w
            ) * self._wp["R_per_micron"]

        R_wire_bit_mux_dec_out = (
            self._num_subarrays_per_row
            * sub_cols
            * self._wp["R_per_micron"]
            * self._cell_w
        )
        R_wire_sa_mux_dec_out = (
            self._num_subarrays_per_row
            * sub_cols
            * self._wp["R_per_micron"]
            * self._cell_w
        )

        C_ld_bit_mux_dec_out = 0.0
        # The mux-decoder output wire load spans the mat (num_subarrays_per_row
        # subarrays wide) and uses the SEMI-GLOBAL layer (wire_inside_mat), not the
        # local wire -- mat.cc:134/141/147.
        if self._deg_bl_muxing > 1:
            C_ld_bit_mux_dec_out = (
                2
                * self._num_subarrays_per_mat
                * sub_cols
                / self._deg_bl_muxing
            ) * self.gate_C(
                self._tp["w_nmos_b_mux"],
                0,
                is_dram=self._is_dram,
                is_wl_tr=False,
            ) + self._num_subarrays_per_row * sub_cols * self._C_per_micron_mat * self._cell_w

        C_ld_sa_mux_lev_1_dec_out = 0.0
        if self._Ndsam_lev_1 > 1:
            C_ld_sa_mux_lev_1_dec_out = (
                self._num_subarrays_per_mat
                * number_sa_subarray
                / self._Ndsam_lev_1
            ) * self.gate_C(
                self._tp["w_nmos_sa_mux"],
                0,
                is_dram=self._is_dram,
                is_wl_tr=False,
            ) + self._num_subarrays_per_row * sub_cols * self._C_per_micron_mat * self._cell_w

        C_ld_sa_mux_lev_2_dec_out = 0.0
        if self._Ndsam_lev_2 > 1:
            C_ld_sa_mux_lev_2_dec_out = (
                self._num_subarrays_per_mat
                * number_sa_subarray
                / (self._Ndsam_lev_1 * self._Ndsam_lev_2)
            ) * self.gate_C(
                self._tp["w_nmos_sa_mux"],
                0,
                is_dram=self._is_dram,
                is_wl_tr=False,
            ) + self._num_subarrays_per_row * sub_cols * self._C_per_micron_mat * self._cell_w

        if self._num_subarrays_per_row >= 2:
            R_wire_bit_mux_dec_out /= 2.0
            R_wire_sa_mux_dec_out /= 2.0

        # 4. Instantiate Final Stage Decoders.
        # NOTE: CactiDecoder/PredecBlk/PredecBlkDrv/Predec derive everything they
        # need from the loads passed to them; they do NOT take the array-shape
        # (total_rows/cols/num_subarrays/num_mats/associativity) arguments. Those
        # were being passed positionally here and shadowed the real constructor
        # parameters (num_dec_signals, flag_way_select, ...), which is a TypeError.
        self.row_dec = CactiDecoder(
            f"{name}_row_dec",
            cacti_params,
            num_dec_signals=num_row_dec_signals,
            flag_way_select=False,
            C_ld_dec_out=self.subarray._C_wl,
            R_wire_dec_out=R_wire_wl_drv_out,
            is_wl_tr=True,
            cell_h=active_cell_h,
            cell_w=active_cell_w,
        )

        self.bit_mux_dec = CactiDecoder(
            f"{name}_bit_mux_dec",
            cacti_params,
            num_dec_signals=self._deg_bl_muxing,
            flag_way_select=False,
            C_ld_dec_out=C_ld_bit_mux_dec_out,
            R_wire_dec_out=R_wire_bit_mux_dec_out,
            is_wl_tr=False,
            cell_h=active_cell_h,
            cell_w=active_cell_w,
        )

        self.sa_mux_lev_1_dec = CactiDecoder(
            f"{name}_sa_mux_lev_1_dec",
            cacti_params,
            num_dec_signals=self._deg_senseamp_muxing_non_associativity,
            flag_way_select=(
                True if self._number_way_select_signals_mat else False
            ),
            C_ld_dec_out=C_ld_sa_mux_lev_1_dec_out,
            R_wire_dec_out=R_wire_sa_mux_dec_out,
            is_wl_tr=False,
            cell_h=active_cell_h,
            cell_w=active_cell_w,
        )

        self.sa_mux_lev_2_dec = CactiDecoder(
            f"{name}_sa_mux_lev_2_dec",
            cacti_params,
            num_dec_signals=self._Ndsam_lev_2,
            flag_way_select=False,
            C_ld_dec_out=C_ld_sa_mux_lev_2_dec_out,
            R_wire_dec_out=R_wire_sa_mux_dec_out,
            is_wl_tr=False,
            cell_h=active_cell_h,
            cell_w=active_cell_w,
        )

        # 5. Wire Loads for Predecoders (Global predecode signals spanning the Mat)
        # mat.cc:212/214-215 uses g_tp.wire_inside_mat for the predecode wire
        # load. g_tp.wire_inside_mat is itself indexed by g_ip->wire_is_mat_type
        # (technology.cc:2993-3000) -- the same selector self._C_per_micron_mat/
        # _R_per_micron_mat above already dispatch on via _layer_for() -- so
        # reusing them here is correct, not an approximation (confirmed via a
        # live probe, Phase 29; an earlier Phase 29 attempt "fixed" this site
        # to always use the inside_mat/index-1 tier, which was wrong -- the
        # real bug turned out to be in cacti_wire_params.py's R_per_micron_
        # global formula, see the native reference).
        # FA/CAM (mat.cc:216-220) drops the num_subarrays_per_row factor -- the
        # predecode wire load is just subarray.num_rows * cam_cell.h there.
        if self._is_fa or self._is_pure_cam:
            L_wire_predec_blk_out = self.subarray._rows * active_cell_h
        else:
            L_wire_predec_blk_out = (
                self._num_subarrays_per_row
                * self.subarray._rows
                * active_cell_h
            )
        C_wire_predec_blk_out = L_wire_predec_blk_out * self._C_per_micron_mat
        R_wire_predec_blk_out = L_wire_predec_blk_out * self._R_per_micron_mat

        # In CACTI, num_dec_per_predec is the number of row decoders that share
        # one predecoder, i.e. the number of subarrays per mat (mat.cc passes
        # num_subarrays_per_mat) -- NOT the subarray row count.
        num_dec_per_predec = self._num_subarrays_per_mat

        # 6. Instantiate Predecoders
        self.r_predec_blk1 = CactiPredecBlk(
            f"{name}_r_predec_blk1",
            cacti_params,
            num_row_dec_signals,
            self.row_dec,
            C_wire_predec_blk_out,
            R_wire_predec_blk_out,
            num_dec_per_predec,
            True,
        )
        self.r_predec_blk2 = CactiPredecBlk(
            f"{name}_r_predec_blk2",
            cacti_params,
            num_row_dec_signals,
            self.row_dec,
            C_wire_predec_blk_out,
            R_wire_predec_blk_out,
            num_dec_per_predec,
            False,
        )

        self.r_predec_drv1 = CactiPredecBlkDrv(
            f"{name}_r_predec_drv1", cacti_params, False, self.r_predec_blk1
        )
        self.r_predec_drv2 = CactiPredecBlkDrv(
            f"{name}_r_predec_drv2", cacti_params, False, self.r_predec_blk2
        )

        self.r_predec = CactiPredec(
            f"{name}_r_predec",
            cacti_params,
            self.r_predec_drv1,
            self.r_predec_drv2,
        )

        # Only create bitline MUX predecoders if we actually multiplex the bitlines.
        # For the bit-mux predecoder the decoder output load is carried by the
        # bit_mux_dec itself; CACTI passes zero wire load and num_dec_per_predec=1.
        if self._deg_bl_muxing > 1:
            self.b_mux_predec_blk1 = CactiPredecBlk(
                f"{name}_b_mux_predec_blk1",
                cacti_params,
                self._deg_bl_muxing,
                self.bit_mux_dec,
                0.0,
                0.0,
                1,
                True,
            )
            self.b_mux_predec_blk2 = CactiPredecBlk(
                f"{name}_b_mux_predec_blk2",
                cacti_params,
                self._deg_bl_muxing,
                self.bit_mux_dec,
                0.0,
                0.0,
                1,
                False,
            )

            self.b_mux_predec_drv1 = CactiPredecBlkDrv(
                f"{name}_b_mux_predec_drv1",
                cacti_params,
                False,
                self.b_mux_predec_blk1,
            )
            self.b_mux_predec_drv2 = CactiPredecBlkDrv(
                f"{name}_b_mux_predec_drv2",
                cacti_params,
                False,
                self.b_mux_predec_blk2,
            )

            self.b_mux_predec = CactiPredec(
                f"{name}_b_mux_predec",
                cacti_params,
                self.b_mux_predec_drv1,
                self.b_mux_predec_drv2,
            )
            self._has_b_mux = True
        else:
            self._has_b_mux = False

        # Sense-amp mux level-1/2 predecoders (mat.cc:244-247). Like the bit-mux
        # predecoder, the decoder output load is carried by the sa_mux decoders
        # themselves, so CACTI passes zero wire load and num_dec_per_predec=1.
        # For narrow mux degrees the PredecBlk/PredecBlkDrv simply do not "exist"
        # and contribute zero (ram_test: Ndsam1=Ndsam2=1 -> all zero).
        self.sa_mux_lev_1_predec_blk1 = CactiPredecBlk(
            f"{name}_sa_mux_lev_1_predec_blk1",
            cacti_params,
            self._deg_senseamp_muxing_non_associativity,
            self.sa_mux_lev_1_dec,
            0.0,
            0.0,
            1,
            True,
        )
        self.sa_mux_lev_1_predec_blk2 = CactiPredecBlk(
            f"{name}_sa_mux_lev_1_predec_blk2",
            cacti_params,
            self._deg_senseamp_muxing_non_associativity,
            self.sa_mux_lev_1_dec,
            0.0,
            0.0,
            1,
            False,
        )
        self.sa_mux_lev_1_predec_drv1 = CactiPredecBlkDrv(
            f"{name}_sa_mux_lev_1_predec_drv1",
            cacti_params,
            False,
            self.sa_mux_lev_1_predec_blk1,
        )
        self.sa_mux_lev_1_predec_drv2 = CactiPredecBlkDrv(
            f"{name}_sa_mux_lev_1_predec_drv2",
            cacti_params,
            False,
            self.sa_mux_lev_1_predec_blk2,
        )
        self.sa_mux_lev_1_predec = CactiPredec(
            f"{name}_sa_mux_lev_1_predec",
            cacti_params,
            self.sa_mux_lev_1_predec_drv1,
            self.sa_mux_lev_1_predec_drv2,
        )

        self.sa_mux_lev_2_predec_blk1 = CactiPredecBlk(
            f"{name}_sa_mux_lev_2_predec_blk1",
            cacti_params,
            self._Ndsam_lev_2,
            self.sa_mux_lev_2_dec,
            0.0,
            0.0,
            1,
            True,
        )
        self.sa_mux_lev_2_predec_blk2 = CactiPredecBlk(
            f"{name}_sa_mux_lev_2_predec_blk2",
            cacti_params,
            self._Ndsam_lev_2,
            self.sa_mux_lev_2_dec,
            0.0,
            0.0,
            1,
            False,
        )
        self.sa_mux_lev_2_predec_drv1 = CactiPredecBlkDrv(
            f"{name}_sa_mux_lev_2_predec_drv1",
            cacti_params,
            False,
            self.sa_mux_lev_2_predec_blk1,
        )
        self.sa_mux_lev_2_predec_drv2 = CactiPredecBlkDrv(
            f"{name}_sa_mux_lev_2_predec_drv2",
            cacti_params,
            False,
            self.sa_mux_lev_2_predec_blk2,
        )
        self.sa_mux_lev_2_predec = CactiPredec(
            f"{name}_sa_mux_lev_2_predec",
            cacti_params,
            self.sa_mux_lev_2_predec_drv1,
            self.sa_mux_lev_2_predec_drv2,
        )

        # Way-select driver (mat.cc:247-258, Phase 30's area_mat_center_
        # circuitry term). dummy_way_sel_predec_blk1 is a 1-signal PredecBlk
        # (num_dec_signals=1 -> num_addr_bits_dec=0 -> exist stays False, so
        # this object contributes zero delay/energy anywhere it's read) that
        # exists only to give way_sel_drv1 a `.dec` (sa_mux_lev_1_dec). The
        # mirrored "2" pair (dummy_way_sel_predec_blk2/dummy_way_sel_predec_
        # blk_drv2) is confirmed dead in real CACTI -- constructed and
        # deleted there but never read by anything else -- so it is not
        # ported here.
        self.dummy_way_sel_predec_blk1 = CactiPredecBlk(
            f"{name}_dummy_way_sel_predec_blk1",
            cacti_params,
            1,
            self.sa_mux_lev_1_dec,
            0.0,
            0.0,
            0,
            True,
        )
        self.way_sel_drv1 = CactiPredecBlkDrv(
            f"{name}_way_sel_drv1",
            cacti_params,
            self._number_way_select_signals_mat,
            self.dummy_way_sel_predec_blk1,
        )

        # 7. Bitline / sense-amp / precharge datapath (the dominant read/write
        #    energy). A single read/write port is assumed (RWP=1, ERP=EWP=SCHP=0).
        #    number_sa_subarray = subarray.num_cols / deg_bl_muxing (mat.cc:1510):
        #    every bitline column has a sense amp; the sa-mux only selects which
        #    outputs propagate (it does NOT reduce the sense-amp count).
        self._num_sa_subarray = max(
            1, self.subarray._cols // self._deg_bl_muxing
        )

        self._power_bitline = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self._power_sa = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self._power_subarray_out_drv = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self._power_comparator = ComponentPower(
            PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
        )
        self._out_wire_dynamic = 0.0
        # Phase 29: zero-initialized ONCE here (mirrors mat.cc:60's
        # `delay_subarray_out_drv(0)` constructor init), accumulated via +=
        # across compute_subarray_out_drv's 4 stages with NO reset inside
        # that method -- matches real CACTI exactly. A validation harness
        # must construct a fresh CactiMat per test case, never call
        # compute_subarray_out_drv/compute_delays twice on one instance.
        self.delay_subarray_out_drv = 0.0
        self.delay_wl_reset = 0.0
        self.delay_bl_restore = 0.0
        self.delay_bitline = 0.0
        self.delay_sa = 0.0
        self.compute_bitline_power()
        self.compute_sa_power()
        self.compute_subarray_out_drv_power()
        # Comparator (mat.cc:641, is_tag && !fully_assoc -- FA is out of scope, so
        # just cfg.is_tag). Stays permanently 0 for data arrays, matching CACTI
        # exactly (compute_comparator_delay is simply never called there).
        if cfg.is_tag:
            self.compute_comparator_power()

        # Bitline precharge / equalization driver (mat.cc: one Driver charges all
        # the precharge PMOS gates across the subarray's columns, plus the wire).
        p_to_n = self.pmos_to_nmos_sz_ratio()
        w_pmos_bl_precharge = 6 * p_to_n * self._tp["min_w_nmos"]
        w_pmos_bl_eq = p_to_n * self._tp["min_w_nmos"]
        if self._is_fa or self._is_pure_cam:
            # mat.cc:273-296: FA/CAM builds TWO precharge drivers -- one for the
            # CAM (tag) columns (cam_cell.w), one for the RAM (data) columns
            # (cell.w, only when not pure_cam).
            driver_c_gate_load = self.subarray._num_cols_fa_cam * self.gate_C(
                2 * w_pmos_bl_precharge + w_pmos_bl_eq,
                0,
                is_dram=self._is_dram,
            )
            driver_c_wire_load = (
                self.subarray._num_cols_fa_cam
                * self._cam_cell_w
                * self._C_per_micron_out
            )
            driver_r_wire_load = (
                self.subarray._num_cols_fa_cam
                * self._cam_cell_w
                * self._R_per_micron_out
            )
            self.cam_bl_precharge_eq_drv = CactiDriver(
                f"{name}_cam_bl_precharge_eq_drv",
                cacti_params,
                driver_c_gate_load,
                driver_c_wire_load,
                driver_r_wire_load,
            )
            # REAL CACTI BUG (grepped every mat.cc call site): unlike every other
            # Driver built in this file, `cam_bl_precharge_eq_drv->compute_delay()`
            # is NEVER called anywhere -- its widths are computed (constructor
            # calls compute_widths/compute_area) but `power` is only populated
            # inside compute_delay(), which nothing invokes. So its dynamic/
            # leakage/gate_leakage are permanently zero in real CACTI, despite
            # the driver existing with correctly-sized transistors. CactiDriver
            # computes power eagerly in __init__ (matching every OTHER driver's
            # C++ compute_delay() being called somewhere), so zero it out here
            # to replicate the bug verbatim.
            self.cam_bl_precharge_eq_drv._power = ComponentPower(
                PowerStats(0, 0, 0), PowerStats(0, 0, 0), PowerStats(0, 0, 0)
            )

            if not self._is_pure_cam:
                driver_c_gate_load = (
                    self.subarray._num_cols_fa_ram
                    * self.gate_C(
                        2 * w_pmos_bl_precharge + w_pmos_bl_eq,
                        0,
                        is_dram=self._is_dram,
                    )
                )
                driver_c_wire_load = (
                    self.subarray._num_cols_fa_ram
                    * self._cell_w
                    * self._C_per_micron_out
                )
                driver_r_wire_load = (
                    self.subarray._num_cols_fa_ram
                    * self._cell_w
                    * self._R_per_micron_out
                )
                self.bl_precharge_eq_drv = CactiDriver(
                    f"{name}_bl_precharge_eq_drv",
                    cacti_params,
                    driver_c_gate_load,
                    driver_c_wire_load,
                    driver_r_wire_load,
                )
        else:
            driver_c_gate_load = self.subarray._cols * self.gate_C(
                2 * w_pmos_bl_precharge + w_pmos_bl_eq,
                0,
                is_dram=self._is_dram,
            )
            # mat.cc:301-303 loads the precharge driver wire with wire_OUTSIDE_mat
            # (g_ip->wire_os_mat_type), not the in-mat wire used by the other
            # in-mat routing wires above.
            driver_c_wire_load = (
                self.subarray._cols * self._cell_w * self._C_per_micron_out
            )
            driver_r_wire_load = (
                self.subarray._cols * self._cell_w * self._R_per_micron_out
            )
            self.bl_precharge_eq_drv = CactiDriver(
                f"{name}_bl_precharge_eq_drv",
                cacti_params,
                driver_c_gate_load,
                driver_c_wire_load,
                driver_r_wire_load,
            )

        if self._is_cam:
            self.compute_cam_search_power()

            # Search-chain Driver objects (mat.cc:823-953) -- their own
            # logical-effort-sized inverter-chain energy (CactiDriver), on top of
            # the manual comparator/NAND/NOR/hit-miss chain above.
            num_cols_fa_cam = self.subarray._num_cols_fa_cam
            num_rows = self.subarray._rows
            c_searchline_metal = self._cam_cell_h * self._wp["C_per_micron"]
            r_searchline_metal = self._cam_cell_h * self._wp["R_per_micron"]
            p_to_n = self.pmos_to_nmos_sz_ratio()
            w_pmos_bl_precharge = 6 * p_to_n * self._tp["min_w_nmos"]
            w_pmos_bl_eq = p_to_n * self._tp["min_w_nmos"]
            Wfaprechp = w_pmos_bl_precharge
            Wdummyn = self._tp["cam_cell_nmos_w"]
            W_hit_miss_n = Wdummyn

            # Searchline precharge/equalization driver (mat.cc:825-833). Routes
            # horizontally, so like the bitline precharge driver it loads the
            # OUTSIDE-mat wire.
            driver_c_gate_load = num_cols_fa_cam * self.gate_C(
                2 * w_pmos_bl_precharge + w_pmos_bl_eq,
                0,
                is_dram=self._is_dram,
            )
            driver_c_wire_load = (
                num_cols_fa_cam * self._cam_cell_w * self._C_per_micron_out
            )
            driver_r_wire_load = (
                num_cols_fa_cam * self._cam_cell_w * self._R_per_micron_out
            )
            self.sl_precharge_eq_drv = CactiDriver(
                f"{name}_sl_precharge_eq_drv",
                cacti_params,
                driver_c_gate_load,
                driver_c_wire_load,
                driver_r_wire_load,
            )

            # Searchline data driver (mat.cc:835-844) -- one gate per row + dummy
            # row; local wire (searchline metal), not outside-mat.
            driver_c_gate_load = (num_rows + 1) * self.gate_C(
                Wdummyn, 0, is_dram=self._is_dram
            )
            driver_c_wire_load = (num_rows + 1) * c_searchline_metal
            driver_r_wire_load = (num_rows + 1) * r_searchline_metal
            self.sl_data_drv = CactiDriver(
                f"{name}_sl_data_drv",
                cacti_params,
                driver_c_gate_load,
                driver_c_wire_load,
                driver_r_wire_load,
            )

            # Matchline precharge driver (mat.cc:862-872) -- also local wire.
            driver_c_gate_load = (num_rows + 1) * self.gate_C(
                Wfaprechp, 0, is_dram=self._is_dram
            )
            driver_c_wire_load = (num_rows + 1) * c_searchline_metal
            driver_r_wire_load = (num_rows + 1) * r_searchline_metal
            self.ml_precharge_drv = CactiDriver(
                f"{name}_ml_precharge_drv",
                cacti_params,
                driver_c_gate_load,
                driver_c_wire_load,
                driver_r_wire_load,
            )

            # Matchline -> RAM wordline driver (mat.cc:929-938): loads the RAM-
            # only wordline wire (subarray.C_wl_ram/R_wl_ram), not the combined
            # C_wl. Its own dynamic energy is added at the CactiBank level
            # (bank.cc:187), not here (mat.cc never sums it into the mat total --
            # the "power.searchOp.dynamic += ml_to_ram_wl_drv..." line is
            # commented out at mat.cc:1590/1644).
            driver_c_gate_load = self.gate_C(
                W_hit_miss_n, 0, is_dram=self._is_dram
            )
            driver_c_wire_load = self.subarray._C_wl_ram
            driver_r_wire_load = self.subarray._R_wl_ram
            self.ml_to_ram_wl_drv = CactiDriver(
                f"{name}_ml_to_ram_wl_drv",
                cacti_params,
                driver_c_gate_load,
                driver_c_wire_load,
                driver_r_wire_load,
            )

        self.compute_power()

    def compute_bitline_power(self):
        # Port of the SRAM energy terms of Mat::compute_bitline_delay (the timing
        # terms -- tau, tstep, Vth, Vbitpre -- are omitted as they do not affect
        # energy). Energies here are PER BITLINE COLUMN (read/write) and PER CELL
        # (leakage); they are scaled by the column / cell counts in static/dynamic
        # aggregation, matching compute_power_energy.
        deg_bl_muxing = self._deg_bl_muxing
        deg_senseamp_muxing = self._Ndsam_lev_1 * self._Ndsam_lev_2
        Vdd = self._tp["Vdd"]  # cell supply (single-Vdd port -> peri Vdd)
        V_b_sense = self._V_b_sense

        C_bl = self.subarray._C_bl

        # SRAM cell static leakage. is_cell selects the cell device; this port's
        # leakage helpers currently reuse the peripheral I_off, so this is an
        # upper bound on true (higher-Vth) cell leakage.
        Iport = self.cmos_Isub_leakage(
            self._tp["sram_cell_a_w"],
            0,
            1,
            "NMOS",
            is_dram=self._is_dram,
            is_cell=True,
        )
        # A read-only (single-ended) port stacks two access transistors (fanin 2).
        Iport_erp = self.cmos_Isub_leakage(
            self._tp["sram_cell_a_w"],
            0,
            2,
            "NMOS",
            is_dram=self._is_dram,
            is_cell=True,
        )
        Icell = (
            self.cmos_Isub_leakage(
                self._tp["sram_cell_nmos_w"],
                self._tp["sram_cell_pmos_w"],
                1,
                "INV",
                is_dram=self._is_dram,
                is_cell=True,
            )
            * 2
        )
        Ig_cell = self.cmos_Ig_leakage(
            self._tp["sram_cell_nmos_w"],
            self._tp["sram_cell_pmos_w"],
            1,
            "INV",
            is_dram=self._is_dram,
            is_cell=True,
        )
        Ig_port_erp = self.cmos_Ig_leakage(
            self._tp["sram_cell_a_w"],
            0,
            1,
            "NMOS",
            is_dram=self._is_dram,
            is_cell=True,
        )

        # Drain / gate caps of the bit-mux, isolation, sense-amp latch and sa-mux
        # transistors that load the bitline. The drain fold width is the cell width
        # shared across all read paths, i.e. divided by (RWP+ERP+SCHP) -- mat.cc:1147-1154.
        # camFlag ternary-precedence bug, replicated verbatim (same as the delay
        # side, compute_bitline_delay): `camFlag ? cam_cell.w : cell.w * deg / div`
        # gives the BARE, undivided cam_cell.w for FA/CAM. Dividing it by the port
        # count here (as this method used to) is harmless at RWP+ERP == 1 but
        # under-folds FA arrays with 2 RW ports (the dcache Miss/Fill/prefetch/WB
        # buffers), ~1.2-5e-4 off on their read/search energy.
        fold_div = self._num_rw_ports + self._num_r_ports  # SCHP = 0
        bitmux_w = (
            self._cam_cell_w if self._is_cam else self._cell_w / (2 * fold_div)
        )
        samux_w = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing / fold_div
        )
        C_drain_bit_mux = self.drain_C(
            self._tp["w_nmos_b_mux"],
            "NCH",
            1,
            0,
            bitmux_w,
            is_dram=self._is_dram,
        )
        C_drain_sense_amp_iso = self.drain_C(
            self._tp["w_iso"], "PCH", 1, 0, samux_w, is_dram=self._is_dram
        )
        C_sense_amp_latch = (
            self.gate_C(
                self._tp["w_sense_p"] + self._tp["w_sense_n"],
                0,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_n"],
                "NCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_p"],
                "PCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
        )
        C_drain_sense_amp_mux = self.drain_C(
            self._tp["w_nmos_sa_mux"],
            "NCH",
            1,
            0,
            samux_w,
            is_dram=self._is_dram,
        )

        dyn_read = 0.0
        dyn_write = 0.0
        if deg_bl_muxing > 1:
            dyn_read += (C_bl + 2 * C_drain_bit_mux) * 2 * V_b_sense * Vdd
            dyn_read += (
                (
                    2 * C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
                * 2
                * V_b_sense
                * Vdd
                * (1.0 / deg_bl_muxing)
            )
            dyn_write += (
                ((1.0 / deg_bl_muxing) / deg_senseamp_muxing)
                * self._num_act_mats_hor_dir
                * (C_bl + 2 * C_drain_bit_mux)
                * Vdd
                * Vdd
                * 2
            )
        else:
            dyn_read += (
                (
                    C_bl
                    + 2 * C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
                * 2
                * V_b_sense
                * Vdd
            )
            dyn_write += (
                ((1.0 / deg_bl_muxing) / deg_senseamp_muxing)
                * self._num_act_mats_hor_dir
                * C_bl
                * Vdd
                * Vdd
                * 2
            )

        self._power_bitline.read.dynamic = dyn_read
        self._power_bitline.write.dynamic = dyn_write
        # Per-cell static leakage (mat.cc:1217-1230): the cross-coupled inverters
        # plus one RW/WR access transistor per (RWP+EWP) port and one read-port
        # access transistor per ERP read port. For a single R/W port this reduces
        # to cc_inverters + acc_tr.
        RWP = self._num_rw_ports
        EWP = self._num_w_ports
        ERP = self._num_r_ports
        self._power_bitline.read.leakage = (
            Icell * Vdd + Iport * Vdd * (RWP + EWP) + Iport_erp * Vdd * ERP
        )
        self._power_bitline.read.gate_leakage = (
            Ig_cell * Vdd + Ig_port_erp * Vdd * ERP
        )

    def compute_bitline_delay(self, inrisetime):
        # Port of the timing terms of Mat::compute_bitline_delay (mat.cc:
        # 1070-1282), SRAM (is_dram=False) arm only -- the energy terms are
        # already ported above in compute_bitline_power(). Phase 29, additive
        # only -- not called from __init__.
        #
        # v_th_mem_cell/V_wl/V_b_pre all reuse self._tp['Vth']/['Vdd']: real
        # CACTI reads these from g_tp.sram_cell.Vth/.Vdd (V_b_pre via
        # g_tp.sram.Vbitpre = g_tp.sram_cell.Vdd, technology.cc:1980) --  a
        # DIFFERENT tech-param struct than this port's self._tp, which is
        # wired from g_tp.peri_global. Confirmed exact (not approximate) two
        # ways: (1) a live GM_SENSE_AMP_PROBE run  printed both
        # g_tp.sram_cell.{Vth,Vdd} and g_tp.peri_global.{Vth,Vdd} for two real
        # tech corners (40nm/LOP and one HP corner from the real ARM_A9 XML)
        # and found them bit-identical; (2) both structs are populated by the
        # same alpha-interpolation loop indexed by ram_cell_tech_type vs.
        # peri_global_tech_type respectively (technology.cc), and every real
        # McPAT trace in mcpat/sweep_traces/*.jsonl has those two fields equal
        # in 100% of 1988 checked records (same underlying enum in practice).
        # The same argument (same table, same index variable) extends to
        # R_nch_on/R_pch_on below -- this port's existing tr_R_on() already
        # ignores its is_cell argument entirely and always reads
        # self._tp['R_nch_on']/['R_pch_on'] (peri_global-indexed); given the
        # proven ram_cell_tech_type == peri_global_tech_type equivalence,
        # that is exact for R_cell_pull_down/R_cell_acc below too, not a new
        # approximation -- this phase is simply the first call site in the
        # port to exercise tr_R_on(is_cell=True) at all.
        v_th_mem_cell = self._tp["Vth"]
        V_wl = self._tp["Vdd"]
        V_b_pre = self._tp["Vdd"]
        Vdd = self._tp["Vdd"]
        deg_bl_muxing = self._deg_bl_muxing

        # camFlag ternary-precedence bug (mat.cc:1078, replicated verbatim --
        # see CactiMat.compute_delays' hazard note): `camFlag ? cam_cell.h :
        # cell.h * R_per_um` parses as `camFlag ? cam_cell.h : (cell.h *
        # R_per_um)` -- `?:` binds looser than `*`, so for CAM arrays R_b_metal
        # is the BARE cell height (no R_per_um factor at all), not a dropped
        # divisor like the cell_w sites below. Dimensionally wrong in real
        # CACTI; ported as-is, not "fixed".
        R_b_metal = (
            self._cam_cell_h
            if self._is_cam
            else self._cell_h * self._wp["R_per_micron"]
        )
        R_bl = self.subarray._rows * R_b_metal
        C_bl = self.subarray._C_bl

        R_cell_pull_down = self.tr_R_on(
            self._tp["sram_cell_nmos_w"],
            "NCH",
            1,
            is_dram=self._is_dram,
            is_cell=True,
        )
        R_cell_acc = self.tr_R_on(
            self._tp["sram_cell_a_w"],
            "NCH",
            1,
            is_dram=self._is_dram,
            is_cell=True,
        )

        # Same drain-cap terms as compute_bitline_power's tau-input loads
        # (identical formulas, cacti_component.py:2041-2051) plus the two new
        # resistance terms (R_bit_mux, R_sense_amp_iso) the energy method
        # never needed.
        #
        # camFlag ternary-precedence bug (mat.cc:1148-1155, replicated
        # verbatim): `camFlag ? cam_cell.w : cell.w / divisor` -- for CAM
        # arrays the result is the BARE (undivided) cam_cell.w, since `?:`
        # binds looser than `/`. bitmux_w/samux_w below hold the correct
        # per-site divided value for cell.w and the bare cam_cell.w for CAM,
        # matching each ternary exactly (first-call-site novelty, per
        # the native reference: compute_bitline_delay/compute_sa_delay/
        # compute_subarray_out_drv are exercised with self._is_cam=True for
        # the first time by the is_fa branch of compute_delays).
        fold_div = (
            self._num_rw_ports + self._num_r_ports
        )  # (RWP+ERP+SCHP), SCHP=0
        bitmux_w = (
            self._cam_cell_w if self._is_cam else self._cell_w / (2 * fold_div)
        )
        samux_w = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing / fold_div
        )
        C_drain_bit_mux = self.drain_C(
            self._tp["w_nmos_b_mux"],
            "NCH",
            1,
            0,
            bitmux_w,
            is_dram=self._is_dram,
        )
        R_bit_mux = self.tr_R_on(
            self._tp["w_nmos_b_mux"], "NCH", 1, is_dram=self._is_dram
        )
        C_drain_sense_amp_iso = self.drain_C(
            self._tp["w_iso"], "PCH", 1, 0, samux_w, is_dram=self._is_dram
        )
        R_sense_amp_iso = self.tr_R_on(
            self._tp["w_iso"], "PCH", 1, is_dram=self._is_dram
        )
        C_sense_amp_latch = (
            self.gate_C(
                self._tp["w_sense_p"] + self._tp["w_sense_n"],
                0,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_n"],
                "NCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_p"],
                "PCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
        )
        C_drain_sense_amp_mux = self.drain_C(
            self._tp["w_nmos_sa_mux"],
            "NCH",
            1,
            0,
            samux_w,
            is_dram=self._is_dram,
        )

        # Two-branch tau, replicating the real asymmetry exactly (with-mux
        # adds 2*C_drain_bit_mux and an R_bit_mux Elmore term the no-mux
        # branch omits entirely -- genuine CACTI behavior, not a bug).
        if deg_bl_muxing > 1:
            tau = (
                (R_cell_pull_down + R_cell_acc)
                * (
                    C_bl
                    + 2 * C_drain_bit_mux
                    + 2 * C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
                + R_bl
                * (
                    C_bl / 2
                    + 2 * C_drain_bit_mux
                    + 2 * C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
                + R_bit_mux
                * (
                    C_drain_bit_mux
                    + 2 * C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
                + R_sense_amp_iso
                * (
                    C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
            )
        else:
            tau = (
                (R_cell_pull_down + R_cell_acc)
                * (
                    C_bl
                    + C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
                + R_bl * C_bl / 2
                + R_sense_amp_iso
                * (
                    C_drain_sense_amp_iso
                    + C_sense_amp_latch
                    + C_drain_sense_amp_mux
                )
            )
        tstep = tau * log(V_b_pre / (V_b_pre - self._V_b_sense))

        # Rise-time-aware final combine (mat.cc:1240-1249) -- the only place
        # inrisetime is actually used. inrisetime==0.0 is a real, reachable
        # input (row_dec_outrisetime is exactly 0.0 whenever row_dec doesn't
        # physically exist -- Decoder.compute_delays' own `if not
        # self._exist: return 0.0`, mirroring decoder.cc). C++'s
        # `V_wl / 0.0` silently yields +inf (IEEE 754), which then makes
        # `(V_wl - v_th_mem_cell) / m` and `tstep <= ...` both evaluate to
        # (something, 0.0) -> False -> the else branch reduces to exactly
        # `delay_bitline = tstep` (the /(2*m) term vanishes). Python raises
        # ZeroDivisionError instead of returning inf for this exact division,
        # so this replicates the C++ float semantics explicitly rather than
        # crashing on a real, valid-in-CACTI input (found via exhaustive
        # partition-search enumeration, the native reference -- prior
        # validation never exercised a row_dec.exist==False geometry).
        m = V_wl / inrisetime if inrisetime != 0.0 else float("inf")
        if tstep <= (0.5 * (V_wl - v_th_mem_cell) / m):
            self.delay_bitline = sqrt(2 * tstep * (V_wl - v_th_mem_cell) / m)
        else:
            self.delay_bitline = tstep + (V_wl - v_th_mem_cell) / (2 * m)

        # Real CACTI quirk: this function ALWAYS returns 0.0 regardless of
        # the computed delay_bitline (mat.cc:1280-1281) -- delay_bitline is a
        # real Mat member field used downstream elsewhere, but the RETURN
        # value used for signal-chaining is always 0. Replicated verbatim,
        # not "fixed".
        return 0.0

    def compute_sa_power(self):
        # Port of the energy terms of Mat::compute_sa_delay. The sense amp drives
        # its own latch plus the sense-precharge / iso / sa-mux drain loads.
        Vdd = self._tp["Vdd"]
        deg_bl_muxing = self._deg_bl_muxing

        fold_div = (
            self._num_rw_ports + self._num_r_ports
        )  # (RWP+ERP+SCHP), SCHP=0
        # camFlag ternary-precedence bug (mat.cc:1307-1310): bare cam_cell.w for
        # FA/CAM -- see compute_bitline_power's note.
        samux_w = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing / fold_div
        )
        C_ld = (
            self.gate_C(
                self._tp["w_sense_p"] + self._tp["w_sense_n"],
                0,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_n"],
                "NCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_p"],
                "PCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_iso"], "PCH", 1, 0, samux_w, is_dram=self._is_dram
            )
            + self.drain_C(
                self._tp["w_nmos_sa_mux"],
                "NCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
        )
        # per-sense-amp energy / leakage (scaled by num_sa_subarray later)
        self._power_sa.read.dynamic = C_ld * Vdd * Vdd
        lkg_idle = self.simplified_nmos_leakage(
            self._tp["w_sense_en"], is_dram=self._is_dram
        )
        self._power_sa.read.leakage = lkg_idle * Vdd

    def _gm_sense_amp_latch(self):
        # Port of the g_tp.gm_sense_amp_latch derivation (technology.cc:
        # 1968-1970), computed on the fly rather than persisted as a
        # cacti_tech_params.py dict key (a derived scalar, like
        # pmos_to_nmos_sz_ratio() elsewhere). Uses the peri_global-indexed
        # inputs wired in Phase 29 (l_elec/mobility_eff/Vdsat/gmp_to_gmn_mult).
        mobility_eff = self._tp["mobility_eff"]
        vdsat = self._tp["Vdsat"]
        l_elec = self._tp["l_elec"]
        gmp_to_gmn_mult = self._tp["gmp_to_gmn_mult"]
        Vdd = self._tp["Vdd"]
        Vdd_default = self._tp["Vdd_default"]
        Vth = self._tp["Vth"]

        gmn = (
            (mobility_eff / 2)
            * self._tp["c_ox"]
            * (self._tp["w_sense_n"] / l_elec)
            * vdsat
        )
        gmp = gmp_to_gmn_mult * gmn
        return gmn + gmp * ((Vdd - Vth) / (Vdd_default - Vth)) ** 1.3 / (
            Vdd / Vdd_default
        )

    def compute_sa_delay(self, inrisetime):
        # Port of the timing terms of Mat::compute_sa_delay (mat.cc:1286-1323)
        # -- the energy terms are already ported above in compute_sa_power().
        # Phase 29, additive only -- not called from __init__. `inrisetime` is
        # genuinely unused in the C++ body -- kept for signature symmetry with
        # the other three delay methods, not synthesized a use for.
        #
        # Reuses compute_sa_power's exact C_ld formula verbatim (same 5-term
        # gate_C/drain_C sum, cacti_component.py:2088-2094), except this
        # delay counterpart also replicates the camFlag ternary-precedence
        # bug (mat.cc:1308-1311, same pattern as compute_bitline_delay's
        # samux_w -- see that method's comment) since it is exercised with
        # self._is_cam=True for the first time by the is_fa branch of
        # compute_delays (the native reference). compute_sa_power itself is
        # NOT touched here -- out of scope, see the native reference finding
        # re: a possible latent connection to Finding 3.
        deg_bl_muxing = self._deg_bl_muxing
        fold_div = (
            self._num_rw_ports + self._num_r_ports
        )  # (RWP+ERP+SCHP), SCHP=0
        samux_w = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing / fold_div
        )
        C_ld = (
            self.gate_C(
                self._tp["w_sense_p"] + self._tp["w_sense_n"],
                0,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_n"],
                "NCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_sense_p"],
                "PCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_iso"], "PCH", 1, 0, samux_w, is_dram=self._is_dram
            )
            + self.drain_C(
                self._tp["w_nmos_sa_mux"],
                "NCH",
                1,
                0,
                samux_w,
                is_dram=self._is_dram,
            )
        )
        tau = C_ld / self._gm_sense_amp_latch()
        self.delay_sa = tau * log(self._tp["Vdd"] / self._V_b_sense)

        # Real CACTI quirk: always returns 0.0 regardless of the computed
        # delay_sa (mat.cc:1321-1322), same as compute_bitline_delay.
        # Replicated verbatim.
        return 0.0

    def compute_subarray_out_drv_power(self):
        # Port of Mat::compute_subarray_out_drv energy terms: the buffer chain
        # that moves a sensed bit through the two sense-amp mux levels out to the
        # mat edge. Each of these is a PER-OUTPUT-BIT energy, scaled by
        # num_do_b_mat in aggregation. The repeated global output wire itself is
        # in self._out_wire_dynamic (repeated_wire_energy_per_um).
        Vdd = self._tp["Vdd"]
        p_to_n = self.pmos_to_nmos_sz_ratio()
        deg_bl_muxing = self._deg_bl_muxing
        min_w = self._tp["min_w_nmos"]
        cell_h_def = self._tp["cell_h_def"]

        # Output wire first: it fixes the repeater size/spacing that the final
        # driver stage loads. subarray.cc:58 area.w (cl_vertical) is the wire
        # length -- CactiSubarray.area_w already ports this exactly (both
        # the non-FA and FA/CAM branches, including the ceil-on-integer-
        # division floor bug), so reuse it instead of re-deriving here.
        subarray_width = self.subarray.area_w
        self._out_wire_dynamic = (
            self.repeated_wire_energy_per_um() * subarray_width
        )

        # Drain fold width shared across all read paths (mat.cc:1334/1376).
        fold_div = (
            self._num_rw_ports + self._num_r_ports
        )  # (RWP+ERP+SCHP), SCHP=0
        # camFlag ternary-precedence bug (mat.cc:1332/1361/1374): bare
        # cam_cell.w for FA/CAM -- same as compute_subarray_out_drv's delay side.
        samux_w1 = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing / fold_div
        )
        samux_w2 = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing * self._Ndsam_lev_1 / fold_div
        )

        dyn = 0.0
        # Stage 1: first sense-amp-mux pass transistor -> inverter buffer input.
        C_ld = self._Ndsam_lev_1 * self.drain_C(
            self._tp["w_nmos_sa_mux"],
            "NCH",
            1,
            0,
            samux_w1,
            is_dram=self._is_dram,
        ) + self.gate_C(min_w + p_to_n * min_w, 0.0, is_dram=self._is_dram)
        dyn += C_ld * 0.5 * Vdd * Vdd
        # Stage 2: inverter-buffer internal.
        C_ld = (
            self.drain_C(min_w, "NCH", 1, 1, cell_h_def, is_dram=self._is_dram)
            + self.drain_C(
                p_to_n * min_w, "PCH", 1, 1, cell_h_def, is_dram=self._is_dram
            )
            + self.gate_C(min_w + p_to_n * min_w, 0.0, is_dram=self._is_dram)
        )
        dyn += C_ld * 0.5 * Vdd * Vdd
        # Stage 3: inverter driving second-level sense-amp mux.
        C_ld = (
            self.drain_C(min_w, "NCH", 1, 1, cell_h_def, is_dram=self._is_dram)
            + self.drain_C(
                p_to_n * min_w, "PCH", 1, 1, cell_h_def, is_dram=self._is_dram
            )
            + self.drain_C(
                self._tp["w_nmos_sa_mux"],
                "NCH",
                1,
                0,
                samux_w2,
                is_dram=self._is_dram,
            )
        )
        dyn += C_ld * 0.5 * Vdd * Vdd
        # Stage 4: second-level sense-amp mux pass transistor drain PLUS the gate
        # of the output wire's first (scaled) repeater. mat.cc:1376.
        C_ld = self._Ndsam_lev_2 * self.drain_C(
            self._tp["w_nmos_sa_mux"],
            "NCH",
            1,
            0,
            samux_w2,
            is_dram=self._is_dram,
        ) + self.gate_C(
            self._out_wire_repeater_size
            * (subarray_width / self._out_wire_repeater_spacing_um)
            * min_w
            * (1 + p_to_n),
            0.0,
            is_dram=self._is_dram,
        )
        dyn += C_ld * 0.5 * Vdd * Vdd

        self._power_subarray_out_drv.read.dynamic = dyn

        # Per-output-bit leakage of the driver stages: stages 1 & 4 are sense-amp
        # mux pass transistors (zero subthreshold leakage, gate leakage only);
        # stages 2 & 3 are min-sized inverters (mat.cc compute_subarray_out_drv).
        self._power_subarray_out_drv.read.leakage = (
            2
            * self.cmos_Isub_leakage(
                min_w, p_to_n * min_w, 1, "INV", is_dram=self._is_dram
            )
            * Vdd
        )
        self._power_subarray_out_drv.read.gate_leakage = (
            2
            * self.cmos_Ig_leakage(
                self._tp["w_nmos_sa_mux"], 0, 1, "NMOS", is_dram=self._is_dram
            )
            * Vdd
            + 2
            * self.cmos_Ig_leakage(
                min_w, p_to_n * min_w, 1, "INV", is_dram=self._is_dram
            )
            * Vdd
        )
        # Output wire leakage over the subarray width (per output bit).
        self._out_wire_leakage = self._out_wire_leakage_per_um * subarray_width
        self._out_wire_gate_leakage = (
            self._out_wire_gate_leakage_per_um * subarray_width
        )
        # number_output_drivers_subarray = num_sa_subarray / (Ndsam1 * Ndsam2)
        self._number_output_drivers_subarray = max(
            1, self._num_sa_subarray // (self._Ndsam_lev_1 * self._Ndsam_lev_2)
        )

    def compute_subarray_out_drv(self, inrisetime):
        # Port of the timing terms of Mat::compute_subarray_out_drv (mat.cc:
        # 1327-1390) -- the energy terms are already ported above in
        # compute_subarray_out_drv_power(). Phase 29, additive only -- not
        # called from __init__. The one function of the four Phase 29 ports
        # whose return value is real (properly chained through all 4 stages),
        # not hardcoded to 0.
        #
        # Reuses compute_subarray_out_drv_power's exact per-stage C_ld
        # formulas verbatim (cacti_component.py:2251+, including stage 4's
        # reuse of self._out_wire_repeater_size/_spacing_um, already set as a
        # side effect of that method's own repeated_wire_energy_per_um()
        # call), adding only the missing rd/tf/horowitz/feed-forward parts.
        # self.delay_subarray_out_drv is NOT reset here -- see its __init__
        # comment (zero-initialized once, accumulated via += across every
        # call, matching the C++ member field exactly).
        p_to_n = self.pmos_to_nmos_sz_ratio()
        deg_bl_muxing = self._deg_bl_muxing
        min_w = self._tp["min_w_nmos"]
        cell_h_def = self._tp["cell_h_def"]
        fold_div = (
            self._num_rw_ports + self._num_r_ports
        )  # (RWP+ERP+SCHP), SCHP=0
        subarray_width = self.subarray.area_w

        # camFlag ternary-precedence bug (mat.cc:1333/1362/1375), replicated
        # verbatim -- same pattern as compute_bitline_delay's bitmux_w/samux_w
        # (see that method's comment). samux_w1 feeds stage 1's divisor
        # (deg_bl_muxing only); samux_w2 feeds stages 3/4's divisor
        # (deg_bl_muxing * Ndsam_lev_1) -- both bare cam_cell_w when
        # self._is_cam, matching each C++ ternary independently.
        samux_w1 = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing / fold_div
        )
        samux_w2 = (
            self._cam_cell_w
            if self._is_cam
            else self._cell_w * deg_bl_muxing * self._Ndsam_lev_1 / fold_div
        )

        # Stage 1: first sense-amp-mux pass transistor -> inverter buffer input.
        rd = self.tr_R_on(
            self._tp["w_nmos_sa_mux"], "NCH", 1, is_dram=self._is_dram
        )
        C_ld = self._Ndsam_lev_1 * self.drain_C(
            self._tp["w_nmos_sa_mux"],
            "NCH",
            1,
            0,
            samux_w1,
            is_dram=self._is_dram,
        ) + self.gate_C(min_w + p_to_n * min_w, 0.0, is_dram=self._is_dram)
        tf = rd * C_ld
        this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
        self.delay_subarray_out_drv += this_delay
        inrisetime = this_delay / (1.0 - 0.5)

        # Stage 2: inverter-buffer internal delay.
        rd = self.tr_R_on(min_w, "NCH", 1, is_dram=self._is_dram)
        C_ld = (
            self.drain_C(min_w, "NCH", 1, 1, cell_h_def, is_dram=self._is_dram)
            + self.drain_C(
                p_to_n * min_w, "PCH", 1, 1, cell_h_def, is_dram=self._is_dram
            )
            + self.gate_C(min_w + p_to_n * min_w, 0.0, is_dram=self._is_dram)
        )
        tf = rd * C_ld
        this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
        self.delay_subarray_out_drv += this_delay
        inrisetime = this_delay / (1.0 - 0.5)

        # Stage 3: inverter driving second-level sense-amp mux drain.
        rd = self.tr_R_on(min_w, "NCH", 1, is_dram=self._is_dram)
        C_ld = (
            self.drain_C(min_w, "NCH", 1, 1, cell_h_def, is_dram=self._is_dram)
            + self.drain_C(
                p_to_n * min_w, "PCH", 1, 1, cell_h_def, is_dram=self._is_dram
            )
            + self.drain_C(
                self._tp["w_nmos_sa_mux"],
                "NCH",
                1,
                0,
                samux_w2,
                is_dram=self._is_dram,
            )
        )
        tf = rd * C_ld
        this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
        self.delay_subarray_out_drv += this_delay
        inrisetime = this_delay / (1.0 - 0.5)

        # Stage 4: second-level sense-amp mux pass transistor -> subarray
        # output driver input (the repeated output wire's first repeater gate).
        rd = self.tr_R_on(
            self._tp["w_nmos_sa_mux"], "NCH", 1, is_dram=self._is_dram
        )
        C_ld = self._Ndsam_lev_2 * self.drain_C(
            self._tp["w_nmos_sa_mux"],
            "NCH",
            1,
            0,
            samux_w2,
            is_dram=self._is_dram,
        ) + self.gate_C(
            self._out_wire_repeater_size
            * (subarray_width / self._out_wire_repeater_spacing_um)
            * min_w
            * (1 + p_to_n),
            0.0,
            is_dram=self._is_dram,
        )
        tf = rd * C_ld
        this_delay = self.horowitz(inrisetime, tf, 0.5, 0.5, 1)
        self.delay_subarray_out_drv += this_delay
        inrisetime = this_delay / (1.0 - 0.5)

        return inrisetime

    def compute_comparator_power(self):
        # Port of Mat::compute_comparator_delay (mat.cc:1394-1480), energy-only
        # (skip horowitz/rise-time delay terms) -- tag arrays only (mat.cc:641).
        # 4 quarter-comparators per associativity way; tag_assoc == assoc always
        # (io.cc:error_checking), stored as self._associativity.
        A = self._associativity
        tagbits_ = (
            self.dp.tagbits // 4
        )  # mat.cc:1398, tagbits already a multiple of 4
        Vdd = self._tp["Vdd"]
        cell_h_def = self._tp["cell_h_def"]

        dyn = 0.0
        lkg = 0.0
        glkg = 0.0

        # Stage 1: first inverter.
        Ceq = (
            self.gate_C(
                self._tp["w_comp_inv_n2"] + self._tp["w_comp_inv_p2"],
                0,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_comp_inv_p1"],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_comp_inv_n1"],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
        )
        dyn += 0.5 * Ceq * Vdd * Vdd * 4 * A
        lkg += (
            self.cmos_Isub_leakage(
                self._tp["w_comp_inv_n1"],
                self._tp["w_comp_inv_p1"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )
        glkg += (
            self.cmos_Ig_leakage(
                self._tp["w_comp_inv_n1"],
                self._tp["w_comp_inv_p1"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )

        # Stage 2: second inverter.
        Ceq = (
            self.gate_C(
                self._tp["w_comp_inv_n3"] + self._tp["w_comp_inv_p3"],
                0,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_comp_inv_p2"],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_comp_inv_n2"],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
        )
        dyn += 0.5 * Ceq * Vdd * Vdd * 4 * A
        lkg += (
            self.cmos_Isub_leakage(
                self._tp["w_comp_inv_n2"],
                self._tp["w_comp_inv_p2"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )
        glkg += (
            self.cmos_Ig_leakage(
                self._tp["w_comp_inv_n2"],
                self._tp["w_comp_inv_p2"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )

        # Stage 3: third inverter (feeds the eval inverter).
        Ceq = (
            self.gate_C(
                self._tp["w_eval_inv_n"] + self._tp["w_eval_inv_p"],
                0,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_comp_inv_p3"],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_comp_inv_n3"],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
        )
        dyn += 0.5 * Ceq * Vdd * Vdd * 4 * A
        lkg += (
            self.cmos_Isub_leakage(
                self._tp["w_comp_inv_n3"],
                self._tp["w_comp_inv_p3"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )
        glkg += (
            self.cmos_Ig_leakage(
                self._tp["w_comp_inv_n3"],
                self._tp["w_comp_inv_p3"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )

        # Final stage: virtual-ground driver discharging the compare part. The
        # gate_C(WmuxdrvNANDn+WmuxdrvNANDp, 0) term in c1 (mat.cc:1449) is omitted --
        # both constants are permanently 0 in real CACTI (const.h:94-95).
        c2 = (
            tagbits_
            * (
                self.drain_C(
                    self._tp["w_comp_n"],
                    "NCH",
                    1,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
                + self.drain_C(
                    self._tp["w_comp_n"],
                    "NCH",
                    2,
                    1,
                    cell_h_def,
                    is_dram=self._is_dram,
                )
            )
            + self.drain_C(
                self._tp["w_eval_inv_p"],
                "PCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_eval_inv_n"],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
        )
        c1 = tagbits_ * (
            self.drain_C(
                self._tp["w_comp_n"],
                "NCH",
                1,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
            + self.drain_C(
                self._tp["w_comp_n"],
                "NCH",
                2,
                1,
                cell_h_def,
                is_dram=self._is_dram,
            )
        ) + self.drain_C(
            self._tp["w_comp_p"],
            "PCH",
            1,
            1,
            cell_h_def,
            is_dram=self._is_dram,
        )
        dyn += 0.5 * c2 * Vdd * Vdd * 4 * A
        dyn += c1 * Vdd * Vdd * (A - 1)
        lkg += (
            self.cmos_Isub_leakage(
                self._tp["w_eval_inv_n"],
                self._tp["w_eval_inv_p"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )
        lkg += (
            self.cmos_Isub_leakage(
                self._tp["w_comp_n"],
                self._tp["w_comp_n"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )
        glkg += (
            self.cmos_Ig_leakage(
                self._tp["w_eval_inv_n"],
                self._tp["w_eval_inv_p"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )
        glkg += (
            self.cmos_Ig_leakage(
                self._tp["w_comp_n"],
                self._tp["w_comp_n"],
                1,
                "INV",
                is_dram=self._is_dram,
            )
            * 4
            * A
        )

        self._power_comparator.read.dynamic = dyn
        self._power_comparator.read.leakage = lkg * Vdd
        self._power_comparator.read.gate_leakage = glkg * Vdd

    def compute_cam_search_power(self):
        # Energy-only port of Mat::compute_cam_delay's SEARCH-op chain (mat.cc:
        # 707-1041): comparator -> NAND2 -> dummy-row inverter/NOR2 -> RAM
        # wordline driver, plus the peripheral hit/miss logic. Horowitz/tr_R_on
        # delay terms and the linear_scaling dead branch are skipped entirely
        # (energy-only scope); the Driver objects this function's C++ sibling
        # also builds (sl_precharge_eq_drv/sl_data_drv/ml_precharge_drv/
        # ml_to_ram_wl_drv) carry their OWN energy via CactiDriver and are
        # aggregated separately (dynamic_power/static_power, CactiBank).
        Vdd = self._tp["Vdd"]
        cell_h_def = self._tp["cell_h_def"]
        is_dram = self._is_dram

        Wdummyn = self._tp["cam_cell_nmos_w"]
        Wdummyinvn = self._tp["w_dummy_inv_n"]
        Wdummyinvp = self._tp["w_dummy_inv_p"]
        Waddrnandn = self._tp["w_addr_nand_n"]
        Waddrnandp = self._tp["w_addr_nand_p"]
        Wfanorn = self._tp["w_fa_nor_n"]
        Wfanorp = self._tp["w_fa_nor_p"]
        p_to_n = self.pmos_to_nmos_sz_ratio()
        Wfaprechp = (
            6 * p_to_n * self._tp["min_w_nmos"]
        )  # == w_pmos_bl_precharge
        W_hit_miss_n = Wdummyn
        W_hit_miss_p = self._tp["min_w_nmos"] * p_to_n

        num_cols_fa_cam = self.subarray._num_cols_fa_cam
        num_rows = self.subarray._rows
        c_searchline_metal = self._cam_cell_h * self._wp["C_per_micron"]
        c_matchline_metal = self._cam_cell_w * self._wp["C_per_micron"]

        Htagbits = int(ceil(num_cols_fa_cam / 2.0))

        dyn = 0.0

        # Stage 1: comparators, both halves (mat.cc:878-899).
        c_intrinsic = Htagbits * (
            2 * self.drain_C(Wdummyn, "NCH", 2, 1, cell_h_def, is_dram=is_dram)
            + self.drain_C(Wfaprechp, "PCH", 1, 1, cell_h_def, is_dram=is_dram)
            / Htagbits
        )
        Cwire = c_matchline_metal * Htagbits
        c_gate_load = self.gate_C(Waddrnandn + Waddrnandp, 0, is_dram=is_dram)
        dyn += (
            (c_intrinsic + Cwire + c_gate_load)
            * (num_rows + 1)
            * Vdd
            * Vdd
            * 2
        )

        # Stage 2: NAND2 gates combining both halves (mat.cc:901-912).
        c_intrinsic = (
            self.drain_C(Waddrnandn, "NCH", 2, 1, cell_h_def, is_dram=is_dram)
            + self.drain_C(
                Waddrnandp, "PCH", 1, 1, cell_h_def, is_dram=is_dram
            )
            * 2
        )
        c_gate_load = self.gate_C(Wdummyinvn + Wdummyinvp, 0, is_dram=is_dram)
        dyn += (
            c_intrinsic * (num_rows + 1) * Vdd * Vdd
            + c_gate_load * 2 * Vdd * Vdd
        )

        # Stage 3: dummy-row inverter driving the NOR2 (mat.cc:914-925).
        c_intrinsic = self.drain_C(
            Wdummyinvn, "NCH", 1, 1, cell_h_def, is_dram=is_dram
        ) + self.drain_C(Wdummyinvp, "NCH", 1, 1, cell_h_def, is_dram=is_dram)
        Cwire = (
            c_matchline_metal * Htagbits
            + c_searchline_metal * (num_rows + 1) / 2
        )
        c_gate_load = self.gate_C(Wfanorn + Wfanorp, 0, is_dram=is_dram)
        dyn += (c_intrinsic + Cwire + num_rows * c_gate_load) * Vdd * Vdd

        # Stage 4: NOR2 -> RAM wordline driver (mat.cc:927-953). The driver's own
        # gate load ("c_gate_load energy is computed in ml_to_ram_wl_drv", mat.cc:952)
        # is charged inside ml_to_ram_wl_drv's own CactiDriver, not here.
        c_intrinsic = 2 * self.drain_C(
            Wfanorn, "NCH", 1, 1, cell_h_def, is_dram=is_dram
        ) + self.drain_C(Wfanorp, "NCH", 1, 1, cell_h_def, is_dram=is_dram)
        dyn += c_intrinsic * Vdd * Vdd

        # Hit/miss precharge (mat.cc:956-968).
        c_intrinsic = 2 * self.drain_C(
            W_hit_miss_p, "NCH", 2, 1, cell_h_def, is_dram=is_dram
        )
        Cwire = c_searchline_metal * num_rows
        c_gate_load = (
            self.drain_C(
                W_hit_miss_n, "NCH", 1, 1, cell_h_def, is_dram=is_dram
            )
            * num_rows
        )
        dyn += (c_intrinsic + Cwire + c_gate_load) * Vdd * Vdd

        # Hit/miss evaluation (mat.cc:970-984). Same Cwire/c_gate_load as the
        # precharge stage; only c_intrinsic changes (mat.cc uses NCH for both the
        # p- and n-named hit/miss widths here -- replicated verbatim).
        c_intrinsic = 2 * self.drain_C(
            W_hit_miss_n, "NCH", 2, 1, cell_h_def, is_dram=is_dram
        )
        dyn += (c_intrinsic + Cwire + c_gate_load) * Vdd * Vdd

        self._power_matchline_search_dynamic = dyn

        # ---- leakage (mat.cc:990-1015) ----
        Iport = self.cmos_Isub_leakage(
            self._tp["cam_cell_a_w"],
            0,
            1,
            "NMOS",
            is_dram=is_dram,
            is_cell=True,
        )
        Iport_erp = self.cmos_Isub_leakage(
            self._tp["cam_cell_a_w"],
            0,
            2,
            "NMOS",
            is_dram=is_dram,
            is_cell=True,
        )
        Icell = (
            self.cmos_Isub_leakage(
                self._tp["cam_cell_nmos_w"],
                self._tp["cam_cell_pmos_w"],
                1,
                "INV",
                is_dram=is_dram,
                is_cell=True,
            )
            * 2
        )
        Icell_comparator = (
            self.cmos_Isub_leakage(
                Wdummyn, Wdummyn, 1, "INV", is_dram=is_dram, is_cell=True
            )
            * 2
        )

        RWP = self._num_rw_ports
        EWP = self._num_w_ports
        ERP = self._num_r_ports
        SCHP = self._num_sr_ports

        leak = (
            Icell * Vdd
            + Icell_comparator * Vdd
            + Iport * Vdd
            + Iport * Vdd * (RWP + EWP - 1)
            + Iport_erp * Vdd * ERP
            + 0 * SCHP
        )
        leak *= (num_rows + 1) * num_cols_fa_cam
        leak += (
            (num_rows + 1)
            * self.cmos_Isub_leakage(0, Wfaprechp, 1, "PMOS", is_dram=is_dram)
            * Vdd
        )
        leak += (
            (num_rows + 1)
            * self.cmos_Isub_leakage(
                Waddrnandn, Waddrnandp, 2, "NAND", is_dram=is_dram
            )
            * Vdd
        )
        leak += (
            (num_rows + 1)
            * self.cmos_Isub_leakage(
                Wfanorn, Wfanorp, 2, "NOR", is_dram=is_dram
            )
            * Vdd
        )
        # Hit/miss subthreshold leakage is a deliberate CACTI no-op here (mat.cc:
        # 1013-1015 -- "in idle states, the hit/miss txs are closed (on)").
        self._power_matchline_search_leakage = leak

        Ig_port_erp = self.cmos_Ig_leakage(
            self._tp["cam_cell_a_w"],
            0,
            1,
            "NMOS",
            is_dram=is_dram,
            is_cell=True,
        )
        Ig_cell = (
            self.cmos_Ig_leakage(
                self._tp["cam_cell_nmos_w"],
                self._tp["cam_cell_pmos_w"],
                1,
                "INV",
                is_dram=is_dram,
                is_cell=True,
            )
            * 2
        )
        Ig_cell_comparator = (
            self.cmos_Ig_leakage(
                Wdummyn, Wdummyn, 1, "INV", is_dram=is_dram, is_cell=True
            )
            * 2
        )

        # mat.cc:1024 -- CACTI bug: the RD-port gate-leak term uses sram_cell.Vdd
        # instead of cam_cell.Vdd, unlike its subthreshold sibling above. Both are
        # the single peripheral Vdd in this port's device model, so numerically
        # identical here; kept as a distinct term for fidelity.
        gleak = (
            Ig_cell * Vdd
            + Ig_cell_comparator * Vdd
            + Ig_port_erp * Vdd * ERP
            + 0 * SCHP
        )
        gleak *= (num_rows + 1) * num_cols_fa_cam
        gleak += (
            (num_rows + 1)
            * self.cmos_Ig_leakage(0, Wfaprechp, 1, "PMOS", is_dram=is_dram)
            * Vdd
        )
        gleak += (
            (num_rows + 1)
            * self.cmos_Ig_leakage(
                Waddrnandn, Waddrnandp, 2, "NAND", is_dram=is_dram
            )
            * Vdd
        )
        gleak += (
            (num_rows + 1)
            * self.cmos_Ig_leakage(Wfanorn, Wfanorp, 2, "NOR", is_dram=is_dram)
            * Vdd
        )
        # Hit/miss gate leakage IS active here (mat.cc:1036-1037): num_rows scales
        # only the nmos term; the pmos term is added once due to an operator-
        # precedence quirk in the original (`a + b*c +\n + d*e` parses as
        # `a + b*c + d*e`, not `a + b*(c+d)*e`) -- replicated verbatim.
        gleak += (
            num_rows
            * self.cmos_Ig_leakage(W_hit_miss_n, 0, 1, "NMOS", is_dram=is_dram)
            * Vdd
            + self.cmos_Ig_leakage(0, W_hit_miss_p, 1, "PMOS", is_dram=is_dram)
            * Vdd
        )
        self._power_matchline_search_gate_leakage = gleak

    def compute_cam_delay(self, inrisetime):
        # Delay-only port of Mat::compute_cam_delay's SEARCH-op chain (mat.cc:
        # 706-1039) -- the counterpart to compute_cam_search_power's energy-only
        # port of the same C++ function. linear_scaling is always False in real
        # CACTI's own compute_delays() call path (the dead `if (linear_scaling)`
        # branch there is never taken), so only the else-branch width literals
        # are needed here -- same ones compute_cam_search_power already uses
        # (Wdummyn/Wdummyinvn/Wdummyinvp/Waddrnandn/Waddrnandp/Wfanorn/Wfanorp/
        # Wfaprechp/W_hit_miss_n/W_hit_miss_p), reused verbatim.
        #
        # Real CACTI quirk, replicated verbatim: the hit/miss EVALUATION stage
        # (mat.cc:978) computes `rd = tr_R_on(W_hit_miss_n, PCH, ...)` -- the
        # n-named width fed into a PCH (pmos) resistance lookup, not a copy/paste
        # typo fixed here.
        Vdd = self._tp["Vdd"]
        cell_h_def = self._tp["cell_h_def"]
        is_dram = self._is_dram

        Wdummyn = self._tp["cam_cell_nmos_w"]
        Wdummyinvn = self._tp["w_dummy_inv_n"]
        Wdummyinvp = self._tp["w_dummy_inv_p"]
        Waddrnandn = self._tp["w_addr_nand_n"]
        Waddrnandp = self._tp["w_addr_nand_p"]
        Wfanorn = self._tp["w_fa_nor_n"]
        Wfanorp = self._tp["w_fa_nor_p"]
        p_to_n = self.pmos_to_nmos_sz_ratio()
        Wfaprechp = (
            6 * p_to_n * self._tp["min_w_nmos"]
        )  # == g_tp.w_pmos_bl_precharge
        W_hit_miss_n = Wdummyn
        W_hit_miss_p = self._tp["min_w_nmos"] * p_to_n

        num_cols_fa_cam = self.subarray._num_cols_fa_cam
        num_rows = self.subarray._rows
        c_matchline_metal = self._cam_cell_w * self._wp["C_per_micron"]
        r_matchline_metal = self._cam_cell_w * self._wp["R_per_micron"]
        c_searchline_metal = self._cam_cell_h * self._wp["C_per_micron"]
        r_searchline_metal = self._cam_cell_h * self._wp["R_per_micron"]

        Htagbits = int(ceil(num_cols_fa_cam / 2.0))

        self.delay_matchchline = 0.0

        # ---- Searchline precharge/restore (mat.cc:823-856) ----
        # Precharge routes horizontally like the bitline precharge driver; the
        # Driver object itself (sl_precharge_eq_drv) was already built in
        # __init__ with the matching c_gate_load/c_wire_load/r_wire_load.
        self.sl_precharge_eq_drv.compute_delay(0)
        R_bl_precharge = self.tr_R_on(Wfaprechp, "PCH", 1, is_dram=is_dram)
        r_b_metal = self._cam_cell_h * self._wp["R_per_micron"]
        R_bl = (num_rows + 1) * r_b_metal
        C_bl = self.subarray._C_bl_cam
        V_bitpre = (
            Vdd  # g_tp.cam.Vbitpre == g_tp.cam_cell.Vdd (technology.cc:1981);
        )
        # g_tp.cam_cell.Vdd == g_tp.peri_global.Vdd is the same
        # alpha-interpolation-loop argument already proven for
        # g_tp.sram.Vbitpre (compute_bitline_delay's docstring)
        # but was NOT independently probed for cam_cell here --
        # confirmed instead by the corpus-wide bit-exact MATCH
        # on delay_cam_sl_restore/delay_cam_ml_reset (both scale
        # with log(V_bitpre)) across all 566 FA/CAM corpus arms,
        # see the native reference.
        self.delay_cam_sl_restore = self.sl_precharge_eq_drv.delay + log(
            V_bitpre
        ) * (R_bl_precharge * C_bl + R_bl * C_bl / 2)

        # Searchline data driver -- feeds the comparator chain's input ramp.
        out_time_ramp = self.sl_data_drv.compute_delay(inrisetime)
        self.delay_matchchline += self.sl_data_drv.delay

        # ---- Matchline precharge/reset (mat.cc:861-889) ----
        self.ml_precharge_drv.compute_delay(0)
        rd = self.tr_R_on(Wdummyn, "NCH", 2, is_dram=is_dram)
        c_intrinsic = Htagbits * (
            2 * self.drain_C(Wdummyn, "NCH", 2, 1, cell_h_def, is_dram=is_dram)
            + self.drain_C(Wfaprechp, "PCH", 1, 1, cell_h_def, is_dram=is_dram)
            / Htagbits
        )
        Cwire = c_matchline_metal * Htagbits
        Rwire = r_matchline_metal * Htagbits
        c_gate_load = self.gate_C(Waddrnandn + Waddrnandp, 0, is_dram=is_dram)

        R_ml_precharge = self.tr_R_on(Wfaprechp, "PCH", 1, is_dram=is_dram)
        R_ml = Rwire
        C_ml = Cwire + c_intrinsic
        self.delay_cam_ml_reset = self.ml_precharge_drv.delay + log(
            V_bitpre
        ) * (R_ml_precharge * C_ml + R_ml * C_ml / 2)

        # ---- Stage 2: comparator XOR pulldown -> NAND gate load (mat.cc:876-899) ----
        tf = rd * (c_intrinsic + Cwire / 2 + c_gate_load) + Rwire * (
            Cwire / 2 + c_gate_load
        )
        this_delay = self.horowitz(out_time_ramp, tf, VTHFA2, VTHFA3, 0)
        self.delay_matchchline += this_delay
        out_time_ramp = this_delay / VTHFA3

        # ---- Stage 3: NAND2 -> dummy-row inverter (mat.cc:901-912) ----
        rd = self.tr_R_on(Waddrnandn, "NCH", 2, is_dram=is_dram)
        c_intrinsic = (
            self.drain_C(Waddrnandn, "NCH", 2, 1, cell_h_def, is_dram=is_dram)
            + self.drain_C(
                Waddrnandp, "PCH", 1, 1, cell_h_def, is_dram=is_dram
            )
            * 2
        )
        c_gate_load = self.gate_C(Wdummyinvn + Wdummyinvp, 0, is_dram=is_dram)
        tf = rd * (c_intrinsic + c_gate_load)
        this_delay = self.horowitz(out_time_ramp, tf, VTHFA3, VTHFA4, 1)
        out_time_ramp = this_delay / (1 - VTHFA4)
        self.delay_matchchline += this_delay

        # ---- Stage 4: dummy inverter -> NOR2 driving RAM wordline (mat.cc:913-924) ----
        rd = self.tr_R_on(Wdummyinvn, "NCH", 1, is_dram=is_dram)
        c_intrinsic = self.drain_C(
            Wdummyinvn, "NCH", 1, 1, cell_h_def, is_dram=is_dram
        ) + self.drain_C(Wdummyinvp, "NCH", 1, 1, cell_h_def, is_dram=is_dram)
        Cwire = (
            c_matchline_metal * Htagbits
            + c_searchline_metal * (num_rows + 1) / 2
        )
        Rwire = (
            r_matchline_metal * Htagbits
            + r_searchline_metal * (num_rows + 1) / 2
        )
        c_gate_load = self.gate_C(Wfanorn + Wfanorp, 0, is_dram=is_dram)
        tf = rd * (c_intrinsic + Cwire + c_gate_load) + Rwire * (
            Cwire / 2 + c_gate_load
        )
        this_delay = self.horowitz(out_time_ramp, tf, VTHFA4, VTHFA5, 0)
        out_time_ramp = this_delay / VTHFA5
        self.delay_matchchline += this_delay

        # ---- Final stage: NOR2 -> ml_to_ram_wl_drv's own gate (mat.cc:926-952) ----
        rd = self.tr_R_on(Wfanorn, "NCH", 1, is_dram=is_dram)
        c_intrinsic = 2 * self.drain_C(
            Wfanorn, "NCH", 1, 1, cell_h_def, is_dram=is_dram
        ) + self.drain_C(Wfanorp, "NCH", 1, 1, cell_h_def, is_dram=is_dram)
        c_gate_load = self.gate_C(
            self.ml_to_ram_wl_drv._width_n[0]
            + self.ml_to_ram_wl_drv._width_p[0],
            0,
            is_dram=is_dram,
        )
        tf = rd * (c_intrinsic + c_gate_load)
        this_delay = self.horowitz(out_time_ramp, tf, 0.5, 0.5, 1)
        out_time_ramp = this_delay / (1 - 0.5)
        self.delay_matchchline += this_delay

        out_time_ramp = self.ml_to_ram_wl_drv.compute_delay(out_time_ramp)

        # ---- Hit/miss logic (mat.cc:955-984) ----
        c_intrinsic = 2 * self.drain_C(
            W_hit_miss_p, "NCH", 2, 1, cell_h_def, is_dram=is_dram
        )
        Cwire = c_searchline_metal * num_rows
        Rwire = r_searchline_metal * num_rows
        c_gate_load = (
            self.drain_C(
                W_hit_miss_n, "NCH", 1, 1, cell_h_def, is_dram=is_dram
            )
            * num_rows
        )
        rd = self.tr_R_on(W_hit_miss_p, "PCH", 1, is_dram=is_dram)
        R_hit_miss = Rwire
        C_hit_miss = Cwire + c_intrinsic
        self.delay_hit_miss_reset = log(V_bitpre) * (
            rd * C_hit_miss + R_hit_miss * C_hit_miss / 2
        )

        c_intrinsic = 2 * self.drain_C(
            W_hit_miss_n, "NCH", 2, 1, cell_h_def, is_dram=is_dram
        )
        rd = self.tr_R_on(
            W_hit_miss_n, "PCH", 1, is_dram=is_dram
        )  # bug: n-width, PCH lookup -- verbatim
        tf = rd * (c_intrinsic + Cwire / 2 + c_gate_load) + Rwire * (
            Cwire / 2 + c_gate_load
        )
        self.delay_hit_miss = self.horowitz(0, tf, 0.5, 0.5, 0)

        # is_fa-only term (mat.cc:980-981) -- NOT shared with pure_cam, which
        # this port does not implement (see PROGRESS.md: 0 pure_cam records in
        # the corpus). Coded unconditionally here since compute_cam_delay is
        # only ever called from the is_fa branch of compute_delays.
        if self._is_fa:
            self.delay_matchchline += max(
                self.ml_to_ram_wl_drv.delay, self.delay_hit_miss
            )

        return out_time_ramp

    def repeated_wire_energy_per_um(self):
        # Per-micron dynamic energy of CACTI's delay-optimal repeated wire (the
        # static Wire::global that Wire(Global,...) reads), used by the mat's
        # subarray output driver. Hard-coded to overhead=0 (Global) and the
        # outside-mat geometry McPAT selects via wire_is_mat_type (see __init__'s
        # _out_wire_* selection) -- the subarray output wire is always constructed
        # as new Wire(Global, ...) in mat.cc regardless of the array's g_ip->wt.
        point = self._wire_model(
            self._out_wire_pitch_um,
            self._out_wire_aspect,
            self._out_wire_horiz,
            self._out_wire_vert,
            self._out_wire_ild,
            self._out_wire_fringe,
            overhead=0,
            is_dram=self._is_dram,
        )
        # Expose the repeater sizing (global.area.h / global.area.w) so the output
        # driver's final stage can load the wire's first repeater gate.
        self._out_wire_repeater_size = point["repeater_size"]
        self._out_wire_repeater_spacing_um = point["repeater_spacing_um"]
        self._out_wire_leakage_per_um = point["leak_per_um"]
        self._out_wire_gate_leakage_per_um = point["gleak_per_um"]
        return point["dynamic_per_um"]  # J per micron

    def _dynamic_power_fa(self):
        # Port of Mat::compute_power_energy's `is_fa` branch (mat.cc:1534-1592).
        # KEY QUIRK, verified against the source line-by-line: for FA, NONE of
        # the per-component READ energies (precharge, sense-amp, bitline, row
        # decoder) are multiplied by num_subarrays_per_mat -- unlike the non-FA
        # path, which scales all of them. Only the SEARCH-side components
        # (power_matchline, searchline, matchline-precharge) get the
        # `num_subarrays_per_mat` multiply (mat.cc:1574-1580). This asymmetry is
        # exactly what CACTI computes; replicated verbatim, not "fixed."
        n_sub = self._num_subarrays_per_mat
        num_cols_fa_cam = self.subarray._num_cols_fa_cam
        num_cols_fa_ram = self.subarray._num_cols_fa_ram

        b_mux_dyn = (
            self.b_mux_predec._power.read.dynamic if self._has_b_mux else 0.0
        )
        row_dec_dyn = (
            self.row_dec._power.read.dynamic if self.row_dec._exist else 0.0
        )
        bit_mux_dec_dyn = (
            self.bit_mux_dec._power.read.dynamic
            if self.bit_mux_dec._exist
            else 0.0
        )
        sa_mux_l1_dec_dyn = (
            self.sa_mux_lev_1_dec._power.read.dynamic
            if self.sa_mux_lev_1_dec._exist
            else 0.0
        )
        sa_mux_l2_dec_dyn = (
            self.sa_mux_lev_2_dec._power.read.dynamic
            if self.sa_mux_lev_2_dec._exist
            else 0.0
        )

        # mat.cc:1499-1500: row_dec dynamic is UNSCALED for FA (the *n_sub only
        # applies in the non-FA branch).
        decode_dyn = (
            self.r_predec._power.read.dynamic
            + b_mux_dyn
            + self.sa_mux_lev_1_predec._power.read.dynamic
            + self.sa_mux_lev_2_predec._power.read.dynamic
            + row_dec_dyn
            + bit_mux_dec_dyn
            + sa_mux_l1_dec_dyn
            + sa_mux_l2_dec_dyn
        )

        # mat.cc:1538-1540: neither precharge driver is scaled by n_sub for FA.
        precharge_read_dyn = (
            0
            if self._is_pure_cam
            else self.bl_precharge_eq_drv._power.read.dynamic
        ) + self.cam_bl_precharge_eq_drv._power.read.dynamic
        self._power_bl_precharge_eq_drv_search_dynamic = (
            0
            if self._is_pure_cam
            else self.bl_precharge_eq_drv._power.read.dynamic
        )

        # mat.cc:1542-1545: sense-amp energy scaled by column count, not n_sub.
        num_sa_subarray = (
            num_cols_fa_cam + num_cols_fa_ram
        ) // self._deg_bl_muxing
        num_sa_subarray_search = num_cols_fa_ram // self._deg_bl_muxing
        self._power_sa_search_dynamic = (
            self._power_sa.read.dynamic * num_sa_subarray_search
        )
        sa_read_dyn = self._power_sa.read.dynamic * num_sa_subarray

        # mat.cc:1549-1552: bitline energy scaled by column count, not n_sub.
        # power_bitline.searchOp is captured from the PRE-scaled per-column value.
        self._power_bitline_search_dynamic = (
            self._power_bitline.read.dynamic * num_cols_fa_ram
        )
        bitline_read_dyn = self._power_bitline.read.dynamic * (
            num_cols_fa_cam + num_cols_fa_ram
        )
        bitline_write_dyn = self._power_bitline.write.dynamic * (
            num_cols_fa_cam + num_cols_fa_ram
        )

        # mat.cc:1555-1559: subarray output driver, scaled by num_so/do_b_mat.
        out_drv_unit = (
            self._power_subarray_out_drv.read.dynamic + self._out_wire_dynamic
        )
        self._power_subarray_out_drv_search_dynamic = (
            out_drv_unit * self._num_so_b_mat
        )
        out_drv_read_dyn = out_drv_unit * self._num_do_b_mat

        self._bitline_read_mat = bitline_read_dyn
        self._bitline_write_mat = bitline_write_dyn
        self._sa_read_mat = sa_read_dyn

        self._power.read.dynamic = (
            decode_dyn
            + precharge_read_dyn
            + sa_read_dyn
            + bitline_read_dyn
            + out_drv_read_dyn
            + self._power_comparator.read.dynamic
        )
        # Same write recomputation as the non-FA path (uca.cc:273/406-421 apply
        # identically once is_tag==false, which FA always is).
        self._power.write.dynamic = (
            self._power.read.dynamic
            - self._bitline_read_mat
            - self._sa_read_mat
            + self._bitline_write_mat
        )

        # ---- search energy: power_cam_all_active (mat.cc:1573-1589) ----
        matchline_search_dyn = self._power_matchline_search_dynamic * n_sub
        searchline_precharge_dyn = (
            self.sl_precharge_eq_drv._power.read.dynamic * n_sub
        )
        searchline_dyn = (
            self.sl_data_drv._power.read.dynamic * num_cols_fa_cam * n_sub
        )
        matchline_precharge_dyn = (
            self.ml_precharge_drv._power.read.dynamic * n_sub
        )

        self._power.search.dynamic = (
            matchline_search_dyn
            + searchline_precharge_dyn
            + searchline_dyn
            + matchline_precharge_dyn
        )

    def dynamic_power(self):
        if self._is_cam:
            self._dynamic_power_fa()
            return
        # Read/write per-access energy of the mat, following Mat::compute_power_energy
        # (non-FA path).
        n_sub = self._num_subarrays_per_mat
        num_cols = self.subarray._cols

        b_mux_dyn = (
            self.b_mux_predec._power.read.dynamic if self._has_b_mux else 0.0
        )
        row_dec_dyn = (
            self.row_dec._power.read.dynamic if self.row_dec._exist else 0.0
        )
        bit_mux_dec_dyn = (
            self.bit_mux_dec._power.read.dynamic
            if self.bit_mux_dec._exist
            else 0.0
        )
        sa_mux_l1_dec_dyn = (
            self.sa_mux_lev_1_dec._power.read.dynamic
            if self.sa_mux_lev_1_dec._exist
            else 0.0
        )
        sa_mux_l2_dec_dyn = (
            self.sa_mux_lev_2_dec._power.read.dynamic
            if self.sa_mux_lev_2_dec._exist
            else 0.0
        )

        # Predecode + decode + wordline, shared by read and write. The predecoder
        # drivers (r_predec, b_mux_predec, sa_mux_lev_1/2_predec) are shared by all
        # subarrays in the mat; only the row decoder is replicated per subarray.
        decode_dyn = (
            self.r_predec._power.read.dynamic
            + b_mux_dyn
            + self.sa_mux_lev_1_predec._power.read.dynamic
            + self.sa_mux_lev_2_predec._power.read.dynamic
            + row_dec_dyn * n_sub
            + bit_mux_dec_dyn
            + sa_mux_l1_dec_dyn
            + sa_mux_l2_dec_dyn
        )
        precharge_dyn = self.bl_precharge_eq_drv._power.read.dynamic * n_sub

        # Subarray output drivers + output wire, one path per output data bit.
        out_drv_dyn = (
            self._power_subarray_out_drv.read.dynamic + self._out_wire_dynamic
        ) * self._num_do_b_mat

        # Mat-aggregated bitline/sense-amp energies (power_bitline/power_sa AFTER
        # the mat.cc:1510-1516 scaling). Stored so the UCA wrapper can apply the
        # num_act_mats_hor_dir scaling of the is_tag write recomputation.
        self._bitline_read_mat = (
            self._power_bitline.read.dynamic * n_sub * num_cols
        )
        self._bitline_write_mat = (
            self._power_bitline.write.dynamic * n_sub * num_cols
        )
        self._sa_read_mat = (
            self._power_sa.read.dynamic * self._num_sa_subarray * n_sub
        )

        # Read: bitline read swing + sense amps + output drivers + comparator
        # (mat.cc:1531; comparator dynamic is summed in UNSCALED -- it already
        # includes its own 4*tag_assoc factor from compute_comparator_power, unlike
        # its leakage/gate-leakage which get an additional num_do_b_mat*(RWP+ERP)
        # scale in static_power. Permanently 0 for non-tag arrays.)
        self._power.read.dynamic = (
            decode_dyn
            + precharge_dyn
            + self._sa_read_mat
            + self._bitline_read_mat
            + out_drv_dyn
            + self._power_comparator.read.dynamic
        )

        # Write energy per access (uca.cc:273): CACTI takes the full read energy and
        # swaps the bitline READ swing + sense amps for the differential bitline
        # WRITE swing -- everything else (decode, predecode, wordline, precharge,
        # output-driver path) fires on a write too. This is the single-mat (num_act
        # =1) form; the UCA wrapper recomputes it with num_act scaling and H-tree
        # deltas for the general case.
        self._power.write.dynamic = (
            self._power.read.dynamic
            - self._bitline_read_mat
            - self._sa_read_mat
            + self._bitline_write_mat
        )

    def _static_power_fa(self):
        """FA/CAM aggregation; pure CAM omits the RAM leakage terms."""
        # Port of Mat::compute_power_energy's `is_fa` leakage block (mat.cc:
        # 1810-1889). Unlike the dynamic side, EVERY component here IS scaled by
        # num_subarrays_per_mat -- and by (RWP+ERP+SCHP), not just (RWP+ERP).
        n_sub = self._num_subarrays_per_mat
        num_rows = self.subarray._rows
        num_cols = self.subarray._cols
        n_ports = self._num_rw_ports + self._num_r_ports + self._num_sr_ports
        n_out_drv = self._number_output_drivers_subarray

        bitline_leak = (
            0
            if self._is_pure_cam
            else self._power_bitline.read.leakage * num_rows * num_cols * n_sub
        )
        bitline_gleak = (
            0
            if self._is_pure_cam
            else self._power_bitline.read.gate_leakage
            * num_rows
            * num_cols
            * n_sub
        )
        precharge_leak = (
            0
            if self._is_pure_cam
            else self.bl_precharge_eq_drv._power.read.leakage * n_sub
        )
        precharge_gleak = (
            0
            if self._is_pure_cam
            else self.bl_precharge_eq_drv._power.read.gate_leakage * n_sub
        )
        precharge_search_leak = (
            self.cam_bl_precharge_eq_drv._power.read.leakage * n_sub
        )
        precharge_search_gleak = (
            self.cam_bl_precharge_eq_drv._power.read.gate_leakage * n_sub
        )
        sa_leak = (
            self._power_sa.read.leakage
            * self._num_sa_subarray
            * n_sub
            * n_ports
        )
        sa_gleak = (
            self._power_sa.read.gate_leakage
            * self._num_sa_subarray
            * n_sub
            * n_ports
        )
        out_drv_leak = (
            (
                self._power_subarray_out_drv.read.leakage
                + self._out_wire_leakage
            )
            * n_out_drv
            * n_sub
            * n_ports
        )
        out_drv_gleak = (
            (
                self._power_subarray_out_drv.read.gate_leakage
                + self._out_wire_gate_leakage
            )
            * n_out_drv
            * n_sub
            * n_ports
        )

        row_dec_leak = (
            (self.row_dec._power.read.leakage if self.row_dec._exist else 0.0)
            * num_rows
            * n_sub
        )
        row_dec_gleak = (
            (
                self.row_dec._power.read.gate_leakage
                if self.row_dec._exist
                else 0.0
            )
            * num_rows
            * n_sub
        )

        if self._is_pure_cam:
            row_dec_leak *= (
                self._num_rw_ports + self._num_r_ports + self._num_w_ports
            )
            row_dec_gleak *= (
                self._num_rw_ports + self._num_r_ports + self._num_w_ports
            )
        self._power.read.leakage = (
            bitline_leak
            + precharge_leak
            + precharge_search_leak
            + sa_leak
            + out_drv_leak
            + self.r_predec._power.read.leakage
            + row_dec_leak
        )
        self._power.read.gate_leakage = (
            bitline_gleak
            + precharge_gleak
            + precharge_search_gleak
            + sa_gleak
            + out_drv_gleak
            + self.r_predec._power.read.gate_leakage
            + row_dec_gleak
        )

        # power_cam_all_active.searchOp.{leakage,gate_leakage} (mat.cc:1841-1849,
        # 1880-1887) -- both add `ml_precharge_drv->power.readOp.dynamic` (NOT
        # .leakage/.gate_leakage): a real CACTI copy-paste bug, replicated
        # verbatim in both accumulators.
        cam_active_leak = (
            self._power_matchline_search_leakage
            + self.sl_precharge_eq_drv._power.read.leakage
            + self.sl_data_drv._power.read.leakage
            * self.subarray._num_cols_fa_cam
            + self.ml_precharge_drv._power.read.dynamic
        ) * n_sub
        cam_active_gleak = (
            self._power_matchline_search_gate_leakage
            + self.sl_precharge_eq_drv._power.read.gate_leakage
            + self.sl_data_drv._power.read.gate_leakage
            * self.subarray._num_cols_fa_cam
            + self.ml_precharge_drv._power.read.dynamic
        ) * n_sub

        self._power.read.leakage += cam_active_leak
        self._power.read.gate_leakage += cam_active_gleak

    def static_power(self):
        if self._is_cam:
            self._static_power_fa()
            return
        n_sub = self._num_subarrays_per_mat
        num_rows = self.subarray._rows
        num_cols = self.subarray._cols

        b_mux_leak = (
            self.b_mux_predec._power.read.leakage if self._has_b_mux else 0.0
        )
        b_mux_gleak = (
            self.b_mux_predec._power.read.gate_leakage
            if self._has_b_mux
            else 0.0
        )
        row_dec_leak = (
            self.row_dec._power.read.leakage if self.row_dec._exist else 0.0
        )
        row_dec_gleak = (
            self.row_dec._power.read.gate_leakage
            if self.row_dec._exist
            else 0.0
        )
        bit_mux_dec_leak = (
            self.bit_mux_dec._power.read.leakage
            if self.bit_mux_dec._exist
            else 0.0
        )
        bit_mux_dec_gleak = (
            self.bit_mux_dec._power.read.gate_leakage
            if self.bit_mux_dec._exist
            else 0.0
        )
        sa_mux_l1_dec_leak = (
            self.sa_mux_lev_1_dec._power.read.leakage
            if self.sa_mux_lev_1_dec._exist
            else 0.0
        )
        sa_mux_l1_dec_gleak = (
            self.sa_mux_lev_1_dec._power.read.gate_leakage
            if self.sa_mux_lev_1_dec._exist
            else 0.0
        )
        sa_mux_l2_dec_leak = (
            self.sa_mux_lev_2_dec._power.read.leakage
            if self.sa_mux_lev_2_dec._exist
            else 0.0
        )
        sa_mux_l2_dec_gleak = (
            self.sa_mux_lev_2_dec._power.read.gate_leakage
            if self.sa_mux_lev_2_dec._exist
            else 0.0
        )

        # The sense amps and subarray output drivers are replicated per read path,
        # i.e. per (RWP + ERP) port (mat.cc:1660/1665).
        n_ports_rd = self._num_rw_ports + self._num_r_ports

        # Subarray output driver + output wire leakage, per output bit, scaled by
        # the number of output drivers per subarray (mat.cc:1663).
        n_out_drv = self._number_output_drivers_subarray
        out_drv_leak = (
            (
                self._power_subarray_out_drv.read.leakage
                + self._out_wire_leakage
            )
            * n_out_drv
            * n_sub
            * n_ports_rd
        )
        out_drv_gleak = (
            (
                self._power_subarray_out_drv.read.gate_leakage
                + self._out_wire_gate_leakage
            )
            * n_out_drv
            * n_sub
            * n_ports_rd
        )

        # Comparator leakage/gate-leakage, scaled by num_do_b_mat*(RWP+ERP)
        # (mat.cc:1681,1768) -- NOT the same scale as its dynamic term above.
        # Permanently 0 for non-tag arrays.
        comparator_leak = (
            self._power_comparator.read.leakage
            * self._num_do_b_mat
            * n_ports_rd
        )
        comparator_gleak = (
            self._power_comparator.read.gate_leakage
            * self._num_do_b_mat
            * n_ports_rd
        )

        # Cell array + datapath leakage. There is one row-decoder driver per row,
        # and one SRAM cell per (row, col); hence the row/col multipliers. The
        # sa-mux decoders are weighted by their mux degree (mat.cc:1700-1701) and
        # the sa-mux predecoders are shared (per-mat, x1).
        self._power.read.leakage = (
            self._power_bitline.read.leakage * num_rows * num_cols * n_sub
            + self.bl_precharge_eq_drv._power.read.leakage * n_sub
            + self._power_sa.read.leakage
            * self._num_sa_subarray
            * n_sub
            * n_ports_rd
            + out_drv_leak
            + self.r_predec._power.read.leakage
            + b_mux_leak
            + self.sa_mux_lev_1_predec._power.read.leakage
            + self.sa_mux_lev_2_predec._power.read.leakage
            + row_dec_leak * num_rows * n_sub
            + bit_mux_dec_leak * self._deg_bl_muxing
            + sa_mux_l1_dec_leak * self._Ndsam_lev_1
            + sa_mux_l2_dec_leak * self._Ndsam_lev_2
            + comparator_leak
        )

        self._power.read.gate_leakage = (
            self._power_bitline.read.gate_leakage * num_rows * num_cols * n_sub
            + self.bl_precharge_eq_drv._power.read.gate_leakage * n_sub
            + out_drv_gleak
            + self.r_predec._power.read.gate_leakage
            + b_mux_gleak
            + self.sa_mux_lev_1_predec._power.read.gate_leakage
            + self.sa_mux_lev_2_predec._power.read.gate_leakage
            + row_dec_gleak * num_rows * n_sub
            + bit_mux_dec_gleak * self._deg_bl_muxing
            + sa_mux_l1_dec_gleak * self._Ndsam_lev_1
            + sa_mux_l2_dec_gleak * self._Ndsam_lev_2
            + comparator_gleak
        )

    def compute_delays(self, inrisetime):
        # Additive-only orchestrator wiring together the Phase 29 delay-chain
        # methods (compute_bitline_delay/compute_sa_delay/
        # compute_subarray_out_drv, CactiDriver.compute_delay) and the
        # already-ported  row_dec/r_predec/side-mux predecoder
        # chains, mirroring Mat::compute_delays' "normal SRAM/DRAM,
        # non-FA/non-CAM" branch (mat.cc:588-653) call order exactly. Not
        # called from __init__ or anywhere in the energy pipeline -- purely
        # additive, same discipline as Phase 26/28.
        #
        # Out of scope (see the native reference): compute_comparator_delay
        # (mat.cc:641-644, separate component, tag arrays only).
        # delay_subarray_out_drv_htree (mat.cc:638 = delay_subarray_out_drv +
        # subarray_out_wire->delay) IS computed here (Task 3) -- see its own
        # comment below.
        #
        # CAM search uses the matchline chain. Plain reads/writes evaluate
        # their separate decoder and sense-amplifier chain.
        if self._is_pure_cam:
            search_rise = self.compute_cam_delay(inrisetime)
            search_rise = self.compute_subarray_out_drv(search_rise)
            self.delay_subarray_out_drv_htree = (
                self.delay_subarray_out_drv + self._subarray_out_wire_delay()
            )
            rise = self.r_predec.compute_delays(inrisetime)
            row_rise = self.row_dec.compute_delays(rise)
            if self._has_b_mux:
                self.bit_mux_dec.compute_delays(
                    self.b_mux_predec.compute_delays(inrisetime)
                )
            self.sa_mux_lev_1_dec.compute_delays(
                self.sa_mux_lev_1_predec.compute_delays(inrisetime)
            )
            self.sa_mux_lev_2_dec.compute_delays(
                self.sa_mux_lev_2_predec.compute_delays(inrisetime)
            )
            self.compute_sa_delay(self.compute_bitline_delay(row_rise))
            return search_rise
        if self._is_fa:
            return self._compute_delays_fa(inrisetime)

        p_to_n = self.pmos_to_nmos_sz_ratio()
        w_pmos_bl_precharge = 6 * p_to_n * self._tp["min_w_nmos"]

        # Precharge-driver delay (mat.cc:590), always invoked with a literal
        # hardcoded 0 -- never chained from any upstream signal.
        self.bl_precharge_eq_drv.compute_delay(0)

        # delay_wl_reset (mat.cc:591-601): an independent estimate from
        # row_dec's own already-sized final-stage widths (set by
        # compute_widths() in CactiDecoder.__init__, unaffected by
        # compute_delays() below) -- horowitz(0, ...), also hardcoded.
        if self.row_dec._exist:
            k = self.row_dec._num_gates - 1
            rd = self.tr_R_on(
                self.row_dec._w_dec_n[k],
                "NCH",
                1,
                is_dram=self._is_dram,
                is_wl_tr=True,
            )
            C_intrinsic = self.drain_C(
                self.row_dec._w_dec_p[k],
                "PCH",
                1,
                1,
                4 * self._cell_h,
                is_dram=self._is_dram,
                is_wl_tr=True,
            ) + self.drain_C(
                self.row_dec._w_dec_n[k],
                "NCH",
                1,
                1,
                4 * self._cell_h,
                is_dram=self._is_dram,
                is_wl_tr=True,
            )
            C_ld = self.row_dec._C_ld_dec_out
            tf = (
                rd * (C_intrinsic + C_ld)
                + self.row_dec._R_wire_dec_out * C_ld / 2
            )
            self.delay_wl_reset = self.horowitz(0, tf, 0.5, 0.5, 1)

        # delay_bl_restore (mat.cc:602-616), SRAM arm only (is_dram=False
        # project-wide). V_b_pre reuses self._tp['Vdd'] -- see
        # compute_bitline_delay's docstring for the proof this is exact
        # (g_tp.sram.Vbitpre = g_tp.sram_cell.Vdd = g_tp.peri_global.Vdd).
        R_bl_precharge = self.tr_R_on(
            w_pmos_bl_precharge, "PCH", 1, is_dram=self._is_dram
        )
        r_b_metal = self._cell_h * self._wp["R_per_micron"]
        R_bl = self.subarray._rows * r_b_metal
        C_bl = self.subarray._C_bl
        V_b_pre = self._tp["Vdd"]
        self.delay_bl_restore = self.bl_precharge_eq_drv.delay + log(
            (V_b_pre - 0.1 * self._V_b_sense) / (V_b_pre - self._V_b_sense)
        ) * (R_bl_precharge * C_bl + R_bl * C_bl / 2)

        # Row decode chain -- row_dec_outrisetime is the only one of the four
        # predecoder/decoder outrisetimes below that survives to feed
        # bitline delay (mat.cc:621-631: the other three overwrite the same
        # local `outrisetime` in the C++ and are called for side-effect
        # parity only -- their own .delay members get populated, consumed
        # elsewhere/later, not propagated further here).
        outrisetime = self.r_predec.compute_delays(inrisetime)
        row_dec_outrisetime = self.row_dec.compute_delays(outrisetime)

        if self._has_b_mux:
            outrisetime = self.b_mux_predec.compute_delays(inrisetime)
            self.bit_mux_dec.compute_delays(outrisetime)

        outrisetime = self.sa_mux_lev_1_predec.compute_delays(inrisetime)
        self.sa_mux_lev_1_dec.compute_delays(outrisetime)

        outrisetime = self.sa_mux_lev_2_predec.compute_delays(inrisetime)
        self.sa_mux_lev_2_dec.compute_delays(outrisetime)

        outrisetime = self.compute_bitline_delay(
            row_dec_outrisetime
        )  # always 0.0
        outrisetime = self.compute_sa_delay(outrisetime)  # always 0.0
        outrisetime = self.compute_subarray_out_drv(
            outrisetime
        )  # real chained value

        # delay_subarray_out_drv_htree (mat.cc:638): subarray_out_wire's own
        # delay, added on top of delay_subarray_out_drv. subarray_out_wire is
        # built as `new Wire(Global, (cl_vertical?area.w:area.h), 1, 1,
        # inside_mat)` (mat.cc:266). g_ip->cl_vertical is NOT False by
        # default -- InputParameter's own constructor default is TRUE
        # (io.cc:61), and only a "-CLDriver vertical" cfg line (parsed at
        # io.cc:574-582) can override it; no cfg/XML this project uses sets
        # one (grepped icache_test.cfg, .ARM_A9_2GHz_gem5.xml, and McPAT's
        # XML_Parse.cc, which has no CLDriver translation at all), so
        # cl_vertical is really always True here -- the opposite of what it
        # looks like at a glance. Confirmed empirically: area_w matches the
        # MAT_DELAY_HTREE C++ probe to FP precision (ic_dp 1.370397120503816e-11
        # vs target 1.3703971205038162e-11; ae_dp 2.4302702161597202e-11 vs
        # target 2.4302702161597192e-11), area_h is off by 25-40%.
        #
        # It also LOOKS like this should read wire_is_mat_type's geometry
        # (placement=inside_mat), but Wire::calculate_wire_stats' wt==Global
        # branch (wire.cc:148-158) sets `delay = global.delay * wire_length`
        # from the STATIC cached Wire::global, never recomputing from this
        # instance's own wire_placement -- wire_placement only feeds
        # wire_width/wire_spacing (area, not delay) in that branch.
        # Wire::global is populated once per array by the first
        # (default-constructed) Wire, whose wire_placement default is
        # outside_mat (wire.h:52/60) -- i.e. keyed by wire_os_mat_type, not
        # wire_is_mat_type. This is the exact same quirk already ported for
        # this wire's ENERGY in repeated_wire_energy_per_um()
        # (self._out_wire_* fields, wire.cc verified line-by-line); reuse
        # those same fields here rather than
        # _wire_layer_geom(wire_is_mat_type), for identical reasons. Only
        # matters when wire_is_mat_type != wire_os_mat_type (L2/L3-style
        # configs); numerically identical to the naive reading whenever they
        # agree (every array validated so far, including both anchors below).
        self.delay_subarray_out_drv_htree = (
            self.delay_subarray_out_drv + self._subarray_out_wire_delay()
        )

        # row_dec.exist==False fallback (mat.cc:646-648), evaluated after
        # r_predec/row_dec have run -- matches the C++'s own line ordering.
        if not self.row_dec._exist:
            self.delay_wl_reset = max(
                self.r_predec.blk1.delay, self.r_predec.blk2.delay
            )

        return outrisetime

    def _subarray_out_wire_delay(self):
        # subarray_out_wire's own delay (mat.cc:266's `new Wire(Global, ...)`,
        # `->delay` read at mat.cc:638/562) -- factored out of compute_delays
        # so both the non-FA tail and _compute_delays_fa (mat.cc:561-564,
        # which reaches this same wire construction unconditionally too) can
        # share it without duplicating the cl_vertical/Wire::global reasoning
        # above verbatim.
        wire_point = self._wire_model(
            self._out_wire_pitch_um,
            self._out_wire_aspect,
            self._out_wire_horiz,
            self._out_wire_vert,
            self._out_wire_ild,
            self._out_wire_fringe,
            overhead=0,
            is_dram=self._is_dram,
        )
        return wire_point["delay_per_um"] * self.subarray.area_w

    def _compute_delays_fa(self, inrisetime):
        # Port of Mat::compute_delays' is_fa branch (mat.cc:536-587), called
        # only for self._is_fa (pure_cam is out of scope, see compute_delays).
        # Structurally separate from the non-FA tail above, not a variant of
        # it: real CACTI's is_fa/pure_cam arm computes the CAM search chain,
        # the RAM-data-half restore/bitline/sense-amp/output-driver chain,
        # and then returns outrisetime_search directly -- it never reaches
        # the non-FA tail's row_dec.exist==False fallback or comparator-delay
        # call. (the native reference.)
        outrisetime_search = self.compute_cam_delay(inrisetime)

        # bl_precharge_eq_drv's own delay (mat.cc:539), always invoked with a
        # literal hardcoded 0, same as the non-FA tail's own precharge-driver
        # call -- feeds delay_bl_restore below.
        self.bl_precharge_eq_drv.compute_delay(0)

        # delay_wl_reset (mat.cc:539-547): derived from ml_to_ram_wl_drv's
        # own already-sized final-stage width, NOT row_dec (FA mats have no
        # row decoder driving the data half directly) -- same horowitz(0,...)
        # shape as the non-FA row_dec case above, different source object.
        # Real CACTI bug, replicated verbatim: BOTH the PCH and NCH drain_C
        # calls use width_n[k] (not width_p[k] for the PCH call).
        k = self.ml_to_ram_wl_drv._num_gates - 1
        w = self.ml_to_ram_wl_drv._width_n[k]
        rd = self.tr_R_on(w, "NCH", 1, is_dram=self._is_dram, is_wl_tr=True)
        C_intrinsic = self.drain_C(
            w,
            "PCH",
            1,
            1,
            4 * self._cell_h,
            is_dram=self._is_dram,
            is_wl_tr=True,
        ) + self.drain_C(
            w,
            "NCH",
            1,
            1,
            4 * self._cell_h,
            is_dram=self._is_dram,
            is_wl_tr=True,
        )
        C_ld = (
            self.ml_to_ram_wl_drv._c_gate_load
            + self.ml_to_ram_wl_drv._c_wire_load
        )
        tf = (
            rd * (C_intrinsic + C_ld)
            + self.ml_to_ram_wl_drv._r_wire_load * C_ld / 2
        )
        self.delay_wl_reset = self.horowitz(0, tf, 0.5, 0.5, 1)

        # delay_bl_restore (mat.cc:549-555): same formula shape as the non-FA
        # case, but r_b_metal uses the CAM cell's height (dummy rows in SRAM
        # are "filled in", per the C++ comment) while C_bl stays the RAM
        # bitline cap (self.subarray._C_bl, not _C_bl_cam).
        p_to_n = self.pmos_to_nmos_sz_ratio()
        w_pmos_bl_precharge = 6 * p_to_n * self._tp["min_w_nmos"]
        R_bl_precharge = self.tr_R_on(
            w_pmos_bl_precharge, "PCH", 1, is_dram=self._is_dram
        )
        r_b_metal = self._cam_cell_h * self._wp["R_per_micron"]
        R_bl = self.subarray._rows * r_b_metal
        C_bl = self.subarray._C_bl
        V_b_pre = self._tp["Vdd"]
        self.delay_bl_restore = self.bl_precharge_eq_drv.delay + log(
            (V_b_pre - 0.1 * self._V_b_sense) / (V_b_pre - self._V_b_sense)
        ) * (R_bl_precharge * C_bl + R_bl * C_bl / 2)

        # RAM-data-half read chain, on the CAM search chain's own outrisetime
        # (mat.cc:557-558) -- compute_bitline_delay/compute_sa_delay are
        # exercised with self._is_cam=True here for the first time (see their
        # own camFlag-bug-replication comments).
        outrisetime_search = self.compute_bitline_delay(outrisetime_search)
        outrisetime_search = self.compute_sa_delay(outrisetime_search)

        outrisetime_search = self.compute_subarray_out_drv(outrisetime_search)
        self.delay_subarray_out_drv_htree = (
            self.delay_subarray_out_drv + self._subarray_out_wire_delay()
        )

        # Predecoder/decoder chains (mat.cc:567-578): side-effect-only, per
        # real CACTI's own TODO comment there ("just for compute plain
        # read/write energy for fa and cam, plain read/write access timing
        # need to be revisited") -- their outputs are NOT chained into
        # outrisetime_search, matching the C++ exactly. b_mux_predec/
        # bit_mux_dec are gated on self._has_b_mux here, same as the non-FA
        # tail's own gate and for the same reason (Phase 33's measured-0.0
        # shortcut for real CACTI's unconditional construction). Re-checked
        # for FA arrays specifically (the native reference): all 566 FA/CAM
        # corpus arms have deg_bl_muxing==1 (self._has_b_mux==False), so this
        # is the ONLY case exercised for FA so far -- CactiUCA.compute_delays'
        # cycle_time MAX against b_mux_predec.delay matched bit-exact across
        # all of them, confirming the shortcut for that case. No FA record
        # with deg_bl_muxing>1 exists in the corpus to test the has_b_mux==
        # True arm for FA specifically.
        outrisetime = self.r_predec.compute_delays(inrisetime)
        self.row_dec.compute_delays(
            outrisetime
        )  # row_dec_outrisetime discarded, matches mat.cc:568-569

        if self._has_b_mux:
            outrisetime = self.b_mux_predec.compute_delays(inrisetime)
            self.bit_mux_dec.compute_delays(outrisetime)

        outrisetime = self.sa_mux_lev_1_predec.compute_delays(inrisetime)
        self.sa_mux_lev_1_dec.compute_delays(outrisetime)

        outrisetime = self.sa_mux_lev_2_predec.compute_delays(inrisetime)
        self.sa_mux_lev_2_dec.compute_delays(outrisetime)

        return outrisetime_search

    def compute_comparators_height(
        self, tagbits, number_ways_in_mat, subarray_mem_cell_area_width
    ):
        # Port of Mat::compute_comparators_height (mat.cc:1057-1065). The
        # call site (mat.cc:1062) passes w_pmos=0 literally into
        # compute_gate_area(NAND, 2, 0, w_comp_n, cell_h_def) -- its
        # `w_pmos<=0` early-return guard  makes this whole
        # function always return 0.0 for every real config (confirmed by a
        # live COMPARATORS_HEIGHT_PROBE run against icache_test.cfg).
        # Ported faithfully anyway, not special-cased -- "replicate real
        # CACTI, don't shortcut it."
        nand2_area = self.compute_gate_area(
            "NAND", 2, 0, self._tp["w_comp_n"], self._tp["cell_h_def"]
        )
        cumulative_area = nand2_area * number_ways_in_mat * tagbits / 4
        return cumulative_area / subarray_mem_cell_area_width

    def compute_bit_mux_sa_precharge_sa_mux_wr_drv_wr_mux_h(self):
        # Port of Mat::compute_bit_mux_sa_precharge_sa_mux_wr_drv_wr_mux_h
        # (mat.cc:658-704). The write-driver/write-mux height term at the
        # end is commented out in real CACTI (mat.cc:691-701, a "TODO:
        # uncommented" note in the source itself) -- not ported, replicating
        # the omission rather than "fixing" it.
        RWP, ERP, EWP, SCHP = (
            self._num_rw_ports,
            self._num_r_ports,
            self._num_w_ports,
            self._num_sr_ports,
        )
        p_to_n = self.pmos_to_nmos_sz_ratio()
        w_pmos_bl_precharge = 6 * p_to_n * self._tp["min_w_nmos"]
        w_pmos_bl_eq = p_to_n * self._tp["min_w_nmos"]

        # camFlag ternary-precedence quirk (mat.cc:661-662, flagged but
        # deferred by the native reference): C++'s `?:` binds looser than
        # `/`, so `camFlag ? cam_cell.w : cell.w / divisor` leaves the
        # cam_cell.w arm UNDIVIDED by the port-count factor -- only the
        # cell.w arm is divided. Python's `A if cond else B/C` has this
        # exact same grouping (the else-branch is the whole `B/C`), so
        # writing it this way replicates the bug exactly, not "avoids" it.
        height = self.compute_tr_width_after_folding(
            w_pmos_bl_precharge,
            (
                self._cam_cell_w
                if self._is_cam
                else self._cell_w / (2 * (RWP + ERP + SCHP))
            ),
        ) + self.compute_tr_width_after_folding(
            w_pmos_bl_eq,
            (
                self._cam_cell_w
                if self._is_cam
                else self._cell_w / (RWP + ERP + SCHP)
            ),
        )

        if self._deg_bl_muxing > 1:
            height += self.compute_tr_width_after_folding(
                self._tp["w_nmos_b_mux"], self._cell_w / (2 * (RWP + ERP))
            )

        # The camFlag/sram_cell.w arm here is commented out in real CACTI
        # (mat.cc:670) -- the live call always uses cell.w, regardless of
        # camFlag. Not a bug to "fix" -- port the live code, not the comment.
        height += self.height_sense_amplifier(
            self._cell_w * self._deg_bl_muxing / (RWP + ERP)
        )

        if self._Ndsam_lev_1 > 1:
            height += self.compute_tr_width_after_folding(
                self._tp["w_nmos_sa_mux"],
                self._cell_w * self._Ndsam_lev_1 / (RWP + ERP),
            )

        if self._Ndsam_lev_2 > 1:
            height += self.compute_tr_width_after_folding(
                self._tp["w_nmos_sa_mux"],
                self._cell_w
                * self._deg_bl_muxing
                * self._Ndsam_lev_1
                / (RWP + ERP),
            )
            height += 2 * self.compute_tr_width_after_folding(
                p_to_n * self._tp["min_w_nmos"],
                self._cell_w * self._Ndsam_lev_2 / (RWP + ERP),
            )
            height += 2 * self.compute_tr_width_after_folding(
                self._tp["min_w_nmos"],
                self._cell_w * self._Ndsam_lev_2 / (RWP + ERP),
            )

        return height

    def compute_area(self):
        # Port of Mat::Mat's area computation (mat.cc:308-470, inlined in
        # the real C++ constructor, not a separate method there). Phase 30.
        # Purely additive -- not called from __init__ or the energy
        # pipeline. Returns (derived_area_w, derived_area_h) and stores
        # them under THOSE names only -- never self.area_w/self.area_h,
        # which remain the given input CactiBank/CactiUCA already consume
        # (see the native reference's naming-collision warning).
        #
        # ver_htree_wires_over_array (mat.cc:394) is hardcoded 0 by McPAT
        # itself (processor.cc:794, never read from XML) -- so for every
        # McPAT-driven array the `if (!g_ip->ver_htree_wires_over_array)`
        # branch (mat.cc:396-413) is unconditionally taken; only that
        # branch's h_non_cell_area formula is ported (the throwaway value
        # at mat.cc:374-376 is always immediately overwritten by it).
        #
        # The is_fa/pure_cam area branch is dead code in real CACTI
        # (mat.cc:441-470 -- the `if(!is_fa)` markers and the entire `else`
        # arm are commented out) -- every mat, FA/CAM or not, uses the
        # single live formula below, not two branches.
        RWP, ERP, EWP, SCHP = (
            self._num_rw_ports,
            self._num_r_ports,
            self._num_w_ports,
            self._num_sr_ports,
        )

        self.row_dec.compute_area()
        area_row_decoder = (
            self.row_dec.area_w
            * self.row_dec._area_h
            * self.subarray._rows
            * (RWP + ERP + EWP)
        )
        w_row_decoder = area_row_decoder / self.subarray.area_h

        h_bit_mux_sense_amp_precharge_sa_mux_write_driver_write_mux = (
            self.compute_bit_mux_sa_precharge_sa_mux_wr_drv_wr_mux_h()
        )

        # h_subarray_out_drv (mat.cc:322-326): subarray_out_wire->area.get_area(),
        # ported the same way compute_subarray_out_drv_power already ports its
        # dynamic-power term (mat.cc:1377) -- both read off the same one-time,
        # closed-form repeater_size/repeater_spacing this port already stores
        # (self._out_wire_repeater_size/_out_wire_repeater_spacing_um, Phase
        # 4/6), NOT a per-instance repeater-count delay-side search (that
        # remains Phase 23's separate, unstarted proposal item -- confirmed
        # inapplicable here since wire.cc's own area.set_area() call shares
        # the identical repeater_spacing_init/repeater_scaling values already
        # feeding the validated power.readOp.dynamic term in the same C++
        # function, wire.cc:598-621).
        p_to_n = self.pmos_to_nmos_sz_ratio()
        subarray_width = self.subarray.area_w
        subarray_out_wire_area = (
            subarray_width / self._out_wire_repeater_spacing_um
        ) * self.compute_gate_area(
            "INV",
            1,
            p_to_n * self._tp["min_w_nmos"] * self._out_wire_repeater_size,
            self._tp["min_w_nmos"] * self._out_wire_repeater_size,
            self._tp["cell_h_def"],
        )
        # subarray.num_cols/(deg_bl_muxing*Ndsam_lev_1*Ndsam_lev_2): all C++
        # operands are unsigned int/int -- truncating integer division.
        h_subarray_out_drv = (
            subarray_out_wire_area
            * (
                self.subarray._cols
                // (
                    self._deg_bl_muxing * self._Ndsam_lev_1 * self._Ndsam_lev_2
                )
            )
            / self.subarray.area_w
        )
        h_subarray_out_drv *= RWP + ERP + SCHP

        # h_comparators (mat.cc:333-338): only (not is_fa) and dp.is_tag.
        h_comparators = 0.0
        if (not self._is_fa) and self.dp.cfg.is_tag:
            h_comparators = self.compute_comparators_height(
                self.dp.tagbits, self._num_do_b_mat, self.subarray.area_w
            )
            h_comparators *= RWP + ERP

        # w_row_predecode_output_wires (mat.cc:368-371). NOTE the swap: the
        # blk1-side branch effort comes from blk2's number_input_addr_bits,
        # and vice versa -- this is the real C++, not a transcription slip.
        branch_effort_predec_blk1_out = (
            1 << self.r_predec_blk2.number_input_addr_bits
        )
        branch_effort_predec_blk2_out = (
            1 << self.r_predec_blk1.number_input_addr_bits
        )
        w_row_predecode_output_wires = (
            (branch_effort_predec_blk1_out + branch_effort_predec_blk2_out)
            * self._pitch_mat
            * (RWP + ERP + EWP)
        )

        w_non_cell_area = max(
            w_row_predecode_output_wires,
            self._num_subarrays_per_row * w_row_decoder,
        )

        h_bit_mux_dec_out_wires = 0.0
        h_senseamp_mux_dec_out_wires = 0.0
        if self._deg_bl_muxing > 1:
            h_bit_mux_dec_out_wires = (
                self._deg_bl_muxing * self._pitch_mat * (RWP + ERP)
            )
        if self._Ndsam_lev_1 > 1:
            h_senseamp_mux_dec_out_wires = (
                self._Ndsam_lev_1 * self._pitch_mat * (RWP + ERP)
            )
        if self._Ndsam_lev_2 > 1:
            h_senseamp_mux_dec_out_wires += (
                self._Ndsam_lev_2 * self._pitch_mat * (RWP + ERP)
            )

        # h_addr_datain_wires (mat.cc:396-406), always the ver_htree_wires_
        # over_array-true branch (see docstring above). (num_di_b_mat +
        # num_do_b_mat)/num_subarrays_per_row is C++ int/int truncating
        # division.
        h_addr_datain_wires = (
            (
                self.dp.number_addr_bits_mat
                + self._number_way_select_signals_mat
                + (self._num_di_b_mat + self._num_do_b_mat)
                // self._num_subarrays_per_row
            )
            * self._pitch_mat
            * (RWP + ERP + EWP)
        )
        if self._is_fa or self._is_pure_cam:
            num_si_b_mat = getattr(self.dp, "num_si_b_mat", 0)
            h_addr_datain_wires += (
                (num_si_b_mat + self._num_so_b_mat)
                // self._num_subarrays_per_row
                * self._pitch_mat
                * SCHP
            )

        h_non_cell_area = (
            (
                h_bit_mux_sense_amp_precharge_sa_mux_write_driver_write_mux
                + h_comparators
                + h_subarray_out_drv
            )
            * (self._num_subarrays_per_mat // self._num_subarrays_per_row)
            + h_addr_datain_wires
            + h_bit_mux_dec_out_wires
            + h_senseamp_mux_dec_out_wires
        )

        # area_mat_center_circuitry (mat.cc:418-437): 20 objects' ->area.
        # get_area(), * (RWP+ERP+EWP). The b_mux_* quartet only exists when
        # self._has_b_mux (deg_bl_muxing>1) -- omitted from the sum
        # otherwise, matching real CACTI where they'd report exist=False/
        # area=0 anyway. The parallel "dummy_way_sel_predec_blk2"/"dummy_
        # way_sel_predec_blk_drv2" pair is confirmed dead in real CACTI
        # (constructed/deleted, never read elsewhere) and is not ported
        # (see __init__'s way_sel_drv1 comment) -- only way_sel_drv1 itself
        # is summed here.
        drv_objs = [
            self.r_predec_drv1,
            self.sa_mux_lev_1_predec_drv1,
            self.sa_mux_lev_2_predec_drv1,
            self.way_sel_drv1,
            self.r_predec_drv2,
            self.sa_mux_lev_1_predec_drv2,
            self.sa_mux_lev_2_predec_drv2,
        ]
        blk_objs = [
            self.r_predec_blk1,
            self.sa_mux_lev_1_predec_blk1,
            self.sa_mux_lev_2_predec_blk1,
            self.r_predec_blk2,
            self.sa_mux_lev_1_predec_blk2,
            self.sa_mux_lev_2_predec_blk2,
        ]
        if self._has_b_mux:
            drv_objs += [self.b_mux_predec_drv1, self.b_mux_predec_drv2]
            blk_objs += [self.b_mux_predec_blk1, self.b_mux_predec_blk2]

        area_mat_center_circuitry = 0.0
        for drv in drv_objs:
            drv.compute_area()
            area_mat_center_circuitry += drv.area
        for blk in blk_objs:
            blk.compute_area()
            area_mat_center_circuitry += blk.area

        self.bit_mux_dec.compute_area()
        self.sa_mux_lev_1_dec.compute_area()
        self.sa_mux_lev_2_dec.compute_area()
        area_mat_center_circuitry += (
            self.bit_mux_dec.area_w * self.bit_mux_dec._area_h
            + self.sa_mux_lev_1_dec.area_w * self.sa_mux_lev_1_dec._area_h
            + self.sa_mux_lev_2_dec.area_w * self.sa_mux_lev_2_dec._area_h
        )
        area_mat_center_circuitry *= RWP + ERP + EWP

        # Final combine (mat.cc:444-447): subarrays tile num_subarrays_per_row
        # wide by (num_subarrays_per_mat // num_subarrays_per_row) tall, each
        # dimension gets its own non-cell strip added, then
        # area_mat_center_circuitry (a pure scalar area, no w/h split) is
        # folded in by treating the just-computed area.h as FIXED and
        # re-deriving area.w from (rectangle_area + circuitry_area)/h -- NOT
        # area.h itself changing. num_subarrays_per_mat/num_subarrays_per_row
        # is C++ int/int truncating division.
        assert self._num_subarrays_per_mat // self._num_subarrays_per_row > 0
        derived_area_h = (
            self._num_subarrays_per_mat // self._num_subarrays_per_row
        ) * self.subarray.area_h + h_non_cell_area
        derived_area_w = (
            self._num_subarrays_per_row * self.subarray.area_w
            + w_non_cell_area
        )
        derived_area_w = (
            derived_area_h * derived_area_w + area_mat_center_circuitry
        ) / derived_area_h

        self.derived_area_w = derived_area_w
        self.derived_area_h = derived_area_h
        return derived_area_w, derived_area_h
