from .cacti_base_params import CactiBaseParams
from .config_input_params import McPATMachineConfig

CU_RESISTIVITY = 0.022
BULK_CU_RESISTIVITY = 0.018


class CactiWireParams(CactiBaseParams):
    def __init__(self, machine: McPATMachineConfig):
        CactiBaseParams.__init__(self, machine._config_params["tech_node"])
        self._ic_type = machine._config_params["interconnect_type"]
        # 'C_per_micron'/'R_per_micron'/'pitch' are the LOCAL wire (CACTI
        # wire_local, pitch 2.5F), used for bitline/wordline metal. The
        # '*_inside_mat' entries are the semi-global wire (CACTI wire_inside_mat,
        # pitch 4F) used by the subarray output wire.
        self._wire_params = {
            "C_per_micron": 0,
            "R_per_micron": 0,
            "pitch": 0,
            "C_per_micron_inside_mat": 0,
            "R_per_micron_inside_mat": 0,
            "pitch_inside_mat": 0,
            # Raw semi-global geometry, kept so the repeated-wire
            # energy can recompute cap/res the wire.cc way (which
            # uses a DIFFERENT formula than wire_capacitance above).
            "aspect_ratio_inside_mat": 0,
            "horiz_dielec_inside_mat": 0,
            "vert_dielec_inside_mat": 0,
            "ild_inside_mat": 0,
            # Raw LOCAL wire geometry (populated at every node),
            # used for the subarray output wire when McPAT sets
            # wire_is_mat_type = 0 (local layer).
            "aspect_ratio_local": 0,
            "horiz_dielec_local": 0,
            "vert_dielec_local": 0,
            "ild_local": 0,
            "fringe_local": 0,
            # Global (outside_mat, wire type 2, 8F) layer, used
            # for NON-embedded configs (McPAT wire_os_mat_type=2).
            # Populated per-node in each init_wire_XXnm() below,
            # same as local/semi-global -- aspect_ratio/
            # ild_thickness/horiz_dielec_const genuinely vary by
            # node in technology.cc's own table (only the 8F pitch
            # formula, vert_dielec=3.9, and fringe_cap=0.115e-15
            # are actually node-independent). Previously computed
            # once with hardcoded values that turned out to be an
            # exact, uncredited copy of the 180nm row, applied to
            # every node regardless of target -- found via the
            # Phase 14 sweep (PROGRESS.md), fixed this phase.
            "C_per_micron_global": 0,
            "R_per_micron_global": 0,
            "pitch_global": 0,
            "aspect_ratio_global": 0,
            "horiz_dielec_global": 0,
            "vert_dielec_global": 0,
            "ild_global": 0,
            "fringe_global": 0,
        }

    def validate_params(self):
        super().validate_params()
        if not (self._ic_type == 0 or self._ic_type == 1):
            raise ValueError(f"IC Type ({interconnect_type}) must be\
                    aggressive (0) or conservative! (1)")

    def wire_capacitance(
        self,
        wire_w,
        wire_t,
        wire_s,
        ild_t,
        miller_val,
        horiz_dielec_const,
        vert_dielec_const,
        fringe_cap,
    ):
        vert_cap = (
            2 * self._perm_free_space * vert_dielec_const * wire_w / ild_t
        )
        sidewall_cap = (
            2
            * self._perm_free_space
            * miller_val
            * horiz_dielec_const
            * wire_t
            / wire_s
        )
        return vert_cap + sidewall_cap + fringe_cap

    def wire_resistance(
        self, resistivity, wire_w, wire_t, barrier_t, dishing_t, alpha_scatter
    ):
        return (
            alpha_scatter
            * resistivity
            / ((wire_t - barrier_t - dishing_t) * (wire_w - 2 * barrier_t))
        )

    def init_wire_params(self):
        interp_params = self.get_alpha()
        for alpha, tech in zip(interp_params[0::2], interp_params[1::2]):
            if tech == 180:
                self.init_wire_180nm(alpha)
            if tech == 90:
                self.init_wire_90nm(alpha)
            if tech == 65:
                self.init_wire_65nm(alpha)
            if tech == 45:
                self.init_wire_45nm(alpha)
            if tech == 32:
                self.init_wire_32nm(alpha)
            if tech == 22:
                self.init_wire_22nm(alpha)

    def init_wire_180nm(self, alpha):
        if self._ic_type == 0:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.017
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.75
            miller_val = 1.5
            horiz_dielec_const = 2.709
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.017
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.75
            miller_val = 1.5
            horiz_dielec_const = 3.038
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_per_micron = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_per_micron = self.wire_resistance(
            CU_RESISTIVITY,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron"] += alpha * wire_c_per_micron
        self._wire_params["R_per_micron"] += alpha * wire_r_per_micron
        self._wire_params["pitch"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_local"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_local"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_local"] += alpha * vert_dielec_const
        self._wire_params["ild_local"] += alpha * ild_thickness
        self._wire_params["fringe_local"] += alpha * fringe_cap

        # Semi-global (inside_mat) wire, technology.cc [ic][1], 180nm. Was entirely
        # missing (same gap Phase 13 found/fixed at 65nm, the native reference
        # Finding 2) -- 180nm/non-embedded arrays that set wire_is_mat_type=1 or
        # wire_os_mat_type=1 hit a ZeroDivisionError in _wire_model, not a wrong
        # number.
        if self._ic_type == 0:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.4
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.017
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.75
            miller_val = 1.5
            horiz_dielec_const = 2.709
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.017
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.75
            miller_val = 1.5
            horiz_dielec_const = 3.038
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_sg = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_sg = self.wire_resistance(
            resistivity,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron_inside_mat"] += alpha * wire_c_sg
        self._wire_params["R_per_micron_inside_mat"] += alpha * wire_r_sg
        self._wire_params["pitch_inside_mat"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_inside_mat"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_inside_mat"] += (
            alpha * horiz_dielec_const
        )
        self._wire_params["vert_dielec_inside_mat"] += (
            alpha * vert_dielec_const
        )
        self._wire_params["ild_inside_mat"] += alpha * ild_thickness

        # Global (outside_mat) wire, technology.cc [ic][2], 180nm. Previously a
        # single init_wire_global() shared unconditionally across every node --
        # in fact an exact copy of this node's own row (aspect_ratio/ild_thickness/
        # horiz_dielec_const below), applied even when the target node wasn't 180
        # (the native reference Finding 1). aspect_ratio/ild_thickness/
        # horiz_dielec_const are the only three fields that actually vary by node
        # in technology.cc's global-wire table -- pitch (8F), vert_dielec (3.9),
        # fringe_cap are node-independent.
        # R: technology.cc never resets barrier_thickness/dishing_thickness/
        # alpha_scatter/resistivity between the local(index0)/semi-global(index1)/
        # global(index2) blocks of a single node -- they're plain C++ locals
        # carried forward, with only dishing_thickness reassigned to
        # 0.1*wire_thickness for the conservative (ic_type==1) global row. A
        # prior "simplified to 0/0/CU_RESISTIVITY" placeholder here was wrong --
        # found and fixed in Phase 29 via a live g_tp.wire_outside_mat probe
        # (R_per_um diverged from this port's R_per_micron_global by ~18%,
        # first caught by Mat::compute_bitline_delay's R_wire_predec_blk_out).
        #
        # Phase 31: that Phase 29 fix only got the ic_type==1 (conservative)
        # row right. For ic_type==0 (aggressive), technology.cc's global row
        # ALSO carries forward barrier_thickness unreset from the local row
        # above (0.017 at this node, both ic_type==0 and ==1) -- the true
        # real-CACTI behavior is "barrier never resets regardless of
        # ic_type", not "barrier resets to 0 for aggressive". Confirmed via
        # a live DELAY_BITLINE_PROBE run : this port's
        # R_wire_predec_blk_out was 25.955555555555556 vs real CACTI's
        # 27.537524117131586 for a real 180nm/HP/ic_type==0 (Alpha21364)
        # icache row-predecoder -- ratio 1.06095, matching
        # (1.584*0.72)/((1.584-0.017)*(0.72-2*0.017)) almost exactly (the
        # wire_resistance() denominator's barrier-thickness sensitivity),
        # not explainable by any other term (C_wire_predec_blk_out, all L1/
        # L2 gate widths, and drv1/drv2's own delays were separately
        # confirmed bit-exact via the same probe).
        if self._ic_type == 0:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.2
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 1.5
            miller_val = 1.5
            horiz_dielec_const = 2.709
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.017
            gl_dishing = 0
            gl_alpha_scatter = 1
        elif self._ic_type == 1:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.2
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 1.98
            miller_val = 1.5
            horiz_dielec_const = 3.038
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.017
            gl_dishing = 0.1 * wire_thickness
            gl_alpha_scatter = 1
        wire_c_gl = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_gl = self.wire_resistance(
            gl_resistivity,
            wire_width,
            wire_thickness,
            gl_barrier,
            gl_dishing,
            gl_alpha_scatter,
        )
        self._wire_params["C_per_micron_global"] += alpha * wire_c_gl
        self._wire_params["R_per_micron_global"] += alpha * wire_r_gl
        self._wire_params["pitch_global"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_global"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_global"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_global"] += alpha * vert_dielec_const
        self._wire_params["ild_global"] += alpha * ild_thickness
        self._wire_params["fringe_global"] += alpha * fringe_cap

    def init_wire_90nm(self, alpha):
        if self._ic_type == 0:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.4
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.01
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.48
            miller_val = 1.5
            horiz_dielec_const = 2.709
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.008
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.48
            miller_val = 1.5
            horiz_dielec_const = 3.038
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_per_micron = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_per_micron = self.wire_resistance(
            CU_RESISTIVITY,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron"] += alpha * wire_c_per_micron
        self._wire_params["R_per_micron"] += alpha * wire_r_per_micron
        self._wire_params["pitch"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_local"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_local"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_local"] += alpha * vert_dielec_const
        self._wire_params["ild_local"] += alpha * ild_thickness
        self._wire_params["fringe_local"] += alpha * fringe_cap

        # Semi-global (inside_mat) wire, technology.cc [ic][1], 90nm. Was entirely
        # missing (the native reference Finding 2, same gap as 180nm/22nm above/
        # below and the 65nm gap Phase 13 fixed).
        if self._ic_type == 0:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.4
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.01
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.48
            miller_val = 1.5
            horiz_dielec_const = 2.709
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.008
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.48
            miller_val = 1.5
            horiz_dielec_const = 3.038
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_sg = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_sg = self.wire_resistance(
            resistivity,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron_inside_mat"] += alpha * wire_c_sg
        self._wire_params["R_per_micron_inside_mat"] += alpha * wire_r_sg
        self._wire_params["pitch_inside_mat"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_inside_mat"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_inside_mat"] += (
            alpha * horiz_dielec_const
        )
        self._wire_params["vert_dielec_inside_mat"] += (
            alpha * vert_dielec_const
        )
        self._wire_params["ild_inside_mat"] += alpha * ild_thickness

        # Global (outside_mat) wire, technology.cc [ic][2], 90nm (PROGRESS.md
        # Phase 14 Finding 1 -- see the 180nm block above for the full explanation;
        # Phase 29 fixed the R formula the same way as that block). Phase 31:
        # ic_type==0's gl_barrier carries forward this node's local
        # barrier_thickness (0.01), same fix as the 180nm block above --
        # this node's own ic_type==0 barrier_thickness is genuinely nonzero
        # (unlike 65/45/32/22nm, where it's 0 and gl_barrier=0 is correct).
        if self._ic_type == 0:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.7
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.96
            miller_val = 1.5
            horiz_dielec_const = 2.709
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.01
            gl_dishing = 0
            gl_alpha_scatter = 1
        elif self._ic_type == 1:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.2
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 1.1
            miller_val = 1.5
            horiz_dielec_const = 3.038
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.008
            gl_dishing = 0.1 * wire_thickness
            gl_alpha_scatter = 1
        wire_c_gl = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_gl = self.wire_resistance(
            gl_resistivity,
            wire_width,
            wire_thickness,
            gl_barrier,
            gl_dishing,
            gl_alpha_scatter,
        )
        self._wire_params["C_per_micron_global"] += alpha * wire_c_gl
        self._wire_params["R_per_micron_global"] += alpha * wire_r_gl
        self._wire_params["pitch_global"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_global"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_global"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_global"] += alpha * vert_dielec_const
        self._wire_params["ild_global"] += alpha * ild_thickness
        self._wire_params["fringe_global"] += alpha * fringe_cap

    def init_wire_65nm(self, alpha):
        if self._ic_type == 0:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.7
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.405
            miller_val = 1.5
            horiz_dielec_const = 2.303
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.006
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.405
            miller_val = 1.5
            horiz_dielec_const = 2.734
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_per_micron = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        # technology.cc:2301 uses BULK_CU_RESISTIVITY for the aggressive
        # (ic_type==0) LOCAL wire wire_r_per_micron[0][0] at tech==65 (and
        # 45/32/22); CU_RESISTIVITY only for conservative ([1][0],
        # technology.cc:2352). 180/90nm use CU for both.
        wire_r_per_micron = self.wire_resistance(
            BULK_CU_RESISTIVITY if self._ic_type == 0 else CU_RESISTIVITY,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron"] += alpha * wire_c_per_micron
        self._wire_params["R_per_micron"] += alpha * wire_r_per_micron
        self._wire_params["pitch"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_local"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_local"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_local"] += alpha * vert_dielec_const
        self._wire_params["ild_local"] += alpha * ild_thickness
        self._wire_params["fringe_local"] += alpha * fringe_cap

        # Semi-global (inside_mat) wire, technology.cc [ic][1], 65nm -- this
        # block was entirely missing : every node's init_wire_XXnm
        # here is supposed to populate BOTH the local (index 0) and
        # semi-global (index 1) wire tables, mirroring technology.cc's own
        # per-node block, but only init_wire_45nm/32nm ever did -- invisible
        # for every anchor before Phase 13 because they either used a
        # 40nm-interpolated node (blended from 45nm+32nm, both of which DO
        # set this) or never exercised wire_is_mat_type/wire_os_mat_type==1
        # at a node other than 40nm. First config to hit the gap (L2 at
        # 65nm/HP) crashed outright (ZeroDivisionError in _wire_model, pitch
        # of exactly 0), not a silent wrong-number bug.
        if self._ic_type == 0:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.7
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = BULK_CU_RESISTIVITY
            ild_thickness = 0.405
            miller_val = 1.5
            horiz_dielec_const = 2.303
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.006
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.405
            miller_val = 1.5
            horiz_dielec_const = 2.734
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_sg = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_sg = self.wire_resistance(
            resistivity,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron_inside_mat"] += alpha * wire_c_sg
        self._wire_params["R_per_micron_inside_mat"] += alpha * wire_r_sg
        self._wire_params["pitch_inside_mat"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_inside_mat"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_inside_mat"] += (
            alpha * horiz_dielec_const
        )
        self._wire_params["vert_dielec_inside_mat"] += (
            alpha * vert_dielec_const
        )
        self._wire_params["ild_inside_mat"] += alpha * ild_thickness

        # Global (outside_mat) wire, technology.cc [ic][2], 65nm (PROGRESS.md
        # Phase 14 Finding 1 -- see the 180nm block above for the full explanation;
        # Phase 29 fixed the R formula the same way as that block).
        if self._ic_type == 0:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.8
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.81
            miller_val = 1.5
            horiz_dielec_const = 2.303
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = BULK_CU_RESISTIVITY
            gl_barrier = 0
            gl_dishing = 0
            gl_alpha_scatter = 1
        elif self._ic_type == 1:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.2
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.77
            miller_val = 1.5
            horiz_dielec_const = 2.734
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.006
            gl_dishing = 0.1 * wire_thickness
            gl_alpha_scatter = 1
        wire_c_gl = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_gl = self.wire_resistance(
            gl_resistivity,
            wire_width,
            wire_thickness,
            gl_barrier,
            gl_dishing,
            gl_alpha_scatter,
        )
        self._wire_params["C_per_micron_global"] += alpha * wire_c_gl
        self._wire_params["R_per_micron_global"] += alpha * wire_r_gl
        self._wire_params["pitch_global"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_global"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_global"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_global"] += alpha * vert_dielec_const
        self._wire_params["ild_global"] += alpha * ild_thickness
        self._wire_params["fringe_global"] += alpha * fringe_cap

    def init_wire_45nm(self, alpha):
        if self._ic_type == 0:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.315
            miller_val = 1.5
            horiz_dielec_const = 1.958
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.004
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.315
            miller_val = 1.5
            horiz_dielec_const = 2.46
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_per_micron = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        # technology.cc:2409 uses BULK_CU_RESISTIVITY for the aggressive
        # (ic_type==0) LOCAL wire wire_r_per_micron[0][0] at tech==45;
        # CU_RESISTIVITY only for conservative ([1][0], technology.cc:2459).
        wire_r_per_micron = self.wire_resistance(
            BULK_CU_RESISTIVITY if self._ic_type == 0 else CU_RESISTIVITY,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron"] += alpha * wire_c_per_micron
        self._wire_params["R_per_micron"] += alpha * wire_r_per_micron
        self._wire_params["pitch"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_local"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_local"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_local"] += alpha * vert_dielec_const
        self._wire_params["ild_local"] += alpha * ild_thickness
        self._wire_params["fringe_local"] += alpha * fringe_cap

        # Semi-global (inside_mat) wire, technology.cc [ic][1], 45nm.
        if self._ic_type == 0:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = BULK_CU_RESISTIVITY
            ild_thickness = 0.315
            miller_val = 1.5
            horiz_dielec_const = 1.958
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.004
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.315
            miller_val = 1.5
            horiz_dielec_const = 2.46
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_sg = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_sg = self.wire_resistance(
            resistivity,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron_inside_mat"] += alpha * wire_c_sg
        self._wire_params["R_per_micron_inside_mat"] += alpha * wire_r_sg
        self._wire_params["pitch_inside_mat"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_inside_mat"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_inside_mat"] += (
            alpha * horiz_dielec_const
        )
        self._wire_params["vert_dielec_inside_mat"] += (
            alpha * vert_dielec_const
        )
        self._wire_params["ild_inside_mat"] += alpha * ild_thickness

        # Global (outside_mat) wire, technology.cc [ic][2], 45nm (PROGRESS.md
        # Phase 14 Finding 1 -- see the 180nm block above for the full explanation;
        # Phase 29 fixed the R formula the same way as that block).
        if self._ic_type == 0:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.63
            miller_val = 1.5
            horiz_dielec_const = 1.958
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = BULK_CU_RESISTIVITY
            gl_barrier = 0
            gl_dishing = 0
            gl_alpha_scatter = 1
        elif self._ic_type == 1:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.2
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.55
            miller_val = 1.5
            horiz_dielec_const = 2.46
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.004
            gl_dishing = 0.1 * wire_thickness
            gl_alpha_scatter = 1
        wire_c_gl = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_gl = self.wire_resistance(
            gl_resistivity,
            wire_width,
            wire_thickness,
            gl_barrier,
            gl_dishing,
            gl_alpha_scatter,
        )
        self._wire_params["C_per_micron_global"] += alpha * wire_c_gl
        self._wire_params["R_per_micron_global"] += alpha * wire_r_gl
        self._wire_params["pitch_global"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_global"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_global"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_global"] += alpha * vert_dielec_const
        self._wire_params["ild_global"] += alpha * ild_thickness
        self._wire_params["fringe_global"] += alpha * fringe_cap

    def init_wire_32nm(self, alpha):
        if self._ic_type == 0:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.21
            miller_val = 1.5
            horiz_dielec_const = 1.664
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.003
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.21
            miller_val = 1.5
            horiz_dielec_const = 2.214
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_per_micron = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        # technology.cc:2517 uses BULK_CU_RESISTIVITY for the aggressive
        # (ic_type==0) LOCAL wire wire_r_per_micron[0][0] at tech==32;
        # CU_RESISTIVITY only for conservative ([1][0], technology.cc:2567).
        wire_r_per_micron = self.wire_resistance(
            BULK_CU_RESISTIVITY if self._ic_type == 0 else CU_RESISTIVITY,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron"] += alpha * wire_c_per_micron
        self._wire_params["R_per_micron"] += alpha * wire_r_per_micron
        self._wire_params["pitch"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_local"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_local"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_local"] += alpha * vert_dielec_const
        self._wire_params["ild_local"] += alpha * ild_thickness
        self._wire_params["fringe_local"] += alpha * fringe_cap

        # Semi-global (inside_mat) wire, technology.cc [ic][1], 32nm.
        if self._ic_type == 0:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = BULK_CU_RESISTIVITY
            ild_thickness = 0.21
            miller_val = 1.5
            horiz_dielec_const = 1.664
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.003
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.21
            miller_val = 1.5
            horiz_dielec_const = 2.214
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_sg = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_sg = self.wire_resistance(
            resistivity,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron_inside_mat"] += alpha * wire_c_sg
        self._wire_params["R_per_micron_inside_mat"] += alpha * wire_r_sg
        self._wire_params["pitch_inside_mat"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_inside_mat"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_inside_mat"] += (
            alpha * horiz_dielec_const
        )
        self._wire_params["vert_dielec_inside_mat"] += (
            alpha * vert_dielec_const
        )
        self._wire_params["ild_inside_mat"] += alpha * ild_thickness

        # Global (outside_mat) wire, technology.cc [ic][2], 32nm (PROGRESS.md
        # Phase 14 Finding 1 -- see the 180nm block above for the full explanation;
        # Phase 29 fixed the R formula the same way as that block).
        if self._ic_type == 0:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.42
            miller_val = 1.5
            horiz_dielec_const = 1.664
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = BULK_CU_RESISTIVITY
            gl_barrier = 0
            gl_dishing = 0
            gl_alpha_scatter = 1
        elif self._ic_type == 1:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.2
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.385
            miller_val = 1.5
            horiz_dielec_const = 2.214
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.003
            gl_dishing = 0.1 * wire_thickness
            gl_alpha_scatter = 1
        wire_c_gl = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_gl = self.wire_resistance(
            gl_resistivity,
            wire_width,
            wire_thickness,
            gl_barrier,
            gl_dishing,
            gl_alpha_scatter,
        )
        self._wire_params["C_per_micron_global"] += alpha * wire_c_gl
        self._wire_params["R_per_micron_global"] += alpha * wire_r_gl
        self._wire_params["pitch_global"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_global"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_global"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_global"] += alpha * vert_dielec_const
        self._wire_params["ild_global"] += alpha * ild_thickness
        self._wire_params["fringe_global"] += alpha * fringe_cap

    def init_wire_22nm(self, alpha):
        if self._ic_type == 0:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            ild_thickness = 0.15
            miller_val = 1.5
            horiz_dielec_const = 1.414
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 2.5 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.003
            dishing_thickness = 0
            alpha_scatter = 1.05
            ild_thickness = 0.15
            miller_val = 1.5
            horiz_dielec_const = 2.104
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_per_micron = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        # technology.cc:2624 uses BULK_CU_RESISTIVITY for the aggressive
        # (ic_type==0) LOCAL wire wire_r_per_micron[0][0] at tech==22;
        # CU_RESISTIVITY only for conservative ([1][0], technology.cc:2712).
        wire_r_per_micron = self.wire_resistance(
            BULK_CU_RESISTIVITY if self._ic_type == 0 else CU_RESISTIVITY,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron"] += alpha * wire_c_per_micron
        self._wire_params["R_per_micron"] += alpha * wire_r_per_micron
        self._wire_params["pitch"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_local"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_local"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_local"] += alpha * vert_dielec_const
        self._wire_params["ild_local"] += alpha * ild_thickness
        self._wire_params["fringe_local"] += alpha * fringe_cap

        # Semi-global (inside_mat) wire, technology.cc [ic][1], 22nm. Was entirely
        # missing (the native reference Finding 2, same gap as 180nm/90nm above and
        # the 65nm gap Phase 13 fixed).
        if self._ic_type == 0:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0
            dishing_thickness = 0
            alpha_scatter = 1
            resistivity = BULK_CU_RESISTIVITY
            ild_thickness = 0.15
            miller_val = 1.5
            horiz_dielec_const = 1.414
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        elif self._ic_type == 1:
            wire_pitch = 4 * self._node_um
            aspect_ratio = 2.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            barrier_thickness = 0.003
            dishing_thickness = 0
            alpha_scatter = 1.05
            resistivity = CU_RESISTIVITY
            ild_thickness = 0.15
            miller_val = 1.5
            horiz_dielec_const = 2.104
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
        wire_c_sg = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_sg = self.wire_resistance(
            resistivity,
            wire_width,
            wire_thickness,
            barrier_thickness,
            dishing_thickness,
            alpha_scatter,
        )
        self._wire_params["C_per_micron_inside_mat"] += alpha * wire_c_sg
        self._wire_params["R_per_micron_inside_mat"] += alpha * wire_r_sg
        self._wire_params["pitch_inside_mat"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_inside_mat"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_inside_mat"] += (
            alpha * horiz_dielec_const
        )
        self._wire_params["vert_dielec_inside_mat"] += (
            alpha * vert_dielec_const
        )
        self._wire_params["ild_inside_mat"] += alpha * ild_thickness

        # Global (outside_mat) wire, technology.cc [ic][2], 22nm (PROGRESS.md
        # Phase 14 Finding 1 -- see the 180nm block above for the full explanation;
        # Phase 29 fixed the R formula the same way as that block).
        if self._ic_type == 0:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 3.0
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.3
            miller_val = 1.5
            horiz_dielec_const = 1.414
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = BULK_CU_RESISTIVITY
            gl_barrier = 0
            gl_dishing = 0
            gl_alpha_scatter = 1
        elif self._ic_type == 1:
            wire_pitch = 8 * self._node_um
            aspect_ratio = 2.2
            wire_width = wire_pitch / 2
            wire_thickness = aspect_ratio * wire_width
            wire_spacing = wire_pitch - wire_width
            ild_thickness = 0.275
            miller_val = 1.5
            horiz_dielec_const = 2.104
            vert_dielec_const = 3.9
            fringe_cap = 0.115e-15
            gl_resistivity = CU_RESISTIVITY
            gl_barrier = 0.003
            gl_dishing = 0.1 * wire_thickness
            gl_alpha_scatter = 1.05
        wire_c_gl = self.wire_capacitance(
            wire_width,
            wire_thickness,
            wire_spacing,
            ild_thickness,
            miller_val,
            horiz_dielec_const,
            vert_dielec_const,
            fringe_cap,
        )
        wire_r_gl = self.wire_resistance(
            gl_resistivity,
            wire_width,
            wire_thickness,
            gl_barrier,
            gl_dishing,
            gl_alpha_scatter,
        )
        self._wire_params["C_per_micron_global"] += alpha * wire_c_gl
        self._wire_params["R_per_micron_global"] += alpha * wire_r_gl
        self._wire_params["pitch_global"] += alpha * wire_pitch
        self._wire_params["aspect_ratio_global"] += alpha * aspect_ratio
        self._wire_params["horiz_dielec_global"] += alpha * horiz_dielec_const
        self._wire_params["vert_dielec_global"] += alpha * vert_dielec_const
        self._wire_params["ild_global"] += alpha * ild_thickness
        self._wire_params["fringe_global"] += alpha * fringe_cap
