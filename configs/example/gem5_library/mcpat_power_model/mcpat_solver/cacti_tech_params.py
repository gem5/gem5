from math import sqrt

from .cacti_base_params import CactiBaseParams
from .config_input_params import McPATMachineConfig

""" Constants used for the tech parameters. """

"""
For all technology nodes, these ratios don't change. They're primarily
used to calculate dynamic/static energies for caches and similar
structures in CACTI. DRAM is the only one that does change.
"""
ASP_RATIO_CAM = 2.92
AREA_CELL_CAM = 292
ASP_RATIO_SRAM = 1.46
AREA_CELL_SRAM = 146


class CactiTechParams(CactiBaseParams):
    def __init__(self, machine: McPATMachineConfig):
        CactiBaseParams.__init__(self, machine._config_params["tech_node"])
        self._mode = self.devtype_to_mode(
            machine._config_params["device_type"]
        )
        self._temp = machine._config_params["temperature"]
        # technology.cc:1987's abort is gated on !g_ip->is_main_mem. The flag
        # is OPTIONAL in the machine-config contract (it is not in
        # McPATMachineConfig._defined_config_params, so existing call sites
        # never pass it and must not be forced to) -- read it via a
        # presence check, defaulting to False, not via .get().
        self._is_main_mem = (
            bool(machine._config_params["is_main_mem"])
            if "is_main_mem" in machine._config_params
            else False
        )
        # User Vdd override (McPAT core.cc:4483-4490, sharedcache.cc:1124-1136,
        # noc.cc:421-427, ...): when a component's XML `vdd` param is > 0,
        # McPAT sets interface_ip.specific_{hp,lop,lstp}_vdd = true and
        # {hp,lop,lstp}_Vdd = vdd (all three at once), and technology.cc
        # then uses it as vdd_real[] for every device flavor at every
        # interpolation node (vdd_real[i] = g_ip->specific_*_vdd ?
        # g_ip->*_Vdd : vdd[i]). vdd_real feeds Vdd (peri_global/sram_cell/
        # cam_cell/Vbitpre), I_on_n (alpha-power law), R_nch_on/R_pch_on
        # (except at 180nm, which uses the ITRS vdd -- technology.cc:257)
        # and I_off_n's (vdd_real/vdd)^k factor (absent at 180nm); Vdd_default
        # stays ITRS. NOT uniform in Vdd: at 130nm (90<->180 interpolated)
        # 1.3V vs ITRS 1.333V gives I_off_n x1.371 (the 90nm term is scaled
        # by (1.3/1.2)^4 while 180nm has no factor) and R_nch_on x1.180 --
        # real CACTI behavior, replicated. OPTIONAL config key (read via a
        # presence check, like is_main_mem); vdd <= 0 (McPAT's XML default
        # 0) means "no override".
        self._specific_vdd = (
            "vdd" in machine._config_params
            and machine._config_params["vdd"] > 0
        )
        self._user_vdd = (
            float(machine._config_params["vdd"])
            if self._specific_vdd
            else None
        )
        self._dram_type = 0  # TODO: Fix to use machine params
        self._tech_params = {
            "Vdd": 0,
            "Vdd_default": 0,
            # technology.cc:1764-1767, 1817-1820 -- the
            # peri_global / sram_cell Vdd/Vdd_default split
            # and their running-sum Vcc_min_default, kept
            # separate from the single 'Vdd'/'Vdd_default'
            # pair above (which the energy pipeline reads).
            # Only consumed by init_tech_params()'s
            # "User defined Vdd is too low" abort check.
            "peri_global_Vdd": 0,
            "peri_global_Vdd_default": 0,
            "peri_global_Vcc_min_default": 0,
            "sram_cell_Vdd": 0,
            "sram_cell_Vdd_default": 0,
            "sram_cell_Vcc_min_default": 0,
            "Vth": 0,
            # Sense-amp transconductance inputs (technology.cc:
            # l_elec/mobility_eff_periph_global/Vdsat_periph_global/
            # gmp_to_gmn_multiplier_periph_global -- feed
            # gm_sense_amp_latch, computed on the fly in
            # CactiMat.compute_sa_delay, Phase 29).
            "l_elec": 0,
            "mobility_eff": 0,
            "Vdsat": 0,
            "gmp_to_gmn_mult": 0,
            "t_ox": 0,
            "c_ox": 0,
            "c_g_ideal": 0,
            "c_fringe": 0,
            "c_junc": 0,
            "c_junc_sidewall": 0.25e-15,
            "c_overlap": 0,
            "hpowerrail": 2 * self._node_um,
            "min_gap_p_to_n_diff": 5 * self._node_um,
            # MIN_GAP_BET_SAME_TYPE_DIFFS (technology.cc:1930).
            "min_gap_bet_same_type_diffs": 1.5 * self._node_um,
            "w_poly_contact": self._node_um,
            "spacing_poly_to_contact": self._node_um,
            "spacing_poly_to_poly": 1.5 * self._node_um,
            "cell_h_def": 50 * self._node_um,
            "min_w_nmos": 3 * self._node_um / 2,
            "max_w_nmos": 100 * self._node_um,
            "max_w_nmos_dec": 100 * self._node_um,
            "w_nmos_b_mux": 6 * (3 * self._node_um / 2),
            "w_nmos_sa_mux": 6 * (3 * self._node_um / 2),
            # Sense-amp / bitline isolation transistor sizes
            # (technology.cc: multiples of the feature size).
            "w_iso": 12.5 * self._node_um,
            "w_sense_n": 3.75 * self._node_um,
            "w_sense_p": 7.5 * self._node_um,
            "w_sense_en": 5 * self._node_um,
            # SRAM cell transistor widths (technology.cc:
            # 1.31/2.08/1.23 * F for access/pull-down/pull-up).
            "sram_cell_nmos_w": 2.08 * self._node_um,
            "sram_cell_pmos_w": 1.23 * self._node_um,
            # CAM cell transistor widths (technology.cc:1897-1899,
            # curr_Wmemcella/pmos/nmos_cam -- numerically identical
            # to the SRAM cell constants at every node, but a
            # distinct named param per CACTI's own g_tp.cam struct).
            "cam_cell_nmos_w": 2.08 * self._node_um,
            "cam_cell_pmos_w": 1.23 * self._node_um,
            # FA/CAM search (matchline/comparator) chain
            # transistor widths (mat.cc:compute_cam_delay,
            # technology.cc-style multiples of F).
            "w_dummy_inv_n": 75 * self._node_um,
            "w_dummy_inv_p": 100 * self._node_um,
            "w_addr_nand_n": 62.5 * self._node_um,
            "w_addr_nand_p": 62.5 * self._node_um,
            "w_fa_nor_n": 6.25 * self._node_um,
            "w_fa_nor_p": 12.5 * self._node_um,
            "R_nch_on": 0,
            "R_pch_on": 0,
            "n_to_p_eff_curr_drv_ratio": 0,
            "I_off_n": 0,
            "I_g_on_n": 0,
            "I_off_p": 0,
            "I_g_on_p": 0,
            "area_cell_dram": 0,
            "area_cell_sram": 0,
            "area_cell_cam": 0,
            "asp_ratio_cell_dram": 0,
            "asp_ratio_cell_sram": 0,
            "asp_ratio_cell_cam": 0,
            "cam_b_w": 0,
            "cam_b_h": 0,
            "cam_cell_a_w": 0,
            "sram_b_w": 0,
            "sram_b_h": 0,
            "sram_cell_a_w": 0,
            "dram_b_w": 0,
            "dram_b_h": 0,
            "dram_cell_a_w": 0,
            "sense_dy_power": 0,
            "sckt_coeff": 0,
            # Layout overhead multipliers on dynamic energy only
            # (technology.cc: node-only, not per-device-type;
            # array.cc's pppm_t leaves leakage un-scaled).
            "macro_layout_overhead": 0,
            "chip_layout_overhead": 0,
            # Undifferentiated-logic scaling factor and core
            # transistor density (technology.cc: ScalingFactor,
            # node-only, not per-device-type -- used by
            # FunctionalUnit's area_t/leakage formulas).
            "logic_scaling_co_eff": 0,
            "core_tx_density": 0,
            # Peripheral-device long-channel leakage reduction
            # factor (technology.cc's long_channel_leakage_reduction[
            # peri_global_tech_type], node- AND device-type-dependent).
            "long_channel_leakage_reduction": 0,
            "h_dec": 4,
            "h_dram": 8,
            # Wordline stitching overhead width added to the
            # subarray every sram_num_cells_wl_stitching_ (16)
            # columns (technology.cc: 7.5 * F).
            "ram_wl_stitching_overhead": 7.5 * self._node_um,
            # Tag-array comparator transistor widths (technology.cc:
            # 1917-1927, all multiples of the feature size).
            "w_comp_inv_p1": 12.5 * self._node_um,
            "w_comp_inv_n1": 7.5 * self._node_um,
            "w_comp_inv_p2": 25 * self._node_um,
            "w_comp_inv_n2": 15 * self._node_um,
            "w_comp_inv_p3": 50 * self._node_um,
            "w_comp_inv_n3": 30 * self._node_um,
            "w_eval_inv_p": 100 * self._node_um,
            "w_eval_inv_n": 50 * self._node_um,
            "w_comp_n": 12.5 * self._node_um,
            "w_comp_p": 37.5 * self._node_um,
        }

    def devtype_to_mode(self, mode):
        if mode == 0:
            return "HP"
        elif mode == 1:
            return "LSTP"
        elif mode == 2:
            return "LOP"
        else:
            return None

    def validate_params(self):
        super().validate_params()
        if self._mode not in ["LOP", "HP", "LSTP"]:
            raise ValueError(f"Mode {self._node}nm is not supported!\
                      Only Low Power (LOP), High Power (HP), and Low\
                    Standby Power (LSTP) are supported by McPAT/CACTI!")

    def init_tech_params(self):
        # technology.cc's `tech == 90` / `tech == 180` blocks populate only
        # the HP (index 0) entries of vdd/vdd_real/v_th/... -- ITRS ships no
        # LSTP/LOP device data from 90nm up to 180nm. Real CACTI reads
        # uninitialised stack for a non-HP device type in that range and its
        # output is undefined (measured: no valid power table). Refuse here
        # rather than propagate garbage (the port would otherwise hit an
        # UnboundLocalError on `vdd` inside init_tech_180nm).
        # The explicit upstream guard is io.cc:1365 (Ip::error_checking):
        # real CACTI aborts when F_sz_um > 0.091 for a non-HP device -- the
        # `>= 91` (>= 91nm) boundary below mirrors that.
        if self._node >= 91 and self._mode != "HP":
            raise ValueError(
                f"Feature size {self._node}nm with {self._mode}: real "
                "CACTI/McPAT only has ITRS HP device data from 90nm to "
                "180nm (technology.cc's tech==90 / tech==180 blocks "
                "populate index 0 / HP only). Use device_type=HP above 90nm."
            )
        interp_params = self.get_alpha()
        for alpha, tech in zip(interp_params[0::2], interp_params[1::2]):
            if tech == 180:
                self.init_tech_180nm(alpha)
            if tech == 90:
                self.init_tech_90nm(alpha)
            if tech == 65:
                self.init_tech_65nm(alpha)
            if tech == 45:
                self.init_tech_45nm(alpha)
            if tech == 32:
                self.init_tech_32nm(alpha)
            if tech == 22:
                self.init_tech_22nm(alpha)
        self._tech_params["cam_b_w"] = sqrt(
            self._tech_params["area_cell_cam"]
            / self._tech_params["asp_ratio_cell_cam"]
        )
        self._tech_params["cam_b_h"] = (
            self._tech_params["asp_ratio_cell_cam"]
            * self._tech_params["cam_b_w"]
        )
        self._tech_params["sram_b_w"] = sqrt(
            self._tech_params["area_cell_sram"]
            / self._tech_params["asp_ratio_cell_sram"]
        )
        self._tech_params["sram_b_h"] = (
            self._tech_params["asp_ratio_cell_sram"]
            * self._tech_params["sram_b_w"]
        )

        # technology.cc:1958-1960 -- C_overlap is assigned ONCE, AFTER the
        # interpolation loop closes, as 0.2 * the fully-accumulated
        # (interpolated) C_g_ideal -- it is NOT an interpolated quantity
        # itself and must NOT be accumulated per node-iteration. The old
        # per-init_tech_XXnm() lines did so inconsistently (90nm/65nm used
        # +=), making c_overlap up to 1.5x too large at an interpolated
        # node in the 66-179nm span (exact-node results were unaffected,
        # since a single alpha==1 iteration collapses to the same value).
        # peri_global == ram_cell tech type in every McPAT SRAM path, so
        # this single key covers technology.cc's peri_global / sram_cell /
        # cam_cell C_overlap (all three: 0.2 * the same C_g_ideal).
        self._tech_params["c_overlap"] = 0.2 * self._tech_params["c_g_ideal"]

        # technology.cc:1987 -- "User defined Vdd is too low". Gated on
        # !g_ip->is_main_mem. With no user Vdd override vdd_real == vdd, so
        # terms 2 and 3 (X < 0.75*X) are always false and only the
        # Vcc_min_default term can actually fire; under a user Vdd override
        # (self._specific_vdd) all three terms are live. At an exact
        # supported node the interpolation loop runs one iteration with
        # alpha == 1, giving Vcc_min_default == 0.6*Vdd -> never aborts
        # (absent an override); at an interpolated node the running sum
        # makes it
        # data-dependent, and real CACTI exits(0) here for e.g. 30/43/62 nm.
        tp = self._tech_params
        if (
            tp["sram_cell_Vcc_min_default"] > tp["sram_cell_Vdd"]
            or tp["peri_global_Vdd"] < tp["peri_global_Vdd_default"] * 0.75
            or tp["sram_cell_Vdd"] < tp["sram_cell_Vdd_default"] * 0.75
        ) and not self._is_main_mem:
            raise ValueError(
                f"User defined Vdd is too low (node {self._node}nm, "
                f"{self._mode}); real CACTI/McPAT aborts here "
                "(technology.cc:1987). This interpolated node is not "
                "evaluable by the reference tool."
            )

        # technology.cc:2024 -- "User defined power-saving supply voltage
        # cannot be lower than Vdd (DVS0)". No user power_gating_vcc
        # (specific_vcc_min) is modelled, so Vcc_min == Vcc_min_default
        # (technology.cc:2010-2011); the sram_cell term is already covered
        # by the check above, the peri_global term can fire under a user
        # Vdd override.
        if (
            tp["sram_cell_Vcc_min_default"] > tp["sram_cell_Vdd"]
            or tp["peri_global_Vcc_min_default"] > tp["peri_global_Vdd"]
        ) and not self._is_main_mem:
            raise ValueError(
                f"User defined power-saving supply voltage cannot be lower "
                f"than Vdd (DVS0) (node {self._node}nm, {self._mode}); real "
                "CACTI/McPAT aborts here (technology.cc:2024)."
            )

    def init_tech_180nm(self, alpha):
        sckt_coeff = 1.11
        macro_layout_overhead = 1.0
        chip_layout_overhead = 1.0
        logic_scaling_co_eff = 1.5
        core_tx_density = 1.25 * 0.7 * 0.7 * 0.4
        SENSE_AMP_D = 0.28e-9
        SENSE_AMP_P = 14.7e-15
        I_off_choices = {
            "HP": [
                7e-10,
                8.26e-10,
                9.74e-10,
                1.15e-9,
                1.35e-9,
                1.6e-9,
                1.88e-9,
                2.29e-9,
                2.7e-9,
                3.19e-9,
                3.76e-9,
            ]
        }
        if self._mode == "HP":
            vdd = 1.5
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.4
            l_phy = 0.12
            l_elec = 0.10
            t_ox = 1.2e-3 * 2
            v_th = 0.4407
            c_ox = 1.79e-14 * 2
            mobility_eff = 302.16 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 0.128 * 2
            c_g_ideal = 6.64e-16 * 2
            c_fringe = 0.08e-15 * 2
            c_junc = 1e-15 * 2
            i_on_n = (
                750e-6 * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = 350e-6
            nmos_eff_resistance_mult = 1.54
            n_to_p_eff_curr_drv_ratio = 2.45
            gmp_to_gmn_mult = 1.22
            # technology.cc:257 -- 180nm (only) uses the ITRS vdd[0] here,
            # not vdd_real[0]; differs from vdd_real only under a user Vdd
            # override.
            rn_channel_on = nmos_eff_resistance_mult * vdd / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1
            I_off_n = I_off_choices[self._mode][(self._temp - 300) // 10]
            I_g_on_n = 1.65e-10

        self._tech_params["Vdd"] += alpha * (vdd_real)
        self._tech_params["Vdd_default"] += alpha * (vdd)
        # technology.cc:1764-1766, 1817-1820 -- peri_global == ram_cell tech
        # type in every McPAT SRAM path, so both use this function's
        # vdd/vdd_real. Vcc_min_default is a RUNNING SUM over the
        # interpolation iterations (uses the partially-accumulated
        # Vdd_default), replicated exactly.
        self._tech_params["peri_global_Vdd"] += alpha * vdd_real
        self._tech_params["peri_global_Vdd_default"] += alpha * vdd
        self._tech_params["peri_global_Vcc_min_default"] += (
            self._tech_params["peri_global_Vdd_default"] * 0.45
        )
        self._tech_params["sram_cell_Vdd"] += alpha * vdd_real
        self._tech_params["sram_cell_Vdd_default"] += alpha * vdd
        self._tech_params["sram_cell_Vcc_min_default"] += (
            self._tech_params["sram_cell_Vdd_default"] * 0.6
        )
        self._tech_params["Vth"] += alpha * (v_th)
        self._tech_params["l_elec"] += alpha * (l_elec)
        self._tech_params["mobility_eff"] += alpha * (mobility_eff)
        self._tech_params["Vdsat"] += alpha * (vd_sat)
        self._tech_params["gmp_to_gmn_mult"] += alpha * (gmp_to_gmn_mult)
        self._tech_params["t_ox"] += alpha * (t_ox)
        self._tech_params["c_ox"] += alpha * (c_ox)
        self._tech_params["c_g_ideal"] += alpha * (c_g_ideal)
        self._tech_params["c_fringe"] += alpha * (c_fringe)
        self._tech_params["c_junc"] += alpha * (c_junc)
        self._tech_params["R_nch_on"] += alpha * (rn_channel_on)
        self._tech_params["R_pch_on"] += alpha * (rp_channel_on)
        self._tech_params["n_to_p_eff_curr_drv_ratio"] += alpha * (
            n_to_p_eff_curr_drv_ratio
        )
        self._tech_params["I_off_n"] += alpha * I_off_n
        self._tech_params["I_off_p"] += alpha * I_off_n
        self._tech_params["I_g_on_n"] += alpha * I_g_on_n
        self._tech_params["I_g_on_p"] += alpha * I_g_on_n
        self._tech_params["area_cell_sram"] += alpha * (
            AREA_CELL_SRAM * (self._node_um**2)
        )
        self._tech_params["area_cell_cam"] += alpha * (
            AREA_CELL_CAM * (self._node_um**2)
        )
        self._tech_params["asp_ratio_cell_sram"] += alpha * ASP_RATIO_SRAM
        self._tech_params["asp_ratio_cell_cam"] += alpha * ASP_RATIO_CAM
        self._tech_params["cam_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sram_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sense_dy_power"] += alpha * (SENSE_AMP_P)
        self._tech_params["sckt_coeff"] += alpha * sckt_coeff
        self._tech_params["macro_layout_overhead"] += (
            alpha * macro_layout_overhead
        )
        self._tech_params["chip_layout_overhead"] += (
            alpha * chip_layout_overhead
        )
        self._tech_params["long_channel_leakage_reduction"] += (
            alpha * long_channel_lkg_reduction
        )
        self._tech_params["logic_scaling_co_eff"] += (
            alpha * logic_scaling_co_eff
        )
        self._tech_params["core_tx_density"] += alpha * core_tx_density

    def init_tech_90nm(self, alpha):
        sckt_coeff = 1.1539
        macro_layout_overhead = 1.1
        chip_layout_overhead = 1.2
        logic_scaling_co_eff = 1
        core_tx_density = 1.25 * 0.7 * 0.7
        SENSE_AMP_D = 0.28e-9
        SENSE_AMP_P = 14.7e-15
        I_off_choices = {
            "HP": [
                3.24e-08,
                4.01e-08,
                4.9e-08,
                5.92e-08,
                7.08e-08,
                8.38e-08,
                9.82e-08,
                1.14e-07,
                1.29e-07,
                1.43e-07,
                1.54e-07,
            ],
            "LSTP": [
                2.81e-12,
                4.76e-12,
                7.82e-12,
                1.25e-11,
                1.94e-11,
                2.94e-11,
                4.36e-11,
                6.32e-11,
                8.95e-11,
                1.25e-10,
                1.7e-10,
            ],
            "LOP": [
                2.14e-09,
                2.9e-09,
                3.87e-09,
                5.07e-09,
                6.54e-09,
                8.27e-08,
                1.02e-07,
                1.2e-07,
                1.36e-08,
                1.52e-08,
                1.73e-08,
            ],
        }
        if self._mode == "HP":
            vdd = 1.2
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.34
            l_phy = 0.037
            l_elec = 0.0266
            t_ox = 1.2e-3
            v_th = 0.23707
            c_ox = 1.79e-14
            mobility_eff = 342.16 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 0.128
            c_g_ideal = 6.64e-16
            c_fringe = 0.08e-15
            c_junc = 1e-15
            i_on_n = (
                1076.9e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = 712.6e-6
            nmos_eff_resistance_mult = 1.54
            n_to_p_eff_curr_drv_ratio = 2.45
            gmp_to_gmn_mult = 1.22
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 4
            I_g_on_n = 1.65e-8
        elif self._mode == "LSTP":
            vdd = 1.3
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.47
            l_phy = 0.075
            l_elec = 0.0486
            t_ox = 2.2e-3
            v_th = 0.48203
            c_ox = 1.22e-14
            mobility_eff = 356.76 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 0.373
            c_g_ideal = 9.15e-16
            c_fringe = 0.08e-15
            c_junc = 1e-15
            i_on_n = (
                503.6e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = 235.1e-6
            nmos_eff_resistance_mult = 1.92
            n_to_p_eff_curr_drv_ratio = 2.44
            gmp_to_gmn_mult = 0.88
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 4
            I_g_on_n = 3.87e-11
        elif self._mode == "LOP":
            vdd = 0.9
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.55
            l_phy = 0.053
            l_elec = 0.0354
            t_ox = 1.5e-3
            v_th = 0.30764
            c_ox = 1.59e-14
            mobility_eff = 460.39 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 0.113
            c_g_ideal = 8.45e-16
            c_fringe = 0.08e-15
            c_junc = 1e-15
            i_on_n = (
                386.6e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = 209.7e-6
            nmos_eff_resistance_mult = 1.77
            n_to_p_eff_curr_drv_ratio = 2.54
            gmp_to_gmn_mult = 0.98
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 5
            I_g_on_n = 4.31e-8

        self._tech_params["Vdd"] += alpha * vdd_real
        self._tech_params["Vdd_default"] += alpha * vdd
        # technology.cc:1764-1766, 1817-1820 -- peri_global == ram_cell tech
        # type in every McPAT SRAM path, so both use this function's
        # vdd/vdd_real. Vcc_min_default is a RUNNING SUM over the
        # interpolation iterations (uses the partially-accumulated
        # Vdd_default), replicated exactly.
        self._tech_params["peri_global_Vdd"] += alpha * vdd_real
        self._tech_params["peri_global_Vdd_default"] += alpha * vdd
        self._tech_params["peri_global_Vcc_min_default"] += (
            self._tech_params["peri_global_Vdd_default"] * 0.45
        )
        self._tech_params["sram_cell_Vdd"] += alpha * vdd_real
        self._tech_params["sram_cell_Vdd_default"] += alpha * vdd
        self._tech_params["sram_cell_Vcc_min_default"] += (
            self._tech_params["sram_cell_Vdd_default"] * 0.6
        )
        self._tech_params["Vth"] += alpha * v_th
        self._tech_params["l_elec"] += alpha * l_elec
        self._tech_params["mobility_eff"] += alpha * mobility_eff
        self._tech_params["Vdsat"] += alpha * vd_sat
        self._tech_params["gmp_to_gmn_mult"] += alpha * gmp_to_gmn_mult
        self._tech_params["t_ox"] += alpha * t_ox
        self._tech_params["c_ox"] += alpha * c_ox
        self._tech_params["c_g_ideal"] += alpha * c_g_ideal
        self._tech_params["c_fringe"] += alpha * c_fringe
        self._tech_params["c_junc"] += alpha * c_junc
        self._tech_params["R_nch_on"] += alpha * (rn_channel_on)
        self._tech_params["R_pch_on"] += alpha * (rp_channel_on)
        self._tech_params["n_to_p_eff_curr_drv_ratio"] += alpha * (
            n_to_p_eff_curr_drv_ratio
        )
        self._tech_params["I_off_n"] += alpha * I_off_n
        self._tech_params["I_off_p"] += alpha * I_off_n
        self._tech_params["I_g_on_n"] += alpha * I_g_on_n
        self._tech_params["I_g_on_p"] += alpha * I_g_on_n
        self._tech_params["area_cell_sram"] += alpha * (
            AREA_CELL_SRAM * (self._node_um**2)
        )
        self._tech_params["area_cell_cam"] += alpha * (
            AREA_CELL_CAM * (self._node_um**2)
        )
        self._tech_params["asp_ratio_cell_sram"] += alpha * ASP_RATIO_SRAM
        self._tech_params["asp_ratio_cell_cam"] += alpha * ASP_RATIO_CAM
        self._tech_params["cam_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sram_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sense_dy_power"] += alpha * (SENSE_AMP_P)
        self._tech_params["sckt_coeff"] += alpha * sckt_coeff
        self._tech_params["macro_layout_overhead"] += (
            alpha * macro_layout_overhead
        )
        self._tech_params["chip_layout_overhead"] += (
            alpha * chip_layout_overhead
        )
        self._tech_params["long_channel_leakage_reduction"] += (
            alpha * long_channel_lkg_reduction
        )
        self._tech_params["logic_scaling_co_eff"] += (
            alpha * logic_scaling_co_eff
        )
        self._tech_params["core_tx_density"] += alpha * core_tx_density

    def init_tech_65nm(self, alpha):
        # Every constant in this function's HP/LSTP/LOP blocks (except vdd,
        # l_phy, the I_off_n tables, I_g_on_n, and long_channel_lkg_reduction
        # -- already correct/fixed in Phase 8) was independently found wrong
        # against technology.cc:566-705 while root-causing a ~4-30% per-
        # component dynamic-energy gap on the first 65nm array anchor
        # (icache, Phase 13) -- not a copy from any other node's table, just
        # scattered transcription errors across the whole physical-constant
        # block. Values below are transcribed directly from technology.cc.
        sckt_coeff = 1.1359
        macro_layout_overhead = 1.1
        chip_layout_overhead = 1.2
        logic_scaling_co_eff = 0.7
        core_tx_density = 1.25 * 0.7
        SENSE_AMP_D = 0.2e-9
        SENSE_AMP_P = 5.7e-15
        I_off_choices = {
            "HP": [
                1.96e-07,
                2.29e-07,
                2.66e-07,
                3.05e-07,
                3.49e-07,
                3.95e-07,
                4.45e-07,
                4.97e-07,
                5.48e-07,
                5.94e-07,
                6.3e-07,
            ],
            "LSTP": [
                9.12e-12,
                1.49e-11,
                2.36e-11,
                3.64e-11,
                5.48e-11,
                8.05e-11,
                1.15e-10,
                1.59e-10,
                2.1e-10,
                2.62e-10,
                3.21e-10,
            ],
            "LOP": [
                4.9e-09,
                6.49e-09,
                8.45e-09,
                1.08e-08,
                1.37e-08,
                1.71e-08,
                2.09e-08,
                2.48e-08,
                2.84e-08,
                3.13e-08,
                3.42e-08,
            ],
        }
        if self._mode == "HP":
            vdd = 1.1
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.27
            l_phy = 0.025
            l_elec = 0.019
            t_ox = 1.1e-3
            v_th = 0.19491
            c_ox = 1.88e-14
            mobility_eff = 436.24 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 7.71e-2
            c_g_ideal = 4.69e-16
            c_fringe = 0.077e-15
            c_junc = 1e-15
            i_on_n = (
                1197.2e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.50
            n_to_p_eff_curr_drv_ratio = 2.41
            gmp_to_gmn_mult = 1.38
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 3.74
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 4
            I_g_on_n = 4.09e-8
        elif self._mode == "LSTP":
            vdd = 1.2
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.40
            l_phy = 0.045
            l_elec = 0.0298
            t_ox = 1.9e-3
            v_th = 0.52354
            c_ox = 1.36e-14
            mobility_eff = 341.21 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 0.128
            c_g_ideal = 6.14e-16
            c_fringe = 0.08e-15
            c_junc = 1e-15
            i_on_n = (
                519.2e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.96
            n_to_p_eff_curr_drv_ratio = 2.23
            gmp_to_gmn_mult = 0.99
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 2.82
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 4
            I_g_on_n = 1.09e-10
        elif self._mode == "LOP":
            vdd = 0.8
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.43
            l_phy = 0.032
            l_elec = 0.0216
            t_ox = 1.2e-3
            v_th = 0.28512
            c_ox = 1.87e-14
            mobility_eff = 495.19 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 0.292
            c_g_ideal = 6e-16
            c_fringe = 0.08e-15
            c_junc = 1e-15
            i_on_n = (
                573.1e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.82
            n_to_p_eff_curr_drv_ratio = 2.28
            gmp_to_gmn_mult = 1.11
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 2.05
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 5
            I_g_on_n = 9.61e-9
        self._tech_params["Vdd"] += alpha * vdd_real
        self._tech_params["Vdd_default"] += alpha * vdd
        # technology.cc:1764-1766, 1817-1820 -- peri_global == ram_cell tech
        # type in every McPAT SRAM path, so both use this function's
        # vdd/vdd_real. Vcc_min_default is a RUNNING SUM over the
        # interpolation iterations (uses the partially-accumulated
        # Vdd_default), replicated exactly.
        self._tech_params["peri_global_Vdd"] += alpha * vdd_real
        self._tech_params["peri_global_Vdd_default"] += alpha * vdd
        self._tech_params["peri_global_Vcc_min_default"] += (
            self._tech_params["peri_global_Vdd_default"] * 0.45
        )
        self._tech_params["sram_cell_Vdd"] += alpha * vdd_real
        self._tech_params["sram_cell_Vdd_default"] += alpha * vdd
        self._tech_params["sram_cell_Vcc_min_default"] += (
            self._tech_params["sram_cell_Vdd_default"] * 0.6
        )
        self._tech_params["Vth"] += alpha * v_th
        self._tech_params["l_elec"] += alpha * l_elec
        self._tech_params["mobility_eff"] += alpha * mobility_eff
        self._tech_params["Vdsat"] += alpha * vd_sat
        self._tech_params["gmp_to_gmn_mult"] += alpha * gmp_to_gmn_mult
        self._tech_params["t_ox"] += alpha * t_ox
        self._tech_params["c_ox"] += alpha * c_ox
        self._tech_params["c_g_ideal"] += alpha * c_g_ideal
        self._tech_params["c_fringe"] += alpha * c_fringe
        self._tech_params["c_junc"] += alpha * c_junc
        self._tech_params["R_nch_on"] += alpha * (rn_channel_on)
        self._tech_params["R_pch_on"] += alpha * (rp_channel_on)
        self._tech_params["n_to_p_eff_curr_drv_ratio"] += alpha * (
            n_to_p_eff_curr_drv_ratio
        )
        self._tech_params["I_off_n"] += alpha * I_off_n
        self._tech_params["I_off_p"] += alpha * I_off_n
        self._tech_params["I_g_on_n"] += alpha * I_g_on_n
        self._tech_params["I_g_on_p"] += alpha * I_g_on_n
        self._tech_params["area_cell_sram"] += alpha * (
            AREA_CELL_SRAM * (self._node_um**2)
        )
        self._tech_params["area_cell_cam"] += alpha * (
            AREA_CELL_CAM * (self._node_um**2)
        )
        self._tech_params["asp_ratio_cell_sram"] += alpha * ASP_RATIO_SRAM
        self._tech_params["asp_ratio_cell_cam"] += alpha * ASP_RATIO_CAM
        self._tech_params["cam_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sram_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sense_dy_power"] += alpha * (SENSE_AMP_P)
        self._tech_params["sckt_coeff"] += alpha * sckt_coeff
        self._tech_params["macro_layout_overhead"] += (
            alpha * macro_layout_overhead
        )
        self._tech_params["chip_layout_overhead"] += (
            alpha * chip_layout_overhead
        )
        self._tech_params["long_channel_leakage_reduction"] += (
            alpha * long_channel_lkg_reduction
        )
        self._tech_params["logic_scaling_co_eff"] += (
            alpha * logic_scaling_co_eff
        )
        self._tech_params["core_tx_density"] += alpha * core_tx_density

    def init_tech_45nm(self, alpha):
        sckt_coeff = 1.1387
        macro_layout_overhead = 1.1
        chip_layout_overhead = 1.2
        logic_scaling_co_eff = 0.7 * 0.7
        core_tx_density = 1.25
        SENSE_AMP_D = 0.04e-9
        SENSE_AMP_P = 3.2e-15
        I_off_choices = {
            "HP": [
                2.8e-07,
                3.28e-07,
                3.81e-07,
                4.39e-07,
                5.02e-07,
                5.69e-07,
                6.42e-07,
                7.2e-07,
                8.03e-07,
                8.91e-07,
                9.84e-07,
            ],
            "LSTP": [
                1.01e-11,
                1.65e-11,
                2.62e-11,
                4.06e-11,
                6.12e-11,
                9.02e-11,
                1.3e-10,
                1.83e-10,
                2.51e-10,
                3.29e-10,
                4.1e-10,
            ],
            "LOP": [
                4.03e-09,
                5.02e-09,
                6.18e-09,
                7.51e-09,
                9.04e-09,
                1.08e-08,
                1.27e-08,
                1.47e-08,
                1.66e-08,
                1.84e-08,
                2.03e-08,
            ],
        }
        I_g_lop_choices = [
            3.24e-8,
            4.01e-8,
            4.9e-8,
            5.92e-8,
            7.08e-8,
            8.38e-8,
            9.82e-8,
            1.14e-7,
            1.29e-7,
            1.43e-7,
            1.54e-7,
        ]
        if self._mode == "HP":
            vdd = 1.0
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.21
            l_phy = 0.018
            l_elec = 0.01345
            t_ox = 0.65e-3
            v_th = 0.18035
            c_ox = 3.77e-14
            mobility_eff = 266.68 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 9.38e-2
            c_g_ideal = 6.78e-16
            c_fringe = 0.05e-15
            c_junc = 1e-15
            i_on_n = (
                2046.6e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.51
            n_to_p_eff_curr_drv_ratio = 2.41
            gmp_to_gmn_mult = 1.38
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 3.546
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 4
            I_g_on_n = 3.59e-8
        elif self._mode == "LSTP":
            vdd = 1.1
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.33
            l_phy = 0.028
            l_elec = 0.0212
            t_ox = 1.4e-3
            v_th = 0.50245
            c_ox = 2.01e-14
            mobility_eff = 363.96 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 9.12e-2
            c_g_ideal = 5.18e-16
            c_fringe = 0.08e-15
            c_junc = 1e-15
            i_on_n = (
                666.2e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.99
            n_to_p_eff_curr_drv_ratio = 2.23
            gmp_to_gmn_mult = 0.99
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 2.08
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 4
            I_g_on_n = 9.47e-12
        elif self._mode == "LOP":
            vdd = 0.7
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.39
            l_phy = 0.022
            l_elec = 0.016
            t_ox = 0.9e-3
            v_th = 0.22599
            c_ox = 2.82e-14
            mobility_eff = 508.9 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 5.71e-2
            c_g_ideal = 6.2e-16
            c_fringe = 0.073e-15
            c_junc = 1e-15
            i_on_n = (
                748.9e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.76
            n_to_p_eff_curr_drv_ratio = 2.28
            gmp_to_gmn_mult = 1.11
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 1.92
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 5
            I_g_on_n = I_g_lop_choices[(self._temp - 300) // 10]

        self._tech_params["Vdd"] += alpha * (vdd_real)
        self._tech_params["Vdd_default"] += alpha * (vdd)
        # technology.cc:1764-1766, 1817-1820 -- peri_global == ram_cell tech
        # type in every McPAT SRAM path, so both use this function's
        # vdd/vdd_real. Vcc_min_default is a RUNNING SUM over the
        # interpolation iterations (uses the partially-accumulated
        # Vdd_default), replicated exactly.
        self._tech_params["peri_global_Vdd"] += alpha * vdd_real
        self._tech_params["peri_global_Vdd_default"] += alpha * vdd
        self._tech_params["peri_global_Vcc_min_default"] += (
            self._tech_params["peri_global_Vdd_default"] * 0.45
        )
        self._tech_params["sram_cell_Vdd"] += alpha * vdd_real
        self._tech_params["sram_cell_Vdd_default"] += alpha * vdd
        self._tech_params["sram_cell_Vcc_min_default"] += (
            self._tech_params["sram_cell_Vdd_default"] * 0.6
        )
        self._tech_params["Vth"] += alpha * (v_th)
        self._tech_params["l_elec"] += alpha * (l_elec)
        self._tech_params["mobility_eff"] += alpha * (mobility_eff)
        self._tech_params["Vdsat"] += alpha * (vd_sat)
        self._tech_params["gmp_to_gmn_mult"] += alpha * (gmp_to_gmn_mult)
        self._tech_params["t_ox"] += alpha * (t_ox)
        self._tech_params["c_ox"] += alpha * (c_ox)
        self._tech_params["c_g_ideal"] += alpha * (c_g_ideal)
        self._tech_params["c_fringe"] += alpha * (c_fringe)
        self._tech_params["c_junc"] += alpha * (c_junc)
        self._tech_params["R_nch_on"] += alpha * (rn_channel_on)
        self._tech_params["R_pch_on"] += alpha * (rp_channel_on)
        self._tech_params["n_to_p_eff_curr_drv_ratio"] += alpha * (
            n_to_p_eff_curr_drv_ratio
        )
        self._tech_params["I_off_n"] += alpha * I_off_n
        self._tech_params["I_off_p"] += alpha * I_off_n
        self._tech_params["I_g_on_n"] += alpha * I_g_on_n
        self._tech_params["I_g_on_p"] += alpha * I_g_on_n
        self._tech_params["area_cell_sram"] += alpha * (
            AREA_CELL_SRAM * (self._node_um**2)
        )
        self._tech_params["area_cell_cam"] += alpha * (
            AREA_CELL_CAM * (self._node_um**2)
        )
        self._tech_params["asp_ratio_cell_sram"] += alpha * ASP_RATIO_SRAM
        self._tech_params["asp_ratio_cell_cam"] += alpha * ASP_RATIO_CAM
        self._tech_params["cam_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sram_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sense_dy_power"] += alpha * (SENSE_AMP_P)
        self._tech_params["sckt_coeff"] += alpha * sckt_coeff
        self._tech_params["macro_layout_overhead"] += (
            alpha * macro_layout_overhead
        )
        self._tech_params["chip_layout_overhead"] += (
            alpha * chip_layout_overhead
        )
        self._tech_params["long_channel_leakage_reduction"] += (
            alpha * long_channel_lkg_reduction
        )
        self._tech_params["logic_scaling_co_eff"] += (
            alpha * logic_scaling_co_eff
        )
        self._tech_params["core_tx_density"] += alpha * core_tx_density

    def init_tech_32nm(self, alpha):
        sckt_coeff = 1.1111
        macro_layout_overhead = 1.1
        chip_layout_overhead = 1.2
        logic_scaling_co_eff = 0.7 * 0.7 * 0.7
        core_tx_density = 1.25 / 0.7
        SENSE_AMP_D = 0.03e-9
        SENSE_AMP_P = 2.16e-15
        I_off_choices = {
            "HP": [
                1.52e-07,
                1.55e-07,
                1.59e-07,
                1.68e-07,
                1.9e-07,
                2.69e-07,
                5.32e-07,
                1.02e-06,
                1.62e-06,
                2.73e-06,
                6.1e-06,
            ],
            "LSTP": [
                2.06e-11,
                3.3e-11,
                5.15e-11,
                7.83e-11,
                1.16e-10,
                1.69e-10,
                2.4e-10,
                3.34e-10,
                4.54e-10,
                5.96e-10,
                7.44e-10,
            ],
            "LOP": [
                5.94e-08,
                7.23e-08,
                8.7e-08,
                1.04e-07,
                1.22e-07,
                1.43e-07,
                1.65e-07,
                1.9e-07,
                2.15e-07,
                2.39e-07,
                2.63e-07,
            ],
        }
        if self._mode == "HP":
            vdd = 0.9
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.19
            l_phy = 0.013
            l_elec = 0.01013
            t_ox = 0.5e-3
            v_th = 0.21835
            c_ox = 4.11e-14
            mobility_eff = 361.84 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 5.09e-2
            c_g_ideal = 5.34e-16
            c_fringe = 0.04e-15
            c_junc = 1e-15
            i_on_n = (
                2211.7e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.49
            n_to_p_eff_curr_drv_ratio = 2.41
            gmp_to_gmn_mult = 1.38
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 3.706
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 2
            I_g_on_n = 6.55e-8
        elif self._mode == "LSTP":
            vdd = 1.0
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.27
            l_phy = 0.020
            l_elec = 0.0173
            t_ox = 1.2e-3
            v_th = 0.513
            c_ox = 2.29e-14
            mobility_eff = 347.46 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 8.64e-2
            c_g_ideal = 4.58e-16
            c_fringe = 0.053e-15
            c_junc = 1e-15
            i_on_n = (
                683.6e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.99
            n_to_p_eff_curr_drv_ratio = 2.23
            gmp_to_gmn_mult = 0.99
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 1.93
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            )
            I_g_on_n = 3.73e-11
        elif self._mode == "LOP":
            vdd = 0.6
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.26
            l_phy = 0.016
            l_elec = 0.01232
            t_ox = 0.9e-3
            v_th = 0.24227
            c_ox = 2.84e-14
            mobility_eff = 513.52 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 4.64e-2
            c_g_ideal = 4.54e-16
            c_fringe = 0.057e-15
            c_junc = 1e-15
            i_on_n = (
                827.8e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.73
            n_to_p_eff_curr_drv_ratio = 2.28
            gmp_to_gmn_mult = 1.11
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 1.89
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 5
            I_g_on_n = 2.93e-9
        self._tech_params["Vdd"] += alpha * (vdd_real)
        self._tech_params["Vdd_default"] += alpha * (vdd)
        # technology.cc:1764-1766, 1817-1820 -- peri_global == ram_cell tech
        # type in every McPAT SRAM path, so both use this function's
        # vdd/vdd_real. Vcc_min_default is a RUNNING SUM over the
        # interpolation iterations (uses the partially-accumulated
        # Vdd_default), replicated exactly.
        self._tech_params["peri_global_Vdd"] += alpha * vdd_real
        self._tech_params["peri_global_Vdd_default"] += alpha * vdd
        self._tech_params["peri_global_Vcc_min_default"] += (
            self._tech_params["peri_global_Vdd_default"] * 0.45
        )
        self._tech_params["sram_cell_Vdd"] += alpha * vdd_real
        self._tech_params["sram_cell_Vdd_default"] += alpha * vdd
        self._tech_params["sram_cell_Vcc_min_default"] += (
            self._tech_params["sram_cell_Vdd_default"] * 0.6
        )
        self._tech_params["Vth"] += alpha * (v_th)
        self._tech_params["l_elec"] += alpha * (l_elec)
        self._tech_params["mobility_eff"] += alpha * (mobility_eff)
        self._tech_params["Vdsat"] += alpha * (vd_sat)
        self._tech_params["gmp_to_gmn_mult"] += alpha * (gmp_to_gmn_mult)
        self._tech_params["t_ox"] += alpha * (t_ox)
        self._tech_params["c_ox"] += alpha * (c_ox)
        self._tech_params["c_g_ideal"] += alpha * (c_g_ideal)
        self._tech_params["c_fringe"] += alpha * (c_fringe)
        self._tech_params["c_junc"] += alpha * (c_junc)
        self._tech_params["R_nch_on"] += alpha * (rn_channel_on)
        self._tech_params["R_pch_on"] += alpha * (rp_channel_on)
        self._tech_params["n_to_p_eff_curr_drv_ratio"] += alpha * (
            n_to_p_eff_curr_drv_ratio
        )
        self._tech_params["I_off_n"] += alpha * I_off_n
        self._tech_params["I_off_p"] += alpha * I_off_n
        self._tech_params["I_g_on_n"] += alpha * I_g_on_n
        self._tech_params["I_g_on_p"] += alpha * I_g_on_n
        self._tech_params["area_cell_sram"] += alpha * (
            AREA_CELL_SRAM * (self._node_um**2)
        )
        self._tech_params["area_cell_cam"] += alpha * (
            AREA_CELL_CAM * (self._node_um**2)
        )
        self._tech_params["asp_ratio_cell_sram"] += alpha * ASP_RATIO_SRAM
        self._tech_params["asp_ratio_cell_cam"] += alpha * ASP_RATIO_CAM
        self._tech_params["cam_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sram_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sense_dy_power"] += alpha * (SENSE_AMP_P)
        self._tech_params["sckt_coeff"] += alpha * sckt_coeff
        self._tech_params["macro_layout_overhead"] += (
            alpha * macro_layout_overhead
        )
        self._tech_params["chip_layout_overhead"] += (
            alpha * chip_layout_overhead
        )
        self._tech_params["long_channel_leakage_reduction"] += (
            alpha * long_channel_lkg_reduction
        )
        self._tech_params["logic_scaling_co_eff"] += (
            alpha * logic_scaling_co_eff
        )
        self._tech_params["core_tx_density"] += alpha * core_tx_density

    def init_tech_22nm(self, alpha):
        sckt_coeff = 1.1296
        macro_layout_overhead = 1.1
        chip_layout_overhead = 1.2
        logic_scaling_co_eff = 0.7 * 0.7 * 0.7 * 0.7
        core_tx_density = 1.25 / 0.7 / 0.7
        SENSE_AMP_D = 0.03e-9
        SENSE_AMP_P = 2.16e-15
        I_off_choices = {
            "HP": [
                1.52e-07,
                1.55e-07,
                1.59e-07,
                1.68e-07,
                1.9e-07,
                2.69e-07,
                5.32e-07,
                1.02e-06,
                1.62e-06,
                2.73e-06,
                6.1e-06,
            ],
            "LSTP": [
                2.43e-11,
                4.85e-11,
                9.68e-11,
                1.94e-10,
                3.87e-10,
                7.73e-10,
                3.55e-10,
                3.09e-09,
                6.19e-09,
                1.24e-08,
                2.48e-08,
            ],
            "LOP": [
                1.31e-08,
                2.6e-08,
                5.14e-08,
                1.02e-07,
                2.02e-07,
                3.99e-07,
                7.91e-07,
                1.09e-06,
                2.09e-06,
                4.04e-06,
                4.48e-06,
            ],
        }

        if self._mode == "HP":
            vdd = 0.8
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.2
            l_phy = 0.009
            l_elec = 0.00468
            t_ox = 0.55e-3
            v_th = 0.1395
            c_ox = 3.63e-14
            mobility_eff = 426.07 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 2.33e-2
            c_g_ideal = 3.27e-16
            c_fringe = 0.06e-15
            c_junc = 0
            i_on_n = (
                2626.4e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.45
            n_to_p_eff_curr_drv_ratio = 2
            gmp_to_gmn_mult = 1.38
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 3.274
            I_off_n = (
                (I_off_choices[self._mode][(self._temp - 300) // 10])
                / 1.5
                * 1.2
                * (vdd_real / vdd) ** 2
            )
            I_g_on_n = 1.81e-9
        elif self._mode == "LSTP":
            vdd = 0.8
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.23
            l_phy = 0.014
            l_elec = 0.008
            t_ox = 1.1e-3
            v_th = 0.40126
            c_ox = 2.30e-14
            mobility_eff = 738.09 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 6.64e-2
            c_g_ideal = 3.22e-16
            c_fringe = 0.08e-15
            c_junc = 0
            i_on_n = (
                727.6e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.99
            n_to_p_eff_curr_drv_ratio = 2
            gmp_to_gmn_mult = 0.99
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 1.89
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            )
            I_g_on_n = 4.51e-10
        if self._mode == "LOP":
            vdd = 0.6
            vdd_real = self._user_vdd if self._specific_vdd else vdd
            alpha_power_law = 1.21
            l_phy = 0.011
            l_elec = 0.00604
            t_ox = 0.8e-3
            v_th = 0.2315
            c_ox = 2.87e-14
            mobility_eff = 698.37 * (1e-2 * 1e6 * 1e-2 * 1e6)
            vd_sat = 1.81e-2
            c_g_ideal = 3.16e-16
            c_fringe = 0.08e-15
            c_junc = 0
            i_on_n = (
                916.1e-6
                * ((vdd_real - v_th) / (vdd - v_th)) ** alpha_power_law
            )
            i_on_p = i_on_n / 2
            nmos_eff_resistance_mult = 1.73
            n_to_p_eff_curr_drv_ratio = 2
            gmp_to_gmn_mult = 1.11
            rn_channel_on = nmos_eff_resistance_mult * vdd_real / i_on_n
            rp_channel_on = n_to_p_eff_curr_drv_ratio * rn_channel_on
            long_channel_lkg_reduction = 1 / 2.38
            I_off_n = (I_off_choices[self._mode][(self._temp - 300) // 10]) * (
                vdd_real / vdd
            ) ** 5
            I_g_on_n = 2.74e-9
        self._tech_params["Vdd"] += alpha * (vdd_real)
        self._tech_params["Vdd_default"] += alpha * (vdd)
        # technology.cc:1764-1766, 1817-1820 -- peri_global == ram_cell tech
        # type in every McPAT SRAM path, so both use this function's
        # vdd/vdd_real. Vcc_min_default is a RUNNING SUM over the
        # interpolation iterations (uses the partially-accumulated
        # Vdd_default), replicated exactly.
        self._tech_params["peri_global_Vdd"] += alpha * vdd_real
        self._tech_params["peri_global_Vdd_default"] += alpha * vdd
        self._tech_params["peri_global_Vcc_min_default"] += (
            self._tech_params["peri_global_Vdd_default"] * 0.45
        )
        self._tech_params["sram_cell_Vdd"] += alpha * vdd_real
        self._tech_params["sram_cell_Vdd_default"] += alpha * vdd
        self._tech_params["sram_cell_Vcc_min_default"] += (
            self._tech_params["sram_cell_Vdd_default"] * 0.6
        )
        self._tech_params["Vth"] += alpha * (v_th)
        self._tech_params["l_elec"] += alpha * (l_elec)
        self._tech_params["mobility_eff"] += alpha * (mobility_eff)
        self._tech_params["Vdsat"] += alpha * (vd_sat)
        self._tech_params["gmp_to_gmn_mult"] += alpha * (gmp_to_gmn_mult)
        self._tech_params["t_ox"] += alpha * (t_ox)
        self._tech_params["c_ox"] += alpha * (c_ox)
        self._tech_params["c_g_ideal"] += alpha * (c_g_ideal)
        self._tech_params["c_fringe"] += alpha * (c_fringe)
        self._tech_params["c_junc"] += alpha * (c_junc)
        self._tech_params["R_nch_on"] += alpha * (rn_channel_on)
        self._tech_params["R_pch_on"] += alpha * (rp_channel_on)
        self._tech_params["n_to_p_eff_curr_drv_ratio"] += alpha * (
            n_to_p_eff_curr_drv_ratio
        )
        self._tech_params["I_off_n"] += alpha * I_off_n
        self._tech_params["I_off_p"] += alpha * I_off_n
        self._tech_params["I_g_on_n"] += alpha * I_g_on_n
        self._tech_params["I_g_on_p"] += alpha * I_g_on_n
        self._tech_params["area_cell_sram"] += alpha * (
            AREA_CELL_SRAM * (self._node_um**2)
        )
        self._tech_params["area_cell_cam"] += alpha * (
            AREA_CELL_CAM * (self._node_um**2)
        )
        self._tech_params["asp_ratio_cell_sram"] += alpha * ASP_RATIO_SRAM
        self._tech_params["asp_ratio_cell_cam"] += alpha * ASP_RATIO_CAM
        self._tech_params["cam_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sram_cell_a_w"] += alpha * (1.31 * self._node_um)
        self._tech_params["sense_dy_power"] += alpha * (SENSE_AMP_P)
        self._tech_params["sckt_coeff"] += alpha * sckt_coeff
        self._tech_params["macro_layout_overhead"] += (
            alpha * macro_layout_overhead
        )
        self._tech_params["chip_layout_overhead"] += (
            alpha * chip_layout_overhead
        )
        self._tech_params["long_channel_leakage_reduction"] += (
            alpha * long_channel_lkg_reduction
        )
        self._tech_params["logic_scaling_co_eff"] += (
            alpha * logic_scaling_co_eff
        )
        self._tech_params["core_tx_density"] += alpha * core_tx_density
