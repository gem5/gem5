from math import (
    ceil,
    comb,
    log,
    sqrt,
)

from .cacti_params import CactiParams

"""

This file encapsulates all the circuit operations found in CACTI,
these are used in several components to get dynamic and leakage
energies.

"""

UNI_LEAK_STACK_FACTOR = 0.43


class CactiCircuit:
    def __init__(self, comp_params: CactiParams):
        self._cacti_params = comp_params
        self._tp = comp_params._tech_params
        self._wp = comp_params._wire_params
        self._machine_config = comp_params._machine_config
        self._node = comp_params._node
        self._node_um = comp_params._node_um

    def _long_channel_reduction(self, device_ty="core"):
        """Port of basic_components.cc:longer_channel_device_reduction, for
        the logic-side components in mcpat_logic_components.py -- every one
        of their real call sites (core.cc's construction of selection_logic/
        dep_resource_conflict_check/Pipeline/UndiffCore/FunctionalUnit/
        inst_decoder/interconnect) passes Device_ty::Core_device and
        coredynp.core_ty, so device_ty defaults to "core" and core_ooo is
        derived here from the same core0["machine_type"]==0 convention
        already used throughout this file (Core_type: OOO=0, Inorder=1,
        confirmed against basic_components.h and core.cc:4346's
        `coredynp.core_ty = (enum Core_type)XML->sys.core[ithCore].machine_type`).
        Mirrors CactiArrayST._long_channel_reduction (cacti_component.py) --
        not shared with it directly since CactiArrayST is not a CactiCircuit
        subclass and its own version is already independently validated
        ; left untouched.
        """
        core0 = self._machine_config._config_params["core0"]
        core_ooo = core0["machine_type"] == 0
        peri = self._tp["long_channel_leakage_reduction"]
        if device_ty == "core":
            pct = 0.56 if core_ooo else 0.8
        elif device_ty == "uncore":
            pct = 0.82
        else:  # llc
            pct = 1.0
        return (1.0 - pct) + pct * peri

    def drain_C(
        self,
        width,
        channel,
        stack,
        folding_w_or_h,
        fold_dim,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        assert channel == "NCH" or channel == "PCH"
        drain_C_metal_cnt_folded_tr = 0
        if folding_w_or_h == 0:
            w_folded_tr = fold_dim
        else:
            h_tr_region = fold_dim - 2 * self._tp["hpowerrail"]
            p_to_n_ratio = 2 / 3
            w_folded_tr = h_tr_region - self._tp["min_gap_p_to_n_diff"]
            if channel == "NCH":
                w_folded_tr *= 1 - p_to_n_ratio
            else:
                w_folded_tr *= p_to_n_ratio

        num_folded_tr = int(ceil(width / w_folded_tr))
        if num_folded_tr < 2:
            w_folded_tr = width

        total_drain_w = (
            self._tp["w_poly_contact"]
            + 2 * self._tp["spacing_poly_to_contact"]
            + (stack - 1) * self._tp["spacing_poly_to_poly"]
        )
        drain_h_for_sidewall = w_folded_tr
        total_drain_h_wrt_gate = w_folded_tr + 2 * w_folded_tr * (stack - 1)

        if num_folded_tr > 1:
            total_drain_w += (num_folded_tr - 2) * (
                self._tp["w_poly_contact"]
                + 2 * self._tp["spacing_poly_to_contact"]
            ) + (num_folded_tr - 1) * (
                (stack - 1) * self._tp["spacing_poly_to_poly"]
            )
            if num_folded_tr % 2 == 0:
                drain_h_for_sidewall = 0
            total_drain_h_wrt_gate *= num_folded_tr
            drain_C_metal_cnt_folded_tr = (
                self._wp["C_per_micron"] * total_drain_w
            )

        drain_C_area = self._tp["c_junc"] * total_drain_w * w_folded_tr
        drain_C_sidewall = self._tp["c_junc_sidewall"] * (
            drain_h_for_sidewall + 2 * total_drain_w
        )
        drain_C_wrt_gate = (
            2 * self._tp["c_fringe"] + 2 * self._tp["c_overlap"]
        ) * total_drain_h_wrt_gate

        return (
            drain_C_area
            + drain_C_sidewall
            + drain_C_wrt_gate
            + drain_C_metal_cnt_folded_tr
        )

    def gate_C(
        self,
        width,
        wire_length,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        return width * (
            self._tp["c_g_ideal"]
            + self._tp["c_overlap"]
            + 3 * self._tp["c_fringe"]
        )

    def tr_R_on(
        self,
        width,
        channel,
        stack,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        if channel == "NCH":
            restrans = self._tp["R_nch_on"]
        elif channel == "PCH":
            restrans = self._tp["R_pch_on"]
        else:
            raise ValueError(f"Channel {channel} must be NCH or PCH!")
        return stack * restrans / width

    def cmos_Ig_n(
        self,
        nWidth,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        return nWidth * self._tp["I_g_on_n"]

    def cmos_Ig_p(
        self,
        pWidth,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        return pWidth * self._tp["I_g_on_p"]

    def _cacti_factorial(self, n, m=1):
        # CACTI basic_circuit.cc factorial(n, m) = m*(m+1)*...*n.
        fa = m
        for i in range(m + 1, n + 1):
            fa *= i
        return fa

    def _cacti_combination(self, n, m):
        # CACTI basic_circuit.cc combination(n, m) = factorial(n, m+1)/factorial(n-m).
        # This is NOT the standard binomial coefficient (e.g. combination(2,2)==3,
        # combination(3,3)==4); it is what CACTI's leakage model actually uses, so
        # we replicate it verbatim (integer division, as in the C++).
        return self._cacti_factorial(n, m + 1) // self._cacti_factorial(n - m)

    def cmos_Ig_leakage(
        self,
        nWidth,
        pWidth,
        fanin,
        gate_type,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        assert fanin >= 1
        nmos_leak = self.cmos_Ig_n(nWidth)
        pmos_leak = self.cmos_Ig_p(pWidth)
        num_states = 2**fanin
        Ig_on = 0
        if gate_type == "NMOS":
            Ig_on = nmos_leak * fanin
            for i in range(1, fanin):
                Ig_on += nmos_leak * self._cacti_combination(fanin, i) * i / 2
            Ig_on /= num_states
        elif gate_type == "PMOS":
            Ig_on = pmos_leak * fanin
            for i in range(1, fanin):
                Ig_on += pmos_leak * self._cacti_combination(fanin, i) * i / 2
            Ig_on /= num_states
        elif gate_type == "INV":
            Ig_on = (nmos_leak + pmos_leak) / 2
        elif gate_type == "NAND":
            for i in range(1, fanin + 1):
                Ig_on += pmos_leak * self._cacti_combination(fanin, i) * i
            Ig_on += nmos_leak * fanin
            for i in range(1, fanin):
                Ig_on += nmos_leak * self._cacti_combination(fanin, i) * i / 2
            Ig_on /= num_states
        elif gate_type == "NOR":
            Ig_on += pmos_leak * fanin
            for i in range(1, fanin):
                Ig_on += pmos_leak * self._cacti_combination(fanin, i) * i / 2
            for i in range(1, fanin + 1):
                Ig_on += nmos_leak * self._cacti_combination(fanin, i) * i
            Ig_on /= num_states
        elif gate_type == "TRI":
            Ig_on += (2 * nmos_leak + 2 * pmos_leak) / 2
            Ig_on += (nmos_leak + pmos_leak) / 2
            Ig_on /= 2
        elif gate_type == "TG":
            Ig_on += (nmos_leak + pmos_leak) / 2
        return Ig_on

    def cmos_Isub_leakage(
        self,
        nWidth,
        pWidth,
        fanin,
        gate_type,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        assert fanin >= 1
        nmos_leak = self.simplified_nmos_leakage(
            nWidth, is_dram, is_cell, is_wl_tr, is_sleep_tx
        )
        pmos_leak = self.simplified_pmos_leakage(
            pWidth, is_dram, is_cell, is_wl_tr, is_sleep_tx
        )
        num_states = 2**fanin
        Isub = 0
        if gate_type == "NMOS":
            # CACTI special-cases fanin==1 (single tx); fanin>1 defaults to the
            # series (stacked) topology. combination() must NOT run for fanin==1
            # (combination(1,1)==2 would double the single-transistor leakage).
            if fanin == 1:
                Isub = nmos_leak / num_states
            else:
                for i in range(1, fanin + 1):
                    Isub += (
                        nmos_leak
                        * UNI_LEAK_STACK_FACTOR ** (i - 1)
                        * self._cacti_combination(fanin, i)
                    )
                Isub /= num_states
        elif gate_type == "PMOS":
            if fanin == 1:
                Isub = pmos_leak / num_states
            else:
                for i in range(1, fanin + 1):
                    Isub += (
                        pmos_leak
                        * UNI_LEAK_STACK_FACTOR ** (i - 1)
                        * self._cacti_combination(fanin, i)
                    )
                Isub /= num_states
        elif gate_type == "INV":
            Isub = (nmos_leak + pmos_leak) / 2
        elif gate_type == "NAND":
            Isub += fanin * pmos_leak
            for i in range(1, fanin + 1):
                Isub += (
                    nmos_leak
                    * UNI_LEAK_STACK_FACTOR ** (i - 1)
                    * self._cacti_combination(fanin, i)
                )
            Isub /= num_states
        elif gate_type == "NOR":
            for i in range(1, fanin + 1):
                Isub += (
                    pmos_leak
                    * UNI_LEAK_STACK_FACTOR ** (i - 1)
                    * self._cacti_combination(fanin, i)
                )
            Isub += fanin * nmos_leak
            Isub /= num_states
        elif gate_type == "TRI":
            Isub += (nmos_leak + pmos_leak) / 2
            Isub += nmos_leak * UNI_LEAK_STACK_FACTOR
            Isub /= 2
        elif gate_type == "TG":
            Isub = (nmos_leak + pmos_leak) / 2
        return Isub

    def simplified_nmos_leakage(
        self,
        nWidth,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        return nWidth * self._tp["I_off_n"]

    def simplified_pmos_leakage(
        self,
        pWidth,
        is_dram=False,
        is_cell=False,
        is_wl_tr=False,
        is_sleep_tx=False,
    ):
        return pWidth * self._tp["I_off_p"]

    def pmos_to_nmos_sz_ratio(self):
        return self._cacti_params.get_pmos_to_nmos_sz_ratio()

    def _wire_layer_geom(self, type_idx):
        # CACTI technology.cc indexes wire_pitch[ic_proj_type][type_idx] (and the
        # matching R/C/dielectric tables) identically for wire_inside_mat and
        # wire_outside_mat -- only the type_idx (wire_is_mat_type / wire_os_mat_type)
        # differs. type_idx: 0=local (2.5F), 1=semi-global/"inside_mat" (4F),
        # 2=global (8F pitch -- pitch is the only node-independent piece;
        # aspect_ratio/ild_thickness/horiz_dielec_const vary by node like the
        # other two tiers, the native reference Finding 1). Returns
        # (pitch_um, aspect, horiz, vert, ild_um, fringe_F_per_um).
        wp = self._wp
        if type_idx == 0:
            return (
                wp["pitch"],
                wp["aspect_ratio_local"],
                wp["horiz_dielec_local"],
                wp["vert_dielec_local"],
                wp["ild_local"],
                wp["fringe_local"],
            )
        elif type_idx == 2:
            return (
                wp["pitch_global"],
                wp["aspect_ratio_global"],
                wp["horiz_dielec_global"],
                wp["vert_dielec_global"],
                wp["ild_global"],
                wp["fringe_global"],
            )
        else:
            # The semi-global table has no separate fringe entry (fringe_cap is
            # 0.115 fF/um at every layer/node).
            return (
                wp["pitch_inside_mat"],
                wp["aspect_ratio_inside_mat"],
                wp["horiz_dielec_inside_mat"],
                wp["vert_dielec_inside_mat"],
                wp["ild_inside_mat"],
                0.115e-15,
            )

    def _cpm(
        self,
        pitch_um,
        aspect,
        horiz,
        vert,
        ild,
        fringe,
        w_scale,
        s_scale,
        adj_microns_bug=False,
    ):
        # Port of Wire::wire_cap (wire.cc:297-371), returning tot_cap in F/m
        # (i.e. Wire::wire_cap(len)/len). Both _wire_model's internal Cpm term
        # and the public wire_cap() method go through here so the width/space
        # scaling is defined in exactly one place.
        #
        # wire.cc:341  wire_height = wire_width/w_scale*aspect_ratio  -- the
        # /w_scale exactly cancels the w_scale baked into wire_width by the
        # constructor, so wire_height (hence the `sidewall` term) is
        # INDEPENDENT of w_scale; the comment at wire.cc:342-345 says this is
        # deliberate ("assuming height does not change ... as wire width
        # increases"). `sidewall` scales with s_scale via wire_spacing;
        # `adj` scales with w_scale via wire_width. So:
        #   wire_width   = pitch_um*1e-6/2 * w_scale   (C++ wire_width, m)
        #   wire_spacing = pitch_um*1e-6/2 * s_scale   (C++ wire_spacing, m)
        #   wire_height  = pitch_um*1e-6/2 * aspect    (= wire_width/w_scale*aspect)
        #
        # adj_microns_bug mirrors wire_cap(call_from_outside=False) invoked on
        # a freshly-built Wire whose wire_width is still in MICRONS (the
        # constructor's closing wire_width*=1e6, wire.cc:59): the `adj` term's
        # `wire_width/(ild_thickness*1e-6)` then divides microns by metres and
        # comes out 1e6x too large. This is what router.cc:81's Cw3 (the only
        # call_from_outside=False site) actually feeds downstream; replicated,
        # not fixed. arbiter.cc:103 passes call_from_outside=True and gets the
        # clean metres form (adj_microns_bug=False).
        eps0 = 8.8542e-12
        miller = 1.5  # g_tp.wire_*.miller_value
        ww = (
            pitch_um * 1e-6 / 2.0
        )  # wire_width / w_scale (unscaled half-pitch, m)
        ws = ww * s_scale  # C++ wire_spacing (m)
        wh = ww * aspect  # C++ wire_height (uses wire_width/w_scale)
        adj_w = ww * w_scale  # C++ wire_width (m)
        if adj_microns_bug:
            adj_w = adj_w * 1e6
        return (
            miller * horiz * (wh / ws) * eps0
            + miller * vert * adj_w / (ild * 1e-6) * eps0
            + fringe * 1e6
        )  # F/m

    def wire_cap(
        self, length_m, call_from_outside=False, w_scale=1.0, s_scale=1.0
    ):
        # Port of Wire::wire_cap (mcpat/cacti/wire.cc:297). Returns load
        # capacitance in F for a wire of length_m metres, with the given
        # width/space scaling (Wire(g_ip->wt, len, 1, w_scale, s_scale)).
        #
        # Geometry tier: Wire's wire_placement defaults to outside_mat, so
        # wire.cc:307 reads g_tp.wire_outside_mat.*, which CACTI fills from
        # g_ip->wire_os_mat_type. Every consumer in this file's scope
        # (crossbar.cc / arbiter.cc / router.cc, all reached through NoC::NoC)
        # runs with the Embedded setting wire_os_mat_type=1 (noc.cc:64-66),
        # i.e. the semi-global / "inside_mat" tier -- _wire_layer_geom(1).
        # (A non-embedded NoC would use wire_os_mat_type=2; out of scope here.)
        #
        # call_from_outside mirrors the C++ flag: True (arbiter.cc:103) ->
        # clean metres form; False (router.cc:81) -> the 1e6-inflated `adj`
        # term (see _cpm's adj_microns_bug note).
        pitch_um, aspect, horiz, vert, ild, fringe = self._wire_layer_geom(1)
        cpm = self._cpm(
            pitch_um,
            aspect,
            horiz,
            vert,
            ild,
            fringe,
            w_scale,
            s_scale,
            adj_microns_bug=not call_from_outside,
        )
        return cpm * length_m

    def _wire_model(
        self,
        pitch_um,
        aspect,
        horiz,
        vert,
        ild,
        fringe,
        overhead,
        is_dram=False,
        w_scale=1.0,
        s_scale=1.0,
    ):
        # Port of Wire::delay_optimal_wire + Wire::wire_model + (for overhead>0)
        # Wire::init_wire/update_fullswing (wire.cc). delay_optimal_wire/wire_model
        # recompute the wire cap/res via Wire::wire_cap / Wire::wire_res, which use a
        # DIFFERENT formula than technology.cc's wire_capacitance (the source itself
        # flags this inconsistency) -- so this does not reuse the C_per_micron_*
        # tables directly, only the raw geometry (pitch/aspect/dielectrics/ild).
        # w_scale/s_scale default to 1 (every energy-pipeline caller); the
        # crossbar/arbiter ports (Wire winit(4,4), Wire(g_ip->wt,len,1,3,3))
        # pass non-unit values. CU resistivity, alpha_scatter=1.05 are
        # hard-coded in Wire::wire_res, independent of the technology's own
        # resistivity table.
        #
        # overhead: 0 (delay-optimal, closed-form) or 5/10/20/30 (%-delay-overhead
        # point chosen by init_wire's repeater grid search + update_fullswing's
        # loosest-first pruned greedy selection).
        #
        # Returns dict: dynamic_per_um (J), leak_per_um (W), gleak_per_um (W),
        # repeater_size, repeater_spacing_um, Cpm (F/m, for Htree2's wire_cap()),
        # delay_per_um (s/um, chosen candidate's per-meter delay scaled to per-um).
        #
        # Pure function of the geometry args + self._tp, so memoize it on the
        # CactiParams instance -- the partition search calls this ~10^4 times
        # per run across only 2-3 distinct arg tuples. Callers get a fresh copy.
        _wm_cache = self._cacti_params.__dict__.setdefault(
            "_wire_model_cache", {}
        )
        _wm_key = (
            pitch_um,
            aspect,
            horiz,
            vert,
            ild,
            fringe,
            overhead,
            is_dram,
            w_scale,
            s_scale,
        )
        if _wm_key in _wm_cache:
            return dict(_wm_cache[_wm_key])
        tp = self._tp
        Vdd = tp["Vdd"]
        minn = tp["min_w_nmos"]
        minp = self.pmos_to_nmos_sz_ratio() * minn
        beta = self.pmos_to_nmos_sz_ratio()

        resistivity = 0.022  # CU_RESISTIVITY (static-global default)
        alpha_scatter = 1.05  # hard-coded in Wire::wire_res
        ww = (
            pitch_um * 1e-6 / 2.0
        )  # wire_width / w_scale (unscaled half-pitch, m)
        Cpm = self._cpm(
            pitch_um, aspect, horiz, vert, ild, fringe, w_scale, s_scale
        )  # F/m
        # Wire::wire_res: denom = (aspect*wire_width/w_scale) * wire_width
        #               = (aspect * ww) * (ww * w_scale)
        Rpm = (
            alpha_scatter
            * resistivity
            * 1e-6
            / ((aspect * ww) * (ww * w_scale))
        )  # ohm/m

        input_cap = self.gate_C(minn + minp, 0)
        out_cap = self.drain_C(
            minp, "PCH", 1, 1, tp["cell_h_def"]
        ) + self.drain_C(minn, "NCH", 1, 1, tp["cell_h_def"])
        out_res = (
            self.tr_R_on(minn, "NCH", 1) + self.tr_R_on(minp, "PCH", 1)
        ) / 2.0

        def wire_model_point(spacing_m, size):
            # Wire::wire_model(space, size, &delay) -- NOTE the 3rd tc term uses
            # out_cap (wire_model), unlike Wire::delay_optimal_wire (which uses
            # input_cap there); this is the formula CACTI actually stores into
            # Wire::global/global_5/../global_30.
            switching = (
                (size * (input_cap + out_cap) + spacing_m * Cpm) * Vdd * Vdd
            )
            tc = (
                out_res * (input_cap + out_cap)
                + out_res * Cpm * spacing_m / size
                + Rpm * spacing_m * out_cap * size
                + 0.5 * Rpm * Cpm * spacing_m * spacing_m
            )
            delay = 0.693 * tc / spacing_m
            short_ckt = Vdd * minn * 65e-6 * 1.0986 * size * tc
            dyn = (1.0 / spacing_m) * (switching + short_ckt)
            leak = (
                (1.0 / spacing_m)
                * Vdd
                * self.cmos_Isub_leakage(
                    minn * size, beta * minn * size, 1, "INV", is_dram=is_dram
                )
            )
            gleak = (
                (1.0 / spacing_m)
                * Vdd
                * self.cmos_Ig_leakage(
                    minn * size, beta * minn * size, 1, "INV", is_dram=is_dram
                )
            )
            return delay, dyn, leak, gleak

        # delay_optimal_wire: delay-optimal repeater size and spacing (metres).
        repeater_size0 = sqrt(out_res * Cpm / (Rpm * input_cap))
        repeater_spacing0 = sqrt(
            2 * out_res * (out_cap + input_cap) / (Rpm * Cpm)
        )
        delay0, dyn0, leak0, gleak0 = wire_model_point(
            repeater_spacing0, repeater_size0
        )

        if overhead == 0:
            repeater_size, repeater_spacing_um = (
                repeater_size0,
                repeater_spacing0 * 1e6,
            )
            dyn, leak, gleak = dyn0, leak0, gleak0
            delay_val = delay0
        else:
            # Wire::init_wire: grid search over repeater spacing (j, microns,
            # sp -> <4*sp step 100) x repeater size (i, a DOUBLE, si -> >1 step -1).
            sp_um = repeater_spacing0 * 1e6
            si = repeater_size0
            candidates = []
            j = sp_um
            while j < 4 * sp_um:
                i = si
                while i > 1:
                    d, dy, lk, gl = wire_model_point(j * 1e-6, i)
                    candidates.append((d, dy, lk, gl, j, i))
                    i -= 1.0
                j += 100.0

            # Wire::update_fullswing: loosest-first (30/20/10/5%), each round prunes
            # the candidate list permanently (delay > threshold discarded) before the
            # tighter next round; per-round pick is min-cost among survivors, first
            # encountered wins ties (matches C++'s strict '<').
            remaining = candidates
            chosen = {}
            for penalty, label in (
                (1.30, 30),
                (1.20, 20),
                (1.10, 10),
                (1.05, 5),
            ):
                threshold = delay0 * penalty
                remaining = [c for c in remaining if c[0] <= threshold]
                best, best_cost = None, None
                for c in remaining:
                    cost = c[1] / dyn0 + c[2] / leak0
                    if best_cost is None or cost < best_cost:
                        best_cost, best = cost, c
                chosen[label] = best
            delay_val, dyn, leak, gleak, repeater_spacing_um, repeater_size = (
                chosen[overhead]
            )

        _wm_result = dict(
            dynamic_per_um=dyn * 1e-6,
            leak_per_um=leak * 1e-6,
            gleak_per_um=gleak * 1e-6,
            repeater_size=repeater_size,
            repeater_spacing_um=repeater_spacing_um,
            Cpm=Cpm,
            delay_per_um=delay_val * 1e-6,
        )
        _wm_cache[_wm_key] = _wm_result
        return dict(_wm_result)

    def horowitz(self, input_ramp_time, tf, vs1, vs2, rise):
        # Port of CACTI basic_circuit.cc horowitz() (basic_circuit.cc:366-392).
        if input_ramp_time == 0 and vs1 == vs2:
            val = -log(vs1) if (vs1 < 1) else log(vs1)
            return tf * val
        a = input_ramp_time / tf
        if rise == True:
            b = 0.5
            td = tf * sqrt(log(vs1) ** 2 + 2 * a * b * (1 - vs1)) + tf * (
                log(vs1) - log(vs2)
            )
        else:
            b = 0.4
            td = tf * sqrt(log(1 - vs1) ** 2 + 2 * a * b * vs1) + tf * (
                log(1 - vs1) - log(1 - vs2)
            )
        return td
