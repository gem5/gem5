# SPDX-License-Identifier: BSD-3-Clause
from dataclasses import dataclass
from math import (
    ceil,
    log,
    log2,
    sqrt,
)

from .cacti_circuit import CactiCircuit
from .cacti_mats import CactiMat
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


class CactiHtree(CactiCircuit):
    """Energy-only port of Htree2 (in/out htree, htree2.cc).

    For ndbl==ndwl==2 (single mat, single bank), h=v=0 and the per-link loop
    never runs -- the tree carries exactly zero energy (matching CACTI). For
    ndwl/2>1 and/or ndbl/2>1 (multi-mat / multi-bank), this ports the closed-form
    wire-length geometry (in_htree/out_htree) and the per-link traversal loop
    (wire segment energy via CactiCircuit._wire_model's per-technology constant,
    plus NAND/tristate-driver energy at each repeater tap for the intra-bank/
    intra-mat level), plus the per-link horowitz delay accumulation (delay is
    NOT scaled by wire_bw/init_wire_bw -- only the energies are).

    FA/CAM search trees (search_in_bits/search_out_bits/search_tree): per
    bank.cc's is_fa branch, Bank builds htree_in_search/htree_out_search as
    DUPLICATES of htree_in_data/htree_out_data -- same Htree_type (so the same
    wire_bw/init_wire_bw, NOT a distinct search bit-width: the enum's
    Search_in_htree/Search_out_htree cases exist in htree2.cc's switch but are
    never actually passed as h_type anywhere in the codebase, confirmed by grep
    -- dead code) but constructed with uca_tree=true (treating the intra-bank
    tree as if it were inter-bank, closed-form geometry) and, for the in-tree
    only, search_tree=true (forces the per-link wire_bw doubling that's
    otherwise gated to `not uca_tree`). search_in_bits/search_out_bits
    (num_si/so_b_bank_per_port * SCHP) are passed into the constructor of ALL
    FIVE of a bank's H-trees (not just the two search ones) and enter the
    GEOMETRY formula only, added into the same term as add_bits -- confirmed
    line-by-line against htree2.cc's in_htree/out_htree; default 0 for every
    non-FA/CAM caller, exactly preserving prior behavior.
    """

    def __init__(
        self,
        cacti_params,
        wire_type_idx,
        overhead,
        mat_w,
        mat_h,
        add_bits,
        data_in_bits,
        data_out_bits,
        ndbl,
        ndwl,
        bits_selector,
        is_out_htree,
        uca_tree=False,
        is_dram=False,
        search_in_bits=0,
        search_out_bits=0,
        search_tree=False,
    ):
        # bits_selector: which of add_bits/data_in_bits/data_out_bits is this
        # H-tree's own wire_bw / init_wire_bw (Add_htree/Data_in_htree ->
        # in_htree; Data_out_htree -> out_htree). is_out_htree selects the
        # driver (input_nand vs output_buffer) and, per htree2.cc, is otherwise
        # geometrically identical to in_htree.
        super().__init__(cacti_params)
        assert ndbl >= 2 and ndwl >= 2
        assert bits_selector in ("add", "data_in", "data_out")

        self._is_dram = is_dram
        self._uca_tree = uca_tree
        min_w_nmos = self._tp["min_w_nmos"]
        min_w_pmos = self.pmos_to_nmos_sz_ratio() * min_w_nmos
        Vdd = self._tp["Vdd"]
        pton_size = self.pmos_to_nmos_sz_ratio()

        wire_geom = self._wire_layer_geom(wire_type_idx)
        point = self._wire_model(
            *wire_geom, overhead=overhead, is_dram=is_dram
        )
        repeater_spacing = point["repeater_spacing_um"]
        repeater_size = point["repeater_size"]
        Cpm = point["Cpm"]  # F/m, for the output-buffer wire_cap() term
        pitch = wire_geom[0]

        h = _htree_log2_floor(ndwl // 2)
        v = _htree_log2_floor(ndbl // 2)

        # ---- geometry (htree2.cc:262-315 / 468-522; identical for in/out htree) ----
        # add_bits is combined with search_in_bits/search_out_bits in every
        # branch (htree2.cc's "add_bits + (search_data_in_bits +
        # search_data_out_bits)" term) -- 0 for every non-FA/CAM caller.
        add_and_search_bits = add_bits + search_in_bits + search_out_bits
        if uca_tree:
            ht_temp = (
                mat_h * ndbl / 2
                + (add_and_search_bits + data_in_bits + data_out_bits)
                * pitch
                * 2
                * (1 - 0.5**h)
            ) / 2
            len_temp = (
                mat_w * ndwl / 2
                + (add_and_search_bits + data_in_bits + data_out_bits)
                * pitch
                * 2
                * (1 - 0.5**v)
            ) / 2
        elif ndwl == ndbl:
            ht_temp = (
                mat_h * ndbl / 2
                + add_and_search_bits * (ndbl / 2 - 1) * pitch
                + (data_in_bits + data_out_bits) * pitch * h
            ) / 2
            len_temp = (
                mat_w * ndwl / 2
                + add_and_search_bits * (ndwl / 2 - 1) * pitch
                + (data_in_bits + data_out_bits) * pitch * v
            ) / 2
        elif ndwl > ndbl:
            excess_part = h - v
            ht_temp = (
                mat_h * ndbl / 2
                + add_and_search_bits * ((ndbl / 2 - 1) + excess_part) * pitch
                + (data_in_bits + data_out_bits)
                * pitch
                * (2 * (1 - 0.5 ** (h - v)) + 0.5 ** (v - h) * v)
            ) / 2
            len_temp = (
                mat_w * ndwl / 2
                + add_and_search_bits * (ndwl / 2 - 1) * pitch
                + (data_in_bits + data_out_bits) * pitch * v
            ) / 2
        else:  # ndwl < ndbl
            excess_part = v - h
            ht_temp = (
                mat_h * ndbl / 2
                + add_and_search_bits * ((ndwl / 2 - 1) + excess_part) * pitch
                + (data_in_bits + data_out_bits) * pitch * h
            ) / 2
            len_temp = (
                mat_w * ndwl / 2
                + add_and_search_bits * ((ndwl / 2 - 1) + excess_part) * pitch
                + (data_in_bits + data_out_bits)
                * pitch
                * (h + 2 * (1 - 0.5 ** (v - h)))
            ) / 2

        self.area_h = ht_temp * 2
        self.area_w = len_temp * 2

        init_wire_bw = {
            "add": add_bits,
            "data_in": data_in_bits,
            "data_out": data_out_bits,
        }[bits_selector]
        wire_bw = init_wire_bw

        dynamic = 0.0
        delay = 0.0
        leakage = 0.0
        gate_leakage = 0.0
        search_dynamic = 0.0
        length = len_temp
        height = ht_temp / 2
        delay_per_um = point["delay_per_um"]
        repeater_spacing_um = point["repeater_spacing_um"]
        Vth = self._tp["Vth"]

        def driver_energy(s1, s2, l_eff, wbw):
            if is_out_htree:
                return self._output_buffer(
                    s1,
                    s2,
                    l_eff,
                    wbw,
                    min_w_nmos,
                    min_w_pmos,
                    Vdd,
                    pton_size,
                    Cpm,
                    delay_per_um,
                    repeater_spacing_um,
                    Vth,
                )
            else:
                return self._input_nand(
                    s1,
                    s2,
                    l_eff,
                    wbw,
                    min_w_nmos,
                    min_w_pmos,
                    Vdd,
                    pton_size,
                    delay_per_um,
                    repeater_spacing_um,
                    Vth,
                )

        # ---- traversal loop (htree2.cc:327-452 / 534-658) ----
        # len_temp / ht_temp (already set by the geometry step above) are REUSED
        # as persistent mutable loop state exactly as the C++ does: option 0 only
        # updates len_temp; option 1 updates both; option 2 only updates ht_temp.
        # s1/s2/s3 read whatever they currently hold, which can be stale from an
        # earlier iteration (or the pre-loop geometry value) -- replicated
        # verbatim, not "fixed".
        while v > 0 or h > 0:
            if h > v:
                len1, len2 = length, length / 2
                len_temp = length
                length = length / 2
                option = 0
                h -= 1
            elif v > 0 and h > 0:
                len1, len2 = length, height
                len_temp, ht_temp = length, height
                length = length / 2
                height = height / 2
                v -= 1
                h -= 1
                option = 1
            else:
                assert h == 0
                len1, len2 = height, height / 2
                ht_temp = height
                height = height / 2
                v -= 1
                option = 2

            dynamic += point["dynamic_per_um"] * len1
            delay += delay_per_um * len1
            # htree2.cc:373/576 -- in_htree scales by the LIVE wire_bw (before
            # this iteration's doubling below); out_htree scales by the
            # CONSTANT init_wire_bw instead -- a confirmed asymmetry, not a
            # transcription choice. Only read by the FA/CAM search trees
            # (bank.cc's htree_in_search/htree_out_search); harmless elsewhere.
            search_dynamic += (
                point["dynamic_per_um"]
                * len1
                * (init_wire_bw if is_out_htree else wire_bw)
            )
            leakage += point["leak_per_um"] * len1 * wire_bw
            gate_leakage += point["gleak_per_um"] * len1 * wire_bw
            if ((not uca_tree) and option == 2) or search_tree:
                wire_bw *= 2

            if not uca_tree:
                if len_temp > repeater_spacing:
                    s1 = repeater_size
                    l_eff = repeater_spacing
                else:
                    s1 = (len_temp / repeater_spacing) * repeater_size
                    l_eff = len_temp

                if ht_temp > repeater_spacing:
                    s2 = repeater_size
                else:
                    s2 = (len_temp / repeater_spacing) * repeater_size

                drv_delay, dyn, lk, gl = driver_energy(s1, s2, l_eff, wire_bw)
                delay += drv_delay
                dynamic += dyn
                leakage += lk
                gate_leakage += gl

            if option != 1:
                continue

            # second level (only when option == 1, i.e. this iteration took one
            # horizontal AND one vertical link): len2 is the vertical (option 1)
            # link's length, computed above as `height` (pre-halving).
            dynamic += point["dynamic_per_um"] * len2
            delay += delay_per_um * len2
            search_dynamic += (
                point["dynamic_per_um"]
                * len2
                * (init_wire_bw if is_out_htree else wire_bw)
            )
            leakage += point["leak_per_um"] * len2 * wire_bw
            gate_leakage += point["gleak_per_um"] * len2 * wire_bw

            # Second-level wtemp2 leakage/gate-leakage is added AGAIN here
            # (htree2.cc:421-431 -- identical in both the uca_tree and non-
            # uca_tree branches), on top of the unconditional add just above.
            # Replicated verbatim, not "fixed".
            leakage += point["leak_per_um"] * len2 * wire_bw
            gate_leakage += point["gleak_per_um"] * len2 * wire_bw

            if not uca_tree:
                wire_bw *= 2
                if ht_temp > repeater_spacing:
                    s3 = repeater_size
                    l_eff = repeater_spacing
                else:
                    s3 = (len_temp / repeater_spacing) * repeater_size
                    l_eff = ht_temp
                drv_delay, dyn, lk, gl = driver_energy(s2, s3, l_eff, wire_bw)
                delay += drv_delay
                dynamic += dyn
                leakage += lk
                gate_leakage += gl

        # htree2.cc:319/525 -- delay is never scaled by wire_bw/init_wire_bw,
        # unlike dynamic/leakage/gate_leakage below.
        self.delay = delay
        # htree2.cc:94-95 -- dynamic scaled by the FULL bit-width only once at the
        # end; leakage/gate-leakage were already scaled incrementally above by
        # wire_bw (which can exceed init_wire_bw due to the doublings).
        self.dynamic = dynamic * init_wire_bw
        self.leakage = leakage
        self.gate_leakage = gate_leakage
        # power.searchOp.dynamic (htree2.cc) -- accumulated directly during the
        # loop above, no final scaling. Only meaningful for the FA/CAM search
        # trees; left correct-but-unused for the other three (bank.cc never
        # reads their .searchOp.dynamic).
        self.search_dynamic = search_dynamic

    def _input_nand(
        self,
        s1,
        s2,
        l_eff,
        wire_bw,
        min_w_nmos,
        min_w_pmos,
        Vdd,
        pton_size,
        delay_per_um,
        repeater_spacing_um,
        Vth,
    ):
        # Port of Htree2::input_nand (htree2.cc:104-131), dynamic+leak+gleak+delay
        # (searchOp terms skipped -- energy-only, search bits always 0 here).
        nsize = s1 * (1 + pton_size) / (2 + pton_size)
        nsize = max(nsize, 1.0)
        wire_out_rise_time = (
            (delay_per_um * l_eff) * (repeater_spacing_um * 1e-6) / Vth
        )
        tc = (
            2
            * self.tr_R_on(nsize * min_w_nmos, "NCH", 1)
            * (
                self.drain_C(
                    nsize * min_w_nmos, "NCH", 1, 1, self._tp["cell_h_def"]
                )
                * 2
                + 2 * self.gate_C(s2 * (min_w_nmos + min_w_pmos), 0)
            )
        )
        nand_delay = self.horowitz(
            wire_out_rise_time, tc, Vth / Vdd, Vth / Vdd, True
        )
        dyn = (
            0.5
            * (
                2
                * self.drain_C(
                    pton_size * nsize * min_w_pmos,
                    "PCH",
                    1,
                    1,
                    self._tp["cell_h_def"],
                )
                + self.drain_C(
                    nsize * min_w_nmos, "NCH", 1, 1, self._tp["cell_h_def"]
                )
                + 2 * self.gate_C(s2 * (min_w_nmos + min_w_pmos), 0)
            )
            * Vdd
            * Vdd
        )
        leak = (
            wire_bw
            * self.cmos_Isub_leakage(
                min_w_nmos * (nsize * 2), min_w_pmos * nsize * 2, 2, "NAND"
            )
            * Vdd
        )
        gleak = (
            wire_bw
            * self.cmos_Ig_leakage(
                min_w_nmos * (nsize * 2), min_w_pmos * nsize * 2, 2, "NAND"
            )
            * Vdd
        )
        return nand_delay, dyn, leak, gleak

    def _output_buffer(
        self,
        s1,
        s2,
        l_eff,
        wire_bw,
        min_w_nmos,
        min_w_pmos,
        Vdd,
        pton_size,
        Cpm,
        delay_per_um,
        repeater_spacing_um,
        Vth,
    ):
        # Port of Htree2::output_buffer (htree2.cc:136-243), dynamic+leak+gleak+
        # delay. The two uca_tree leakage branches are textually identical in the
        # source (including power_gated_leakage's Vcc_min use, irrelevant here
        # since power-gating isn't modelled) -- single formula, no branch.
        cell_h_def = self._tp["cell_h_def"]
        size = s1 * (1 + pton_size) / (2 + pton_size + 1 + 2 * pton_size)
        gate_c_s2 = self.gate_C(s2 * (min_w_nmos + min_w_pmos), 0)
        wire_cap = Cpm * (l_eff * 1e-6)  # Wire::wire_cap(l_eff um -> m), F
        s_eff = (gate_c_s2 + wire_cap) / gate_c_s2
        tr_size = (
            self.gate_C(s1 * (min_w_nmos + min_w_pmos), 0)
            / 2.0
            / (s_eff * self.gate_C(min_w_pmos, 0))
        )
        size = max(size, 1.0)

        wire_out_rise_time = (
            (delay_per_um * l_eff) * (repeater_spacing_um * 1e-6) / Vth
        )
        res_nor = 2 * self.tr_R_on(size * min_w_pmos, "PCH", 1)
        res_ptrans = self.tr_R_on(tr_size * min_w_nmos, "NCH", 1)
        cap_nand_out = (
            self.drain_C(size * min_w_nmos, "NCH", 1, 1, cell_h_def)
            + self.drain_C(size * min_w_pmos, "PCH", 1, 1, cell_h_def) * 2
            + self.gate_C(tr_size * min_w_pmos, 0)
        )
        cap_ptrans_out = 2 * (
            self.drain_C(tr_size * min_w_pmos, "PCH", 1, 1, cell_h_def)
            + self.drain_C(tr_size * min_w_nmos, "NCH", 1, 1, cell_h_def)
        ) + self.gate_C(s1 * (min_w_nmos + min_w_pmos), 0)
        tc = res_nor * cap_nand_out + (res_nor + res_ptrans) * cap_ptrans_out
        buf_delay = self.horowitz(
            wire_out_rise_time, tc, Vth / Vdd, Vth / Vdd, True
        )

        dyn = (
            0.5
            * (
                2 * self.drain_C(size * min_w_pmos, "PCH", 1, 1, cell_h_def)
                + self.drain_C(size * min_w_nmos, "NCH", 1, 1, cell_h_def)
                + self.gate_C(tr_size * min_w_pmos, 0)
            )
            * Vdd
            * Vdd
        )  # nand
        dyn += (
            0.5
            * (
                self.drain_C(size * min_w_pmos, "PCH", 1, 1, cell_h_def)
                + self.drain_C(size * min_w_nmos, "NCH", 1, 1, cell_h_def)
                + self.gate_C(size * (min_w_nmos + min_w_pmos), 0)
            )
            * Vdd
            * Vdd
        )  # not
        dyn += (
            0.5
            * (
                self.drain_C(size * min_w_pmos, "PCH", 1, 1, cell_h_def)
                + 2 * self.drain_C(size * min_w_nmos, "NCH", 1, 1, cell_h_def)
                + self.gate_C(tr_size * (min_w_nmos + min_w_pmos), 0)
            )
            * Vdd
            * Vdd
        )  # nor
        dyn += (
            0.5
            * (
                (
                    self.drain_C(tr_size * min_w_pmos, "PCH", 1, 1, cell_h_def)
                    + self.drain_C(
                        tr_size * min_w_nmos, "NCH", 1, 1, cell_h_def
                    )
                )
                * 2
                + self.gate_C(s1 * (min_w_nmos + min_w_pmos), 0)
            )
            * Vdd
            * Vdd
        )  # output transistor

        leak = (
            self.cmos_Isub_leakage(
                min_w_nmos * tr_size * 2, min_w_pmos * tr_size * 2, 1, "INV"
            )
            * Vdd
            * wire_bw
        )
        leak += (
            self.cmos_Isub_leakage(
                min_w_nmos * size * 3, min_w_pmos * size * 3, 2, "NAND"
            )
            * Vdd
            * wire_bw
        )
        leak += (
            self.cmos_Isub_leakage(
                min_w_nmos * size * 3, min_w_pmos * size * 3, 2, "NOR"
            )
            * Vdd
            * wire_bw
        )
        gleak = (
            self.cmos_Ig_leakage(
                min_w_nmos * tr_size * 2, min_w_pmos * tr_size * 2, 1, "INV"
            )
            * Vdd
            * wire_bw
        )
        gleak += (
            self.cmos_Ig_leakage(
                min_w_nmos * size * 3, min_w_pmos * size * 3, 2, "NAND"
            )
            * Vdd
            * wire_bw
        )
        gleak += (
            self.cmos_Ig_leakage(
                min_w_nmos * size * 3, min_w_pmos * size * 3, 2, "NOR"
            )
            * Vdd
            * wire_bw
        )
        return buf_delay, dyn, leak, gleak


class CactiBank:
    """Port of Bank::compute_power_energy (bank.cc, non-FA/non-CAM path).

    Aggregates the single active mat's per-access dynamic energy (scaled by the
    number of active mats in the data-out direction) and the whole-bank leakage
    (scaled by the total number of mats), plus the inter-mat request/reply
    H-trees.
    """

    def __init__(self, mat, dp):
        self.mat = mat
        cfg = dp.cfg
        num_mats_h = dp.num_mats_h_dir
        num_mats_v = dp.num_mats_v_dir
        is_fa = dp.fully_assoc or dp.pure_cam

        # bank.cc:61-84: bank-facing bit widths (port-weighted).
        RWP, ERP, EWP = cfg.num_rw_ports, cfg.num_rd_ports, cfg.num_wr_ports
        self.total_addrbits = (
            dp.number_addr_bits_mat + dp.number_subbanks_decode
        ) * (RWP + ERP + EWP)
        self.datainbits = dp.num_di_b_bank_per_port * (RWP + EWP)
        self.dataoutbits = dp.num_do_b_bank_per_port * (RWP + ERP)
        if cfg.fast_access and not cfg.is_tag:
            self.dataoutbits *= cfg.data_assoc

        # bank.cc:72-84 (is_fa branch): search bit widths -- passed into the
        # constructor of ALL FIVE of a bank's H-trees (not just the two search
        # ones), where they enter the GEOMETRY term alongside add_bits. 0 for
        # non-FA/CAM, exactly preserving prior geometry.
        if is_fa:
            self.search_in_bits = (
                dp.num_si_b_bank_per_port * cfg.num_search_ports
            )
            self.search_out_bits = (
                dp.num_so_b_bank_per_port * cfg.num_search_ports
            )
        else:
            self.search_in_bits = 0
            self.search_out_bits = 0

        # Inter-mat H-trees inside the bank (ndbl/ndwl = num_mats_*_dir * 2).
        htree_kwargs = dict(
            cacti_params=mat._cacti_params,
            wire_type_idx=cfg.wire_os_mat_type,
            overhead=cfg.wt_overhead,
            mat_w=mat.area_w,
            mat_h=mat.area_h,
            add_bits=self.total_addrbits,
            data_in_bits=self.datainbits,
            data_out_bits=self.dataoutbits,
            ndbl=num_mats_v * 2,
            ndwl=num_mats_h * 2,
            uca_tree=False,
            search_in_bits=self.search_in_bits,
            search_out_bits=self.search_out_bits,
        )
        self.htree_in_add = CactiHtree(
            bits_selector="add", is_out_htree=False, **htree_kwargs
        )
        self.htree_in_data = CactiHtree(
            bits_selector="data_in", is_out_htree=False, **htree_kwargs
        )
        self.htree_out_data = CactiHtree(
            bits_selector="data_out", is_out_htree=True, **htree_kwargs
        )

        # bank.cc: area = the data-in H-tree's own geometry.
        self.area_w = self.htree_in_data.area_w
        self.area_h = self.htree_in_data.area_h

        # Task 4 (derived-area chain, Phase 30 follow-on): if mat.compute_area()
        # has been run (mat.derived_area_w/h present), rebuild the data-in
        # H-tree's geometry off the DERIVED mat dims instead of the given
        # mat.area_w/h, to get the bank's own derived area. Additive only --
        # self.area_w/h above (consumed by every existing caller) are untouched.
        # A full extra CactiHtree also computes energy/delay we don't need
        # here, but its geometry-only formula (lines 3386-3422) has no
        # cheaper standalone entry point, and this is a one-off per Bank.
        if hasattr(mat, "derived_area_w"):
            _derived_htree = CactiHtree(
                bits_selector="data_in",
                is_out_htree=False,
                **dict(
                    htree_kwargs,
                    mat_w=mat.derived_area_w,
                    mat_h=mat.derived_area_h,
                ),
            )
            self.derived_area_w = _derived_htree.area_w
            self.derived_area_h = _derived_htree.area_h

        if not is_fa:
            # bank.cc:151-173: dynamic scales with the active mats, leakage with all mats.
            self.read_dynamic = (
                mat._power.read.dynamic * dp.num_act_mats_hor_dir
                + self.htree_in_add.dynamic
                + self.htree_out_data.dynamic
            )
            self.read_leakage = (
                mat._power.read.leakage * dp.num_mats
                + self.htree_in_add.leakage
                + self.htree_in_data.leakage
                + self.htree_out_data.leakage
            )
            self.read_gate_leakage = (
                mat._power.read.gate_leakage * dp.num_mats
                + self.htree_in_add.gate_leakage
                + self.htree_in_data.gate_leakage
                + self.htree_out_data.gate_leakage
            )
            self.search_dynamic = 0.0
        else:
            # bank.cc:72-115 (is_fa branch) additionally routes search in/out
            # H-trees. Real CACTI quirk (confirmed line-by-line against
            # htree2.cc/bank.cc, not a guess): htree_in_search/htree_out_search
            # are NOT built with a distinct search bit-width -- the enum's
            # Search_in_htree/Search_out_htree cases exist in Htree2's
            # constructor switch but are never passed as h_type anywhere in the
            # codebase (grepped). Instead bank.cc constructs them as DUPLICATES
            # of htree_in_data/htree_out_data (same Data_in_htree/Data_out_htree
            # type, so the same wire_bw = datainbits/dataoutbits) but with
            # uca_tree=True (the closed-form inter-bank geometry, even though
            # these are intra-bank trees) and, for the in-tree only,
            # search_tree=True (bank.cc: `Htree2(..., Data_in_htree, true,
            # true)` vs `Htree2(..., Data_out_htree, true)` -- only ONE extra
            # bool for the out-tree, so its search_tree stays the default
            # False). search_tree forces the per-link wire_bw doubling that's
            # otherwise gated to uca_tree==false.
            search_htree_kwargs = dict(htree_kwargs)
            search_htree_kwargs["uca_tree"] = True
            self.htree_in_search = CactiHtree(
                bits_selector="data_in",
                is_out_htree=False,
                search_tree=True,
                **search_htree_kwargs,
            )
            self.htree_out_search = CactiHtree(
                bits_selector="data_out",
                is_out_htree=True,
                search_tree=False,
                **search_htree_kwargs,
            )

            # bank.cc:178-206 (is_fa branch): num_act_mats_hor_dir is always 1 for
            # FA, so mat.power.readOp.dynamic is added unscaled either way.
            self.read_dynamic = (
                mat._power.read.dynamic
                + self.htree_in_add.dynamic
                + self.htree_out_data.dynamic
            )
            self.read_leakage = (
                mat._power.read.leakage * dp.num_mats
                + self.htree_in_add.leakage
                + self.htree_in_data.leakage
                + self.htree_out_data.leakage
                + self.htree_in_search.leakage
                + self.htree_out_search.leakage
            )
            self.read_gate_leakage = (
                mat._power.read.gate_leakage * dp.num_mats
                + self.htree_in_add.gate_leakage
                + self.htree_in_data.gate_leakage
                + self.htree_out_data.gate_leakage
                + self.htree_in_search.gate_leakage
                + self.htree_out_search.gate_leakage
            )

            # bank.cc:182-187: search dynamic = mat.power.searchOp.dynamic *
            # num_mats PLUS four raw mat-level search components and
            # ml_to_ram_wl_drv's OWN dynamic energy, all WITHOUT a num_mats
            # multiply -- an asymmetry verified against bank.cc, replicated
            # verbatim -- plus the search-routing H-trees' OWN .search_dynamic
            # (htree2.cc's power.searchOp.dynamic, a distinct accumulator from
            # .dynamic -- see CactiHtree).
            self.search_dynamic = (
                mat._power.search.dynamic * dp.num_mats
                + mat._power_bl_precharge_eq_drv_search_dynamic
                + mat._power_sa_search_dynamic
                + mat._power_bitline_search_dynamic
                + mat._power_subarray_out_drv_search_dynamic
                + mat.ml_to_ram_wl_drv._power.read.dynamic
                + self.htree_in_search.search_dynamic
                + self.htree_out_search.search_dynamic
            )

    def compute_delays(self, inrisetime):
        # Port of Bank::compute_delays (bank.cc:138-140) -- a trivial
        # passthrough. Not called from __init__ -- purely additive, same
        # discipline as every Phase 26-32 increment. CactiMat.compute_delays'
        # own NotImplementedError for FA/CAM arrays propagates through
        # unguarded (see the native reference). CactiMat.compute_delays
        # accumulates several fields via += without resetting them (mirroring
        # the C++'s own member-field semantics) -- callers must not invoke
        # this (or mat.compute_delays directly) more than once per CactiMat
        # instance.
        return self.mat.compute_delays(inrisetime)


class CactiUCA:
    """Port of UCA::compute_power_energy (uca.cc, non-FA/non-CAM, non-DRAM).

    Adds the inter-bank routing H-trees to the bank power, then computes write
    energy: the burst-zeroed recomputation for is_tag==false, or the routing/
    H-tree-delta formula (uca.cc:272-284, never overwritten for tag arrays) for
    is_tag==true. A single R/W port is assumed and burst_len/int_prefetch_w
    cancel (the remaining-words-in-burst term is 0), matching the McPAT core
    arrays. Exposes the raw (pre-long-channel-reduction) per-bank read/write
    dynamic energy and subthreshold/gate leakage.
    """

    def __init__(self, mat, dp, nbanks):
        self.mat = mat
        self.dp = dp
        self.nbanks = nbanks
        self.bank = CactiBank(mat, dp)
        cfg = dp.cfg

        # Inter-bank routing H-trees. nbanks is split across the two dimensions
        # by the REAL bank aspect ratio (uca.cc:43): the tall-bank branch (ver =
        # 1<<(log2(nbanks)/2)) when bank.area_h > bank.area_w, else the wide-bank
        # branch (ver = 1<<(log2(nbanks) - log2(nbanks)//2)). For nbanks==1 both
        # give ndbl==ndwl==2 -> zero, matching the old hard-coded case.
        log2n = _htree_log2_floor(nbanks)
        if self.bank.area_h > self.bank.area_w:
            num_banks_ver = 1 << (log2n // 2)
        else:
            num_banks_ver = 1 << (log2n - log2n // 2)
        num_banks_hor = nbanks // num_banks_ver

        # uca.cc:61-65: bank-level bit widths -- the SAME dp fields as bank.cc's
        # total_addrbits/datainbits/dataoutbits (bank.cc and uca.cc both read
        # dp.number_addr_bits_mat/number_subbanks_decode/num_di_b_bank_per_port/
        # num_do_b_bank_per_port directly), so reuse CactiBank's already-computed
        # values rather than re-deriving them. Same for the search bit widths
        # (uca.cc:64-65 num_si/so_b_bank == bank.cc:76-77 searchin/outbits,
        # identical dp fields and SCHP) -- reuse CactiBank.search_in/out_bits.
        # uca.cc:83-96: for fully_assoc/pure_cam, num_si_b_bank/num_so_b_bank are
        # threaded into ALL FIVE UCA-level H-trees (not just the two search
        # ones), exactly like bank.cc's own is_fa branch (CactiBank above) --
        # they enter the geometry term alongside add_bits, so they change the
        # wire length (and hence dynamic/leakage) of htree_in_add/in_data/
        # out_data too, not just the new search trees. 0 for non-FA/CAM,
        # exactly preserving prior behavior.
        htree_kwargs = dict(
            cacti_params=mat._cacti_params,
            wire_type_idx=cfg.wire_os_mat_type,
            overhead=cfg.wt_overhead,
            mat_w=self.bank.area_w,
            mat_h=self.bank.area_h,
            add_bits=self.bank.total_addrbits,
            data_in_bits=self.bank.datainbits,
            data_out_bits=self.bank.dataoutbits,
            ndbl=num_banks_ver * 2,
            ndwl=num_banks_hor * 2,
            uca_tree=True,
            search_in_bits=self.bank.search_in_bits,
            search_out_bits=self.bank.search_out_bits,
        )
        self.htree_in_add = CactiHtree(
            bits_selector="add", is_out_htree=False, **htree_kwargs
        )
        self.htree_in_data = CactiHtree(
            bits_selector="data_in", is_out_htree=False, **htree_kwargs
        )
        self.htree_out_data = CactiHtree(
            bits_selector="data_out", is_out_htree=True, **htree_kwargs
        )

        # uca_org_t::find_area (cacti_interface.cc:117-129): cache_ht/cache_len
        # ("mem_array::height/width" for pure_ram/fully_assoc arrays, the shape
        # every McPAT structure feeding EXECU's bypass-network wire length --
        # core.cc:1183-1300 -- takes) is uca->area.h/.w = htree_in_data->area.h/.w
        # directly, unscaled by ArrayST's later sckt_co_eff/macro/chip_layout
        # overhead (that scaling is energy-only, area.cc never touches area).
        # For nbanks==1 (every bypass-feeding array in this project's anchors)
        # this reduces algebraically to exactly self.bank.area_h/area_w -- the
        # uca_tree geometry's log2(ndwl/2)==0 term vanishes -- so this is a
        # provably-exact restatement, not a new approximation.
        self.area_h = self.htree_in_data.area_h
        self.area_w = self.htree_in_data.area_w

        # Task 4 (derived-area chain, Phase 30 follow-on): same pattern as
        # CactiBank above -- if the bank exposes derived_area_w/h (i.e.
        # mat.compute_area() was run before this UCA was built), rebuild the
        # data-in H-tree's geometry off the bank's derived dims to get the
        # UCA's own derived area. Additive only -- self.area_w/h above are
        # untouched.
        if hasattr(self.bank, "derived_area_w"):
            _derived_uca_htree_kwargs = dict(
                htree_kwargs,
                mat_w=self.bank.derived_area_w,
                mat_h=self.bank.derived_area_h,
            )
            _derived_htree = CactiHtree(
                bits_selector="data_in",
                is_out_htree=False,
                **_derived_uca_htree_kwargs,
            )
            self.derived_area_w = _derived_htree.area_w
            self.derived_area_h = _derived_htree.area_h

        # power_routing_to_bank (uca.cc:243-259). Read uses in_add+out_data; write
        # uses in_add+in_data (uca.cc:243-244).
        routing_read = self.htree_in_add.dynamic + self.htree_out_data.dynamic
        routing_write = self.htree_in_add.dynamic + self.htree_in_data.dynamic
        routing_leak = (
            self.htree_in_add.leakage
            + self.htree_in_data.leakage
            + self.htree_out_data.leakage
        )
        routing_gleak = (
            self.htree_in_add.gate_leakage
            + self.htree_in_data.gate_leakage
            + self.htree_out_data.gate_leakage
        )

        num_act = dp.num_act_mats_hor_dir

        # uca.cc:86-95/245-248/262-264: fully_assoc/pure_cam additionally builds
        # htree_in_search/htree_out_search (SAME Data_in_htree/Data_out_htree
        # type as the non-search trees -- Search_in/out_htree are dead enum
        # values, per CactiBank's own docstring) at uca_tree=True with no
        # explicit search_tree bool (unlike bank.cc's htree_in_search, which
        # passes search_tree=True) -- uca.cc's 5 Htree2(...) calls all end at
        # the uca_tree arg, so search_tree defaults False for every one of
        # them. This is a no-op either way here: search_tree only matters via
        # `(not uca_tree and option==2) or search_tree`, and uca_tree=True
        # already for all 5 UCA-level trees, so the doubling never fires
        # regardless of search_tree. power_routing_to_bank.searchOp.dynamic =
        # htree_in_search.search_dynamic + htree_out_search.search_dynamic
        # (uca.cc:247, each tree's OWN searchOp.dynamic accumulator, not
        # .dynamic). readOp.leakage/gate_leakage also gain the two search
        # trees' contributions (uca.cc:262-264).
        # uca.cc:266: power.searchOp.dynamic += power_routing_to_bank.searchOp.
        # dynamic, on top of power.searchOp.dynamic = bank.power.searchOp.
        # dynamic (== bank.search_dynamic) set at uca.cc:241 (power = bank.
        # power). routing_search is exactly 0 for nbanks==1 (ndbl==ndwl==2 ->
        # the CactiHtree traversal loop never runs), so this doesn't change
        # the already-validated ITLB/DTLB/L1Directory(nbanks=1) anchors.
        if dp.fully_assoc or dp.pure_cam:
            self.htree_in_search = CactiHtree(
                bits_selector="data_in", is_out_htree=False, **htree_kwargs
            )
            self.htree_out_search = CactiHtree(
                bits_selector="data_out", is_out_htree=True, **htree_kwargs
            )
            routing_search = (
                self.htree_in_search.search_dynamic
                + self.htree_out_search.search_dynamic
            )
            routing_leak += (
                self.htree_in_search.leakage + self.htree_out_search.leakage
            )
            routing_gleak += (
                self.htree_in_search.gate_leakage
                + self.htree_out_search.gate_leakage
            )
            self.search = self.bank.search_dynamic + routing_search

        read = self.bank.read_dynamic + routing_read
        self.read = read
        # Write energy: uca.cc:272-284 ALWAYS computes power.writeOp.dynamic (no
        # is_tag gate there at all) as read + a bitline-read-for-write-swing swap
        # + the UCA-level routing read/write delta + the BANK-level H-tree
        # read/write delta [- sense amps if !is_dram]. uca.cc:406-421 then
        # OVERWRITES it with a burst-zeroed formula, but ONLY when is_tag==false --
        # so for is_tag==true, the 272-284 value (WITH the routing/H-tree deltas)
        # is what actually ships; only non-tag arrays get the simplified,
        # delta-free form. (Read energy happens to be identical either way, since
        # the burst term the 406-421 override adds is 0 for the single-burst
        # configs in scope -- that coincidence is why this looked at first like
        # "no formula for tag arrays" rather than "a different one".)
        if cfg.is_tag:
            self.write = (
                read
                - mat._bitline_read_mat * num_act
                + mat._bitline_write_mat * num_act
                - routing_read
                + routing_write
                + self.bank.htree_in_data.dynamic
                - self.bank.htree_out_data.dynamic
                - mat._sa_read_mat * num_act
            )
        else:
            self.write = (
                read
                - mat._bitline_read_mat * num_act
                + mat._bitline_write_mat * num_act
                - mat._sa_read_mat * num_act
            )

        self.leakage = self.bank.read_leakage + routing_leak
        self.gate_leakage = self.bank.read_gate_leakage + routing_gleak

    def compute_delays(self, inrisetime):
        """Port of UCA::compute_delays (uca.cc:128-233), non-FA/non-CAM,
        non-main-mem, non-DRAM SRAM branch only. Not called from __init__ --
        purely additive (see the native reference). Top-level callers should
        always invoke this as uca.compute_delays(0.0) -- the real C++ seeds
        the whole chain with inrisetime=0.0 from UCA::UCA's own constructor
        (uca.cc:105-106), never anything else. Calling this (or
        bank.compute_delays/mat.compute_delays) more than once on the same
        CactiUCA/CactiBank/CactiMat instance is NOT safe -- several
        downstream fields (e.g. CactiMat.delay_subarray_out_drv,
        CactiDriver.delay, CactiDecoder/CactiPredec*.delay) accumulate via
        += without resetting, exactly mirroring the C++'s own member-field
        semantics (a fresh object must be constructed per test case).

        Scope narrowing vs. the real C++ (verified by direct read of
        uca.cc:128-233, not guessed):
         - dp.is_main_mem is unreachable: CactiDynamicParameter.__init__
           already raises NotImplementedError for cfg.is_main_mem before a
           dp object can exist (cacti_dynamic_params.py), so uca.cc's two
           `if (dp.is_main_mem)` blocks (the access_time override at
           uca.cc:167-173, and the cycle_time/precharge_delay/
           multisubbank_interleave_cycle_time override at uca.cc:213-220)
           are omitted entirely here, not guarded against.
         - is_dram is never modelled anywhere in this port (no dp.is_dram
           attribute exists at all) -- the DRAM-only terms (delay_writeback
           at uca.cc:182, dram_refresh_period/dram_array_availability at
           uca.cc:226-230) are omitted, not guarded -- there is no live
           input path that could reach them.
         - fully_assoc : now ported -- see the `self.dp.fully_assoc`
           branches below for access_time (uca.cc:152-161) and cycle_time
           (uca.cc:191-200). pure_cam is still unreached: CactiMat.
           compute_delays raises NotImplementedError for it unconditionally,
           before self.bank.compute_delays below can return (0 pure_cam
           records exist in the corpus to validate against).
         - `access_time = bank.mat.delay_comparator` (uca.cc:149) is
           confirmed dead C++ code: it is unconditionally overwritten by
           either the fully_assoc or else branch immediately below in every
           reachable path (dp.is_main_mem being unreachable here removes the
           one other place it could have mattered) -- omitted.
         - `MAX(temp, bank.htree_in_add.max_unpipelined_link_delay)`
           (uca.cc:203-206, only when g_ip->rpters_in_htree==false) is
           confirmed dead in the C++ itself: max_unpipelined_link_delay is
           hardcoded to 0.0 in htree2.cc:63 ("//TODO", never assigned
           anywhere else, grepped) and temp is always a sum of non-negative
           delays, so this MAX is a permanent no-op regardless of
           rpters_in_htree's value -- omitted.
        """
        outrisetime = self.bank.compute_delays(inrisetime)

        mat = self.bank.mat
        # NOTE (Phase 33 finding): real CACTI unconditionally allocates
        # b_mux_predec (mat.cc:241-262) and unconditionally calls its
        # compute_delays (mat.cc:623), regardless of deg_bl_muxing. This
        # port's Phase 29 CactiMat.compute_delays only builds/calls it when
        # self._has_b_mux (deg_bl_muxing>1), a pre-existing scope narrowing
        # (see the native reference's own energy-side b_mux_dyn precedent at
        # cacti_component.py:2729/2801/2910-2911, "X if self._has_b_mux else
        # 0.0"). Reused here for consistency; b_mux_col_path is the "col_path"
        # candidate of the outer MAX below, so this only matters when it is
        # the winning arm -- see the native reference for the measured
        # confirmation that it never wins on the validated anchors.
        b_mux_delay = mat.b_mux_predec.delay if mat._has_b_mux else 0.0
        bit_mux_delay = mat.bit_mux_dec.delay if mat._has_b_mux else 0.0
        delay_array_to_mat = (
            self.htree_in_add.delay + self.bank.htree_in_add.delay
        )
        max_delay_before_row_decoder = delay_array_to_mat + mat.r_predec.delay
        self.delay_array_to_sa_mux_lev_1_decoder = (
            delay_array_to_mat
            + mat.sa_mux_lev_1_predec.delay
            + mat.sa_mux_lev_1_dec.delay
        )
        self.delay_array_to_sa_mux_lev_2_decoder = (
            delay_array_to_mat
            + mat.sa_mux_lev_2_predec.delay
            + mat.sa_mux_lev_2_dec.delay
        )
        delay_inside_mat = mat.row_dec.delay + mat.delay_bitline + mat.delay_sa

        self.delay_before_subarray_output_driver = max(
            max(
                max_delay_before_row_decoder + delay_inside_mat,  # row_path
                delay_array_to_mat
                + b_mux_delay
                + bit_mux_delay
                + mat.delay_sa,
            ),  # col_path
            max(
                self.delay_array_to_sa_mux_lev_1_decoder,
                self.delay_array_to_sa_mux_lev_2_decoder,
            ),
        )  # sa_mux paths
        self.delay_from_subarray_out_drv_to_out = (
            mat.delay_subarray_out_drv_htree
            + self.bank.htree_out_data.delay
            + self.htree_out_data.delay
        )

        # access_time (uca.cc:149-161): `= bank.mat.delay_comparator` (dead,
        # already omitted above) is unconditionally overwritten by exactly
        # one of these two arms -- fully_assoc replaces the whole
        # delay_before_subarray_output_driver-based formula (not just adds a
        # term), routing through the CAM/matchline chain instead .
        if self.dp.fully_assoc:
            ram_delay_inside_mat = mat.delay_bitline + mat.delay_matchchline
            self.access_time = (
                self.htree_in_add.delay
                + self.bank.htree_in_add.delay
                + ram_delay_inside_mat
                + self.delay_from_subarray_out_drv_to_out
            )
        else:
            self.access_time = (
                self.delay_before_subarray_output_driver
                + self.delay_from_subarray_out_drv_to_out
            )

        # cycle_time (uca.cc:191-200): the FA arm uses a different term set
        # (CAM restore/reset terms instead of r_predec.delay) and, notably,
        # omits r_predec.delay from the MAX entirely .
        if self.dp.fully_assoc:
            ram_delay_inside_mat = mat.delay_bitline + mat.delay_matchchline
            temp = (
                ram_delay_inside_mat
                + mat.delay_cam_sl_restore
                + mat.delay_cam_ml_reset
                + mat.delay_bl_restore
                + mat.delay_hit_miss_reset
                + mat.delay_wl_reset
            )
            temp = max(temp, b_mux_delay)
            temp = max(temp, mat.sa_mux_lev_1_predec.delay)
            temp = max(temp, mat.sa_mux_lev_2_predec.delay)
        else:
            temp = delay_inside_mat + mat.delay_wl_reset + mat.delay_bl_restore
            temp = max(temp, mat.r_predec.delay)
            temp = max(temp, b_mux_delay)
            temp = max(temp, mat.sa_mux_lev_1_predec.delay)
            temp = max(temp, mat.sa_mux_lev_2_predec.delay)
        self.cycle_time = temp

        self.multisubbank_interleave_cycle_time = max(
            max_delay_before_row_decoder,
            self.delay_from_subarray_out_drv_to_out,
        )
        self.precharge_delay = 0.0  # is_main_mem-only in C++, unreachable here

        return outrisetime


class CactiArrayST:
    """Port of the ArrayST wrapper scalings (array.cc:240-307), energy-only.

    McPAT wraps every CACTI array in an ArrayST that applies, after the optimizer
    has chosen a partition:
      * a socket coupling / layout-overhead multiplier to the per-access DYNAMIC
        energy -- this scaled value is exactly what dump_ae_to_xml writes to
        activation_energies.xml (the energy coefficient the McPAT power model uses);
      * an nbanks multiplier to leakage, and a longer-channel device reduction to
        produce longer_channel_leakage (leakage itself is NOT scaled by the layout
        overhead -- pppm_t = {overhead, 1, 1, overhead}, so leakage uses y[1]=1).

    The socket/overhead/long-channel constants are read from the array's own
    CactiParams (cacti_tech_params.py: 'sckt_coeff', 'macro_layout_overhead',
    'chip_layout_overhead', 'long_channel_leakage_reduction'), interpolated per
    node/device-type exactly like every other tech parameter, matching
    technology.cc's per-node table (sckt_co_eff/layout overhead are node-only;
    long_channel_leakage_reduction is node- AND device-type-dependent).
    """

    def __init__(self, uca, nbanks, device_ty="core", core_ooo=False):
        self.uca = uca
        self.nbanks = nbanks
        cacti_params = uca.mat._cacti_params
        self._cacti_params = cacti_params
        tp = cacti_params._tech_params
        overhead = tp["macro_layout_overhead"] * tp["chip_layout_overhead"]
        sckt = tp["sckt_coeff"]

        # mem_array::height/width (cache_ht/cache_len, cacti_interface.cc:117-129)
        # -- unscaled by ArrayST's dynamic-energy overhead/leakage nbanks scaling
        # above (area.cc never touches area), fed to EXECU's bypass-network wire
        # length (core.cc:1183-1300, e.g. rfu->int_regfile_height).
        self.area_h = uca.area_h
        self.area_w = uca.area_w

        # Dynamic per-access energy exactly as dumped to activation_energies.xml.
        self.read = uca.read * sckt * overhead
        self.write = uca.write * sckt * overhead
        # powerDef::operator* (io.cc:843-851) applies the SAME pppm_t scale to
        # searchOp as read/writeOp -- FA/CAM arrays only.
        if uca.dp.fully_assoc or uca.dp.pure_cam:
            self.search = uca.search * sckt * overhead

        # Leakage: nbanks scaling only (no layout overhead). longer_channel applies
        # the McPAT device reduction (basic_components.cc:longer_channel_device_reduction).
        # gate_leakage is NOT scaled by nbanks (array.cc:234-300: only leakage and
        # power_gated_leakage get *= l_ip.nbanks; gate_leakage's pppm_t slot is 1).
        self.leakage = uca.leakage * nbanks
        self.gate_leakage = uca.gate_leakage
        self.longer_channel_leakage = (
            self.leakage * self._long_channel_reduction(device_ty, core_ooo)
        )

    def _long_channel_reduction(self, device_ty, core_ooo):
        # Guards a real bug: a caller passing the C++ Core_device enum's int
        # form instead of this API's string enum silently fell through to
        # the "llc" branch below (pct=1.0), not an error.
        if device_ty not in ("core", "uncore", "llc"):
            raise ValueError(
                f"device_ty must be one of 'core'/'uncore'/'llc' (a string), "
                f"got {device_ty!r} -- pass the string enum, not the C++ "
                f"Core_device int (see sweep_harness.py's DEVICE_TY_MAP)"
            )
        peri = self._cacti_params._tech_params[
            "long_channel_leakage_reduction"
        ]
        if device_ty == "core":
            pct = 0.56 if core_ooo else 0.8
        elif device_ty == "uncore":
            pct = 0.82
        else:  # llc
            pct = 1.0
        return (1.0 - pct) + pct * peri
