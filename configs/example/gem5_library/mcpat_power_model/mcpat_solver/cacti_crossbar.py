"""Literal Python port of ``mcpat/cacti/crossbar.cc`` (the ``Crossbar`` class).

A matrix (tri-state buffer) crossbar as instantiated by ``Router::cb_stats``
(``mcpat/cacti/router.cc:206``) via ``Crossbar c_b(I, O, flit_size); c_b.compute_power();``.
Consumed by Task 5 (Router) as ``CactiCrossbar(circuit, I, O, flit_size)``.

Every wire in ``compute_power`` is a ``Wire(g_ip->wt, len)`` built AFTER
``Wire winit(4, 4)`` re-initialises the static ``Wire::global*`` members with
width/space scaling ``(4, 4)`` -- the port has no global ``Wire`` state, so the
``_wire_model`` calls that back ``w1``/``w2``/``wdriver`` (and the ``w1`` inside
``output_buffer``) pass ``w_scale=4.0, s_scale=4.0`` explicitly.

For the Embedded NoC path (``NoC::NoC``, ``mcpat/noc.cc:60-65``) ``g_ip->wt`` is
``Global_30`` and ``wire_os_mat_type`` is ``1`` -- i.e. the 30%-delay-overhead
repeater point on the semi-global ("inside_mat") wire tier: ``_wire_model`` with
``overhead=30`` and the ``_wire_layer_geom(1)`` geometry.

The CB_ADJ recursion bug (``crossbar.cc:106-147``) is replicated verbatim, NOT
fixed: when ``aspect_ratio_cb < ASPECT_THRESHOLD`` and both port counts exceed 2,
``CB_ADJ`` is bumped by 0.2 and ``compute_power()`` recurses; on unwind, the outer
frame overwrites ``power.readOp.*`` / ``delay`` from its own stale (pre-recursion)
``w1``/``w2``, while ``area.w`` / ``area.h`` -- member-assigned before the
recursion, never re-touched -- retain the DEEPEST frame's values.  ``delay`` is
formed in the outer frame but reads the (now recursion-overwritten, i.e. deepest)
``area_w`` / ``area_h``.
"""

from math import ceil

try:
    from .cacti_component import CactiComponent
except ImportError:  # vendored copy uses package-relative imports
    from cacti_component import CactiComponent

ASPECT_THRESHOLD = 0.8  # crossbar.cc:34  (#define ASPECT_THRESHOLD .8)
ADJ = 1  # crossbar.cc:35  (#define ADJ 1)


class CactiCrossbar(CactiComponent):
    """Port of ``Crossbar``.  ``__init__`` runs ``compute_power()``.

    Attributes after construction:
      ``area_w`` / ``area_h`` (um), ``delay`` (s),
      ``read_dynamic`` (J) / ``read_leakage`` (W) / ``read_gate_leakage`` (W),
      ``tri_inp_cap`` / ``tri_out_cap`` / ``tri_ctr_cap`` / ``tri_int_cap`` (F),
      ``CB_ADJ`` (final value) + ``cb_adj_recursions`` (recursion count),
      ``aspect_ratio_cb`` (the FIRST / pre-recursion frame's value).
    Method: ``output_buffer() -> float`` (sum of the three tri caps).
    """

    def __init__(self, circuit, n_inp, n_out, flit_size, device="peri_global"):
        if device != "peri_global":
            # crossbar.cc always uses &(g_tp.peri_global); the port has no
            # separate device object (peri_global == the SRAM tech type in every
            # McPAT path -- cacti_tech_params.py:233).  Guard rather than
            # silently ignore (cf. Phase 49 device_ty int-vs-string fallthrough).
            raise ValueError(
                f"CactiCrossbar device must be 'peri_global', got {device!r}"
            )
        super().__init__("Crossbar", circuit._cacti_params)

        self.n_inp = float(n_inp)
        self.n_out = float(n_out)
        self.flit_size = float(flit_size)

        # ctor body (crossbar.cc:37-47)
        self._minw = self._tp["min_w_nmos"]  # g_tp.min_w_nmos_
        self.min_w_pmos = self._tp["n_to_p_eff_curr_drv_ratio"] * self._minw
        self.Vdd = self._tp["Vdd"]  # dt->Vdd
        self._Vth = self._tp["Vth"]  # deviceType->Vth
        self.CB_ADJ = 1.0  # crossbar.cc:46
        self.cb_adj_recursions = 0

        self._cell_h_def = self._tp["cell_h_def"]
        # g_tp.wire_outside_mat.* (filled from g_ip->wire_os_mat_type == 1 for the
        # Embedded NoC -> the semi-global / "inside_mat" wire tier).
        self._wire_pitch = self._wp["pitch_inside_mat"]
        self._wire_R_per_um = self._wp["R_per_micron_inside_mat"]
        self._wire_C_per_um = self._wp["C_per_micron_inside_mat"]
        self._wire_geom = circuit._wire_layer_geom(1)

        # set by output_buffer()
        self.TriS1 = 0.0
        self.TriS2 = 0.0
        self.tri_inp_cap = 0.0
        self.tri_out_cap = 0.0
        self.tri_ctr_cap = 0.0
        self.tri_int_cap = 0.0

        self.area_w = 0.0
        self.area_h = 0.0
        self.delay = 0.0
        self.read_dynamic = 0.0
        self.read_leakage = 0.0
        self.read_gate_leakage = 0.0
        self.aspect_ratio_cb = None

        self.compute_power()

    # ------------------------------------------------------------------
    # Wire(g_ip->wt = Global_30, len) built under the winit(4, 4) static state.
    # ------------------------------------------------------------------
    def _wm(self):
        # Pure function of the tier-1 geometry + (overhead=30, 4, 4); _wire_model
        # memoises it on the CactiParams instance, so this is a dict lookup after
        # the first call.
        return self._wire_model(
            *self._wire_geom, overhead=30, w_scale=4.0, s_scale=4.0
        )

    def _wire_stats(self, length_um):
        """Port of ``Wire(Global_30, length_um)``'s member reads used by the
        crossbar: total dynamic (J) / leakage (W) / gate_leakage (W) over the
        wire, plus the (winit-scaled) repeater size and spacing (um).

        ``calculate_wire_stats`` (wire.cc:194-204) sets
        ``power.readOp.X = global_30.power.readOp.X * wire_length`` with
        ``wire_length`` in metres; ``global_30.power.readOp.X`` is per-metre, so
        ``per_um * length_um`` is the same product.
        """
        wm = self._wm()
        return {
            "dyn": wm["dynamic_per_um"] * length_um,
            "leak": wm["leak_per_um"] * length_um,
            "gleak": wm["gleak_per_um"] * length_um,
            "repeater_size": wm["repeater_size"],
            "repeater_spacing": wm["repeater_spacing_um"],
        }

    def _wire_signal_rise_time(self):
        """Port of ``Wire::signal_rise_time`` (wire.cc:259-279) -- a pure
        tech-parameter quantity (independent of wire length / winit scaling), so
        the same value backs w1/w2/wdriver."""
        minw = self._minw
        min_w_pmos = self.min_w_pmos
        cell_h = self._cell_h_def
        Vdd, Vth = self.Vdd, self._Vth
        timeconst = (
            self.drain_C(minw, "NCH", 1, 1, cell_h)
            + self.drain_C(min_w_pmos, "PCH", 1, 1, cell_h)
            + self.gate_C(min_w_pmos + minw, 0)
        ) * self.tr_R_on(minw, "NCH", 1)
        rt = self.horowitz(0, timeconst, Vth / Vdd, Vth / Vdd, True) / Vth
        timeconst = (
            self.drain_C(minw, "NCH", 1, 1, cell_h)
            + self.drain_C(min_w_pmos, "PCH", 1, 1, cell_h)
            + self.gate_C(min_w_pmos + minw, 0)
        ) * self.tr_R_on(min_w_pmos, "PCH", 1)
        ft = self.horowitz(rt, timeconst, Vth / Vdd, Vth / Vdd, False) / (
            Vdd - Vth
        )
        return ft

    # ------------------------------------------------------------------
    # Crossbar::output_buffer  (crossbar.cc:51-89)
    # ------------------------------------------------------------------
    def output_buffer(self):
        minw = self._minw
        min_w_pmos = self.min_w_pmos
        cell_h = self._cell_h_def

        l_eff = self.n_inp * self.flit_size * self._wire_pitch
        w1 = self._wire_stats(l_eff)
        rep_size = w1["repeater_size"]
        rep_spacing = w1["repeater_spacing"]
        s1 = rep_size * (
            l_eff * ADJ / rep_spacing if l_eff < rep_spacing else ADJ
        )
        pton_size = self._tp["n_to_p_eff_curr_drv_ratio"]
        # input capacitance of the wire driver == input cap of nand + nor
        self.TriS1 = s1 * (1 + pton_size) / (2 + pton_size + 1 + 2 * pton_size)
        self.TriS2 = s1  # driver transistor

        if self.TriS1 < 1:
            self.TriS1 = 1

        TriS1, TriS2 = self.TriS1, self.TriS2

        input_cap = self.gate_C(
            TriS1 * (2 * min_w_pmos + minw), 0
        ) + self.gate_C(TriS1 * (min_w_pmos + 2 * minw), 0)
        # crossbar.cc:75-80  (note: the *NCH* drain_C_ calls pass a pmos width --
        # a real CACTI quirk, replicated verbatim).
        self.tri_int_cap = (
            self.drain_C(TriS1 * minw, "NCH", 1, 1, cell_h)
            + self.drain_C(TriS1 * min_w_pmos, "PCH", 1, 1, cell_h) * 2
            + self.gate_C(TriS2 * minw, 0)
            + self.drain_C(TriS1 * min_w_pmos, "NCH", 1, 1, cell_h) * 2
            + self.drain_C(TriS1 * min_w_pmos, "PCH", 1, 1, cell_h)
            + self.gate_C(TriS2 * min_w_pmos, 0)
        )
        output_cap = self.drain_C(
            TriS2 * minw, "NCH", 1, 1, cell_h
        ) + self.drain_C(TriS2 * min_w_pmos, "PCH", 1, 1, cell_h)
        ctr_cap = self.gate_C(TriS2 * (min_w_pmos + minw), 0)

        self.tri_inp_cap = input_cap
        self.tri_out_cap = output_cap
        self.tri_ctr_cap = ctr_cap
        return input_cap + output_cap + ctr_cap

    # ------------------------------------------------------------------
    # Crossbar::compute_power  (crossbar.cc:91-147)
    # ------------------------------------------------------------------
    def compute_power(self):
        minw = self._minw
        min_w_pmos = self.min_w_pmos
        cell_h = self._cell_h_def
        Vdd, Vth = self.Vdd, self._Vth

        # Wire winit(4, 4);  -- modelled by the (w_scale=4, s_scale=4) args
        # threaded through every _wire_stats / _wm call below.
        tri_cap = self.output_buffer()
        assert tri_cap > 0

        # area of a tristate logic (crossbar.cc:98-101)
        g_area = self.compute_gate_area(
            "INV", 1, self.TriS2 * minw, self.TriS2 * min_w_pmos, cell_h
        )
        g_area *= 2  # to model area of output transistors
        g_area += self.compute_gate_area(
            "NAND", 2, self.TriS1 * 2 * minw, self.TriS1 * min_w_pmos, cell_h
        )
        g_area += self.compute_gate_area(
            "NOR", 2, self.TriS1 * minw, self.TriS1 * 2 * min_w_pmos, cell_h
        )

        width = g_area / (self.CB_ADJ * cell_h)  # per tristate
        # effective no. of tristate buffers that need to be laid side by side
        ntri = int(ceil(cell_h / self._wire_pitch))
        wire_len = max(
            width * ntri * self.n_out,
            self.flit_size * self._wire_pitch * self.n_out,
        )
        w1 = self._wire_stats(wire_len)

        self.area_w = wire_len
        self.area_h = (
            self._wire_pitch * self.n_inp * self.flit_size * self.CB_ADJ
        )
        w2 = self._wire_stats(self.area_h)

        aspect_ratio_cb = (self.area_h / self.area_w) * (
            self.n_out / self.n_inp
        )
        if aspect_ratio_cb > 1:
            aspect_ratio_cb = 1 / aspect_ratio_cb
        if (
            self.aspect_ratio_cb is None
        ):  # keep the FIRST / pre-recursion value
            self.aspect_ratio_cb = aspect_ratio_cb

        if aspect_ratio_cb < ASPECT_THRESHOLD:
            if self.n_out > 2 and self.n_inp > 2:
                self.CB_ADJ += 0.2
                self.cb_adj_recursions += 1
                if self.CB_ADJ < 4:
                    self.compute_power()

        # --- outer-frame overwrite (the mandated bug) -------------------------
        # w1 / w2 are THIS frame's locals (pre-recursion geometry); area_w /
        # area_h below are read live -> the recursion has already overwritten
        # them with the DEEPEST frame's values.
        self.read_dynamic = (
            w1["dyn"]
            + w2["dyn"]
            + (
                self.tri_inp_cap * self.n_out
                + self.tri_out_cap * self.n_inp
                + self.tri_ctr_cap
                + self.tri_int_cap
            )
            * Vdd
            * Vdd
        ) * self.flit_size

        self.read_leakage = (
            self.n_inp
            * self.n_out
            * self.flit_size
            * (
                self.cmos_Isub_leakage(
                    minw * self.TriS2 * 2,
                    min_w_pmos * self.TriS2 * 2,
                    1,
                    "INV",
                )
                * Vdd
                + self.cmos_Isub_leakage(
                    minw * self.TriS1 * 3,
                    min_w_pmos * self.TriS1 * 3,
                    2,
                    "NAND",
                )
                * Vdd
                + self.cmos_Isub_leakage(
                    minw * self.TriS1 * 3,
                    min_w_pmos * self.TriS1 * 3,
                    2,
                    "NOR",
                )
                * Vdd
                + w1["leak"]
                + w2["leak"]
            )
        )

        self.read_gate_leakage = (
            self.n_inp
            * self.n_out
            * self.flit_size
            * (
                self.cmos_Ig_leakage(
                    minw * self.TriS2 * 2,
                    min_w_pmos * self.TriS2 * 2,
                    1,
                    "INV",
                )
                * Vdd
                + self.cmos_Ig_leakage(
                    minw * self.TriS1 * 3,
                    min_w_pmos * self.TriS1 * 3,
                    2,
                    "NAND",
                )
                * Vdd
                + self.cmos_Ig_leakage(
                    minw * self.TriS1 * 3,
                    min_w_pmos * self.TriS1 * 3,
                    2,
                    "NOR",
                )
                * Vdd
                + w1["gleak"]
                + w2["gleak"]
            )
        )

        # delay calculation (crossbar.cc:139-144)
        l_eff = self.n_inp * self.flit_size * self._wire_pitch
        wdriver = self._wire_stats(l_eff)
        res = self._wire_R_per_um * (self.area_w + self.area_h) + self.tr_R_on(
            minw * wdriver["repeater_size"], "NCH", 1
        )
        cap = (
            self._wire_C_per_um * (self.area_w + self.area_h)
            + self.n_out * self.tri_inp_cap
            + self.n_inp * self.tri_out_cap
        )
        self.delay = self.horowitz(
            self._wire_signal_rise_time(),
            res * cap,
            Vth / Vdd,
            Vth / Vdd,
            True,
        )
        # Wire wreset();  -- most-vexing-parse no-op, ignored (crossbar.cc:146)
