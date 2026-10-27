"""Literal Python port of ``mcpat/cacti/arbiter.cc`` (the ``Arbiter`` class).

A matrix-arbiter as instantiated twice per router by ``Router::init_router``
(``mcpat/cacti/router.cc:234-237``)::

    Arbiter vcarb (vc_count, flit_size, buffer.area.w);   vcarb.compute_power();
    Arbiter cbarb (n_out,    flit_size, crossbar.area.w);  cbarb.compute_power();

Consumed by Task 5 (Router) as ``CactiArbiter(circuit, n_req, flit_size,
output_len_um)``; the router combines them as ``vcarb.dyn*I + cbarb.dyn*O``.

Replicated CACTI quirks (NOT fixed):

* **Raw ``g_ip->F_sz_um``** (``arbiter.cc:44``): ``technology = g_ip->F_sz_um``
  with NO ``*1e-6`` -- unlike ``router.cc`` which multiplies by ``1e-6``.  So
  every ``NTn1``/``PTn1``/``NTi``/``PTi``/``NTtr``/``PTtr`` here is a transistor
  width in *micrometres*, and is handed straight to ``gate_C`` / ``drain_C``
  (which want um -- ``router.cc::gate_cap(w)`` does ``gate_C(w*1e6, 0)``, i.e.
  converts m->um, so arbiter's direct um call is the matching convention).
  ``self._node_um`` == ``node_nm / 1000`` == ``g_ip->F_sz_um`` exactly.
* **``drain_C_`` channel arg 0 vs 1**: ``const.h:111-112`` define ``NCH 1`` /
  ``PCH 0``.  ``basic_circuit.cc:225`` ``if (nchannel)`` takes the
  ``(1 - ratio_p_to_n)`` fold branch, which is the port's ``channel == "NCH"``
  branch (``cacti_circuit.py:65``).  So ``arbiter.cc``'s ``drain_C_(X, 0, ...)``
  -> ``drain_C(X, "PCH", ...)`` and ``drain_C_(X, 1, ...)`` ->
  ``drain_C(X, "NCH", ...)`` -- matching ``crossbar.cc``'s symbolic
  ``NCH``/``PCH`` -> port ``"NCH"``/``"PCH"``.  It looks like a typo that
  ``NTi`` (an nmos width) is folded with the ``"PCH"`` ratio and ``PTi`` (a
  pmos width) with ``"NCH"`` -- that is exactly what ``arbiter.cc`` does.
* **leakage args are width x width** (``arbiter.cc:90-95``):
  ``cmos_Isub_leakage(g_tp.min_w_nmos_ * NTn1 * 2, ...)`` -- ``NTn1`` is
  already a um width, so this is ``0.06 * 0.27 * 2``; replicated verbatim.
* The ``//FIXME include priority table leakage`` term is simply absent -- port
  what is there.
* ``flit_size`` is stored but never read by ``compute_power`` (``arbiter.cc``
  only prints it).
"""

try:
    from .cacti_component import CactiComponent
except ImportError:  # vendored copy uses package-relative imports
    from cacti_component import CactiComponent


class CactiArbiter(CactiComponent):
    """Port of ``Arbiter``.  ``__init__`` runs ``compute_power()``.

    Attributes after construction:
      ``read_dynamic`` (J) / ``read_leakage`` (W) / ``read_gate_leakage`` (W).
    Methods: ``arb_req()``, ``arb_pri()``, ``arb_grant()``, ``arb_int()``,
      ``crossbar_ctrline()``, ``transmission_buf_ctrcap()``, ``Cw3(length_m)``.
    """

    def __init__(
        self, circuit, n_req, flit_size, output_len_um, device="peri_global"
    ):
        if device != "peri_global":
            # arbiter.cc always uses &(g_tp.peri_global); the port has no
            # separate device object (peri_global == the SRAM tech type in
            # every McPAT path -- cacti_tech_params.py:233).  Guard rather than
            # silently ignore (cf. cacti_crossbar.py's identical guard, and
            # Phase 49's device_ty int-vs-string fallthrough).
            raise ValueError(
                f"CactiArbiter device must be 'peri_global', got {device!r}"
            )
        super().__init__("Arbiter", circuit._cacti_params)

        # ctor body (arbiter.cc:34-53)
        self.R = float(n_req)  # R(n_req)
        self.flit_size = float(flit_size)  # stored, unused
        self.o_len = float(output_len_um)  # o_len(output_len)

        self._minw = self._tp["min_w_nmos"]  # g_tp.min_w_nmos_
        self.min_w_pmos = self._tp["n_to_p_eff_curr_drv_ratio"] * self._minw
        self.Vdd = self._tp["Vdd"]  # dt->Vdd
        self._cell_h_def = self._tp["cell_h_def"]  # g_tp.cell_h_def

        technology = self._node_um  # g_ip->F_sz_um (RAW, no *1e-6)
        self.NTn1 = 13.5 * technology / 2
        self.PTn1 = 76 * technology / 2
        self.NTn2 = 13.5 * technology / 2
        self.PTn2 = 76 * technology / 2
        self.NTi = 12.5 * technology / 2
        self.PTi = 25 * technology / 2
        self.NTtr = 10 * technology / 2  # transmission gate's nmos tr. length
        self.PTtr = 20 * technology / 2  # pmos tr. length

        self.read_dynamic = 0.0
        self.read_leakage = 0.0
        self.read_gate_leakage = 0.0

        self.compute_power()

    # ------------------------------------------------------------------
    # arbiter.cc:57-63
    # ------------------------------------------------------------------
    def arb_req(self):
        ch = self._cell_h_def
        return (
            (self.R - 1)
            * (2 * self.gate_C(self.NTn1, 0) + self.gate_C(self.PTn1, 0))
            + 2 * self.gate_C(self.NTn2, 0)
            + self.gate_C(self.PTn2, 0)
            + self.gate_C(self.NTi, 0)
            + self.gate_C(self.PTi, 0)
            + self.drain_C(self.NTi, "PCH", 1, 1, ch)
            + self.drain_C(self.PTi, "NCH", 1, 1, ch)
        )

    # ------------------------------------------------------------------
    # arbiter.cc:65-70  (switching capacitance of flip-flop is ignored)
    # ------------------------------------------------------------------
    def arb_pri(self):
        return 2 * (2 * self.gate_C(self.NTn1, 0) + self.gate_C(self.PTn1, 0))

    # ------------------------------------------------------------------
    # arbiter.cc:73-77
    # ------------------------------------------------------------------
    def arb_grant(self):
        ch = self._cell_h_def
        return (
            self.drain_C(self.NTn1, "PCH", 1, 1, ch) * 2
            + self.drain_C(self.PTn1, "NCH", 1, 1, ch)
            + self.crossbar_ctrline()
        )

    # ------------------------------------------------------------------
    # arbiter.cc:79-84
    # ------------------------------------------------------------------
    def arb_int(self):
        ch = self._cell_h_def
        return (
            self.drain_C(self.NTn1, "PCH", 1, 1, ch) * 2
            + self.drain_C(self.PTn1, "NCH", 1, 1, ch)
            + 2 * self.gate_C(self.NTn2, 0)
            + self.gate_C(self.PTn2, 0)
        )

    # ------------------------------------------------------------------
    # arbiter.cc:100-105 -- wire cap with triple width/spacing.
    # C++: Wire wc(g_ip->wt, length, 1, 3, 3); wc.wire_cap(length, true);
    # ------------------------------------------------------------------
    def Cw3(self, length):
        return self.wire_cap(
            length, call_from_outside=True, w_scale=3.0, s_scale=3.0
        )

    # ------------------------------------------------------------------
    # arbiter.cc:107-113 -- o_len is passed in MICROMETRES, then *1e-6 -> m.
    # ------------------------------------------------------------------
    def crossbar_ctrline(self):
        ch = self._cell_h_def
        return (
            self.Cw3(self.o_len * 1e-6)
            + self.drain_C(self.NTi, "PCH", 1, 1, ch)
            + self.drain_C(self.PTi, "NCH", 1, 1, ch)
            + self.gate_C(self.NTi, 0)
            + self.gate_C(self.PTi, 0)
        )

    # ------------------------------------------------------------------
    # arbiter.cc:115-119 -- ported for interface completeness (unused by
    # compute_power, consumed nowhere in the C++ router path either).
    # ------------------------------------------------------------------
    def transmission_buf_ctrcap(self):
        return self.gate_C(self.NTtr, 0) + self.gate_C(self.PTtr, 0)

    # ------------------------------------------------------------------
    # arbiter.cc:86-98
    # ------------------------------------------------------------------
    def compute_power(self):
        R = self.R
        Vdd = self.Vdd
        minw = self._minw  # g_tp.min_w_nmos_
        min_w_pmos = self.min_w_pmos

        self.read_dynamic = (
            R * self.arb_req() * Vdd * Vdd / 2
            + R * self.arb_pri() * Vdd * Vdd / 2
            + self.arb_grant() * Vdd * Vdd
            + self.arb_int() * 0.5 * Vdd * Vdd
        )

        nor1_leak = self.cmos_Isub_leakage(
            minw * self.NTn1 * 2, min_w_pmos * self.PTn1 * 2, 2, "NOR"
        )
        nor2_leak = self.cmos_Isub_leakage(
            minw * self.NTn2 * R, min_w_pmos * self.PTn2 * R, 2, "NOR"
        )
        not_leak = self.cmos_Isub_leakage(
            minw * self.NTi, min_w_pmos * self.PTi, 1, "INV"
        )

        nor1_leak_gate = self.cmos_Ig_leakage(
            minw * self.NTn1 * 2, min_w_pmos * self.PTn1 * 2, 2, "NOR"
        )
        nor2_leak_gate = self.cmos_Ig_leakage(
            minw * self.NTn2 * R, min_w_pmos * self.PTn2 * R, 2, "NOR"
        )
        not_leak_gate = self.cmos_Ig_leakage(
            minw * self.NTi, min_w_pmos * self.PTi, 1, "INV"
        )

        # //FIXME include priority table leakage  (arbiter.cc:96 -- omission ported)
        self.read_leakage = (nor1_leak + nor2_leak + not_leak) * Vdd
        self.read_gate_leakage = (
            nor1_leak_gate * Vdd + nor2_leak_gate * Vdd + not_leak_gate * Vdd
        )
