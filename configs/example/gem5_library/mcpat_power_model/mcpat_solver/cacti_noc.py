"""CACTI router buffers, arbiters, crossbars and McPAT NoC composition.

Geometry and coefficients are pure Python; runtime activity is supplied
by the gem5 adapters. Preserves the native router/node leakage attribution."""

from math import ceil

try:
    from .cacti_arbiter import CactiArbiter
    from .cacti_component import CactiMat
    from .cacti_crossbar import CactiCrossbar
    from .cacti_dynamic_params import (
        CactiArrayConfig,
        CactiDynamicParameter,
    )
except ImportError:  # vendored copy uses package-relative imports
    from cacti_arbiter import CactiArbiter
    from cacti_component import CactiMat
    from cacti_crossbar import CactiCrossbar
    from cacti_dynamic_params import (
        CactiArrayConfig,
        CactiDynamicParameter,
    )


class CactiRouter:
    """Port of ``Router``.  ``__init__`` runs ``calc_router_parameters()``
    (``get_router_delay`` -> ``get_router_power`` -> ``get_router_area``).

    Attributes after construction (see module docstring for the two scope
    rulings and the cell-override quirk):
      ``buffer_read_dynamic`` / ``buffer_write_dynamic`` (J, always equal --
        the replicated ``//FIXME`` bug), ``buffer_read_leakage`` /
        ``buffer_read_gate_leakage`` (W), ``buffer_area_w`` / ``buffer_area_h``
        (given inputs, um).
      ``crossbar`` (a ``CactiCrossbar``).
      ``arbiter_read_dynamic`` / ``arbiter_read_leakage`` /
        ``arbiter_read_gate_leakage`` (the ``vcarb*I + cbarb*O`` combine).
      ``power_read_dynamic`` / ``power_read_leakage`` / ``power_read_gate_leakage``
        (the ``get_router_power`` TDP-style aggregate).
      ``area_w`` / ``area_h`` (``get_router_area``).
      ``vc_buffer_size`` / ``vc_count`` / ``flit_size`` / ``I`` / ``O`` / ``M``.
    Method: ``ae() -> dict`` -- the unscaled per-access sub-component
      ``readOp.dynamic`` coefficients ``noc.cc::computeEnergy(is_tdp=false)``
      actually reads (NOT ``power_read_dynamic``, which is the TDP aggregate).
    """

    def __init__(
        self,
        circuit,
        flit_size,
        vc_buf,
        vc_count,
        in_ports,
        out_ports,
        M=1.0,
        device="peri_global",
        buffer_area_w=157.6911042651648,
        buffer_area_h=58.30992283469314,
    ):
        if device != "peri_global":
            # router.cc's Router ctor always defaults dt=&(g_tp.peri_global), and
            # NoC::init_router's only real call site passes exactly that -- same
            # guard discipline as CactiCrossbar/CactiArbiter.
            raise ValueError(
                f"CactiRouter device must be 'peri_global', got {device!r}"
            )
        self._circuit = circuit
        self._tp = circuit._tp
        self._wp = circuit._wp

        # ctor body (router.cc:37-63)
        self.flit_size = float(flit_size)
        self.vc_buffer_size = float(
            vc_buf
        )  # vc_buffer_size = vc_buf (verbatim)
        self.vc_count = float(vc_count)
        self.I = float(in_ports)
        self.O = float(out_ports)
        self.M = float(M)

        self.min_w_pmos = (
            self._tp["n_to_p_eff_curr_drv_ratio"] * self._tp["min_w_nmos"]
        )
        self.Vdd = self._tp["Vdd"]

        # Crossbar-related transistor widths (router.cc:44-63) -- router.cc
        # multiplies by *1e-6 (unlike arbiter.cc's raw-um convention). These
        # back ONLY the dead `else` branch of cb_stats() (router.cc:203-220,
        # `if (1) {...} else {tr_crossbar_power()...}` -- the else branch is
        # unreachable in every real McPAT run); kept for interface/ctor-field
        # completeness per the task brief, not read by get_router_power/ae().
        self.technology = circuit._node_um  # g_ip->F_sz_um
        self.NTtr = 10 * self.technology * 1e-6 / 2
        self.PTtr = 20 * self.technology * 1e-6 / 2
        self.wt = 15 * self.technology * 1e-6 / 2
        self.ht = 15 * self.technology * 1e-6 / 2
        self.NTi = 12.5 * self.technology * 1e-6 / 2
        self.PTi = 25 * self.technology * 1e-6 / 2
        self.NTid = 60 * self.technology * 1e-6 / 2
        self.PTid = 120 * self.technology * 1e-6 / 2
        self.NTod = 60 * self.technology * 1e-6 / 2
        self.PTod = 120 * self.technology * 1e-6 / 2

        # Given-input mat area (ruling 1 -- see module docstring).
        self.buffer_area_w = float(buffer_area_w)
        self.buffer_area_h = float(buffer_area_h)

        self.calc_router_parameters()

    # ------------------------------------------------------------------
    def calc_router_parameters(self):
        self.get_router_delay()
        self.get_router_power()
        self.get_router_area()

    # ------------------------------------------------------------------
    # get_router_delay (router.cc:257-268). NOT needed for AE (per the task
    # brief) and kept only for structural completeness: real CACTI's
    # `g_tp.FO4` (fan-out-of-4 delay) is a horowitz-derived quantity this
    # project's energy-only tech-params dict does not carry (grepped --
    # `FO4` appears nowhere in this port). Rather than fabricate an
    # unvalidated FO4 value, `max_cyc`/the FREQUENCY-adjustment branch are
    # left unevaluated; `FREQUENCY`/`cycle_time`/`delay` (the three fields
    # that do NOT depend on FO4) are ported directly.
    # ------------------------------------------------------------------
    def get_router_delay(self):
        self.FREQUENCY = 5
        self.cycle_time = (1.0 / self.FREQUENCY) * 1e3  # ps
        self.delay = 4
        self.max_cyc = None  # g_tp.FO4 not ported in this project -- see above

    # ------------------------------------------------------------------
    # buffer_stats (router.cc:141-201): builds the router's VC-buffer Mat
    # from the fixed preset (ruling 2 -- see module docstring), NOT a
    # search_partition() call.
    # ------------------------------------------------------------------
    def buffer_stats(self):
        flit_size = int(self.flit_size)
        vc_count = int(self.vc_count)
        vc_buffer_size = int(self.vc_buffer_size)

        # g_ip-level fields (embedded NoC: mcpat/noc.cc:60-65 --
        # wire_is_mat_type=0/local, wire_os_mat_type=1/inside_mat, matching
        # cacti_crossbar.py's/cacti_arbiter.py's identical embedded-NoC
        # assumption). capacity/block_sz/nbanks/assoc are NOT read anywhere
        # downstream for this preset (use_inp_params=1 bypasses
        # CactiDynamicParameter's normal capacity-driven derivation entirely
        # -- verified by grepping every `dp.`/`self.dp.`/`dyn_p.` read in
        # cacti_component.py's CactiMat), so their values are placeholders.
        cfg = CactiArrayConfig(
            capacity=flit_size * vc_buffer_size,
            block_sz=1,
            assoc=1,
            nbanks=1,
            out_w=flit_size,
            is_cache=False,
            is_tag=False,
            wire_is_mat_type=0,
            wire_os_mat_type=1,
            num_rw_ports=0,
            num_rd_ports=1,
            num_wr_ports=vc_count,
            num_se_rd_ports=0,
            num_search_ports=0,
        )

        # A CactiDynamicParameter-SHAPED object built directly from
        # router.cc:141-201's raw field list (use_inp_params=1 means real
        # CACTI skips DynamicParameter's normal capacity-driven derivation
        # entirely too -- this mirrors that, not merely approximates it).
        #
        # `CactiDynamicParameter.__new__(...)` bypasses `__init__` entirely --
        # unlike the real C++ `DynamicParameter` (whose bare/default ctor,
        # `parameter.cc:187-190`, at least zero-initializes every member),
        # a Python attribute that is never assigned here simply DOES NOT
        # EXIST; any read of it would raise AttributeError, not silently
        # return a wrong default. That is a real construction-fragility risk
        # for a shared class this port's `search_partition()`/`CactiMemory`
        # callers rely on staying self-consistent -- worth documenting as
        # explicitly as the `use_inp_params`/`add_ecc`/`V_b_sense` findings
        # above, not left implicit.
        #
        # THE COMPLETE FIELD CONTRACT (verified 2026-09-12 by grepping every
        # `self.dp.`/`dp.`/`dyn_p.` read reachable from `CactiMat.__init__`,
        # `set_dynamic_parameters`, `CactiSubarray.__init__`/`calc_dimensions`,
        # and everything `dynamic_power`/`static_power`/`compute_delays` call
        # -- i.e. every attribute access this preset can actually reach,
        # since `buffer_stats()` never constructs a `CactiBank`/`CactiUCA`
        # wrapper around this bare `Mat`): every field assigned below this
        # comment (`cfg`, `fully_assoc`, `add_ecc`, `valid`, `Nspd` through
        # `V_b_sense`) IS read somewhere on this path. `router.cc:141-201`'s
        # own preset additionally sets four fields this hand-built `dp` does
        # NOT set -- each checked individually, not assumed safe by the
        # fixture's relerr 0.0 alone:
        #   - `ram_cell_tech_type` / `pure_ram`: grepped, ZERO reads anywhere
        #     in cacti_component.py (the one `ram_cell_tech_type` hit is an
        #     unrelated comment about tech-param equivalence, not a `dp`
        #     attribute access). Genuinely dead for this port's whole energy
        #     pipeline, not just this preset -- safe to omit.
        #   - `is_dram`: this port has NO `dp.is_dram` attribute anywhere,
        #     on ANY array, not just the router's -- already documented at
        #     `CactiUCA.compute_delays`'s own docstring ("is_dram is never
        #     modelled anywhere in this port -- no dp.is_dram attribute
        #     exists at all"). `CactiComponent.__init__` hardcodes
        #     `self._is_dram = False` unconditionally instead (DRAM is out
        #     of scope project-wide, CLAUDE.md). Safe to omit.
        #   - `is_main_mem`: read only inside `CactiUCA.compute_delays`
        #     (uca.cc's `if (dp.is_main_mem)` blocks, ported there behind a
        #     "verified unreachable" note since the normal
        #     `CactiDynamicParameter.__init__` already raises
        #     `NotImplementedError` for `cfg.is_main_mem` before a `dp` can
        #     exist) -- and `buffer_stats()` never builds a `CactiUCA`/
        #     `CactiBank` around this Mat at all, so that code is doubly
        #     unreachable here. Safe to omit.
        #   - `is_tag` (set on `cfg` above, but real router.cc also sets it
        #     on `dyn_p` itself): this port's OWN convention -- for EVERY
        #     array, not just the router's -- keeps `is_tag` as a `cfg`-only
        #     field; `CactiDynamicParameter.__init__` (cacti_dynamic_params.py)
        #     never assigns `self.is_tag` on itself at all (confirmed:
        #     `CactiArrayConfig.__init__` is the only `self.is_tag = ...` in
        #     that file). Every `is_tag` read in cacti_component.py goes
        #     through `cfg.is_tag`/`self.dp.cfg.is_tag`, never a bare
        #     `dp.is_tag`/`dyn_p.is_tag` -- grepped, zero such reads exist.
        #     `cfg.is_tag=False` above already covers every one of them; this
        #     is a pre-existing project-wide convention difference from the
        #     C++, not a router-specific gap `fully_assoc` needed to close
        #     the same way (`fully_assoc` genuinely IS a `dp`-level field in
        #     this port, cacti_dynamic_params.py:135, so it does need setting
        #     on `dp` in addition to being implied by `cfg.assoc`).
        dp = CactiDynamicParameter.__new__(CactiDynamicParameter)
        dp.cfg = cfg
        dp.fully_assoc = False
        # add_ecc=True, NOT False (router.cc:141-201 never sets a `dp.add_ecc`
        # field at all -- it doesn't exist on the real DynamicParameter class;
        # ECC is applied by Subarray's ctor reading the GLOBAL g_ip->add_ecc_b_
        # flag directly (subarray.cc:51), independent of which DynamicParameter
        # ctor built dp. McPAT sets that flag true for the whole run
        # (processor.cc), so every array -- including this router buffer --
        # gets it. Verified directly against a temporary router.cc probe:
        # real CACTI's buffer subarray is 8 rows x 144 cols, not 128
        # (=flit_size*vc_count) -- the extra 16 = ceil(128/8), the ECC check-bit
        # overhead this port's CactiSubarray(add_ecc=...) applies generically.
        dp.add_ecc = True
        dp.valid = True
        dp.Nspd = 1
        dp.Ndwl = 1
        dp.Ndbl = 1
        dp.Ndcm = 1
        dp.Ndsam_lev_1 = 1
        dp.Ndsam_lev_2 = 1
        dp.num_subarrays = dp.Ndwl * dp.Ndbl
        dp.num_mats = 1
        dp.num_r_subarray = vc_buffer_size
        dp.num_c_subarray = flit_size * vc_count
        dp.num_mats_h_dir = 1
        dp.num_mats_v_dir = 1
        dp.num_subarrays_per_mat = dp.num_subarrays // dp.num_mats
        dp.num_subarrays_per_row = max(1, dp.Ndwl // dp.num_mats_h_dir)
        dp.deg_bl_muxing = 1
        dp.deg_senseamp_muxing_non_associativity = 1
        dp.number_way_select_signals_mat = 1
        dp.num_di_b_mat = flit_size
        dp.num_do_b_mat = flit_size
        dp.num_act_mats_hor_dir = 1
        dp.number_addr_bits_mat = 8
        dp.number_subbanks_decode = 0
        dp.num_do_b_subbank = flit_size
        dp.num_di_b_subbank = flit_size
        dp.num_do_b_bank_per_port = flit_size
        dp.num_di_b_bank_per_port = flit_size
        dp.V_b_sense = (
            self.Vdd
        )  # unused downstream (CactiComponent.init_v_b_sense
        # derives its own _V_b_sense from self._tp['Vdd']
        # directly -- kept only for preset-field fidelity)

        # router.cc:190-201's cell-dimension override -- see module docstring.
        pitch_os = self._wp["pitch_inside_mat"]
        num_wr, num_rw = cfg.num_wr_ports, cfg.num_rw_ports
        num_rd, num_se = cfg.num_rd_ports, cfg.num_se_rd_ports
        cell_h = self._tp["sram_b_h"] + 2 * pitch_os * (
            num_wr + num_rw - 1 + num_rd
        )
        cell_w = (
            self._tp["sram_b_w"]
            + 2 * pitch_os * (num_rw - 1 + (num_rd - num_se) + num_wr)
            + pitch_os * num_se
        )

        # router.cc:174's V_b_sense override (`dyn_p.V_b_sense = Vdd`, full peri
        # Vdd, not the usual 5%-of-Vdd sense margin) -- see module docstring /
        # CactiMat's v_b_sense_override docstring for why this matters for
        # energy (deg_bl_muxing==1's dynRdEnergy term, mat.cc:1177-1178) and
        # cannot be left to CactiComponent.init_v_b_sense()'s own default.
        mat = CactiMat(
            "router_buffer",
            self._circuit._cacti_params,
            dp,
            cell_h_override=cell_h,
            cell_w_override=cell_w,
            v_b_sense_override=self.Vdd,
        )
        # NOT calling mat.compute_delays(0) here (real router.cc:196 does,
        # before compute_power_energy()). This port's CactiMat.compute_power()
        # already runs automatically in __init__ (unlike the C++), and
        # compute_delays is a purely additive delay-only side-channel that
        # writes no field dynamic_power/static_power reads (confirmed by
        # grepping every `self.delay*` write target against those methods'
        # own reads) -- so skipping it is energy-neutral. It is also actively
        # harmful to call here: delay_bl_restore's `log((V_b_pre - 0.1*V_b) /
        # (V_b_pre - V_b))` divides by exactly zero once v_b_sense_override
        # equals V_b_pre (both == Vdd, router.cc's own V_b_sense=Vdd quirk --
        # real C++ silently gets +inf here via IEEE754; Python raises
        # ZeroDivisionError). Since this delay value has no validated
        # consumer for AE, it is simply not computed rather than adding a
        # third IEEE754-emulation guard for a dead value.
        self._buffer_mat = mat

        self.buffer_read_dynamic = mat._power.read.dynamic
        # //FIXME (router.cc:200): buffer.power.writeOp = buffer.power.readOp --
        # a real CACTI bug, replicated verbatim, not fixed.
        self.buffer_write_dynamic = mat._power.read.dynamic
        self.buffer_read_leakage = mat._power.read.leakage
        self.buffer_read_gate_leakage = mat._power.read.gate_leakage
        # self.buffer_area_w / buffer_area_h: given inputs (ruling 1), set in
        # __init__ already -- router.cc's `buffer.area = buff.area` is not
        # re-derived here.

    # ------------------------------------------------------------------
    # cb_stats (router.cc:203-221): always takes the `if (1)` branch in real
    # CACTI (the `else` -- tr_crossbar_power() -- is dead code).
    # ------------------------------------------------------------------
    def cb_stats(self):
        self.crossbar = CactiCrossbar(
            self._circuit, self.I, self.O, self.flit_size
        )

    # ------------------------------------------------------------------
    # get_router_power (router.cc:223-254).
    # ------------------------------------------------------------------
    def get_router_power(self):
        self.buffer_stats()
        self.cb_stats()

        vcarb = CactiArbiter(
            self._circuit, self.vc_count, self.flit_size, self.buffer_area_w
        )
        cbarb = CactiArbiter(
            self._circuit, self.I, self.flit_size, self.crossbar.area_w
        )
        self._vcarb = vcarb
        self._cbarb = cbarb

        self.arbiter_read_dynamic = (
            vcarb.read_dynamic * self.I + cbarb.read_dynamic * self.O
        )
        self.arbiter_read_leakage = (
            vcarb.read_leakage * self.I + cbarb.read_leakage * self.O
        )
        self.arbiter_read_gate_leakage = (
            vcarb.read_gate_leakage * self.I + cbarb.read_gate_leakage * self.O
        )

        # TDP-style aggregate (router.cc:249-251). This is NOT what ae()
        # returns -- see module docstring / advisor checkpoint.
        self.power_read_dynamic = (
            (
                (self.buffer_read_dynamic + self.buffer_write_dynamic)
                + self.crossbar.read_dynamic
                + self.arbiter_read_dynamic
            )
            * min(self.I, self.O)
            * self.M
        )

        # pppm_t = {1, I, I, 1}; power = power + (buffer.power*pppm_t +
        # crossbar.power + arbiter.power) * pppm_lkg, pppm_lkg = {0,1,1,0}
        # (const.h:262) -- reduces to this for the readOp component (worked
        # out by hand against the fixture in the module docstring / task-5
        # report; power.readOp.dynamic is unaffected -- pppm_lkg zeroes that
        # term, leaving the TDP aggregate above untouched).
        self.power_read_leakage = (
            self.buffer_read_leakage * self.I
            + self.crossbar.read_leakage
            + self.arbiter_read_leakage
        )
        self.power_read_gate_leakage = (
            self.buffer_read_gate_leakage * self.I
            + self.crossbar.read_gate_leakage
            + self.arbiter_read_gate_leakage
        )

    # ------------------------------------------------------------------
    # get_router_area (router.cc:270-274).
    # ------------------------------------------------------------------
    def get_router_area(self):
        self.area_h = self.I * self.buffer_area_h
        self.area_w = self.buffer_area_w + self.crossbar.area_w

    # ------------------------------------------------------------------
    def ae(self):
        """The unscaled per-access sub-component ``readOp.dynamic`` values
        ``noc.cc::computeEnergy(is_tdp=false)`` reads -- NOT
        ``power_read_dynamic`` (the TDP aggregate; see module docstring)."""
        return {
            "NoCBuffer": {
                "Read": self.buffer_read_dynamic,
                "Write": self.buffer_write_dynamic,
            },
            "NoCCrossbar": {"Read": self.crossbar.read_dynamic},
            "NoCArbiter": {"Read": self.arbiter_read_dynamic},
        }


class CactiNoC:
    """Port of ``NoC`` (``mcpat/noc.cc``) -- the per-network facade that wraps
    a ``CactiRouter`` (``type==1``, mesh/torus -- what MESI_Two_Level, this
    plan's real target, uses) or a bus/global-link wire model (``type==0``
    or ``has_global_link``), and computes runtime dynamic energy + static
    leakage from it. Task 6 of the ruby-mesi-noc-support port; see this
    module's top docstring for ``CactiRouter``'s own scope notes (all of
    which this class inherits unchanged).

    ``CactiNoC(circuit, params: dict, embedded: bool, link_len=None)``.

    ``params`` mirrors a McPAT ``<component id="system.NoCn">`` block's
    fields (``noc.cc::set_noc_param``), matched to a Python dict the same
    way this project's other config-from-dict constructors work (cf.
    ``CactiArrayConfig``). Unlike the array-config classes, several
    ``params`` keys are read with a fallback default (``dict.get``,
    NOT the project's usual "no ``.get()``" tech-param-dict rule -- that
    rule is specifically about ``self._tp``/``self._wp``, per CLAUDE.md;
    ``params`` here is an XML-record-shaped external input, the same
    category as the ``rec.get(...)`` calls already used throughout this
    project's trace/graft tooling, e.g. ``sweep_harness.py``,
    ``validate_real_cache_search.py``). The fallback defaults below are
    for fields that are provably UNREAD by the ``type==1``,
    ``has_global_link=False`` combination this task's own fixture and
    every real MESI_Two_Level config in this plan's scope actually
    exercises -- confirmed against ``noc.cc::computeEnergy(is_tdp=False)``
    line by line, not assumed:
      - ``duty_cycle``: only read inside ``computeEnergy(is_tdp=True)``
        (the peak/TDP path, ``M=nocdynp.duty_cycle``) -- never in the
        ``is_tdp=False`` RTP branch this class's ``runtime_dynamic_energy``
        ports. Default ``1.0`` (inert placeholder).
      - ``horizontal_nodes``/``vertical_nodes``: only feed ``total_nodes``
        (an AREA-aggregation multiplier, ``init_router``) and the
        ``link_len`` scaling inside ``init_link_bus`` (bus/global-link
        path only). Default ``1`` each.
      - ``has_global_link``: default ``False`` (no global link).
      - ``link_throughput``/``link_latency``/``chip_coverage``/
        ``route_over_perc``: only read inside ``init_link_bus``
        (``type==0`` or ``has_global_link`` paths). Defaults are inert
        placeholders (``1.0``/``1.0``/``1.0``/``0.5``, the last matching
        ``interconnect.h``'s own ``route_over_perc_=0.5`` header default)
        for the ``type==1``/no-global-link path that never reads them.

    Every field the REQUIRED (``type==1``, no global link) path actually
    reads -- ``clockrate_mhz``, ``flit_bits``, ``input_ports``,
    ``output_ports``, ``M_traffic_pattern`` -- is read via plain
    ``dict[...]`` (KeyError, not a silent default, if missing), matching
    the project's real convention for genuinely load-bearing fields.
    ``virtual_channel_per_port``/``input_buffer_entries_per_vc`` are the
    one exception among "router" fields: real ``set_noc_param`` reads them
    unconditionally (outside the ``if(type)`` guard) but only the
    ``type==1`` path ever consumes them, and real bus-type NoC0 XML blocks
    never define them at all (ParseXML defaults an absent int field to
    ``0``) -- so they default to ``0`` here too, matching that real
    behavior rather than being treated as always-required.

    Embedded-NoC scope (checked, not assumed -- see ``cacti_circuit.py``'s
    ``wire_cap`` docstring and ``cacti_crossbar.py``/``cacti_arbiter.py``'s
    module docstrings): ``CactiRouter``'s buffer geometry AND
    ``CactiCrossbar``/``CactiArbiter`` all hardcode
    ``wire_os_mat_type==1`` (the Embedded-NoC tier) internally, with no
    parameter to select the non-embedded tier (``wire_os_mat_type==2``) --
    a real, pre-existing (Task 3/4/5) scope boundary, not something this
    class introduces. ``CactiNoC`` therefore REQUIRES ``embedded=True`` for
    ``type==1`` and raises ``NotImplementedError`` otherwise, rather than
    silently building a router with the wrong wire tier. The ``type==0``/
    ``has_global_link`` bus path (``init_link_bus``, below) is NOT subject
    to this restriction -- its own wire-tier selection
    (``embedded -> wire_os_mat_type=1``, else ``=2``, ``noc.cc:60-71``) is
    threaded through explicitly, independent of the router's hardcoding.

    Attributes after construction: ``.router`` (a ``CactiRouter``, iff
    ``type==1``) / ``.link_bus`` (a plain ``dict`` -- NOT a class instance,
    see ``init_link_bus``'s docstring -- iff ``type==0`` or
    ``has_global_link``) / ``.type`` / ``.clockRate`` / ``.total_nodes`` /
    ``.min_ports`` / ``.global_linked_ports`` / ``.executionTime`` (``None``
    unless both ``total_cycles``/``target_core_clockrate`` are given in
    ``params`` -- optional, per the brief: "the AE path does not use
    ``executionTime``").

    Methods: ``activation_energies()``, ``runtime_dynamic_energy(total_accesses)``
    (sets ``.runtime_leakage``/``.runtime_gate_leakage`` as a side effect --
    see that method's docstring for why these are separate from the
    returned value), ``static_power()``.
    """

    def __init__(self, circuit, params, embedded, link_len=None):
        self._circuit = circuit
        self.embedded = bool(embedded)
        self.params = dict(params)
        p = self.params

        # --- set_noc_param (noc.cc:345-412) ---------------------------------
        # `type` is the ONE params key that is NOT given a `.get()` default,
        # unlike every other optional field documented above: it SELECTS
        # which path this class builds (router vs. bus), so a missing/
        # mistyped key must raise (KeyError), not silently construct the
        # wrong component. (Advisor checkpoint: an earlier draft defaulted
        # this to `1` because the task-6 brief's own Step 1 fixture omitted
        # it -- that was a fixture-shape gap, not a real optional field;
        # fixed by adding `"type": 1` to `build_ruby_fixture.py`'s
        # `noc.params` instead, matching `gen_ruby_grafts.py`'s
        # `build_graft`, which sets NoC0 `type=1` explicitly.)
        self.type = int(p["type"])
        self.clockrate_mhz = float(p["clockrate_mhz"])
        self.clockRate = self.clockrate_mhz * 1e6

        total_cycles = p.get("total_cycles")
        target_core_clockrate = p.get("target_core_clockrate")
        self.executionTime = (
            total_cycles / (target_core_clockrate * 1e6)
            if total_cycles is not None and target_core_clockrate is not None
            else None
        )

        self.flit_size = float(p["flit_bits"])
        # virtual_channel_per_port / input_buffer_entries_per_vc: set_noc_param
        # (noc.cc:374-375) reads these UNCONDITIONALLY (outside the
        # `if(type)` guard), but only `init_router` (type==1 only) ever
        # consumes them downstream. Real bus-type NoC0 XML blocks (e.g. the
        # base `.ARM_A9_2GHz_gem5.xml` anchor itself, `type=1` with
        # `input_ports=output_ports=1` -- effectively a degenerate
        # single-port router -- and every bus graft derived from it) never
        # define these params at all; ParseXML's own struct defaults an
        # absent int field to 0. Default `0` here reproduces that real
        # XML-absent behavior, not an arbitrary placeholder -- confirmed by
        # reading `.ARM_A9_2GHz_gem5.xml`'s `system.NoC0` block directly.
        self.virtual_channel_per_port = float(
            p.get("virtual_channel_per_port", 0)
        )
        self.input_buffer_entries_per_vc = float(
            p.get("input_buffer_entries_per_vc", 0)
        )
        self.M_traffic_pattern = float(p.get("M_traffic_pattern", 1.0))
        self.duty_cycle = float(p.get("duty_cycle", 1.0))

        if self.type:
            self.input_ports = float(p["input_ports"])
            self.output_ports = float(p["output_ports"])
            self.min_ports = min(self.input_ports, self.output_ports)
            self.global_linked_ports = (self.input_ports - 1) + (
                self.output_ports - 1
            )
        else:
            self.input_ports = 1.0
            self.output_ports = 1.0
            self.min_ports = 1.0
            self.global_linked_ports = 1.0

        self.horizontal_nodes = float(p.get("horizontal_nodes", 1))
        self.vertical_nodes = float(p.get("vertical_nodes", 1))
        self.total_nodes = self.horizontal_nodes * self.vertical_nodes

        self.has_global_link = bool(p.get("has_global_link", False))
        self.link_throughput = float(p.get("link_throughput", 1.0))
        self.link_latency = float(p.get("link_latency", 1.0))
        self.chip_coverage = float(p.get("chip_coverage", 1.0))
        self.route_over_perc = float(p.get("route_over_perc", 0.5))
        assert self.chip_coverage <= 1
        assert self.route_over_perc <= 1

        # Embedded wire selection (noc.cc:60-71). Only the geometry-TIER
        # index is actually threaded anywhere by this class (the type==0/
        # global-link bus path, `init_link_bus` below) -- the Wire_type
        # (`wt`) itself is forced to Global unconditionally inside
        # `interconnect`'s own ctor body regardless of what's passed in
        # (interconnect.cc:74, `wt = Global;` -- the SAME finding already
        # documented in mcpat_logic_components.Interconnect's docstring for
        # the register-bypass network's identical wire model), so there is
        # no `wt` value for this class to store or select.
        self._wire_tier = 1 if self.embedded else 2

        self.router = None
        self.link_bus = None
        self.link_bus_exist = False
        self.router_exist = False

        # --- NoC::NoC ctor body (noc.cc:76-87) -------------------------------
        if self.type:
            if not self.embedded:
                raise NotImplementedError(
                    "CactiNoC type==1 (router) is validated only for the "
                    "Embedded NoC path (wire_os_mat_type=1): CactiRouter's "
                    "buffer geometry and CactiCrossbar/CactiArbiter all "
                    "hardcode that wire tier internally, with no parameter "
                    "to select wire_os_mat_type=2 (see cacti_circuit.py's "
                    "wire_cap docstring / cacti_crossbar.py's and "
                    "cacti_arbiter.py's module docstrings). A non-embedded "
                    "router would need that threaded through all three "
                    "classes first, and this project's corpus has no "
                    "ground truth to validate it against -- raising rather "
                    "than silently returning wrong numbers."
                )
            self.router = CactiRouter(
                circuit,
                self.flit_size,
                self.virtual_channel_per_port
                * self.input_buffer_entries_per_vc,
                self.virtual_channel_per_port,
                self.input_ports,
                self.output_ports,
                M=self.M_traffic_pattern,
            )
            self._finish_init_router()
            self.router_exist = True
        else:
            self.init_link_bus(link_len)

        # Global-link extension (real McPAT: processor.cc:357, called AFTER
        # every component's area is known -- a full-chip concern this
        # single-NoC facade cannot derive `link_len` for on its own; the
        # caller must supply it). Untested by this task's fixture (no real
        # MESI_Two_Level config in this plan's scope sets has_global_link)
        # -- implemented for interface completeness per the brief's
        # `.link_bus (if type==0 or has_global_link)` attribute contract,
        # not validated against ground truth.
        if self.has_global_link and self.type:
            self.init_link_bus(link_len)

    # ------------------------------------------------------------------
    # init_router's longer_channel_leakage addendum (noc.cc:98-131). Only
    # the readOp.leakage (subthreshold) reduction is ported -- the
    # power_gated_leakage lines (noc.cc:116-128) depend on
    # power_gating_leakage_reduction(), which is UNPORTED anywhere in this
    # project (grepped: zero Python hits) and is not part of this task's
    # required interface (`static_power()`'s dict has no power-gated key,
    # and no test exercises it) -- not implemented, not silently faked.
    # ------------------------------------------------------------------
    def _finish_init_router(self):
        r = self.router
        lcr = self._circuit._long_channel_reduction(device_ty="uncore")
        r.power_read_longer_channel_leakage = r.power_read_leakage * lcr
        self._long_channel_device_reduction = lcr

    # ------------------------------------------------------------------
    # init_link_bus (noc.cc:133-160) -- the type==0 (bus) / has_global_link
    # wire model. Reuses `CactiCircuit._wire_model`/`_wire_layer_geom`
    # directly (a fresh, minimal port of `interconnect.cc`'s ctor body +
    # `compute()`) rather than `mcpat_logic_components.Interconnect`: that
    # class is the register-BYPASS network's interconnect (Device_ty=Core,
    # embedded -> LOCAL tier 0, overhead=0 always) -- a different real call
    # site with a different embedded-tier convention
    # (`core.cc`'s "embedded -> local wire (2.5F)" vs `noc.cc`'s "embedded
    # -> wire_os_mat_type=1 (inside_mat, 4F)"), confirmed by reading both
    # C++ call sites; reusing it here would silently apply the WRONG tier.
    #
    # PORTED (not deferred): the `opt_local`/`pipelinable` width-scaling
    # doubling loop (interconnect.cc:120-131, `while (delay > throughput &&
    # width_scaling < 3.0) { width_scaling *= 2; space_scaling *= 2; Wire
    # winit(width_scaling, space_scaling); compute(); }`). Real `noc.cc`'s
    # own call (`init_link_bus`) does not pass `opt_local_`, so it takes
    # `interconnect.h`'s header default `opt_local_=true`; combined with
    # `main.cc`'s own `opt_for_clk=true` default (never overridden by any
    # invocation in this project -- grepped, no `-opt_for_clk` flag
    # anywhere in this repo's scripts), the loop's guard
    # (`opt_for_clk && opt_local`) IS true for a real run -- unlike
    # `mcpat_logic_components.Interconnect`'s IDENTICAL omission of this
    # same loop, which is genuinely unreachable there (every real
    # `core.cc` call site explicitly passes `opt_local=0`). Confirmed
    # real, not hypothesized: a first-pass port WITHOUT this loop measured
    # a uniform ~9.24% relative error on dynamic/leakage/gate_leakage/
    # longer_channel_leakage against a real `./mcpat -infile
    # .ARM_A9_2GHz_gem5_ruby_bus.xml` probe (`mcpat/noc.cc`, temporarily
    # modified then reverted -- see task-6 report); the closed-form
    # `overhead=0`/`w_scale=s_scale=1` delay for this exact config's
    # `link_len` (4032.316 um) is 5.1547e-10s, marginally ABOVE
    # `throughput = link_throughput/clockRate = 5e-10s`, so the loop fires
    # exactly once (`width_scaling: 1.0 -> 2.0`); recomputing `_wire_model`
    # at `w_scale=s_scale=2.0` reproduced all four probed values at
    # relative error ~1e-16 (float-epsilon), confirming this is the
    # complete fix, not a partial one. The `num_pipe_stages` branch
    # (`delay > throughput` even after the doubling loop exhausts at
    # `width_scaling>=3.0`) is ported for completeness but not exercised
    # by this fixture (the loop already satisfies `delay<=throughput`
    # after one iteration here); `delay`/`num_pipe_stages` are stored as
    # attributes but not consumed by `activation_energies`/
    # `runtime_dynamic_energy`/`static_power` (none of which are
    # delay-dependent), matching this port's established "delay ported
    # for completeness, not read downstream" pattern (cf. `CactiRouter.
    # get_router_delay`).
    # ------------------------------------------------------------------
    def init_link_bus(self, link_len):
        if link_len is None:
            raise ValueError(
                "init_link_bus requires an explicit positive link_len -- "
                "real NoC::init_link_bus asserts link_len>0; the real "
                "value (sqrt(total_chip_area*chip_coverage)) is a "
                "full-processor quantity this single-NoC facade cannot "
                "derive on its own (see class docstring)."
            )
        link_len = float(link_len)
        if link_len <= 0:
            raise ValueError(f"link_len must be > 0, got {link_len!r}")

        # noc.cc:146-151
        throughput = self.link_throughput / self.clockRate
        latency = self.link_latency / self.clockRate
        link_len = link_len / (
            (self.horizontal_nodes + self.vertical_nodes) / 2.0
        )
        if self.total_nodes > 1:
            link_len /= 2.0

        # interconnect.cc's ctor: Wire(wt=Global, length, 1, width_scaling,
        # space_scaling) -- overhead=0 is the delay-optimal closed form
        # `wt=Global` structurally always selects (see the module docstring
        # above / mcpat_logic_components.Interconnect's identical finding).
        # width_scaling/space_scaling start at 1.0 and are doubled by the
        # opt_local/pipelinable loop below (see the docstring above this
        # method for why that loop is PORTED, not deferred).
        geom = self._circuit._wire_layer_geom(self._wire_tier)
        width_scaling = 1.0
        space_scaling = 1.0
        wm = self._circuit._wire_model(
            *geom, overhead=0, w_scale=width_scaling, s_scale=space_scaling
        )
        delay = wm["delay_per_um"] * link_len

        # interconnect.cc:124-131 (pipelinable branch): widen the wire
        # (doubling width_scaling/space_scaling together) until the
        # per-length delay-optimal delay meets throughput, capped at
        # width_scaling<3.0 -- exactly mirrors real CACTI's own loop bound.
        num_pipe_stages = None
        while delay > throughput and width_scaling < 3.0:
            width_scaling *= 2.0
            space_scaling *= 2.0
            wm = self._circuit._wire_model(
                *geom, overhead=0, w_scale=width_scaling, s_scale=space_scaling
            )
            delay = wm["delay_per_um"] * link_len
        if delay > throughput:
            # interconnect.cc:130: insert pipeline stages instead of
            # widening further (not exercised by this task's fixture --
            # the loop above already satisfies delay<=throughput for it;
            # ported for completeness, not consumed by any AE/runtime/
            # static output).
            num_pipe_stages = ceil(delay / throughput)
            delay = delay / num_pipe_stages + num_pipe_stages * 0.05 * delay

        sckt = self._circuit._tp["sckt_coeff"]
        data_width = self.flit_size

        # interconnect.cc:144-165: power_bit=power (pre-data_width copy, not
        # separately exposed here -- unread by anything this task's
        # interface needs); power.readOp.dynamic *= data_width; then
        # *= sckRation (leakage/gate_leakage are NOT scaled by sckt_coeff,
        # matching the "only dynamic gets sckt_coeff" rule already
        # established for every other logic/interconnect component in this
        # port, per mcpat_logic_components.Interconnect's own docstring).
        dynamic = wm["dynamic_per_um"] * link_len * data_width * sckt
        leakage = wm["leak_per_um"] * link_len * data_width
        gate_leakage = wm["gleak_per_um"] * link_len * data_width
        lcr = self._circuit._long_channel_reduction(device_ty="uncore")
        longer_channel_leakage = leakage * lcr

        # `.link_bus` is a plain dict, NOT a class instance (unlike real
        # CACTI's `interconnect*` object) -- nothing else in this task
        # needs an object with methods; documented explicitly since the
        # brief describes `.link_bus` as though it mirrors `.router`.
        self.link_bus = {
            "dynamic": dynamic,
            "leakage": leakage,
            "gate_leakage": gate_leakage,
            "longer_channel_leakage": longer_channel_leakage,
            "link_len_um": link_len,
            "throughput_s": throughput,
            "latency_s": latency,
            "width_scaling": width_scaling,
            "space_scaling": space_scaling,
            "delay_s": delay,
            "num_pipe_stages": num_pipe_stages,
        }
        self.link_bus_exist = True
        return self.link_bus

    # ------------------------------------------------------------------
    def activation_energies(self):
        """``{"NoCBuffer": {...}, "NoCCrossbar": {...}, "NoCArbiter": {...}}``
        for ``type==1`` (delegates to ``self.router.ae()``); ``{"Bus":
        {"Read": ...}}`` for ``type==0``. If BOTH a router and a link_bus
        exist (the untested ``has_global_link`` composite case), the dicts
        are merged -- not validated against ground truth, see ``__init__``.
        """
        ae = dict(self.router.ae()) if self.router is not None else {}
        if self.link_bus is not None:
            ae["Bus"] = {"Read": self.link_bus["dynamic"]}
        return ae

    # ------------------------------------------------------------------
    def runtime_dynamic_energy(self, total_accesses):
        """Port of ``noc.cc::computeEnergy(is_tdp=False)``'s RTP branch
        (noc.cc:198-236).

        For ``type==1``: real CACTI computes
        ``buffer.rt_power.dynamic = (buffer.power.readOp.dynamic +
        buffer.power.writeOp.dynamic) * access``, similarly for
        crossbar/arbiter, then
        ``router->rt_power = (buffer.rt + crossbar.rt + arbiter.rt)*pppm_t
        + router->power*pppm_lkg`` with ``pppm_t = {1,0,0,0}`` (so only the
        *.dynamic* slot of the bracketed sum survives) and
        ``pppm_lkg = {0,1,1,0}`` (so the ADDED term contributes only to the
        *leakage*/*gate_leakage* slots, zero to *dynamic*). The upshot:
        ``rt_power.readOp.dynamic`` is EXACTLY
        ``(buffer.read_dyn + buffer.write_dyn + crossbar.read_dyn +
        arbiter.read_dyn) * total_accesses`` -- no leakage term leaks into
        the returned dynamic value -- while ``rt_power.readOp.leakage`` /
        ``.gate_leakage`` pick up the UNSCALED (by total_accesses)
        ``router->power.readOp.leakage`` / ``.gate_leakage`` (the same
        TDP-style aggregate ``static_power()`` reads). This method returns
        only the dynamic part (per the brief: "Return the dynamic part
        only"); the leakage/gate_leakage side effect is exposed as
        ``.runtime_leakage``/``.runtime_gate_leakage`` (set here, not
        returned) so a caller wanting the full ``rt_power`` triple can
        still get it without this method returning a dict and breaking the
        brief's declared ``-> float`` signature.

        For ``type==0``: ``link_bus->rt_power = link_bus->power * pppm_t``
        with ``pppm_t = {access, 1, 1, access}`` (noc.cc:231) -- dynamic
        scales by ``total_accesses``, leakage/gate_leakage do NOT (the 4th
        slot, ``access`` again, is ``short_circuit``, which this port
        never models -- genuinely inert here, not an omission).
        **Scope note (advisor checkpoint)**: unlike the ``type==1`` router
        path, this scaling law is derived from reading ``noc.cc:229-234``
        directly, NOT independently probe-confirmed against a real
        ``link_bus->rt_power.readOp.dynamic`` value -- the checked-in probe
        (``unit_tests/fixtures/probe_output_bus.txt``) only dumped
        ``link_bus->power`` (the CONSTRUCTION-time per-flit coefficients,
        ``NoC::init_link_bus``'s output), which IS what
        ``static_power``/``activation_energies`` and the per-flit
        coefficients this method scales are checked against (those ARE
        ≤1e-6 against real McPAT). ``test_runtime_dynamic_bus`` therefore
        checks this method's OWN internal consistency (``dynamic *
        total_accesses``), not an independently-probed
        ``computeEnergy(is_tdp=False)`` value, for the bus path -- the
        ``type==1`` router test has the same shape but is corroborated by
        ``noc.cc``'s own pre-existing (uncommitted, not this task's)
        ``std::cout`` lines inside that exact RTP branch; no equivalent
        exists for the bus path in this run's stdout capture.

        If both a router and a link_bus exist (untested composite case),
        both contributions are summed -- see ``__init__``'s docstring.
        """
        total_accesses = float(total_accesses)
        dynamic = 0.0
        leakage = 0.0
        gate_leakage = 0.0

        if self.router is not None:
            r = self.router
            dynamic += (
                r.buffer_read_dynamic
                + r.buffer_write_dynamic
                + r.crossbar.read_dynamic
                + r.arbiter_read_dynamic
            ) * total_accesses
            leakage += r.power_read_leakage
            gate_leakage += r.power_read_gate_leakage

        if self.link_bus is not None:
            dynamic += self.link_bus["dynamic"] * total_accesses
            leakage += self.link_bus["leakage"]
            gate_leakage += self.link_bus["gate_leakage"]

        self.runtime_leakage = leakage
        self.runtime_gate_leakage = gate_leakage
        return dynamic

    # ------------------------------------------------------------------
    def static_power(self):
        """``{"subthreshold": .., "gate": .., "subthreshold_longer_channel": ..}``
        from ``router.power_read_leakage`` / ``power_read_gate_leakage`` /
        ``power_read_longer_channel_leakage`` (the last set by
        ``_finish_init_router``, mirroring ``noc.cc::init_router``'s
        ``longer_channel_device_reduction(Uncore_device)`` call -- see that
        method's docstring for why ``power_gating_leakage_reduction`` is
        NOT included here). Only defined when a router exists (``type==1``)
        -- the brief's interface spec keys entirely off router fields; the
        bus path has no analogous required output and is not exercised by
        any test.
        """
        if self.router is None:
            raise NotImplementedError(
                "static_power() is only defined for type==1 (a Router "
                "exists) -- see this method's docstring / class docstring "
                "for why the bus path has no required static_power output "
                "in this task's interface."
            )
        r = self.router
        return {
            "subthreshold": r.power_read_leakage,
            "gate": r.power_read_gate_leakage,
            "subthreshold_longer_channel": r.power_read_longer_channel_leakage,
        }
