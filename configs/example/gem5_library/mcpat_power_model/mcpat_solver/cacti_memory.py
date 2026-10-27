"""CactiMemory: a clean facade over the CACTI/McPAT array energy pipeline.

Construct with the parameters of a cacti memory; the partition search runs
in ``__init__``; read/write/search energies and the static-power
coefficients (subthreshold + gate leakage) are then plain attributes. All
CACTI/McPAT machinery -- geometry derivation, candidate ranking, ArrayST
socket/overhead scaling, the longer-channel leakage reduction -- is
internal.

This is Part 1 of the facade work: pure composition over the already-
validated ``CactiArrayConfig`` -> ``search_partition`` ->
``CactiDynamicParameter`` -> ``CactiMat`` -> ``CactiUCA`` ->
``CactiArrayST`` object graph. It contains no energy or leakage arithmetic
of its own, so it cannot move a validated number unless the composition is
wired wrong -- ``test_cacti_memory.py``'s golden-equivalence suite pins
that at ``abs(diff) < 1e-12``.

See docs/superpowers/specs/2026-08-27-cacti-memory-facade-design.md.
"""

from typing import (
    Any,
    Dict,
    Optional,
    Tuple,
)

from .array_spec import (
    CactiArraySpec,
    build_array,
)
from .cacti_component import (
    CactiArrayST,
    CactiMat,
    CactiUCA,
)
from .cacti_dynamic_params import (
    CactiArrayConfig,
    CactiDynamicParameter,
)
from .cacti_partition_search import optimize_array_partition
from .technology import (
    build_cacti_params,
    clamp_to_cacti_decade,
    mcpat_wire_kwargs,
)

# Real CACTI/McPAT evaluates every partition candidate AND builds the final
# array with one fixed g_ip->wt (Global_30 when Embedded, else Global --
# machine_model.mcpat_wire_kwargs) -- so the search and the rebuild use the
# SAME wire-type overhead here, matching the canonical sweep_harness /
# validate_partition_search path (real McPAT 103/103). For the "search"
# provenance that shared value comes from ``_sys_params``' Embedded flag;
# "given" uses the caller's own wt_overhead for both.

_PartitionDict = dict[str, Any]


def _timing_of(res):
    """Read-only Layer-3 outcome of a partition search: `checked` is False
    when array.cc:113's gate skipped Layer 3 (no targets were tested)."""
    w = res.winner
    checked = hasattr(res, "satisfied")
    return dict(
        checked=checked,
        engaged=checked and res.engaged,
        throughput_ok=res.throughput_ok if checked else True,
        latency_ok=res.latency_ok if checked else True,
        access_time=w.access_time,
        cycle_time=w.cycle_time,
    )


class CactiMemory:
    """One CACTI-modeled memory: a RAM, a fully-associative CAM, or -- with
    ``_has_tag`` -- a data+tag cache pair. Runs CACTI's partition search at
    construction and exposes the resulting per-access dynamic energies and
    the static (leakage) power as attributes.

    Constructor keywords carry a ``_`` prefix (they can collide with
    gem5-side names in adjacent code); the public surface -- every exposed
    attribute (``read``, ``write``, ``search``, ``tag_read``,
    ``subthreshold_leakage``, ``gate_leakage``, ``static_power``,
    ``partition``, ``mat_area``, ...) -- is unprefixed.

    :param _capacity:        total capacity in bytes
    :param _block_sz:        line size in bytes
    :param _assoc:           associativity; 0 selects the fully-associative
                             CAM path (and is the only case in which
                             ``search`` is a float rather than ``None``)
    :param _out_w:           output/read width in bits
    :param _sys_params:      ``CactiParams`` for the target technology
                             corner (from ``machine_model.build_cacti_params``)
    :param _tag_w:           tag width in bits (used only when ``_has_tag``)
    :param _has_tag:         also model a separate ``specific_tag`` tag array
    :param _num_rw_ports:    read/write ports (default 1)
    :param _num_rd_ports:    read-only ports (default 0)
    :param _num_wr_ports:    write-only ports (default 0)
    :param _num_search_ports: search ports, FA/CAM only (default 0)
    :param _device_ty:       ``"core"`` | ``"uncore"`` | ``"llc"`` -- string
                             enum gating the longer-channel static-leakage
                             reduction; a non-string raises ``ValueError``
                             inside ``CactiArrayST`` (Phase 49 guard)
    :param _core_ooo:        ``True`` for an OOO core (selects the OOO
                             reduction factor)
    :param _partition:       how the 6 partition integers + mat area are
                             found:

                               * ``"search"`` -- run the partition search
                                 now (default): ``search_partition()``, or
                                 McPAT's Layer 3 when the array.cc:113 gate
                                 fires (see ``_opt_for_clk``)
                               * ``("given", (Nspd, Ndwl, Ndbl, Ndcm,
                                 Ndsam1, Ndsam2), (mat_w, mat_h))`` -- caller
                                 supplies them verbatim; data-only
                                 (``_has_tag`` raises ``ValueError``)
    :param _nbanks:          number of banks (``g_ip->nbanks``); default 1;
                             >1 for multi-bank caches (BTB=2, L2=8) -- feeds
                             the search cfg AND the rebuild, matching real
                             CACTI which uses one ``g_ip->nbanks`` throughout
    :param _add_ecc:         default ``True``
    :param _wire_is_mat_type: default ``None``: McPAT's core-array wire
                             setting for ``_sys_params``' Embedded flag
                             (``machine_model.mcpat_wire_kwargs``: 0 if
                             embedded, else 2), which then also supplies
                             ``_wire_os_mat_type`` when that is ``None``
    :param _wire_os_mat_type: H-tree (outside-mat) wire layer. ``None`` with
                             an explicit ``_wire_is_mat_type`` means "derive
                             from ``_wire_is_mat_type``"
                             (``CactiArrayConfig``'s own fallback). A
                             non-``None`` value
                             (e.g. ``1`` for an embedded shared L2/L3)
                             splits the H-tree wire layer from the
                             inside-mat layer, changing inter-mat H-tree
                             energy
    :param _wt_overhead:     retained for API stability; no longer feeds the
                             search config. Real CACTI evaluates every
                             candidate and builds the final array with one
                             fixed ``g_ip->wt``, so the search and the
                             rebuild both use ``_rebuild_wt_overhead`` --
                             the Embedded-derived overhead (30 embedded, 0
                             otherwise) for the ``"search"`` provenance, the
                             caller's own ``wt_overhead`` for
        ``"given"``
    :param _data_assoc:      default ``None``
    :param _is_seq_acc:      ``g_ip->is_seq_acc`` -- ``True`` only for a
                             sequential-access cache (McPAT's unified shared
                             L2/L3/Directorycache, ``sharedcache.cc:127``
                             ``access_mode = 1``). Read only by the
                             tag+data combination step of the search
                             (``find_delay``'s ``tag + data`` branch), so it
                             is inert unless ``_has_tag`` is set, and inert
                             for the energy rebuild either way
    :param _fast_access:     ``g_ip->fast_access`` -- ``True`` only for
                             McPAT's ``access_mode=2`` ("fast") structures
                             (``core.cc``'s N>1 branch-predictor arrays,
                             ``io.cc:1317``). Read only by
                             ``cacti_partition_search.py``'s ``_combine()``
                             access-time ranking step during the search, so
                             it is inert for the energy rebuild either way
                             (see ``_build_st``, which deliberately omits it)
    :param _specific_tag:    default follows ``_has_tag``
    :param _machine_params:  the machine-params dict
                             ``machine_model.build_cacti_params`` takes;
                             stored (not otherwise used at construction) so
                             ``update_static_power`` can rebuild
                             ``sys_params`` at a new CACTI temperature
                             decade without re-running the search. Left
                             ``None`` (default), ``update_static_power``
                             raises ``RuntimeError``
    :param _core_params:     the core-params dict paired with
                             ``_machine_params`` for the same purpose
    :param _opt_for_clk:     McPAT's ``-opt_for_clk`` flag; with
                             ``_opt_local`` (the array's ``opt_local``) it
                             gates McPAT's Layer-3 re-search (array.cc:113,
                             ``cacti_partition_search.layer3_gate``) for the
                             ``"search"`` provenance. Default ``False``
                             (plain Layer-1/2 search)
    :param _opt_local:       the array's ``opt_local`` (core0's XML value for
                             core arrays, literal ``True`` for a shared cache)
    :param _throughput:      Layer-3 cycle-time target in seconds
                             (``N_cycles / clockRate``); required only when
                             the gate fires
    :param _latency:         Layer-3 access-time target in seconds

    McPAT's Embedded flag (the default wire kwargs and the ``"search"``
    provenance's ``g_ip->wt``) is read from
    ``_sys_params._machine_config._config_params["embedded"]``.
    """

    def __init__(
        self,
        *,
        _capacity: int,
        _block_sz: int,
        _assoc: int,
        _out_w: int,
        _sys_params: Any,
        _tag_w: int = 0,
        _has_tag: bool = False,
        _num_rw_ports: int = 1,
        _num_rd_ports: int = 0,
        _num_wr_ports: int = 0,
        _num_search_ports: int = 0,
        _device_ty: str = "core",
        _core_ooo: bool = False,
        _partition: Any = "search",
        _nbanks: int = 1,
        _add_ecc: bool = True,
        _wire_is_mat_type: int | None = None,
        _wire_os_mat_type: int | None = None,
        _wt_overhead: int = 0,
        _data_assoc: int | None = None,
        _is_seq_acc: bool = False,
        _fast_access: bool = False,
        _specific_tag: bool | None = None,
        _machine_params: dict[str, Any] | None = None,
        _core_params: dict[str, Any] | None = None,
        _opt_for_clk: bool = False,
        _opt_local: bool = False,
        _throughput: float | None = None,
        _latency: float | None = None,
    ) -> None:
        self._sys_params = _sys_params
        wire = mcpat_wire_kwargs(_sys_params._machine_config._config_params)
        if _wire_is_mat_type is None:
            _wire_is_mat_type = wire["wire_is_mat_type"]
            if _wire_os_mat_type is None:
                _wire_os_mat_type = wire["wire_os_mat_type"]
        self._layer3 = dict(
            opt_for_clk=_opt_for_clk,
            opt_local=_opt_local,
            throughput=_throughput,
            latency=_latency,
        )
        self._nbanks = _nbanks
        self._device_ty = _device_ty
        self._core_ooo = _core_ooo
        self._has_tag = _has_tag
        # Retained so update_static_power() can rebuild CactiParams at a new
        # temperature decade (see machine_model.build_cacti_params); None
        # (the default) makes update_static_power() raise.
        self._machine_params = _machine_params
        self._core_params = _core_params
        if _specific_tag is None:
            _specific_tag = _has_tag
        # Retained so update_static_power() (a later task) can rebuild the
        # ArrayST at a new temperature without re-running the search.
        self._geom: dict[str, Any] = dict(
            capacity=_capacity,
            block_sz=_block_sz,
            assoc=_assoc,
            out_w=_out_w,
            tag_w=_tag_w,
            specific_tag=_specific_tag,
            add_ecc=_add_ecc,
            wire_is_mat_type=_wire_is_mat_type,
            wire_os_mat_type=_wire_os_mat_type,
            data_assoc=_data_assoc,
            is_seq_acc=_is_seq_acc,
            fast_access=_fast_access,
            num_rw_ports=_num_rw_ports,
            num_rd_ports=_num_rd_ports,
            num_wr_ports=_num_wr_ports,
            num_search_ports=_num_search_ports,
        )

        # The wire-type overhead used for BOTH the partition search and the
        # final rebuild -- real CACTI/McPAT evaluates every candidate and
        # builds the winner with one fixed g_ip->wt, and the canonical
        # sweep_harness / validate_partition_search path matches real McPAT
        # 103/103 doing exactly this. The Embedded-derived g_ip->wt for the
        # "search" provenance; "given" uses the caller's own wt_overhead.
        self._rebuild_wt_overhead: int = (
            wire["wt_overhead"] if _partition == "search" else _wt_overhead
        )

        self.timing = None  # set by a "search" partition; see _timing_of
        if _partition == "search":
            data_p, tag_p = self._run_search(_wt_overhead)
        elif (
            isinstance(_partition, tuple)
            and _partition
            and _partition[0] == "given"
        ):
            data_p, tag_p = self._partition_given(_partition)
        else:
            raise ValueError(f"unrecognised _partition: {_partition!r}")

        self._partition: tuple[Any, ...] = data_p["ints"]
        self.partition: tuple[Any, ...] = data_p["ints"]
        self.mat_area: tuple[float, float] = data_p["mat"]
        self._data_st: CactiArrayST = self._build_st(
            data_p["ints"], data_p["mat"], is_tag=False
        )
        self._tag_st: CactiArrayST | None = None
        self._tag_partition: tuple[Any, ...] | None = None
        self.tag_partition: tuple[Any, ...] | None = None
        self.tag_mat_area: tuple[float, float] | None = None
        if _has_tag:
            self._tag_partition = tag_p["ints"]
            self.tag_partition = tag_p["ints"]
            self.tag_mat_area = tag_p["mat"]
            self._tag_st = self._build_st(
                tag_p["ints"], tag_p["mat"], is_tag=True
            )

        self._set_energy_attrs()
        self._set_static_attrs()

    # ---- provenance -------------------------------------------------------

    def _search_cfg(
        self, *, is_tag: bool, wt_overhead: int
    ) -> CactiArrayConfig:
        """Build the ``CactiArrayConfig`` handed to the partition search.

        Same shape gem5-pm's ``_ArraySearch.array()`` uses for its search cfg
        (``is_cache=False``), except ``wt_overhead``: ``_run_search`` passes
        ``self._rebuild_wt_overhead`` -- the SAME overhead the rebuild uses
        -- because real CACTI/McPAT evaluates every candidate with one fixed
        ``g_ip->wt``, matching the canonical ``sweep_harness`` /
        ``validate_partition_search`` path.

        :param is_tag:      build the tag-side config (``specific_tag`` on)
        :param wt_overhead: wire-type overhead points for this config
        :returns:           the search config
        """
        g = self._geom
        return CactiArrayConfig(
            capacity=g["capacity"],
            block_sz=g["block_sz"],
            assoc=g["assoc"],
            nbanks=self._nbanks,
            out_w=g["out_w"],
            is_cache=False,
            add_ecc=g["add_ecc"],
            wire_is_mat_type=g["wire_is_mat_type"],
            wire_os_mat_type=g["wire_os_mat_type"],
            wt_overhead=wt_overhead,
            data_assoc=g["data_assoc"],
            is_seq_acc=g["is_seq_acc"],
            fast_access=g["fast_access"],
            is_tag=is_tag,
            tag_w=g["tag_w"],
            # Thread the caller's specific_tag through for the SINGLE
            # data-side array (_has_tag=False), exactly as ``_ArraySearch.array``
            # does. Inert for non-FA
            # (cacti_dynamic_params only reads it under `if cfg.is_tag`),
            # but load-bearing for FA/CAM: _init_fully_assoc reads
            # cfg.specific_tag to pick tagbits = tag_w vs a derived width,
            # which moves the winner/mat-area/energies. The separate
            # tag-array path (is_tag=True) keeps its own specific_tag.
            specific_tag=(
                g["specific_tag"] if (is_tag or not self._has_tag) else False
            ),
            num_rw_ports=g["num_rw_ports"],
            num_rd_ports=g["num_rd_ports"],
            num_wr_ports=g["num_wr_ports"],
            num_se_rd_ports=0,
            num_search_ports=g["num_search_ports"],
        )

    def _run_search(
        self, wt_overhead: int
    ) -> tuple[_PartitionDict, _PartitionDict | None]:
        """Run CACTI's partition search and extract the winner's 6 integers
        + mat area for the data side (and, when ``_has_tag``, the tag side).

        ``CactiMemory`` never reuses ``winner.data.uca``; only the integers
        and mat area are carried forward into a fresh rebuild, exactly as
        gem5-pm's ``_ArraySearch.array()`` feeds ``d.Nspd...`` into a fresh
        ``CactiArraySpec(...).build()``.

        The search config is built with ``self._rebuild_wt_overhead`` -- the
        SAME overhead the rebuild uses -- because real CACTI/McPAT evaluates
        every candidate with one fixed ``g_ip->wt`` (matching the canonical
        ``sweep_harness`` / ``validate_partition_search`` path). The
        ``wt_overhead`` parameter is kept for API stability but unused.
        McPAT's Layer 3 runs instead of the plain search when the
        array.cc:113 gate fires (``optimize_array_partition``).

        :param wt_overhead: retained for API stability; does not feed the
                            search config
        :returns:           ``(data_partition, tag_partition_or_None)`` where
                            each partition dict has ``"ints"`` (6-tuple) and
                            ``"mat"`` (``(w, h)``)
        """
        data_cfg = self._search_cfg(
            is_tag=False, wt_overhead=self._rebuild_wt_overhead
        )
        tag_cfg = (
            self._search_cfg(
                is_tag=True, wt_overhead=self._rebuild_wt_overhead
            )
            if self._has_tag
            else None
        )
        res = optimize_array_partition(
            data_cfg, self._sys_params, tag_cfg=tag_cfg, **self._layer3
        )
        self.timing = _timing_of(res)
        d = res.winner.data
        data_p: _PartitionDict = dict(
            ints=(d.Nspd, d.Ndwl, d.Ndbl, d.Ndcm, d.Ndsam1, d.Ndsam2),
            mat=(d.uca.mat.area_w, d.uca.mat.area_h),
        )
        tag_p: _PartitionDict | None = None
        if self._has_tag:
            t = res.winner.tag
            tag_p = dict(
                ints=(t.Nspd, t.Ndwl, t.Ndbl, t.Ndcm, t.Ndsam1, t.Ndsam2),
                mat=(t.uca.mat.area_w, t.uca.mat.area_h),
            )
        return data_p, tag_p

    def _partition_given(
        self, spec: tuple[Any, ...]
    ) -> tuple[_PartitionDict, _PartitionDict | None]:
        """``_partition=("given", ints, mat)`` provenance: the caller supplies
        the 6 partition integers and the ``(mat_w, mat_h)`` pair verbatim, so
        no search runs -- they flow straight into ``_build_st``.

        This provenance is data-only: there is no tag-side spec to carry, so
        ``_has_tag`` is unsupported and raises.

        :param spec: the ``("given", (Nspd, Ndwl, Ndbl, Ndcm, Ndsam1,
                     Ndsam2), (mat_w, mat_h))`` tuple
        :returns:    ``(data_partition, None)``
        :raises ValueError: if ``_has_tag`` is set, or if ``spec`` is not a
                            ``("given", (6 ints), (mat_w, mat_h))`` triple
        """
        if len(spec) != 3 or len(spec[1]) != 6 or len(spec[2]) != 2:
            raise ValueError(
                '_partition ("given", ...) must be '
                '("given", (6 ints), (mat_w, mat_h))'
            )
        _, ints, mat = spec
        if self._has_tag:
            raise ValueError(
                '_partition=("given", ...) is data-only; _has_tag is not '
                "supported with given provenance"
            )
        return dict(ints=tuple(ints), mat=tuple(mat)), None

    # ---- rebuild --------------------------------------------------------

    def _build_st(
        self,
        ints: tuple[Any, ...],
        mat: tuple[float, float],
        *,
        is_tag: bool,
    ) -> CactiArrayST:
        """Fresh ``CactiDynamicParameter`` -> ``CactiMat`` -> ``CactiUCA``
        -> ``CactiArrayST`` rebuild from the chosen partition integers + mat
        area, replicating gem5-pm ``CactiArraySpec(...).build()``: ``is_cache=True``,
        ``wt_overhead=self._rebuild_wt_overhead`` (the Embedded-derived one
        for the ``"search"`` provenance, the caller's own
        ``wt_overhead`` for ``"given"``), ``nbanks=self._nbanks``.

        :param ints:   the 6 partition integers
        :param mat:    ``(mat_area_w, mat_area_h)`` in micron
        :param is_tag: this is the tag array (``specific_tag`` on)
        :returns:      the built ``CactiArrayST``
        :raises ValueError: if the integers are invalid for this geometry
        """
        g = self._geom
        cfg = CactiArrayConfig(
            capacity=g["capacity"],
            block_sz=g["block_sz"],
            assoc=g["assoc"],
            nbanks=self._nbanks,
            out_w=g["out_w"],
            is_cache=True,
            add_ecc=g["add_ecc"],
            wire_is_mat_type=g["wire_is_mat_type"],
            wire_os_mat_type=g["wire_os_mat_type"],
            wt_overhead=self._rebuild_wt_overhead,
            data_assoc=g["data_assoc"],
            is_seq_acc=g["is_seq_acc"],
            # fast_access deliberately NOT threaded here (unlike _search_cfg
            # above): it only changes anything inside
            # cacti_partition_search.py's _combine() access-time ranking
            # step during the search, never in a rebuild from already-known
            # partition ints (CactiDynamicParameter/CactiBank's fast_access
            # branches all reduce to a no-op at the data_assoc<=1 shapes
            # this facade's has_tag callers use -- see the task-1 brief /
            # machine_model.CactiArraySpec's "Why not CactiArraySpec" docstring reasoning,
            # same argument applies here). Not a bug -- do not "fix" by
            # adding it.
            is_tag=is_tag,
            tag_w=g["tag_w"],
            # Same specific_tag threading as _search_cfg -- match the final
            # CactiArraySpec ``_ArraySearch.array`` builds.
            specific_tag=(
                g["specific_tag"] if (is_tag or not self._has_tag) else False
            ),
            num_rw_ports=g["num_rw_ports"],
            num_rd_ports=g["num_rd_ports"],
            num_wr_ports=g["num_wr_ports"],
            num_se_rd_ports=0,
            num_search_ports=g["num_search_ports"],
        )
        return build_array(
            "CactiMemory",
            cfg,
            self._sys_params,
            ints,
            mat,
            device_ty=self._device_ty,
            core_ooo=self._core_ooo,
        )

    # ---- attribute setters -------------------------------------------

    def _set_energy_attrs(self) -> None:
        """Set ``read``/``write``/``search``/``tag_read``/``tag_write`` from
        the built ArrayST(s). ``search`` is a float only for the FA/CAM path
        (``assoc == 0``), otherwise ``None`` -- preserving the
        ``hasattr(a, "search")`` / ``is None`` contract callers branch on.
        """
        d, t = self._data_st, self._tag_st
        self.read: float = d.read
        self.write: float = d.write
        # The `assoc == 0` short-circuit is deliberate: CactiArrayST only
        # sets `.search` on the FA/CAM path, so `d.search` must NOT be
        # evaluated for a non-FA array (it would raise AttributeError).
        self.search: float | None = (
            d.search if self._geom["assoc"] == 0 else None
        )
        self.tag_read: float | None = t.read if t is not None else None
        self.tag_write: float | None = t.write if t is not None else None

    def _set_static_attrs(self) -> None:
        """Set the four static-power coefficients (data + tag summed):
        ``subthreshold_leakage`` (longer-channel-reduced),
        ``subthreshold_leakage_raw`` (unreduced), ``gate_leakage``, and
        ``static_power`` (reduced subthreshold + gate).
        """
        d, t = self._data_st, self._tag_st
        sub_red = d.longer_channel_leakage + (
            t.longer_channel_leakage if t is not None else 0.0
        )
        sub_raw = d.leakage + (t.leakage if t is not None else 0.0)
        gate = d.gate_leakage + (t.gate_leakage if t is not None else 0.0)
        self.subthreshold_leakage: float = sub_red
        self.subthreshold_leakage_raw: float = sub_raw
        self.gate_leakage: float = gate
        self.static_power: float = sub_red + gate

    # ---- mutable temperature -------------------------------------------

    def update_static_power(self, temp_k: float) -> None:
        """Re-derive the four static-power coefficients in place at the CACTI
        temperature decade nearest ``temp_k``, reusing the partition integers
        + mat area chosen at construction -- the partition search is NOT
        re-run.

        Only ``subthreshold_leakage``, ``subthreshold_leakage_raw``,
        ``gate_leakage`` and ``static_power`` change; ``read``/``write``/
        ``search``/``tag_read``/``tag_write`` are temperature-independent
        (spec Section 4) and left untouched, as are ``partition``/
        ``mat_area``/``tag_partition``.

        :param temp_k: target temperature in kelvin; snapped onto CACTI's
                       300-400K / 10K decade grid via
                       ``machine_model.clamp_to_cacti_decade``
        :raises RuntimeError: if ``_machine_params``/``_core_params`` were not
                              passed to ``__init__`` (there is no other way
                              to rebuild ``CactiParams`` at a new decade)
        """
        if self._machine_params is None or self._core_params is None:
            raise RuntimeError(
                "update_static_power needs _machine_params/_core_params "
                "passed to __init__"
            )
        decade = clamp_to_cacti_decade(temp_k)
        self._sys_params = build_cacti_params(
            self._machine_params, self._core_params, temperature=decade
        )
        self._data_st = self._build_st(
            self._partition, self.mat_area, is_tag=False
        )
        if self._tag_st is not None:
            self._tag_st = self._build_st(
                self._tag_partition, self.tag_mat_area, is_tag=True
            )
        self._set_static_attrs()

    # ---- machine_model.CactiArraySpec interop ---------------------------------------

    @classmethod
    def from_array(
        cls,
        array: Any,
        sys_params: Any,
        *,
        machine_params: dict[str, Any] | None = None,
        core_params: dict[str, Any] | None = None,
    ) -> "CactiMemory":
        """Build a ``CactiMemory`` that reproduces ``array.build(sys_params)``
        bit-for-bit, by adopting ``array``'s already-chosen 6 partition
        integers + mat area as a ``("given", ...)`` provenance (no search
        runs) and its geometry from ``array._cfg_kwargs``.

        The rebuild wire-type overhead is taken from
        ``array._cfg_kwargs["wt_overhead"]`` -- the array's own value, not
        the Embedded-derived search overhead.

        :param array:          a ``machine_model.CactiArraySpec``
        :param sys_params:     ``CactiParams`` for the target tech corner
        :param machine_params: optional machine-params dict, stored so
                               ``update_static_power`` can rebuild at a new
                               temperature decade
        :param core_params:    optional core-params dict, paired with
                               ``machine_params``
        :returns:              the equivalent ``CactiMemory``
        """
        c = array._cfg_kwargs
        d = array._dp_kwargs
        return cls(
            _capacity=c["capacity"],
            _block_sz=c["block_sz"],
            _assoc=c["assoc"],
            _nbanks=c["nbanks"],
            _out_w=c["out_w"],
            _tag_w=c["tag_w"],
            _specific_tag=c["specific_tag"],
            _has_tag=False,
            _num_rw_ports=c["num_rw_ports"],
            _num_rd_ports=c["num_rd_ports"],
            _num_wr_ports=c["num_wr_ports"],
            _num_search_ports=c["num_search_ports"],
            _wire_is_mat_type=c["wire_is_mat_type"],
            _wire_os_mat_type=c["wire_os_mat_type"],
            _wt_overhead=c["wt_overhead"],
            _data_assoc=c["data_assoc"],
            _is_seq_acc=c["is_seq_acc"],
            _add_ecc=c["add_ecc"],
            _sys_params=sys_params,
            _device_ty=array._device_ty,
            _core_ooo=array._core_ooo,
            _machine_params=machine_params,
            _core_params=core_params,
            _partition=(
                "given",
                (
                    d["Nspd"],
                    d["Ndwl"],
                    d["Ndbl"],
                    d["Ndcm"],
                    d["Ndsam_lev_1"],
                    d["Ndsam_lev_2"],
                ),
                (array._mat_area_w, array._mat_area_h),
            ),
        )

    def to_array(self) -> Any:
        """Emit a plain ``machine_model.CactiArraySpec`` carrying this memory's chosen
        partition integers + mat area (``("given", ...)`` semantics) and
        geometry, so a caller still expecting an ``CactiArraySpec`` gets one whose
        ``build()`` reproduces this ``CactiMemory``.

        Independent of ``self._sys_params`` by construction -- the emitted
        ``CactiArraySpec`` is a pure literal, so it stays correct even after
        ``update_static_power`` has reassigned ``self._sys_params`` in place.

        :returns: the equivalent ``machine_model.CactiArraySpec``
        """
        g = self._geom
        Nspd, Ndwl, Ndbl, Ndcm, Ndsam1, Ndsam2 = self._partition
        return CactiArraySpec(
            "CactiMemory",
            capacity=g["capacity"],
            block_sz=g["block_sz"],
            assoc=g["assoc"],
            nbanks=self._nbanks,
            out_w=g["out_w"],
            add_ecc=g["add_ecc"],
            wire_is_mat_type=g["wire_is_mat_type"],
            wire_os_mat_type=g["wire_os_mat_type"],
            wt_overhead=self._rebuild_wt_overhead,
            data_assoc=g["data_assoc"],
            is_seq_acc=g["is_seq_acc"],
            is_tag=False,
            tag_w=g["tag_w"],
            specific_tag=g["specific_tag"],
            num_rw_ports=g["num_rw_ports"],
            num_rd_ports=g["num_rd_ports"],
            num_wr_ports=g["num_wr_ports"],
            num_se_rd_ports=0,
            num_search_ports=g["num_search_ports"],
            Nspd=Nspd,
            Ndwl=Ndwl,
            Ndbl=Ndbl,
            Ndcm=Ndcm,
            Ndsam_lev_1=Ndsam1,
            Ndsam_lev_2=Ndsam2,
            mat_area_w=self.mat_area[0],
            mat_area_h=self.mat_area[1],
            device_ty=self._device_ty,
            core_ooo=self._core_ooo,
        )
