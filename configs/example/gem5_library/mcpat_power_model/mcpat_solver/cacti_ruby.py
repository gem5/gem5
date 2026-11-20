"""``CactiRubyHierarchy``: the top-level Ruby/MESI hierarchy facade.

Task 8 of the ruby-mesi-noc-support port. Composes:

  * L1I / L1D / L2 caches -- each a data+tag ``cacti_memory.CactiMemory``
    pair (the SAME "cache pair" construction ``test_cacti_memory.py``'s
    ``SearchCachePairGolden`` uses) plus its miss/fill/prefetch/write-back
    buffers, geometry from ``cache_buffers.build_cache_buffers`` and each
    buffer itself a live-searched ``CactiMemory`` (mirroring
    ``cacti_directory.py``'s ``_memory_from_buffer_array`` -- the same
    pattern, not re-derived).
  * A MESI directory cache ("DC") via ``cacti_directory.build_directory``
    (Task 7).
  * A NoC via ``cacti_noc.CactiNoC`` (Task 5/6).

into ONE ``activation_energies()``/``static_power()`` dict. This module is
PURE COMPOSITION -- every energy/leakage number is read off an already-built
``CactiMemory``/``build_directory()`` result/``CactiNoC`` instance; nothing
here computes a new dynamic-energy or leakage term. See each method's
docstring for the exact provenance of every dict entry.

Config-dict shapes (this task's own design -- there is no existing McPAT/
gem5 XML shape this needs to mirror bit-for-bit, unlike ``CactiMemory``'s
own ``_`` -prefixed kwargs, which DO mirror a pre-existing convention):

``_l1i`` / ``_l1d`` / ``_l2`` -- one dict per cache, with keys:
    level            "l1i" | "l1d" | "shared" (l2 is always "shared" --
                     McPAT's own SharedCache class, matching
                     ``machine_model.L2``'s ``device_ty="llc"`` treatment)
    capacity, block_sz, assoc, out_w, tag_w, phy_addr_width   -- required
    entry_counts     {"mshr":, "fill":, "prefetch":, "wbb":} buffer entry
                     counts (required)
    cache_policy     "write_back" | "write_through" (required; gates the
                     l1d WBB exactly as ``build_cache_buffers`` does -- l1i
                     never gets one regardless)
    num_rw_ports, num_rd_ports, num_wr_ports   default 1/0/0
    nbanks           default 1 (L2 callers pass 8, matching
                     ``machine_model.L2``)
    device_ty        default "core" ("llc" for L2, matching
                     ``machine_model.L2``'s own convention)
    core_ooo         default False
    wire_is_mat_type default 0
    wire_os_mat_type default None (L2 passes 1, matching ``machine_model.L2``)
    is_seq_acc       default False (L2 passes True, matching McPAT's
                     ``sharedcache.cc:127`` unified-cache ``access_mode=1``)
    data_assoc       default None (L2 passes 1, matching McPAT's
                     ``io.cc`` "is_seq_acc forces data_assoc=1" rule --
                     see ``cacti_directory.py``'s module docstring, the SAME
                     rule)
    fetch_or_memory_ports, wt_overhead   default 1 / 30

``_directory`` -- forwarded (with ``sys_params`` added) straight into
    ``cacti_directory.build_directory``'s own keyword-argument list
    (``level``, ``dir_cfg``, ``phys_addr_width``, ``buffer_sizes``,
    ``device_ty``, ``cache_policy``, ``clockrate``); see that function's
    docstring for what each means. A real caller should pass
    ``device_ty="llc"`` (see that module's own extensively-documented
    finding).

``_noc`` -- ``{"params": {...}, "embedded": bool, "link_len": float|None}``,
    forwarded straight into ``cacti_noc.CactiNoC``'s own constructor.

``_sys_params`` -- a single ``cacti_params.CactiParams`` instance (from
    ``machine_model.build_cacti_params``), shared by every ``CactiMemory``/
    ``build_directory`` call AND wrapped in one ``CactiCircuit`` for the
    NoC -- exactly ``CactiMemory``'s own ``_sys_params`` convention. **L2 is
    the one exception (Task 5b of the ruby-cache-noc-stress-test plan)**:
    the real McPAT graft this facade is validated against
    (``.ARM_A9_2GHz_gem5_ruby.xml``) gives ``system.L20`` its own
    ``device_type=0`` (HP), distinct from the system-level/every-other-
    Ruby-component ``device_type=2`` (LOP) -- a real, previously-invisible
    config-plumbing gap, not a formula bug. ``__init__`` now also takes
    ``_l2_device_type`` (default ``0``, matching the graft) and builds a
    SEPARATE ``l2_sys_params`` for L2 alone by recovering the raw
    ``machine_params``/``core_params`` dicts ``_sys_params`` was itself
    built from (see ``_l2_sys_params()`` below -- from
    ``_sys_params._machine_config._config_params``, confirmed this session
    to round-trip byte-for-byte), overriding ``device_type`` on a copy, and
    rebuilding via ``machine_model.build_cacti_params`` -- mirroring
    ``machine_model.py``'s ``MachineModel.__init__``'s
    ``l2_machine_params``/``_cacti_params_l2`` pattern exactly (the SAME
    ``build_cacti_params(machine_params, core_params)`` rebuild call, not a
    shortcut that mutates an already-built ``CactiParams``/
    ``McPATMachineConfig`` in place; that is the pre-existing, working
    precedent for this exact problem, and this facade does not invent a new
    design). Recovering the raw dicts from ``_sys_params`` itself (rather
    than adding new required constructor parameters) keeps every existing
    caller that constructs a ``CactiRubyHierarchy`` with only ``_sys_params``
    working unchanged. Only ``_l2`` is built from ``l2_sys_params``; L1I/
    L1D/Directory/NoC all keep using the original, unmodified
    ``_sys_params``.

Optional keys in ``_l1i``/``_l1d``/``_l2`` are read with ``dict.get(...)``
defaults (``_build_cache_pair``/``from_config`` below) rather than direct
``[...]`` indexing -- this sits under the SAME carve-out
``cacti_noc.py``'s own module docstring already documents and cites:
CLAUDE.md's "no ``.get()``" rule is specifically about ``self._tp``/
``self._wp`` (the tech/wire parameter dicts), not an XML/record-shaped
external config dict like these -- the same category as this project's
``rec.get(...)`` trace/graft tooling and ``CactiNoC``'s own ``params``
handling.
"""

try:
    from .machine_model import build_cacti_params
except ImportError:
    from machine_model import build_cacti_params


try:
    from .cache_buffers import build_cache_buffers
    from .cacti_circuit import CactiCircuit
    from .cacti_directory import build_directory
    from .cacti_memory import CactiMemory
    from .cacti_noc import CactiNoC
    from .technology import build_cacti_params
except ImportError:  # vendored copy uses package-relative imports
    from cache_buffers import build_cache_buffers
    from cacti_circuit import CactiCircuit
    from cacti_directory import build_directory
    from cacti_memory import CactiMemory
    from cacti_noc import CactiNoC
    from technology import build_cacti_params

# The exact set of keys McPATMachineConfig._defined_config_params (see
# config_input_params.py) validates a machine_params dict against --
# needed to recover a clean machine_params dict back out of an already-
# built CactiParams (see _l2_sys_params() below). Kept as a copy here
# (not imported) since config_input_params.py builds this set as a class
# attribute assignment inside __init__, not a reusable module-level
# constant.
_MACHINE_PARAMS_KEYS = frozenset(
    {
        "num_cores",
        "num_l2s",
        "private_l2",
        "num_l1_dirs",
        "num_l2_dirs",
        "num_nocs",
        "tech_node",
        "interconnect_type",
        "temperature",
        "device_type",
        "longer_chan_dev",
        "embedded",
        "machine_bits",
        "phy_addr_width",
        "virt_addr_width",
        "vm_pg_size",
    }
)


def _l2_sys_params(sys_params, l2_device_type):
    """Builds L2's own ``CactiParams``, mirroring ``machine_model.py``'s
    ``MachineModel.__init__``'s ``l2_machine_params``/``_cacti_params_l2``
    pattern EXACTLY (verified against ``machine_model.py:1169-1185`` this
    session, Task 5b) -- a fresh ``build_cacti_params(machine_params,
    core_params)`` rebuild with ``device_type`` overridden on a copy of
    ``machine_params``, not a shortcut that mutates an already-built
    ``CactiParams``/``McPATMachineConfig`` in place.

    The raw ``machine_params``/``core_params`` dicts are recovered from
    ``sys_params._machine_config._config_params`` -- ``McPATMachineConfig``'s
    own already-validated flat dict, carrying exactly its
    ``_defined_config_params`` keys (``_MACHINE_PARAMS_KEYS`` above) plus
    two derived entries this function strips back out: ``data_width``
    (``McPATMachineConfig.reconfigure_params()``) and ``core0`` (the
    ``McPATCoreConfig``-processed core dict, ``add_cores()``). Confirmed
    this session, via a real round-trip probe: reconstructing
    ``machine_params``/``core_params`` this way and feeding them back
    through ``machine_model.build_cacti_params`` reproduces the ORIGINAL
    ``machine_params`` dict byte-for-byte, and reproduces bit-identical
    ``_tp``/``_wp`` tech/wire-param dicts (device_type overridden) to
    calling ``build_cacti_params`` directly on a hand-overridden copy of
    the caller's own ``machine_params``/``core_params`` -- not merely
    assumed safe. ``core0``'s dict carries 4 already-computed derived
    fields (``arch_ireg_width`` etc.) that ``McPATCoreConfig.
    reconfigure_params()`` idempotently recomputes on rebuild, and
    ``McPATCoreConfig`` never ``validate()``s its own keys, so their
    presence here is inert -- also confirmed by the same round-trip
    probe, not assumed.

    Recovering the raw dicts this way (rather than adding new required
    ``_machine_params``/``_core_params`` constructor parameters to
    ``CactiRubyHierarchy.__init__``) keeps every existing caller that
    constructs a ``CactiRubyHierarchy`` from only ``_sys_params`` working
    unchanged -- see ``__init__``'s own docstring.
    """

    cfg = sys_params._machine_config._config_params
    machine_params = {k: cfg[k] for k in _MACHINE_PARAMS_KEYS}
    core_params = dict(cfg["core0"])
    machine_params["device_type"] = l2_device_type
    return build_cacti_params(machine_params, core_params)


# cache_buffers.build_cache_buffers's own raw key suffixes (see that
# module's docstring: "shared: <p>MissB, <p>FillB, <p>PrefetchB, <p>WBB
# (always)"; "l1i/l1d: <p>MissBuffer, <p>FillBuffer, <p>prefetchBuffer[,
# <p>WBB]") mapped onto THIS facade's public activation_energies()/
# static_power() role suffix -- "Ifb" for the fill buffer (matching the
# Directory buffer-role naming the task-8 brief itself specifies), and
# "Writebackb" (NOT the Directory's "Wbb" -- a deliberate, brief-specified
# difference, not a typo).
_SHARED_BUFFER_ROLES = (
    ("MissB", "Missb"),
    ("FillB", "Ifb"),
    ("PrefetchB", "Prefetchb"),
    ("WBB", "Writebackb"),
)
_L1_BUFFER_ROLES = (
    ("MissBuffer", "Missb"),
    ("FillBuffer", "Ifb"),
    ("prefetchBuffer", "Prefetchb"),
)

# build_directory()'s own buffer dict keys ({"MissB","FillB","PrefetchB",
# "WBB"}) -> this facade's public Directory<role> key suffix. Exact mapping
# given by the task-8 brief's own Step 1 test code.
_DIRECTORY_ROLE_MAP = {
    "Missb": "MissB",
    "Ifb": "FillB",
    "Prefetchb": "PrefetchB",
    "Wbb": "WBB",
}


def _memory_from_buffer_array(arr, sys_params):
    """Lift one geometry-only buffer ``machine_model.CactiArraySpec`` (as returned by
    ``build_cache_buffers`` with ``partitions=None``) into a live-searched
    ``CactiMemory``, by reading its ``_cfg_kwargs`` -- the mirror image of
    ``CactiMemory.from_array`` (which lifts an CactiArraySpec's ALREADY-KNOWN
    partition as ``("given", ...)``) and the exact same pattern
    ``cacti_directory.py``'s own (private) ``_memory_from_buffer_array``
    uses for the directory's own 4 buffers (this facade always searches,
    matching that function's ``given=None`` branch -- there is no
    given-partition shortcut in this facade's own interface). Buffers are
    single FA/CAM arrays with an embedded tag (``specific_tag=True``,
    ``_has_tag=False``), same shape as ``test_cacti_memory.py``'s
    ``"fa_cam_512B_specific_tag"`` case.
    """
    c = arr._cfg_kwargs
    return CactiMemory(
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
        _add_ecc=c["add_ecc"],
        _wire_is_mat_type=c["wire_is_mat_type"],
        _wire_os_mat_type=c["wire_os_mat_type"],
        _data_assoc=c["data_assoc"],
        _is_seq_acc=c["is_seq_acc"],
        _wt_overhead=c["wt_overhead"],
        _sys_params=sys_params,
        _device_ty=arr._device_ty,
        _core_ooo=False,
        _partition="search",
    )


def _build_cache_pair(cc, sys_params):
    """Build one L1I/L1D/L2-shaped cache: a data+tag ``CactiMemory`` (the
    main array) plus its miss/fill/prefetch/[write-back] buffers, each a
    live-searched ``CactiMemory`` of its own.

    :param cc: one of ``_l1i``/``_l1d``/``_l2`` (see module docstring for
        the exact key contract)
    :returns: ``{"mem": CactiMemory, "buffers": {role: CactiMemory, ...}}``
        -- ``buffers`` has 3 roles ("Missb"/"Ifb"/"Prefetchb") for
        ``level in ("l1i", "l1d")`` with ``cache_policy != "write_back"``,
        4 (adding "Writebackb") for ``level == "l1d"`` with
        ``cache_policy == "write_back"``, and always 4 for
        ``level == "shared"`` (matching ``build_cache_buffers``'s own gate)
    """
    level = cc["level"]
    mem = CactiMemory(
        _capacity=cc["capacity"],
        _block_sz=cc["block_sz"],
        _assoc=cc["assoc"],
        _out_w=cc["out_w"],
        _tag_w=cc["tag_w"],
        _has_tag=True,
        _num_rw_ports=cc.get("num_rw_ports", 1),
        _num_rd_ports=cc.get("num_rd_ports", 0),
        _num_wr_ports=cc.get("num_wr_ports", 0),
        _nbanks=cc.get("nbanks", 1),
        _device_ty=cc.get("device_ty", "core"),
        _core_ooo=cc.get("core_ooo", False),
        _wire_is_mat_type=cc.get("wire_is_mat_type", 0),
        _wire_os_mat_type=cc.get("wire_os_mat_type", None),
        _is_seq_acc=cc.get("is_seq_acc", False),
        _data_assoc=cc.get("data_assoc", None),
        _wt_overhead=cc.get("wt_overhead", 30),
        _sys_params=sys_params,
        _partition="search",
    )

    buf_arrays = build_cache_buffers(
        level=level,
        prefix="buf",
        capacity=cc["capacity"],
        line_bytes=cc["block_sz"],
        assoc=cc["assoc"],
        phy_addr_width=cc["phy_addr_width"],
        entry_counts=cc["entry_counts"],
        cache_policy=cc["cache_policy"],
        fetch_or_memory_ports=cc.get("fetch_or_memory_ports", 1),
        device_ty=cc.get("device_ty", "core"),
        wire_is_mat_type=cc.get("wire_is_mat_type", 0),
        wire_os_mat_type=cc.get("wire_os_mat_type", None),
        wt_overhead=cc.get("wt_overhead", 30),
        partitions=None,
    )

    if level == "shared":
        role_specs = list(_SHARED_BUFFER_ROLES)
    else:
        role_specs = list(_L1_BUFFER_ROLES)
        if level == "l1d" and cc["cache_policy"] == "write_back":
            role_specs.append(("WBB", "Writebackb"))

    buffers = {
        pub: _memory_from_buffer_array(buf_arrays[f"buf{raw}"], sys_params)
        for raw, pub in role_specs
    }
    return {"mem": mem, "buffers": buffers}


def _mem_triple(mem):
    """The ``{subthreshold, gate, subthreshold_longer_channel}`` static-power
    triple read straight off an already-built ``CactiMemory`` -- no new
    arithmetic, matching ``CactiNoC.static_power()``'s own naming
    convention (``subthreshold`` = raw/unreduced, ``subthreshold_longer_
    channel`` = the longer-channel-REDUCED value)."""
    return {
        "subthreshold": mem.subthreshold_leakage_raw,
        "gate": mem.gate_leakage,
        "subthreshold_longer_channel": mem.subthreshold_leakage,
    }


class CactiRubyHierarchy:
    """Facade over one MESI-coherent L1I/L1D/L2 + directory + NoC hierarchy.
    See module docstring for the exact ``_l1i``/``_l1d``/``_l2``/
    ``_directory``/``_noc``/``_sys_params`` config-dict contract.

    Runs every search (the 3 cache pairs' + their buffers', the directory's
    + ITS buffers') and the NoC construction in ``__init__``; the resulting
    per-access dynamic energies and static-power coefficients are then
    read, unmodified, off the already-built ``CactiMemory``/
    ``build_directory()``/``CactiNoC`` objects by ``activation_energies()``/
    ``static_power()``.

    NoC scope note (correct-by-delegation, out of this plan's MESI_Two_Level
    target but not specially guarded here): ``_noc["params"]["type"]==0``
    (the bus path, ``CactiNoC.init_link_bus``) is accepted -- it simply
    changes ``self.noc``'s own shape (``.router`` is ``None``, ``.link_bus``
    is set), which ``activation_energies()`` reflects via a ``"Bus"`` key
    (from ``CactiNoC.activation_energies()`` itself) INSTEAD of
    ``NoCBuffer``/``NoCCrossbar``/``NoCArbiter``. ``static_power()`` will
    raise ``NotImplementedError`` in that case, since it calls
    ``self.noc.static_power()`` directly and that method is, by
    ``CactiNoC``'s own documented design, only defined for the router
    (``type==1``) path -- see that class's ``static_power`` docstring.

    ``_l2_device_type`` (Task 5b): L2 gets its own ``sys_params``
        (device_type overridden, everything else identical) instead of
        sharing ``_sys_params`` with L1I/L1D/Directory/NoC -- see the
        module docstring's ``_sys_params`` paragraph and ``_l2_sys_params()``
        for the full rationale/precedent/mechanism.
    """

    def __init__(
        self,
        *,
        _l1i,
        _l1d,
        _l2,
        _directory,
        _noc,
        _sys_params,
        _l2_device_type=0,
    ):
        self._sys_params = _sys_params
        self._l1i = _build_cache_pair(_l1i, _sys_params)
        self._l1d = _build_cache_pair(_l1d, _sys_params)

        # L2 gets its OWN sys_params -- see _l2_sys_params()'s own
        # docstring for the full mechanism/verification.
        self._l2_device_type = _l2_device_type
        l2_sys_params = _l2_sys_params(_sys_params, _l2_device_type)
        self._l2 = _build_cache_pair(_l2, l2_sys_params)
        self._directory_result = build_directory(
            sys_params=_sys_params, **_directory
        )

        noc_kwargs = dict(_noc)
        params = noc_kwargs.pop("params")
        embedded = noc_kwargs.pop("embedded")
        link_len = noc_kwargs.pop("link_len", None)
        self._circuit = CactiCircuit(_sys_params)
        self.noc = CactiNoC(self._circuit, params, embedded, link_len=link_len)

    # ------------------------------------------------------------------
    def activation_energies(self):
        """``{"DataCacheData": {Read,Write}, "DataCacheTag": {Read,Write},
        "DataCache": {Read,Write} (data+tag summed), "DataCache<role>":
        {Read,Write,Search} for role in (Missb,Ifb,Prefetchb[,Writebackb]),
        ...InstCache*/L2Cache* mirrors..., "Directory": {Read,Write}
        (``build_directory``'s own already-summed data+tag value),
        "DirectoryTag": {Read,Write} (the directory's OWN tag array's
        read/write energy -- the SAME ``*Tag`` pattern ``DataCache``/
        ``InstCache``/``L2Cache`` get above, not a new computation: reads
        ``d["array"].tag_read``/``.tag_write`` off the already-built
        ``CactiMemory`` -- see ``sharedcache.cc:777-780``'s real formula,
        which needs this tag-only value for its ``writeAc.miss*TagWrite_AE``
        term, distinct from the data+tag-combined ``Directory`` entry),
        "Directory<role>": {Read,Write,Search} for role in
        (Missb,Ifb,Prefetchb,Wbb), "NoCBuffer": {Read,Write},
        "NoCCrossbar": {Read}, "NoCArbiter": {Read}}``.

        Every value is read straight off an already-built ``CactiMemory``/
        ``build_directory()`` result/``CactiNoC`` -- ``DataCache``/
        ``InstCache``/``L2Cache``'s own ``{Read,Write}`` entry is the one
        piece of arithmetic this method performs (``mem.read + mem.tag_read``
        / ``mem.write + mem.tag_write``), the SAME "data + tag summed"
        convention ``cacti_directory.py``'s own module docstring documents
        for the single combined directory array (and ``machine_model.py``'s
        BTB/L2 comment) -- not new energy math, just the established
        aggregation rule for a data+tag cache pair.
        """
        ae = {}
        for pub, cache in (
            ("DataCache", self._l1d),
            ("InstCache", self._l1i),
            ("L2Cache", self._l2),
        ):
            mem = cache["mem"]
            ae[f"{pub}Data"] = {"Read": mem.read, "Write": mem.write}
            ae[f"{pub}Tag"] = {"Read": mem.tag_read, "Write": mem.tag_write}
            ae[pub] = {
                "Read": mem.read + mem.tag_read,
                "Write": mem.write + mem.tag_write,
            }
            for role, bmem in cache["buffers"].items():
                ae[f"{pub}{role}"] = {
                    "Read": bmem.read,
                    "Write": bmem.write,
                    "Search": bmem.search,
                }

        d = self._directory_result
        ae["Directory"] = dict(d["activation_energies"])
        # d["array"] is always a _has_tag=True CactiMemory (build_directory
        # hardcodes _has_tag=True for the main array -- see that module's
        # own docstring/source), so .tag_read/.tag_write are never None
        # here; propagated verbatim, no coercion.
        ae["DirectoryTag"] = {
            "Read": d["array"].tag_read,
            "Write": d["array"].tag_write,
        }
        for pub_role, raw_role in _DIRECTORY_ROLE_MAP.items():
            bmem = d["buffers"][raw_role]
            ae[f"Directory{pub_role}"] = {
                "Read": bmem.read,
                "Write": bmem.write,
                "Search": bmem.search,
            }

        ae.update(self.noc.activation_energies())
        return ae

    # ------------------------------------------------------------------
    def static_power(self):
        """``{"DataCache": {...}, "DataCache<role>": {...}, ...InstCache*/
        L2Cache* mirrors..., "Directory": {...}, "Directory<role>": {...},
        "NoC": {...}}``, each a ``{subthreshold, gate,
        subthreshold_longer_channel}`` triple.

        ``DataCache``/``InstCache``/``L2Cache`` report the data+tag-COMBINED
        triple (``CactiMemory``'s own ``subthreshold_leakage_raw``/
        ``gate_leakage``/``subthreshold_leakage`` attributes already sum
        data+tag for a ``_has_tag=True`` memory -- see that class's
        ``_set_static_attrs``) -- there is no separate DataCacheData/
        DataCacheTag static-power split, unlike ``activation_energies()``'s
        dynamic-energy dict: ``CactiMemory``'s public API has no per-side
        leakage attribute to read (only the combined one), and splitting it
        would mean reaching into its private ``_data_st``/``_tag_st``
        instead of delegating to an already-exposed number -- see the
        advisor checkpoint in the task-8 report for this scope decision.
        Every buffer role and ``Directory`` get their own triple (buffers
        and the directory's main array are exposed as single ``CactiMemory``
        objects with nothing to split). ``NoC`` is
        ``self.noc.static_power()`` verbatim (router-level only, matching
        that method's own documented scope).
        """
        sp = {}
        for pub, cache in (
            ("DataCache", self._l1d),
            ("InstCache", self._l1i),
            ("L2Cache", self._l2),
        ):
            sp[pub] = _mem_triple(cache["mem"])
            for role, bmem in cache["buffers"].items():
                sp[f"{pub}{role}"] = _mem_triple(bmem)

        d = self._directory_result
        sp["Directory"] = _mem_triple(d["array"])
        for pub_role, raw_role in _DIRECTORY_ROLE_MAP.items():
            sp[f"Directory{pub_role}"] = _mem_triple(d["buffers"][raw_role])

        sp["NoC"] = self.noc.static_power()
        return sp

    # ------------------------------------------------------------------
    @classmethod
    def from_config(cls, cfg):
        """Build a ``CactiRubyHierarchy`` from one nested config dict, for a
        future gem5-pm bridge caller (Task 9/10). Expected shape::

            {
                "l1i": {...}, "l1d": {...}, "l2": {...},   # see module
                "directory": {...}, "noc": {...},           # docstring
                "sys_params": <CactiParams>,                 # OR:
                "machine_params": {...}, "core_params": {...},
                "l2_device_type": 0,   # optional, default 0 (Task 5b)
            }

        If ``cfg["sys_params"]`` is given it is used directly (matching
        ``CactiMemory``'s own convention of taking a pre-built
        ``CactiParams``); otherwise ``cfg["machine_params"]``/
        ``cfg["core_params"]`` are passed to
        ``machine_model.build_cacti_params`` to build one. A straightforward,
        documented mapping, not an attempt to anticipate every possible
        gem5/XML config shape -- Task 9/10 (the actual gem5-pm bridge
        caller) haven't been built yet, so there is nothing concrete to
        generalize for beyond this.

        ``cfg["l2_device_type"]`` (default ``0``, Task 5b) is forwarded as
        ``_l2_device_type`` -- ``__init__`` recovers the raw machine_params/
        core_params it needs to build L2's own sys_params straight from
        whatever ``sys_params`` ends up being (see ``_l2_sys_params()``),
        so this works the same way whether ``cfg["sys_params"]`` was given
        directly or built here from ``cfg["machine_params"]``/
        ``cfg["core_params"]``.

        :param cfg: the nested config dict described above
        :returns: the constructed ``CactiRubyHierarchy``
        """
        sys_params = cfg.get("sys_params")
        if sys_params is None:
            sys_params = build_cacti_params(
                cfg["machine_params"], cfg["core_params"]
            )
        return cls(
            _l1i=cfg["l1i"],
            _l1d=cfg["l1d"],
            _l2=cfg["l2"],
            _directory=cfg["directory"],
            _noc=cfg["noc"],
            _sys_params=sys_params,
            _l2_device_type=cfg.get("l2_device_type", 0),
        )
