"""build_directory(): MESI directory-cache ("DC") builder.

Models McPAT's ``sharedcache.cc`` ``SharedCache`` class for a Ruby-MESI
directory (``L1Directory``/``L2Directory`` XML sections), on the non-ST
(set-associative, ``dir_ty != ST``) path only -- ST's ``assoc=0``
shadow-tag directory is out of scope for this port (see
``docs`` / the task-7 brief). This is the same C++ class McPAT uses for a
shared L2/L3 cache: a directory is a set-associative array plus 4
miss/fill/prefetch/write-back buffers, with no coherency-protocol logic of
its own (that's genuinely out of scope project-wide, per CLAUDE.md's
"NoC/router/cache-coherency protocol energy is out of scope by design").

Pure composition over already-validated primitives -- no new CACTI
physics:

  * the main directory array -> ``cacti_memory.CactiMemory`` with
    ``_has_tag=True`` (the SAME data+tag "cache pair" construction
    ``test_cacti_memory.py`` uses for icache/dcache/L2, per
    ``CactiMemory``'s own contract, and NOT a `_has_tag=False` single
    array). This matches real CACTI: even though ``sharedcache.cc``
    allocates only ONE ``ArrayST`` for the directory (``specific_tag=1``,
    ``tag_w=<real>`` baked into one ``interface_ip``), CACTI's own
    ``Ucache::find_optimal_uca`` (called inside that ONE ``ArrayST``'s
    ctor) unconditionally enumerates BOTH a tag-side and a data-side
    candidate list and combines them -- a data-only (``is_tag=False``)
    ``MemoryParameter`` always has ``tagbits=0`` (``parameter.cc``: "if
    data array, let tagbits = 0"), so ``tag_w``/``specific_tag`` are
    completely inert unless a genuine tag-side search also runs. The
    single physical array's reported read/write energy is the SUM of the
    tag-side and data-side contributions (``mem.read + mem.tag_read`` /
    ``mem.write + mem.tag_write``), the same "data + tag summed"
    convention ``machine_model.py`` already uses for BTB/L2 (see its own
    comment: "gem5-pm's BTB activation energy is data + tag ... summed").
    The reported partition/mat-area are the DATA side's (``mem.partition``
    / ``mem.mat_area``) -- confirmed against the fixture, which carries
    only one (data-side) partition/mat-area pair per structure.

  * ``device_ty`` should be ``"llc"`` for a real caller, NOT ``"uncore"``:
    verified directly against ``sharedcache.cc:58-70`` -- ``cacheL`` for a
    directory is ``L1Directory``/``L2Directory``, never ``L2``, so the
    ``cacheL==L2 && Private_L2`` branch that would pick ``Core_device``
    never applies, and every directory (array + all 4 buffers) gets
    ``LLC_device`` (this port's ``"llc"`` string). This function still
    accepts ``device_ty`` as a plain pass-through parameter and warns
    (``RuntimeWarning``) rather than raising when a non-``"llc"`` value is
    passed, since a caller may legitimately want to test another value;
    see the ``device_ty`` parameter docstring below for the full source
    citation.

    Critically, ``_data_assoc=1`` is required for this array:
    ``io.cc``'s ``error_checking`` (``cacti_interface.h``'s
    ``tag_assoc``/``data_assoc``) sets ``data_assoc=1`` whenever
    ``is_seq_acc`` (``access_mode==1``) is true, regardless of the real
    associativity -- sequential access reads the tag array first and only
    the ONE matched way's data column, so the data subarray is sized as
    if 1-way. ``cacti_dynamic_params.py`` does NOT auto-derive this from
    ``is_seq_acc`` (it defaults ``data_assoc`` to plain ``assoc`` when not
    given -- see that file's own comment at the ``data_assoc`` default);
    the caller must pass it explicitly, exactly as ``machine_model.py``'s
    own ``L2`` literal does (``data_assoc=1, is_seq_acc=True``). Omitting
    it here reproduces a real, measured ~8x energy / wrong-partition
    divergence (found empirically while implementing this module -- see
    the task-7 report);
  * the 4 buffers -> ``cache_buffers.build_cache_buffers(level="shared",
    ...)`` for geometry, each rebuilt through its own ``CactiMemory``
    search (or, when the caller supplies ``partitions["buffers"]``, a
    direct ``CactiArraySpec.build()`` from the given partition -- no search). The
    buffers are FA/CAM (``assoc=0``) with an embedded tag
    (``specific_tag=True``, ``_has_tag=False``) -- the FA/CAM path's
    geometry derivation is unaffected by ``data_assoc`` (``assoc==0``
    forces ``data_assoc=1`` in real CACTI too, and this port's FA
    candidate enumeration doesn't read it either), so no such adjustment
    is needed there.

``sharedcache.cc``'s non-ST branch (the formulas this module ports):

    idx = ceil(log2(size / line / assoc))
    tag = phys_addr_width - idx - ceil(log2(line)) + EXTRA_TAG_BITS
    num_search_ports = 0
    out_w = line_sz * 8 / 2
    access_mode = 1                      # is_seq_acc=True
    specific_tag = 1
    num_rw_ports = 1; num_rd_ports = 0; num_wr_ports = 0

and, for an Embedded config (this project's ARM_A9 40nm/LOP anchor, which
every ground-truth fixture here is built against --
``unit_tests/fixtures/build_ruby_fixture.py``'s ``meta.embedded: True``):

    wt = Global_30 (wt_overhead=30); wire_is_mat_type = 0; wire_os_mat_type = 1

``interface_ip`` is one mutable struct reused, UNMODIFIED on these three
fields, across all 5 ``ArrayST`` constructions in ``SharedCache``'s ctor
(the main array + all 4 buffers) -- so the same wire config applies to
every one of them here too.
"""

import math
import warnings

try:
    from .cache_buffers import build_cache_buffers
    from .cacti_dynamic_params import EXTRA_TAG_BITS
    from .cacti_memory import CactiMemory
except ImportError:  # vendored copy uses package-relative imports
    from cache_buffers import build_cache_buffers
    from cacti_dynamic_params import EXTRA_TAG_BITS
    from cacti_memory import CactiMemory

_VALID_LEVELS = ("L1", "L2")

# sharedcache.cc's Dir_config/buffer_sizes XML field order maps 1:1 onto
# cachep.missb_size / cachep.fu_size (the fill buffer) / cachep.prefetchb_size
# / cachep.wbb_size, in that order (sharedcache.cc:1197-1200,
# 1236-1239) -- this is the order build_directory's own `buffer_sizes`
# 4-tuple follows, and how it maps onto build_cache_buffers's
# {"mshr", "fill", "prefetch", "wbb"} entry_counts dict.
_BUFFER_ENTRY_KEYS = ("mshr", "fill", "prefetch", "wbb")

# Real McPAT's SharedCache ctor (sharedcache.cc:70-76): for an Embedded
# config it sets wt=Global_30 (this port's wt_overhead=30, via
# CactiMemory's Embedded-derived wt for a "search" provenance),
# wire_is_mat_type=0, wire_os_mat_type=1 -- and never touches those two
# fields again for the rest of the constructor, so the main array AND all
# 4 buffers share this one wire configuration. Matches machine_model.py's
# own L2/L2_BUFFERS literals (wire_is_mat_type=0, wire_os_mat_type=1).
_WIRE_IS_MAT_TYPE = 0
_WIRE_OS_MAT_TYPE = 1

# Global_30's overhead points (see machine_model.mcpat_wire_kwargs).
# CactiMemory applies this automatically for a "search" provenance
# (ignoring whatever _wt_overhead value is passed), but a "given"
# provenance uses the caller's _wt_overhead verbatim -- so
# _memory_from_buffer_array must pass it explicitly for the given-partition
# buffer path to match the searched path's wire config. Measured inert on
# this fixture's winning buffer geometries (bit-identical read/write/search
# at wt_overhead=0 vs 30 -- consistent with Phase 36's "inter-mat H-tree
# term is provably zero for a single-mat geometry" finding, since these
# buffers are single-mat), but passed through anyway rather than left as an
# unaudited asymmetry between the two code paths (CLAUDE.md: "measure,
# don't assert").
_WT_OVERHEAD = 30

# Buffer role -> the plain key this module's own return dict uses.
_BUFFER_ROLES = ("MissB", "FillB", "PrefetchB", "WBB")


def _cache_policy_str(cache_policy):
    """Translate build_directory's own cache_policy representation (an int,
    per the brief's own test call `cache_policy=1`, or already a string)
    into the string enum cache_buffers.build_cache_buffers requires.

    sharedcache.cc always builds all 4 buffers for a directory regardless
    of policy (no write-through variant on the LLC/shared path) -- so this
    translation only needs to satisfy build_cache_buffers's own
    ValueError-on-anything-else guard, not gate anything itself.
    """
    if cache_policy in ("write_back", "write_through"):
        return cache_policy
    if cache_policy == 1:
        return "write_back"
    if cache_policy == 0:
        return "write_through"
    raise ValueError(f"unrecognised cache_policy: {cache_policy!r}")


def _memory_from_buffer_array(arr, sys_params, *, given=None):
    """Build a CactiMemory reproducing one buffer CactiArraySpec's geometry (as
    returned by build_cache_buffers), by lifting its `_cfg_kwargs` into a
    CactiMemory -- either live-searched (`given=None`, the mirror image of
    `CactiMemory.from_array`, which lifts an CactiArraySpec's ALREADY-KNOWN
    partition as `("given", ...)`) or, when `given=(ints, mat)` is
    supplied, adopted verbatim (no search). Buffers are single FA/CAM
    arrays with an embedded tag (specific_tag=True, _has_tag=False) -- the
    same shape as test_cacti_memory.py's "fa_cam_512B_specific_tag" case.

    Always returns a CactiMemory (not an CactiArraySpec/CactiArrayST), so the two
    branches build_directory() dispatches between (searched vs. given per
    buffer) return the SAME type for the caller.
    """
    c = arr._cfg_kwargs
    partition = "search" if given is None else ("given", given[0], given[1])
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
        _wt_overhead=_WT_OVERHEAD,
        _sys_params=sys_params,
        _device_ty=arr._device_ty,
        _core_ooo=False,
        _partition=partition,
    )


def build_directory(
    level,
    dir_cfg,
    phys_addr_width,
    buffer_sizes,
    device_ty,
    cache_policy,
    clockrate,
    sys_params,
    partitions=None,
):
    """Build one MESI directory cache ("DC") -- the set-associative array
    plus its 4 miss/fill/prefetch/write-back buffers.

    :param level: "L1" or "L2" -- maps to McPAT's L1Directory/L2Directory
        (only "L2" is exercised by MESI_Two_Level; "L1" accepted for
        generality). Only used to pick the buffer trace-name prefix here.
    :param dir_cfg: (capacity_bytes, block_w_bytes, assoc, nbanks,
        throughput_cyc, latency_cyc) -- the Dir_config tuple. assoc must be
        > 0 (DC is set-associative; ST's assoc=0 shadow-tag path is out of
        scope).
    :param phys_addr_width: physical address width in bits.
    :param buffer_sizes: (mshr, fill, prefetch, wbb) entry counts -- the
        Dir_config buffer_sizes tuple, in cachep.missb_size / fu_size /
        prefetchb_size / wbb_size order (sharedcache.cc:1197-1200).
    :param device_ty: "core"/"uncore"/"llc" string enum (see
        CactiArrayST._long_channel_reduction) -- forwarded unchanged to the
        main array AND all 4 buffers, matching sharedcache.cc's single
        shared `device_t` local reused across all 5 ArrayST constructions.
        NOTE (verified against sharedcache.cc:58-70, not merely assumed):
        real McPAT sets `device_t = LLC_device` ("llc") for EVERY directory
        (cacheL is L1Directory/L2Directory, never L2, so the
        `cacheL==L2 && Private_L2` branch that picks Core_device never
        applies) -- not `"uncore"`. The task brief this module was written
        from asserted "directory is uncore"; that assertion does not match
        the C++ source. This function still accepts device_ty as a plain
        pass-through parameter (the caller decides), so nothing here is
        gated on which value is "correct" -- this only matters for
        `subthreshold_leakage`/`gate_leakage` (pct=0.82 "uncore" vs 1.0
        "llc" in _long_channel_reduction), which this function's own
        return value doesn't even expose. Flagged for whoever wires a real
        caller in a later task: pass "llc", not "uncore", to match real
        McPAT.
    :param cache_policy: 1/"write_back" or 0/"write_through". Real
        sharedcache.cc builds all 4 buffers for a directory regardless
        (no write-through variant on this path); kept as a parameter only
        to satisfy build_cache_buffers's own signature/validation.
    :param clockrate: accepted for interface symmetry with a real Dir_config
        caller; unused (throughput/latency are pre-converted to cycles in
        dir_cfg, and the energy rebuild here, like CactiMemory's own, is
        clock-independent -- see cacti_partition_search.py's
        Layer-3-only-clock-target note).
    :param sys_params: CactiParams for the target technology corner.
    :param partitions: None (default) -- live-search the main array AND
        every buffer. Otherwise a dict with an optional "buffers" key:

          * "buffers": {"MissB"|"FillB"|"PrefetchB"|"WBB": {"ints": ...,
            "mat": ...}} -- adopt this partition for the named buffer(s)
            verbatim (no search); any role omitted still searches.

        The main array always searches: CactiMemory's ("given", ...)
        provenance is data-only (it raises on _has_tag=True), and the
        main array here genuinely needs _has_tag=True (see module
        docstring) -- so there is no given-partition shortcut for it with
        the current CactiMemory API.

    :returns: {"array": CactiMemory, "buffers": {role: CactiMemory, ...}
        (all 4 roles, searched or given -- homogeneous type either way),
        "geometry": {...}, "activation_energies": {"Read": float,
        "Write": float}}
    """
    if level not in _VALID_LEVELS:
        raise ValueError(
            f"level must be one of {_VALID_LEVELS!r}, got {level!r}"
        )

    if device_ty != "llc":

        warnings.warn(
            f"build_directory({level!r}, ...): device_ty={device_ty!r}, but "
            'real McPAT sets device_t=LLC_device ("llc") for every '
            "directory (sharedcache.cc:58-70 -- cacheL is L1Directory/"
            "L2Directory, never L2, so the Core_device branch never "
            "applies). Passing anything else only changes "
            "subthreshold_leakage/gate_leakage (this function's own "
            "return value doesn't expose those), but a real caller should "
            'pass "llc" to match real McPAT.',
            RuntimeWarning,
            stacklevel=2,
        )

    capacity, block_w, assoc, nbanks, _throughput_cyc, _latency_cyc = dir_cfg
    if assoc <= 0:
        raise ValueError(
            "build_directory: assoc must be > 0 (DC is set-associative; "
            "ST's assoc=0 shadow-tag path is out of scope)"
        )

    idx = int(math.ceil(math.log2(capacity / block_w / assoc)))
    tag_w = (
        phys_addr_width
        - idx
        - int(math.ceil(math.log2(block_w)))
        + EXTRA_TAG_BITS
    )

    # ---- main array -----------------------------------------------------
    # _has_tag=True + _data_assoc=1: see module docstring -- a data-only
    # (_has_tag=False) array leaves tag_w/specific_tag completely inert for
    # geometry, and is_seq_acc=True requires data_assoc=1 (not the real
    # assoc) for the data-side subarray sizing.
    mem = CactiMemory(
        _capacity=capacity,
        _block_sz=block_w,
        _assoc=assoc,
        _out_w=(block_w * 8) // 2,
        _tag_w=tag_w,
        _has_tag=True,
        _num_rw_ports=1,
        _num_rd_ports=0,
        _num_wr_ports=0,
        _num_search_ports=0,
        _nbanks=nbanks,
        _device_ty=device_ty,
        _core_ooo=False,
        _is_seq_acc=True,
        _data_assoc=1,
        _wire_is_mat_type=_WIRE_IS_MAT_TYPE,
        _wire_os_mat_type=_WIRE_OS_MAT_TYPE,
        _sys_params=sys_params,
        _partition="search",
    )

    # ---- buffers ------------------------------------------------------
    policy_str = _cache_policy_str(cache_policy)
    prefix = (
        "Second_Level_Directory" if level == "L2" else "First_Level_Directory"
    )
    mshr, fill, prefetch, wbb = buffer_sizes
    entry_counts = dict(zip(_BUFFER_ENTRY_KEYS, (mshr, fill, prefetch, wbb)))

    given_buffers = (partitions or {}).get("buffers", {})

    # Always geometry-only here: build_cache_buffers's own `partitions`
    # kwarg (when not None) indexes ALL FOUR buffer keys unconditionally
    # (cache_buffers.py's `_arr`: `partitions[key]`), so it can't take a
    # PARTIAL per-role dict -- some roles here may be given, others
    # searched. Each role is resolved individually below instead, via
    # _memory_from_buffer_array's own given=None/given=(ints, mat) switch.
    buf_arrays = build_cache_buffers(
        level="shared",
        prefix=prefix,
        capacity=capacity,
        line_bytes=block_w,
        assoc=assoc,
        phy_addr_width=phys_addr_width,
        entry_counts=entry_counts,
        cache_policy=policy_str,
        fetch_or_memory_ports=1,
        device_ty=device_ty,
        wire_is_mat_type=_WIRE_IS_MAT_TYPE,
        wire_os_mat_type=_WIRE_OS_MAT_TYPE,
        partitions=None,
    )

    buffers = {}
    for role in _BUFFER_ROLES:
        arr = buf_arrays[f"{prefix}{role}"]
        if role in given_buffers:
            spec = given_buffers[role]
            buffers[role] = _memory_from_buffer_array(
                arr,
                sys_params,
                given=(tuple(spec["ints"]), tuple(spec["mat"])),
            )
        else:
            # Geometry-only CactiArraySpec (Nspd=Ndwl=...=1, mat_area=(0,0)) -- this
            # is a "live" caller (build_cache_buffers's own docstring:
            # "partitions=None -> geometry-only (live callers search per
            # buffer)"), so search it via CactiMemory rather than trust
            # those placeholder defaults.
            buffers[role] = _memory_from_buffer_array(arr, sys_params)

    geometry = {
        "idx": idx,
        "tag_w": tag_w,
        "assoc": assoc,
        "line": block_w,
        "size": capacity,
        "num_search_ports": 0,
        "partition": list(mem.partition),
        "mat_area_w": mem.mat_area[0],
        "mat_area_h": mem.mat_area[1],
    }
    # Single-physical-array convention (see module docstring): real
    # McPAT's one combined ArrayST's read/write energy is tag+data summed.
    activation_energies = {
        "Read": mem.read + mem.tag_read,
        "Write": mem.write + mem.tag_write,
    }

    return {
        "array": mem,
        "buffers": buffers,
        "geometry": geometry,
        "activation_energies": activation_energies,
    }
