# SPDX-License-Identifier: BSD-3-Clause
"""Compatibility adapter for live gem5 configuration and McPAT composition."""

import json
from functools import lru_cache
from types import SimpleNamespace

from .gem5_derivation import (
    _DEVICE_MAP,
    _IC_MAP,
    _check_device_type_at_node,
    _derive_buffer_entry_counts,
    _positive,
    derive_core_params,
    derive_machine_params,
    derive_predictor_params,
)
from .mcpat_solver.composition import (
    _CACHE_CONFIG_KEYS,
    _ArraySearch,
    _buffer_entry_counts,
    _cache_shape,
    _cache_tag_width,
    _core_arrays_with_live_geometry,
    _live_l1_cache,
    _live_renaming_issue_rob_arrays,
    _live_shared_cache,
    _live_tlb_arrays,
    _ports,
    _search_geometry_buffers,
    _thread_tag_bits,
    build_machine_model_from_params,
)

_OPTIONAL_OVERRIDE_KEYS = {"machine": ("vdd",)}


def _apply_overrides(groups, overrides):
    """Update each of `groups` ({name: dict}) with `overrides[name]`;
    a name or key that is neither present nor optional raises KeyError."""
    for name, values in overrides.items():
        target = groups[name]
        for key in values:
            if key not in target and not (
                name in _OPTIONAL_OVERRIDE_KEYS
                and key in _OPTIONAL_OVERRIDE_KEYS[name]
            ):
                raise KeyError(f"unknown {name} override key {key!r}")
        target.update(values)


def build_machine_model_from_config(
    board,
    cpu_type,
    l1icache=None,
    l1dcache=None,
    l2cache=None,
    tech_node=None,
    num_threads=None,
    core_device_type=None,
    cache_device_type=None,
    interconnect_type=None,
    cache_configs=None,
    opt_for_clk=False,
    opt_local=False,
    overrides=None,
    buffer_policy="mcpat",
    tag_widths=None,
    reuse=False,
):
    """MachineModel from the live `board`.

    Machine, core and predictor params are derived from the board, then
    `overrides` ({"machine"|"core"|"pred": {key: value}}) replaces McPAT-only
    values gem5 cannot express; an unknown group or key raises KeyError.

    `tech_node` (nm), `core_device_type` / `cache_device_type`
    ("hp"/"lstp"/"lop") and `interconnect_type` ("aggressive"/
    "conservative") select the technology; `None` keeps the anchor value.
    `num_threads` sets McPAT's structural hardware-thread count.
    `l1icache`/`l1dcache`/`l2cache` (cache SimObjects) live-size those arrays
    when given.

    `cache_configs` holds McPAT descriptions gem5 has no source for, keyed as
    the McPAT XML (any subset of `_CACHE_CONFIG_KEYS`): `*_config` tuples
    (shape, banks, cycles; an `L2_config` replaces the anchor L2),
    `*_buffer_sizes` buffer entry counts, `BTB_config`, and
    `itlb_entries`/`dtlb_entries`. A given config wins over the matching
    SimObject. `opt_for_clk`/`opt_local` are McPAT's Layer-3 gate inputs
    (-opt_for_clk, core0 opt_local) for every searched array.
    """
    machine_params = derive_machine_params(
        board,
        tech_node=tech_node,
        device_type=core_device_type,
        interconnect_type=interconnect_type,
    )
    core = board.get_processor().get_cores()[0]
    core_params = derive_core_params(board, core, cpu_type)
    if num_threads is not None:
        core_params["num_threads"] = num_threads
    predictor_params = derive_predictor_params(core)
    core_params["ras_sz"] = predictor_params["ras_entries"]
    _apply_overrides(
        {
            "machine": machine_params,
            "core": core_params,
            "pred": predictor_params,
        },
        {} if overrides is None else overrides,
    )
    changes = overrides or {}
    core_ras = changes.get("core", {}).get("ras_sz")
    pred_ras = changes.get("pred", {}).get("ras_entries")
    if core_ras is not None and pred_ras is not None and core_ras != pred_ras:
        raise ValueError("core ras_sz and predictor ras_entries must agree")
    if core_ras is not None:
        predictor_params["ras_entries"] = core_ras
    core_params["ras_sz"] = predictor_params["ras_entries"]
    l2_device_type = 0
    if cache_device_type is not None:
        _check_device_type_at_node(
            "cache_device_type", cache_device_type, machine_params["tech_node"]
        )
        l2_device_type = _DEVICE_MAP[cache_device_type]
    cache_line_size = (
        int(board.get_cache_line_size())
        if any(c is not None for c in (l1icache, l1dcache, l2cache))
        else None
    )

    keywords = dict(
        cpu_type=cpu_type,
        l1icache=l1icache,
        l1dcache=l1dcache,
        l2cache=l2cache,
        cache_line_size=cache_line_size,
        l2_device_type=l2_device_type,
        cache_configs=cache_configs,
        opt_for_clk=opt_for_clk,
        opt_local=opt_local,
        buffer_policy=buffer_policy,
        tag_widths=tag_widths,
    )
    if not reuse:
        return build_machine_model_from_params(
            machine_params, core_params, predictor_params, **keywords
        )
    for name in ("l1icache", "l1dcache", "l2cache"):
        cache = keywords[name]
        if cache is not None:
            counts = _derive_buffer_entry_counts(cache)
            keywords[name] = dict(
                size=int(cache.size.value), assoc=int(cache.assoc), **counts
            )
    payload = json.dumps(
        dict(
            machine=machine_params,
            core=core_params,
            predictor=predictor_params,
            keywords=keywords,
        ),
        sort_keys=True,
    )
    return _build_resolved_cached(payload)


@lru_cache(maxsize=8)
def _build_resolved_cached(payload):
    """Cache by resolved values, including geometry, ports, policy and clocks."""
    values = json.loads(payload)
    keywords = values["keywords"]
    for name in ("l1icache", "l1dcache", "l2cache"):
        cache = keywords[name]
        if cache is not None:
            keywords[name] = SimpleNamespace(
                size=SimpleNamespace(value=cache["size"]),
                assoc=cache["assoc"],
                mshrs=cache["mshr"],
                write_buffers=cache["wbb"],
                prefetcher=(
                    SimpleNamespace(queue_size=cache["prefetch"])
                    if cache["prefetch"]
                    else None
                ),
            )
    return build_machine_model_from_params(
        values["machine"], values["core"], values["predictor"], **keywords
    )
