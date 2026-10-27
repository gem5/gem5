"""Bridge from gem5 to the vendored mcpat_solver package
(mcpat_power_model/mcpat_solver/): the static anchor parameters and cached
default MachineModel that supply activation energies and leakage. The live,
board-derived path is mcpat_solver_bridge_live.py.
"""

import math

from .mcpat_solver.defaults import (
    _BASE_MACHINE_PARAMS,
    _base_core_params,
    _is_ooo,
)
from .mcpat_solver.machine_model import (
    TLB,
    MachineModel,
)
from .mcpat_solver.presets import anchor_tlbs
from .mcpat_solver_bridge_live import build_machine_model_from_config

_cached_machine_model = None
_cached_cpu_type = None


def build_default_machine_model(
    cpu_type,
    board=None,
    l1icache=None,
    l1dcache=None,
    l2cache=None,
    tech_node=None,
    **config_kwargs,
):
    """One cached MachineModel per cpu_type, or, given `board`, a live model
    from build_machine_model_from_config() (never cached).

    `l1icache`/`l1dcache`/`l2cache` (cache SimObjects) live-size those arrays
    when given; omit them before board._connect_things() has run. `tech_node`
    (nm) overrides the default node and bypasses the cache on the no-`board`
    path. `config_kwargs` (e.g. `cache_configs`, `overrides`) are forwarded to
    the live build and need a `board`."""
    if config_kwargs and board is None:
        raise ValueError(
            f"{sorted(config_kwargs)} need a live `board` to derive from"
        )
    if board is not None:

        return build_machine_model_from_config(
            board,
            cpu_type,
            l1icache=l1icache,
            l1dcache=l1dcache,
            l2cache=l2cache,
            tech_node=tech_node,
            **config_kwargs,
        )

    if tech_node is not None:
        core_params = _base_core_params(cpu_type)
        return MachineModel(
            dict(_BASE_MACHINE_PARAMS, tech_node=int(tech_node)),
            core_params,
            **anchor_tlbs(core_params),
        )

    global _cached_machine_model, _cached_cpu_type
    if _cached_machine_model is not None and _cached_cpu_type == cpu_type:
        return _cached_machine_model

    core_params = _base_core_params(cpu_type)
    _cached_machine_model = MachineModel(
        _BASE_MACHINE_PARAMS, core_params, **anchor_tlbs(core_params)
    )
    _cached_cpu_type = cpu_type
    return _cached_machine_model


def overlay_cpu_activation_energies(act_energies, cpu_type, model=None):
    """Overlay the model's CPU activation energies onto `act_energies`;
    `model` (e.g. a live one) defaults to the static default model."""
    model = (
        model if model is not None else build_default_machine_model(cpu_type)
    )
    for key, value in model.cpu_activation_energies().items():
        if key == "SelLogic" and key not in act_energies:
            continue  # only appears in the O3 branch of init_act_energies()
        act_energies[key] = value
    return act_energies


def overlay_cache_activation_energies(cpu_type, model=None):
    model = (
        model if model is not None else build_default_machine_model(cpu_type)
    )
    return model.cache_activation_energies()
