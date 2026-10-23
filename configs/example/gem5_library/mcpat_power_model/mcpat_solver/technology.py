# SPDX-License-Identifier: BSD-3-Clause
"""Shared technology configuration; no machine composition dependency."""

from .cacti_params import CactiParams
from .config_input_params import McPATMachineConfig


def clamp_to_cacti_decade(temp_k):
    """CACTI/McPAT's I_off table only accepts 300-400K in steps of 10 (the
    real contract, not a bug) -- snap onto it."""
    t = max(300.0, min(400.0, float(temp_k)))
    return int(round(t / 10.0)) * 10


def fresh_machine_config(machine_params, core_params, temperature=None):
    """McPATMachineConfig mutates its input dicts in place and can't be
    built twice from the same dict -- always copy first."""
    mp = dict(machine_params)
    if temperature is not None:
        mp["temperature"] = temperature
    cp = dict(core_params)
    return McPATMachineConfig(mp, cp, mp, mp, mp, mp, mp)


def build_cacti_params(machine_params, core_params, temperature=None):
    return CactiParams(
        fresh_machine_config(machine_params, core_params, temperature)
    )


def mcpat_wire_kwargs(machine_params, shared=False):
    """McPAT's per-array wire setting, from the system-level Embedded flag:
    processor.cc:810-821 (core arrays) and sharedcache.cc:75-86 (shared
    L2/L3 and its buffers, whose embedded H-tree layer is semi-global).
    `machine_params` is the machine-params dict (or a CactiParams'
    `_machine_config._config_params`)."""
    if machine_params["embedded"]:
        return dict(
            wire_is_mat_type=0,
            wire_os_mat_type=1 if shared else 0,
            wt_overhead=30,
        )
    return dict(wire_is_mat_type=2, wire_os_mat_type=2, wt_overhead=0)
