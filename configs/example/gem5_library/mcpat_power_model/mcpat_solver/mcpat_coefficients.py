# SPDX-License-Identifier: BSD-3-Clause
"""Common per-component leakage coefficients in watts."""


def _unit(cacti_params, leakage, longer_channel_leakage, gate_leakage):
    """One leakage-inventory entry. McPAT's XML longer_channel_device selects
    the printed subthreshold leakage (processor.cc/core.cc/sharedcache.cc
    displayEnergy); gate leakage has no reduced form."""
    long_channel = cacti_params._machine_config._config_params[
        "longer_chan_dev"
    ]
    return {
        "leakage": longer_channel_leakage if long_channel else leakage,
        "gate_leakage": gate_leakage,
    }


def _array_unit(cacti_params, a):
    """_unit() of a built array or Interconnect."""
    return _unit(
        cacti_params, a.leakage, a.longer_channel_leakage, a.gate_leakage
    )


def _logic_unit(cacti_params, c):
    """_unit() of a logic block, which keeps its plain values in _power.read."""
    p = c._power.read
    return _unit(
        cacti_params, p.leakage, c.longer_channel_leakage, p.gate_leakage
    )
