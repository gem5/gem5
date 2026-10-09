# SPDX-License-Identifier: BSD-3-Clause
"""Adapt power callbacks to the destination's existing PyFunc capabilities."""


def temperature_kelvin(state):
    method = getattr(state, "getTemperatureKelvin", None)
    if method is not None:
        return method()
    # The destination exports no thermal probe. Unsampled models use their
    # owning PowerModel's configured ambient temperature, in kelvin.
    return float(state.get_parent().ambient_temp.value)


def configure_sampling(state, cycles, ticks, trace, clock_stat, trace_file):
    """Use the sampling parameters provided by the installed gem5 binary."""
    parameters = state._params
    if ticks and "pwr_interval_ticks" not in parameters:
        raise ValueError("this gem5 binary supports cycle sampling only")
    values = dict(
        pwr_interval=cycles,
        pwr_interval_ticks=ticks,
        clock_stat=clock_stat,
        enable_trace=trace,
        trace_file=trace_file,
    )
    for name, value in values.items():
        if name in parameters:
            setattr(state, name, value)
