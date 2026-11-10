# SPDX-License-Identifier: BSD-3-Clause
"""Per-operation energies are joules; leakage coefficients are watts."""

import math
from copy import deepcopy


class StaticCoefficients:
    """Memoized per-component leakage; interpolate independently at 10 K."""

    def __init__(
        self, at_sample, *, partitions=None, timing=None, resolved_config=None
    ):
        self._at_sample = at_sample
        self._samples = {}
        self.partitions = deepcopy(partitions or {})
        self.timing = deepcopy(timing)
        self.resolved_config = deepcopy(resolved_config or {})

    def _sample(self, temperature):
        if temperature not in self._samples:
            rows = self._at_sample(temperature)
            for values in rows.values():
                if set(values) != {"leakage", "gate_leakage"}:
                    raise ValueError("both leakage terms are required")
                if any(not math.isfinite(v) or v < 0 for v in values.values()):
                    raise ValueError("leakage must be finite and nonnegative")
            self._samples[temperature] = deepcopy(rows)
        return self._samples[temperature]

    def per_component(self, temperature):
        t = float(temperature)
        if not math.isfinite(t):
            raise ValueError("temperature must be finite kelvin")
        t = max(300.0, min(400.0, t))
        lo = int(t // 10) * 10
        low = self._sample(lo)
        if t == lo:
            return deepcopy(low)
        high = self._sample(lo + 10)
        if low.keys() != high.keys():
            raise ValueError("temperature samples have different components")
        fraction = (t - lo) / 10
        return {
            name: {
                key: value + fraction * (high[name][key] - value)
                for key, value in values.items()
            }
            for name, values in low.items()
        }

    def _sum(self, field, temperature, components):
        rows = self.per_component(temperature)
        selected = rows.keys() if components is None else components
        return sum(rows[n][field] for n in selected)

    def subthreshold(self, temperature, components=None):
        return self._sum("leakage", temperature, components)

    def gate(self, temperature, components=None):
        return self._sum("gate_leakage", temperature, components)

    def total_power(self, temperature, components=None):
        return self.subthreshold(temperature, components) + self.gate(
            temperature, components
        )
