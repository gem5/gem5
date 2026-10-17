from mcpat_power_model.runtime_support import (
    configure_sampling,
    temperature_kelvin,
)

"""McPAT cache PowerModel wrappers; each owns one cache stage."""

from mcpat_power_model.mcpat_solver.machine_model import (
    interpolated_static_power,
)

import m5
from m5.objects import (
    PowerModel,
    PowerModelPyFunc,
)


class CachePowerOn(PowerModelPyFunc):
    def __init__(
        self,
        stage_cls,
        cache,
        machine_model,
        interval=0,
        interval_ticks=0,
        trace_debug=False,
    ):
        super().__init__()
        self._mm = machine_model
        self._sampling_enabled = interval > 0 or interval_ticks > 0
        self._trace_debug = trace_debug
        self._stage = stage_cls(
            cache,
            machine_model.cache_activation_energies()[stage_cls.STATIC_BUCKET],
            interval,
            interval_ticks,
            machine_model=machine_model,
            sample_clock=self,
        )
        if self._sampling_enabled:
            configure_sampling(
                self,
                interval,
                interval_ticks,
                trace_debug,
                "overallAccesses",
                f"{m5.options.outdir}/power_trace.csv",
            )
        self.dyn = self.dynamic_power
        self.st = self.static_power

    def clear_sample_state(self):
        self._stage.clear_sample_state()

    def static_power(self):
        return interpolated_static_power(
            self._mm, self._stage.STATIC_BUCKET, temperature_kelvin(self)
        )

    def dynamic_power(self):
        sample_mode = self._sampling_enabled and self.inPowerAtInterval()
        self._stage.set_sample_mode(sample_mode)
        power = self._stage.dynamic_power()
        if self._trace_debug:
            print(f"{self._stage.name} power: {power}")
        if sample_mode:
            self._stage.reset_stats_dict()
        return power


class CachePowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class CachePowerModel(PowerModel):
    def __init__(
        self,
        stage_cls,
        cache,
        machine_model,
        power_interval=0,
        power_interval_ticks=0,
        trace_debug=False,
    ):
        super().__init__()
        # ON, CLK_GATED, SRAM_RETENTION, OFF.
        self.pm = [
            CachePowerOn(
                stage_cls,
                cache,
                machine_model,
                power_interval,
                power_interval_ticks,
                trace_debug,
            ),
            CachePowerOff(),
            CachePowerOff(),
            CachePowerOff(),
        ]
