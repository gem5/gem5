import xml.etree.ElementTree as ET

from m5.objects import Root

from ..mcpat_solver.machine_model import _interpolated_leakage
from .base_power_model import AbstractPowerModel


class McPATPowerModel(AbstractPowerModel):
    # McPAT "Core" leakage units this model owns directly (not its children's).
    STATIC_UNITS = ()
    # Attributes holding sub-models (or None) whose static power is included.
    STATIC_CHILDREN = ()
    # True if this stage carries num_pipelines / num_units of the pipeline
    # registers' leakage, like the dynamic pipeline term.
    STATIC_PIPELINE_SHARE = False

    def __init__(
        self,
        simobj,
        act_energies,
        interval=0,
        interval_ticks=0,
        machine_model=None,
    ):
        super().__init__(simobj, interval, interval_ticks)
        self.name = "McPATPowerModel"
        self._act_energies = act_energies
        self._mm = machine_model

    def static_leakage(self, temp_k):
        """(subthreshold, gate) leakage in W at `temp_k` (own units, children,
        and pipeline share), interpolated like the CPU's Core bucket."""
        sub, gate = _interpolated_leakage(
            self._mm, "Core", temp_k, units=self.STATIC_UNITS
        )
        for attr in self.STATIC_CHILDREN:
            child = getattr(self, attr)
            if child is not None:
                child_sub, child_gate = child.static_leakage(temp_k)
                sub += child_sub
                gate += child_gate
        if self.STATIC_PIPELINE_SHARE:
            share = self._num_pipelines / self._num_units
            pipe_sub, pipe_gate = _interpolated_leakage(
                self._mm, "Core", temp_k, units=("Pipeline",)
            )
            sub += pipe_sub * share
            gate += pipe_gate * share
        return sub, gate

    def static_power(self, temp_k) -> float:
        """Returns static power in Watts at `temp_k`."""
        sub, gate = self.static_leakage(temp_k)
        return sub + gate

    def convert_to_watts(self, value: float) -> float:
        """Note that McPAT AEs are already in terms of J,
        no need for conversion"""

        time = self.getExecutionTime()
        if time == 0:
            return 0.0
        return value / time
