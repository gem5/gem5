from mcpat_power_model.runtime_support import (
    configure_sampling,
    temperature_kelvin,
)

import m5
from m5.objects import (
    BaseCPU,
    PowerModel,
    PowerModelPyFunc,
)

from ..mcpat_solver.machine_model import interpolated_static_power
from ..o3_mcpat_power_model.o3_mcpat_cpu_power_model import StagePM
from ..stage_duty_cycles import StageDutyCycles
from .inorder_mcpat_exec_power_model import InorderMcPATExecutePower
from .inorder_mcpat_fetch_power_model import InorderMcPATFetchPower
from .inorder_mcpat_lsu_power_model import InorderMcPATLsuPower
from .inorder_mcpat_mmu_power_model import InorderMcPATMmuPower


def build_stages(
    cpu,
    machine_model,
    duty_cycles,
    act_energies,
    interval=0,
    interval_ticks=0,
    stat_aliases=None,
):
    """Builds the four in-order stage power models for `cpu` (no rename)."""
    pipe = duty_cycles.pipeline
    stages = StagePM(
        fetch=InorderMcPATFetchPower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.ifu,
            interval,
            interval_ticks,
            machine_model=machine_model,
        ),
        rnu=None,
        lsu=InorderMcPATLsuPower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.lsu,
            interval,
            interval_ticks,
            machine_model=machine_model,
        ),
        mmu=InorderMcPATMmuPower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.lsu,
            interval,
            interval_ticks,
            machine_model=machine_model,
        ),
        # For a multithreaded core the unified instruction window is built
        # by McPAT under the Execution Unit, so the exec stage owns it.
        exec_=InorderMcPATExecutePower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.alu,
            interval,
            interval_ticks,
            num_threads=machine_model._core_params["num_threads"],
            machine_model=machine_model,
        ),
    )
    if stat_aliases:
        for stage in stages.stages():
            stage.set_stat_aliases(stat_aliases)
    return stages


class InorderMcPATCpuPowerOn(PowerModelPyFunc):
    def __init__(
        self,
        cpu: BaseCPU,
        machine_model,
        interval=0,
        interval_ticks=0,
        trace_debug=False,
        duty_cycles=StageDutyCycles(),
        stat_aliases=None,
    ):
        """core must be a BaseCPU core"""
        super().__init__()
        self._mm = machine_model
        self._interval = interval
        self._interval_ticks = interval_ticks
        self._sampling_enabled = interval > 0 or interval_ticks > 0
        self._trace_debug = trace_debug
        stages = build_stages(
            cpu,
            machine_model,
            duty_cycles,
            machine_model.cpu_activation_energies(),
            interval,
            interval_ticks,
            stat_aliases,
        )
        for stage in stages.stages():
            stage.set_sample_clock(self)
        self._fetch = stages.fetch
        self._lsu = stages.lsu
        self._mmu = stages.mmu
        self._exec = stages.exec_

        if self._sampling_enabled:
            configure_sampling(
                self,
                interval,
                interval_ticks,
                trace_debug,
                "numCycles",
                f"{m5.options.outdir}/power_trace.csv",
            )

        self.dyn = self.dynamic_power
        self.st = self.static_power

    def _set_sample_mode(self, enable: bool):
        self._fetch.set_sample_mode(enable)
        self._lsu.set_sample_mode(enable)
        self._mmu.set_sample_mode(enable)
        self._exec.set_sample_mode(enable)

    def clear_sample_state(self):
        for sub in [self._fetch, self._lsu, self._mmu, self._exec]:
            if hasattr(sub, "clear_sample_state"):
                sub.clear_sample_state()
            else:
                sub.reset_stats_dict()

    def static_power(self):
        return interpolated_static_power(
            self._mm, "Core", temperature_kelvin(self)
        )

    def dynamic_power(self):
        sample_mode = self._sampling_enabled and self.inPowerAtInterval()
        self._set_sample_mode(sample_mode)

        total = (
            self._fetch.dynamic_power()
            + self._lsu.dynamic_power()
            + self._mmu.dynamic_power()
            + self._exec.dynamic_power()
        )

        if not self._sampling_enabled:
            self.print_mcpat(6, total)

        if sample_mode:
            self.reset_stats_dict()

        return total

    def reset_stats_dict(self):
        self._fetch.reset_stats_dict()
        self._lsu.reset_stats_dict()
        self._mmu.reset_stats_dict()
        self._exec.reset_stats_dict()

    def print_mcpat(self, indent, total):
        print("*" * 80)
        print("Core:")
        print(" " * indent + f"Runtime Dynamic = {total}\n")
        self._fetch.print_mcpat(indent)
        self._lsu.print_mcpat(indent)
        self._mmu.print_mcpat(indent)
        self._exec.print_mcpat(indent)
        print("*" * 80)


class InorderMcPATCpuPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class InorderMcPATCpuPowerModel(PowerModel):
    def __init__(
        self,
        core,
        machine_model,
        interval=0,
        interval_ticks=0,
        trace_debug=False,
        duty_cycles=StageDutyCycles(),
        stat_aliases=None,
    ):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            InorderMcPATCpuPowerOn(
                core,
                machine_model,
                interval,
                interval_ticks,
                trace_debug,
                duty_cycles,
                stat_aliases,
            ),  # ON
            InorderMcPATCpuPowerOff(),  # CLK_GATED
            InorderMcPATCpuPowerOff(),  # SRAM_RETENTION
            InorderMcPATCpuPowerOff(),  # OFF
        ]
