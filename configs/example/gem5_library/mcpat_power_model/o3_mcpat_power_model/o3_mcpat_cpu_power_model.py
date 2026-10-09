import csv
import dataclasses
import pathlib

from mcpat_power_model.runtime_support import (
    configure_sampling,
    temperature_kelvin,
)

import m5
from m5.objects import (
    BaseO3CPU,
    PowerModel,
    PowerModelPyFunc,
)

from ..mcpat_solver.machine_model import interpolated_static_power
from ..stage_duty_cycles import StageDutyCycles
from .o3_mcpat_exec_power_model import O3McPATExecutePower
from .o3_mcpat_fetch_power_model import O3McPATFetchPower
from .o3_mcpat_lsu_power_model import O3McPATLsuPower
from .o3_mcpat_mmu_power_model import O3McPATMmuPower
from .o3_mcpat_renaming_unit_power_model import O3McPATRenamingUnitPower


@dataclasses.dataclass
class StagePM:
    fetch: object
    rnu: object  # None for inorder
    lsu: object
    mmu: object
    exec_: object

    def stages(self):
        s = [self.fetch]
        if self.rnu is not None:
            s.append(self.rnu)
        s += [self.lsu, self.mmu, self.exec_]
        return s


def build_stages(
    cpu,
    machine_model,
    duty_cycles,
    act_energies,
    interval=0,
    interval_ticks=0,
    stat_aliases=None,
):
    """Builds the five O3 stage power models for `cpu`.

    `duty_cycles` is a `StageDutyCycles`; `stat_aliases` maps requested stat
    names to the ones read instead (applied to every stage).
    """
    pipe = duty_cycles.pipeline
    stages = StagePM(
        fetch=O3McPATFetchPower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.ifu,
            interval,
            interval_ticks,
            machine_model=machine_model,
        ),
        rnu=O3McPATRenamingUnitPower(
            cpu,
            act_energies,
            pipe,
            interval,
            interval_ticks,
            machine_model=machine_model,
        ),
        lsu=O3McPATLsuPower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.lsu,
            interval,
            interval_ticks,
            machine_model=machine_model,
        ),
        mmu=O3McPATMmuPower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.lsu,
            interval,
            interval_ticks,
            machine_model=machine_model,
        ),
        exec_=O3McPATExecutePower(
            cpu,
            act_energies,
            pipe,
            duty_cycles.alu,
            interval,
            interval_ticks,
            isEmbedded=bool(machine_model.machine_params["embedded"]),
            machine_model=machine_model,
        ),
    )
    if stat_aliases:
        for stage in stages.stages():
            stage.set_stat_aliases(stat_aliases)
    return stages


class O3McPATCpuPowerOn(PowerModelPyFunc):
    def __init__(
        self,
        cpu: BaseO3CPU,
        machine_model,
        interval=0,
        interval_ticks=0,
        trace_debug=False,
        duty_cycles=StageDutyCycles(),
        stat_aliases=None,
    ):
        """core must be a BaseO3CPU core"""
        super().__init__()
        self._mm = machine_model
        self._interval = interval
        self._interval_ticks = interval_ticks
        self._sampling_enabled = interval > 0 or interval_ticks > 0
        self._trace_debug = trace_debug
        self._trace_prefix = "board.processor.cores.core"
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
        self._rnu = stages.rnu
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
        self._rnu.set_sample_mode(enable)
        self._lsu.set_sample_mode(enable)
        self._mmu.set_sample_mode(enable)
        self._exec.set_sample_mode(enable)

    def clear_sample_state(self):
        for sub in [self._fetch, self._rnu, self._lsu, self._mmu, self._exec]:
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

        fetch = self._fetch.dynamic_power()
        rnu = self._rnu.dynamic_power()
        lsu = self._lsu.dynamic_power()
        mmu = self._mmu.dynamic_power()
        exu = self._exec.dynamic_power()

        total = fetch + rnu + lsu + mmu + exu

        if sample_mode and self._trace_debug:
            self._append_subcomponent_trace(
                [
                    (f"{self._trace_prefix}.fetch", fetch, self._fetch),
                    (f"{self._trace_prefix}.rename", rnu, self._rnu),
                    (f"{self._trace_prefix}.lsu", lsu, self._lsu),
                    (f"{self._trace_prefix}.mmu", mmu, self._mmu),
                    (f"{self._trace_prefix}.execute", exu, self._exec),
                ]
            )
        elif not self._sampling_enabled:
            self.print_mcpat(6, total)

        if sample_mode:
            self.reset_stats_dict()

        return total

    def reset_stats_dict(self):
        self._fetch.reset_stats_dict()
        self._rnu.reset_stats_dict()
        self._lsu.reset_stats_dict()
        self._mmu.reset_stats_dict()
        self._exec.reset_stats_dict()

    def print_mcpat(self, indent, total):
        print("*" * 80)
        print("Core:")
        print(" " * indent + f"Runtime Dynamic = {total}\n")
        self._fetch.print_mcpat(indent)
        self._rnu.print_mcpat(indent)
        self._lsu.print_mcpat(indent)
        self._mmu.print_mcpat(indent)
        self._exec.print_mcpat(indent)
        print("*" * 80)

    def _append_subcomponent_trace(self, rows):
        if (
            not self._sampling_enabled
            or not self._trace_debug
            or not self.inPowerAtInterval()
        ):
            return

        trace_file = getattr(self, "trace_file", "")
        if not trace_file:
            return

        trace_path = pathlib.Path(trace_file)
        file_exists = trace_path.exists()

        with trace_path.open("a", newline="") as fp:
            writer = csv.writer(fp)
            if not file_exists:
                writer.writerow(
                    ["tick", "obj", "dyn_w", "st_w", "total_w", "temp_k"]
                )

            temp_k = temperature_kelvin(self)

            for name, dyn, stage in rows:
                st = stage.static_power(temp_k)
                writer.writerow(
                    [
                        m5.curTick(),
                        name,
                        f"{dyn:.17g}",
                        f"{st:.17g}",
                        f"{dyn + st:.17g}",
                        f"{temp_k:.17g}",
                    ]
                )


class O3McPATCpuPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class O3McPATCpuPowerModel(PowerModel):
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
            O3McPATCpuPowerOn(
                core,
                machine_model,
                interval,
                interval_ticks,
                trace_debug,
                duty_cycles,
                stat_aliases,
            ),  # ON
            O3McPATCpuPowerOff(),  # CLK_GATED
            O3McPATCpuPowerOff(),  # SRAM_RETENTION
            O3McPATCpuPowerOff(),  # OFF
        ]
