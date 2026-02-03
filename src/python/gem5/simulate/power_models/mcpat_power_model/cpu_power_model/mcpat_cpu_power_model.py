from m5.objects import (
    BaseCPU,
    BaseO3CPU,
    PowerModel,
    PowerModelPyFunc,
)

from .mcpat_cpu_exec_power_model import McPATCpuExecutePowerModel
from .mcpat_cpu_fetch_power_model import McPATCpuFetchPowerModel
from .mcpat_cpu_lsu_power_model import McPATCpuLsuPowerModel
from .mcpat_cpu_mmu_power_model import McPATCpuMmuPowerModel
from .mcpat_cpu_renaming_unit_power_model import McPATCpuRenamingUnitPowerModel


class McPATCpuPowerOn(PowerModelPyFunc):
    def __init__(self, cpu: BaseCPU, act_energies):
        """cpu must be a BaseCPU core"""
        super().__init__()
        self._cpu = cpu
        self._fetch = McPATCpuFetchPowerModel(
            cpu=cpu,
            act_energies=act_energies,
            pipeline_act_factor=1.0,
            ifu_act_factor=0.9,
        )
        if isinstance(cpu, BaseO3CPU):
            self._rnu = McPATCpuRenamingUnitPowerModel(
                cpu=cpu, act_energies=act_energies, pipeline_act_factor=1.0
            )
        self._lsu = McPATCpuLsuPowerModel(
            cpu=cpu,
            act_energies=act_energies,
            pipeline_act_factor=1.0,
            lsu_act_factor=0.71,
        )
        self._mmu = McPATCpuMmuPowerModel(
            cpu=cpu,
            act_energies=act_energies,
            pipeline_act_factor=1.0,
            mmu_act_factor=0.71,
        )
        self._exec = McPATCpuExecutePowerModel(
            cpu=cpu,
            act_energies=act_energies,
            pipeline_act_factor=1.0,
            exu_act_factor=0.76,
        )

        self.dyn = self.dynamic_power
        self.st = self.static_power

    def static_power(self):
        return 1.0

    def dynamic_power(self):
        total = (
            self._fetch.dynamic_power()
            + self._lsu.dynamic_power()
            + self._mmu.dynamic_power()
            + self._exec.dynamic_power()
        )
        if isinstance(self._cpu, BaseO3CPU):
            total += self._rnu.dynamic_power()
        self.print_mcpat(6, total)
        return total

    def print_mcpat(self, indent, total):
        print("*" * 80)
        print("Core:")
        print(" " * indent + f"Runtime Dynamic = {total}\n")
        self._fetch.print_mcpat(indent)
        self._lsu.print_mcpat(indent)
        self._mmu.print_mcpat(indent)
        self._exec.print_mcpat(indent)
        if isinstance(self._cpu, BaseO3CPU):
            self._rnu.print_mcpat(indent)
        print("*" * 80)


class McPATCpuPowerOff(PowerModelPyFunc):
    def __init__(self):
        super().__init__()
        self.dyn = lambda: 0.0
        self.st = lambda: 0.0


class McPATCpuPowerModel(PowerModel):
    def __init__(self, cpu: BaseCPU, act_energies):
        super().__init__()
        # Choose a power model for every power state
        self.pm = [
            McPATCpuPowerOn(cpu=cpu, act_energies=act_energies),  # ON
            McPATCpuPowerOff(),  # CLK_GATED
            McPATCpuPowerOff(),  # SRAM_RETENTION
            McPATCpuPowerOff(),  # OFF
        ]
