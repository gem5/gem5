from m5.objects import (
    BaseO3CPU,
    Root,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATCpuInstructionSchedulerPowerModel(BaseMcPATPowerModel):
    # avoid the use of default values
    def __init__(self, cpu: BaseO3CPU, act_energies):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuInstructionSchedulerPowerModel"

    def print_mcpat(self, indent):
        int_iw = self.int_inst_window_energy()
        fp_iw = self.fp_inst_window_energy()
        total_energy = int_iw + fp_iw
        print(" " * indent + f"Instruction Scheduler:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Instruction Window:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(int_iw)} W\n"
        )
        print(" " * (indent + 4) + f"FP Instruction Window:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(fp_iw)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.int_inst_window_energy()
        energy += self.fp_inst_window_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def int_inst_window_energy(self) -> float:
        int_iw_reads = self.get_stat("intInstQueueReads").total
        int_iw_writes = self.get_stat("intInstQueueWrites").total
        int_iw_wakeups = self.get_stat("intInstQueueWakeupAccesses").total

        return (
            int_iw_reads * self._act_energies["IntInstWindow"]["Read"]
            + int_iw_writes * self._act_energies["IntInstWindow"]["Write"]
            + int_iw_wakeups * self._act_energies["IntInstWindow"]["Search"]
            + int_iw_reads * self._act_energies["SelLogic"]
        )

    def fp_inst_window_energy(self) -> float:
        fp_iw_reads = self.get_stat("fpInstQueueReads").total
        fp_iw_writes = self.get_stat("fpInstQueueWrites").total
        fp_iw_wakeups = self.get_stat("fpInstQueueWakeupAccesses").total

        return (
            fp_iw_reads * self._act_energies["FpInstWindow"]["Read"]
            + fp_iw_writes * self._act_energies["FpInstWindow"]["Write"]
            + fp_iw_wakeups * self._act_energies["FpInstWindow"]["Search"]
            + fp_iw_reads * self._act_energies["SelLogic"]
        )
