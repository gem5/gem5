from m5.objects import (
    BaseCPU,
    Root,
)

from .mcpat_power_model import McPATPowerModel


class InorderMcPATInstructionSchedulerPower(McPATPowerModel):
    STATIC_UNITS = ("InstFetchQueue",)

    # avoid the use of default values
    def __init__(
        self,
        cpu: BaseCPU,
        act_energies,
        interval=0,
        interval_ticks=0,
        machine_model=None,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "InorderMcPATInstructionSchedulerPower"

    def print_mcpat(self, indent):
        inst_window = self.inst_window_energy()
        total_energy = inst_window
        print(" " * indent + f"Instruction Scheduler:")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Instruction Window:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(inst_window)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.inst_window_energy()
        return self.convert_to_watts(energy)

    def inst_window_energy(self) -> float:
        int_insts = sum(
            self.get_stat(f"commitStats{t}.numIntInsts").total
            for t in range(self._simobj.numThreads)
        )
        fp_insts = sum(
            self.get_stat(f"commitStats{t}.numFpInsts").total
            for t in range(self._simobj.numThreads)
        )
        iw_reads = iw_writes = int_insts + fp_insts
        iw_wakeups = 2 * iw_reads
        return (
            iw_reads * self._act_energies["IntInstWindow"]["Read"]
            + iw_writes * self._act_energies["IntInstWindow"]["Write"]
            + iw_wakeups * self._act_energies["IntInstWindow"]["Search"]
            + iw_writes * self._act_energies["SelLogic"]
        )
