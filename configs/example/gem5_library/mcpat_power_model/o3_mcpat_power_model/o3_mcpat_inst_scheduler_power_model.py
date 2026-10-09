from m5.objects import (
    BaseO3CPU,
    Root,
)

from .mcpat_power_model import McPATPowerModel


class O3McPATInstructionSchedulerPower(McPATPowerModel):
    STATIC_UNITS = (
        "InstIssueQueue",
        "FPIssueQueue",
        "ReorderBuffer",
        "SelLogic",
    )

    # avoid the use of default values
    def __init__(
        self,
        o3cpu: BaseO3CPU,
        act_energies,
        interval=0,
        interval_ticks=0,
        machine_model=None,
    ):
        super().__init__(
            o3cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "O3McPATInstructionSchedulerPower"

    def print_mcpat(self, indent):
        int_iw = self.int_inst_window_energy()
        fp_iw = self.fp_inst_window_energy()
        rob = self.rob_energy()
        total_energy = int_iw + fp_iw + rob
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
        print(" " * (indent + 4) + f"ROB:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(rob)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.int_inst_window_energy()
        energy += self.fp_inst_window_energy()
        energy += self.rob_energy()
        return self.convert_to_watts(energy)

    def rob_energy(self) -> float:
        rob_reads = self.get_stat("rob.reads").total
        rob_writes = self.get_stat("rob.writes").total
        return (
            rob_reads * self._act_energies["ReorderBuffer"]["Read"]
            + rob_writes * self._act_energies["ReorderBuffer"]["Write"]
        )

    def int_inst_window_energy(self) -> float:
        int_iw_reads = self.get_stat("intInstQueueReads").total
        int_iw_writes = self.get_stat("intInstQueueWrites").total
        int_iw_wakeups = self.get_stat("intInstQueueWakeupAccesses").total

        # NOTE: FOR MULTITHREADED SYSTEMS, THE ABOVE STATS ARE THE INT/FP
        # INSTS FOR R/W. Basically:
        """
        if (threads > 1):
            int_insts = self.get_stat("fetchedIntInsts")
            fp_insts = self.get_stat("fetchedFpInsts")
            int_iw_reads = int_insts + fp_insts
            int_iw_writes = int_insts + fp_insts
            int_iw_wakeups = 2 * (int_insts + fp_insts)
        """

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
            + fp_iw_writes * self._act_energies["SelLogic"]
        )
