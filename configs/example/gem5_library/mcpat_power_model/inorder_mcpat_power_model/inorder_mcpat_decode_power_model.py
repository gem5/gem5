from m5.objects import (
    BaseCPU,
    Root,
)

from .base_power_model import AbstractPowerModel
from .mcpat_power_model import McPATPowerModel


class InorderMcPATDecodePower(McPATPowerModel):
    STATIC_UNITS = ("IDInst", "IDOp", "IDMisc")

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
        self.name = "InorderMcPATDecodePower"
        # No pipeline cost here, since McPAT considers IF + ID as 1 stage

    def print_mcpat(self, indent):
        ib_energy = self.inst_buffer_energy()
        id_energy = self.inst_decode_energy()
        print(" " * indent + f"Instruction Buffer")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(ib_energy)} W\n"
        )
        print(" " * indent + f"Instruction Decoder")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(id_energy)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.inst_buffer_energy()
        energy += self.inst_decode_energy()
        return self.convert_to_watts(energy)

    def inst_buffer_energy(self) -> float:
        decoded_insts = sum(
            self.get_stat(f"fetchStats{t}.numInsts").total
            for t in range(self._simobj.numThreads)
        )
        return (
            decoded_insts * self._act_energies["InstBuffer"]["Read"]
            + decoded_insts * self._act_energies["InstBuffer"]["Write"]
        )

    def inst_decode_energy(self) -> float:
        decoded_insts = sum(
            self.get_stat(f"fetchStats{t}.numInsts").total
            for t in range(self._simobj.numThreads)
        )
        return (
            self._act_energies["IDInst"] * decoded_insts
            + self._act_energies["IDOp"] * decoded_insts
            + self._act_energies["IDMisc"] * decoded_insts
        )
