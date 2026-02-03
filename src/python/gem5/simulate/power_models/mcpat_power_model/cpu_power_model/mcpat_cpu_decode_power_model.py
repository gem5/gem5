from m5.objects import (
    BaseCPU,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel


class McPATCpuDecodePowerModel(BaseMcPATPowerModel):
    def __init__(self, cpu: BaseCPU, act_energies):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuDecodePowerModel"
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

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def dynamic_power(self) -> float:
        energy = self.inst_buffer_energy()
        energy += self.inst_decode_energy()
        return self.convert_to_watts(energy)

    def inst_buffer_energy(self) -> float:
        decoded_insts = self.get_stat("fetchStats0.numInsts").total
        return (
            decoded_insts * self._act_energies["InstBuffer"]["Read"]
            + decoded_insts * self._act_energies["InstBuffer"]["Write"]
        )

    def inst_decode_energy(self) -> float:
        decoded_insts = self.get_stat("fetchStats0.numInsts").total
        return (
            self._act_energies["IDInst"] * decoded_insts
            + self._act_energies["IDOp"] * decoded_insts
            + self._act_energies["IDMisc"] * decoded_insts
        )
