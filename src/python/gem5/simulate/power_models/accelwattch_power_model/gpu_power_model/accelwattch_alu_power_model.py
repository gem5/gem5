from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchALUPowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchALUPowerModel"

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        """ Multiplication factor of 2 is because the
            ratio of the FU's RF clock to the EXECU's
            clock is 2:1 and this is used in AW's
            power calculations for IALU energy.
        """
        energy = 2 * (self.alu_energy())
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def iadd_static_coeff(self) -> float:
        coeff = self._act_energies["ALU"] * 2
        return self.convert_to_watts(coeff)

    def alu_energy(self) -> float:
        alu_accesses = (
            sum(
                self.get_stat(f"CUs{i}.intInsts") for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_ACC"]
        )

        return alu_accesses * self._act_energies["ALU"]

    def alu_accesses(self) -> float:
        return (
            sum(
                self.get_stat(f"CUs{i}.intInsts") for i in range(self._num_cus)
            )
            * self._scaling_factors["INT_ACC"]
        )
