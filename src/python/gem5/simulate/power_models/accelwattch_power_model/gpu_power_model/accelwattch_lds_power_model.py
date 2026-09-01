from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchLDSPowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchLDSPowerModel"

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        energy = self.lds_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def lds_static_coeff(self) -> float:
        coeff = (
            self._act_energies["SharedMemory"]["Read"]
            + self._act_energies["xbar_shared"]
        )
        return self.convert_to_watts(coeff)

    def lds_energy(self) -> float:
        """For the Shmem/LDS, Accelwattch assumes that
        there are no write accesses, and that all accesses
        are considered read hits.
        """
        reads = (
            sum(
                self.get_stat(f"CUs{i}.ldsBankAccesses")
                for i in range(self._num_cus)
            )
            * self._scaling_factors["SHRD_ACC"]
        )
        writes = 0
        energy = (
            reads * self._act_energies["SharedMemory"]["Read"]
            + (reads + writes) * self._act_energies["xbar_shared"]
        )
        return energy
