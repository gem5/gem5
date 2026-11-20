from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchRegisterFilePowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchRegisterFilePowerModel"

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        """ Multiplication factor of 2 is because the
            ratio of the FU's RF clock to the EXECU's
            clock is 2:1 and this is used in AW's
            power calculations (i.e., assume ALUs
            are double clocked).
        """
        energy = 2 * (
            self.reg_file_energy()
            + self.op_collection_energy()
            + self.xbar_reg_file_energy()
            + self.arbiter_reg_file_energy()
        )
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def reg_file_energy(self) -> float:
        reads = (
            sum(
                self.get_stat(f"CUs{i}.vector_register_file{j}.registerReads")
                for i in range(self._num_cus)
                for j in range(self.get_num_simds())
            )
            * self._scaling_factors["REG_RD"]
            / (self.get_wf_size())
            * self.get_simd_width()
        )
        writes = (
            sum(
                self.get_stat(f"CUs{i}.vector_register_file{j}.registerWrites")
                for i in range(self._num_cus)
                for j in range(self.get_num_simds())
            )
            * self._scaling_factors["REG_WR"]
            / (self.get_wf_size())
            * self.get_simd_width()
        )

        return (
            reads * self._act_energies["IRF"]["Read"]
            + writes * self._act_energies["IRF"]["Write"]
        )

    def op_collection_energy(self) -> float:
        reads = (
            sum(
                self.get_stat(f"CUs{i}.vector_register_file{j}.registerReads")
                for i in range(self._num_cus)
                for j in range(self.get_num_simds())
            )
            * self._scaling_factors["REG_RD"]
        )
        return reads * self._act_energies["OPC"]

    def xbar_reg_file_energy(self) -> float:
        reads = (
            sum(
                self.get_stat(f"CUs{i}.vector_register_file{j}.registerReads")
                for i in range(self._num_cus)
                for j in range(self.get_num_simds())
            )
            * self._scaling_factors["REG_RD"]
            / self.get_wf_size()
        )

        return reads * self._act_energies["xbar_rfu"]

    def arbiter_reg_file_energy(self) -> float:
        reads = (
            sum(
                self.get_stat(f"CUs{i}.vector_register_file{j}.registerReads")
                for i in range(self._num_cus)
                for j in range(self.get_num_simds())
            )
            * self._scaling_factors["REG_RD"]
            / self.get_wf_size()
        )

        return reads * self._act_energies["arbiter_rfu"]
