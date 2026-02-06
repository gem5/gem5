from m5.objects import Shader

from ..base_accelwattch_power_model import BaseAccelwattchPowerModel


class AccelwattchPipelinePowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchPipelinePowerModel"
        self._num_units = 4.0
        self._num_pipelines = 1.0

    def dynamic_power(self) -> float:
        self._num_cus = self.get_num_cus()
        energy = self.pipeline_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def pipeline_energy(self) -> float:
        """Accelwattch repeats these calculations for the pipeline
        stages it uses (Fetch/LSU/Exec).
        """
        sim_insts = sum(
            self.get_stat(f"CUs{i}.vALUInsts") for i in range(self._num_cus)
        )
        pipeline_width = int(
            self.get_num_simds()
            + self.get_num_gmem_pipelines()
            + self.get_num_smem_pipelines()
        )
        gpu_duty_cycle = sim_insts / (
            pipeline_width * self.get_wf_size() * self.get_total_cycles()
        )

        rtp_pipeline_coe = (
            gpu_duty_cycle * self.get_total_cycles() * self.get_num_cus()
        )
        energy = self._act_energies["Pipeline"] * sum(
            (self._num_pipelines * rtp_pipeline_coe / self._num_units)
            for i in range(3)
        )
        return energy
