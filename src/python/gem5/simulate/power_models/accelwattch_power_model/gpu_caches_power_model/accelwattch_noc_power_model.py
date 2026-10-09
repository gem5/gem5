from m5.objects import (
    Shader,
)

from gem5.prebuilt.viper.viper_network import SimpleDoubleCrossbar


class AccelwattchNoCPowerModel(BaseAccelwattchPowerModel):
    def __init__(self, gpu: Shader, act_energies, scaling_factors, interval):
        super().__init__(gpu, act_energies, scaling_factors, interval)
        self.name = "AccelwattchNoCPowerModel"
        self._ruby_gpu = None
        self._net_type = None
        self._flit_size = 64

    def dynamic_power(self) -> float:
        self._network = self._simobj._parent.gpu_caches.ruby_gpu.network
        self._net_type = type(self._network)
        energy = self.noc_access_energy()
        return self.convert_to_watts(energy)

    def static_power(self) -> float:
        return 0.0

    def reset_stats_dict(self):
        self.reset_stats_dict()

    def noc_access_energy(self) -> float:
        """Note that this PM doesn't support Garnet because
        Garnet stats like m_flits_injected are inaccessible
        when trying to index them from the Ruby Network
        """
        if self._net_type is SimpleDoubleCrossbar:
            stats_to_track = [
                "Control",
                "Data",
                "Request_Control",
                "Repsonse_Data",
                "Response_Control",
                "Writeback_Control",
                "Unblock_Control",
            ]
            total_msgs = sum(
                self.get_stat("msg_count.{stat}", self._network)
                for stat in stats_to_track
            )
            total_bytes = sum(
                self.get_stat("msg_byte.{stat}", self._network)
                for stat in stats_to_track
            )
            avg_msg_size = total_msgs / total_bytes
            accesses = (
                total_msgs * (avg_msg_size / self._flit_size)
            ) * self._scaling_factors["NOC_A"]
            energy = accesses * (
                self._act_energies["NoCRouter"]["Read"]
                + self._act_energies["NoCRouter"]["Write"]
                + self._act_energies["NoCCrossbar"]
                + self._act_energies["NoCArbiter"]
            )

        return energy
