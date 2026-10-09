from math import ceil

from m5.objects import (
    Root,
    Shader,
)

from ..abstract_power_model import AbstractPowerModel


class BaseAccelwattchPowerModel(AbstractPowerModel):
    def __init__(
        self, simobj: Shader, act_energies, scaling_factors, interval
    ):
        super().__init__(simobj)
        self.name = "BaseAccelwattchPowerModel"
        self._act_energies = act_energies
        self._scaling_factors = scaling_factors
        self._interval = interval
        """ To properly update the delta of each stat, use a dict.
            Might not be the most memory-efficient, but works?
        """
        self._stats = {}
        """ # of CUs is a placeholder, each class will need to change this
            upon execution.
        """
        self._num_cus = 0

    def get_stat(self, stat, simobj=None):
        if simobj == None:
            simobj = self._simobj
        try:
            value = simobj.resolveStat(stat).total
            if self._interval > 0:
                if stat not in self._stats:
                    self._stats[stat] = {"value": value, "active": 1}
                elif self._stats[stat]["active"] == 0:
                    self._stats[stat]["value"] -= value
                    self._stats[stat]["active"] = 1
                return self._stats[stat]["value"]
            return value
        except KeyError as e:
            panic(f"{stat} not found in stats!")
            return 0.0

    def reset_stats_dict(self):
        for stat in self._stats.values():
            stat["active"] = 0

    def get_time(self) -> float:
        """gem5 Simulates 1e12 ticks / s, so this is fine"""
        clk_domain = self._simobj.clk_domain.clock.getValue()[0]
        cycles = self.get_stat("shaderActiveCycles")
        return cycles / (1e12 / clk_domain)

    def convert_to_watts(self, value: float) -> float:
        time = self.get_time()
        return value / time

    def get_num_cus(self) -> float:
        """Get the number of compute units passed to this GPU
        Paticularly useful if you need to sum the stats of
        all CUs into one number
        """
        return len(self._simobj.get_compute_units())

    def get_num_simds(self) -> float:
        """Gets the number of SIMD units on this GPU.
        Similar reasoning as above, useful if you
        need to sum the stats for say, a VRF or SRF.
        """
        return self._simobj.get_compute_units()[0].num_SIMDs

    def get_simd_width(self) -> float:
        return self._simobj.get_compute_units()[0].simd_width.getValue()

    def get_wf_size(self) -> float:
        return self._simobj.get_compute_units()[0].wf_size.getValue()

    def get_num_gmem_pipelines(self) -> float:
        return self._simobj.get_compute_units()[
            0
        ].num_global_mem_pipes.getValue()

    def get_num_smem_pipelines(self) -> float:
        return self._simobj.get_compute_units()[
            0
        ].num_shared_mem_pipes.getValue()

    def get_total_cycles(self) -> float:
        return self.get_stat("shaderActiveCycles")

    def get_avg_idle_cores(self) -> float:
        """Gets the number of avg. idle cores during simulation.
        There's no direct stat, but my proxy is basically
        take, on average, how many cycles the CUs were idle,
        and then get the proprtion of idle / total cycles.
        Once we get this proportion, multiply it by # of
        CUs
        """
        avg_idle_cycles = (
            sum(
                self.get_stat(f"CUs{i}.ExecStage.numCyclesWithNoIssue")
                for i in range(self.get_num_cus())
            )
        ) / self.get_num_cus()

        cycles = self.get_total_cycles()
        return ceil(self.get_num_cus() * (avg_idle_cycles / cycles))

    def get_avg_active_cores(self) -> float:
        idle_cores = self.get_avg_idle_cores()
        cus = self.get_num_cus()
        return (cus - idle_cores) / cus

    def get_gpu_duty_cycle(self) -> float:
        avg_active_cycles = (
            sum(
                self.get_stat(f"CUs{i}.ExecStage.numCyclesWithInstrIssued")
                for i in range(self.get_num_cus())
            )
            / self.get_num_cus()
        )
        cycles = self.get_total_cycles()
        return avg_active_cycles / cycles

    def get_avg_threads_in_wf(self) -> float:
        cus = self.get_num_cus()
        avg_threads = (
            sum(
                util
                for i in range(cus)
                if (util := self.get_stat(f"CUs{i}.vALUUtilization"))
            )
        ) / cus
        return avg_threads / 100 * self.get_wf_size()
