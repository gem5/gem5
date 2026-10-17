from m5.objects import (
    BaseCPU,
    Root,
)

from .mcpat_power_model import McPATPowerModel


class InorderMcPATLsuPower(McPATPowerModel):
    STATIC_UNITS = ("LoadQueue", "LoadStoreQueue")
    STATIC_PIPELINE_SHARE = True

    # avoid the use of default values
    def __init__(
        self,
        cpu: BaseCPU,
        act_energies,
        pipeline_act_factor,
        lsu_act_factor,
        interval=0,
        interval_ticks=0,
        *,
        machine_model,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "InorderMcPATLsuPower"
        """ The Activity Factor of the LSU (default: 0.71): """
        self._lsu_act_factor = lsu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 4.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def print_mcpat(self, indent):
        lsq_energy = self.lsq_energy()
        total_energy = lsq_energy + self.lsu_pipeline_energy()
        print(" " * indent + f"Load Store Unit")
        print(
            " " * (indent + 2)
            + f"Runtime Dynamic = {self.convert_to_watts(total_energy)} W\n"
        )
        print(" " * (indent + 4) + f"Load Store Queue")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(lsq_energy)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = self.lsu_pipeline_energy() + self.lsq_energy()
        return self.convert_to_watts(energy)

    def lsq_energy(self) -> float:
        loads = sum(
            self.get_stat(f"commitStats{t}.numLoadInsts").total
            for t in range(self._simobj.numThreads)
        )
        stores = sum(
            self.get_stat(f"commitStats{t}.numStoreInsts").total
            for t in range(self._simobj.numThreads)
        )
        """ 'Accesses' considers the overhead for flush """
        accesses = (loads + stores) * 2
        return (
            accesses
            * (
                self._act_energies["LoadStoreQueue"]["Read"]
                + self._act_energies["LoadStoreQueue"]["Search"]
            )
            + accesses * self._act_energies["LoadStoreQueue"]["Write"]
        )

    def lsu_pipeline_energy(self) -> float:
        cycles = self.get_stat("numCycles").total
        rtp_pipeline_coe = (
            self._pipeline_act_factor * cycles * self._lsu_act_factor
        )
        total_pipeline_cost = (
            rtp_pipeline_coe * self._num_pipelines / self._num_units
        )
        return total_pipeline_cost * self._act_energies["Pipeline"]
