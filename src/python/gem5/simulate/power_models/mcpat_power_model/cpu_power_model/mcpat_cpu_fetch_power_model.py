from m5.objects import (
    BaseCPU,
    BaseO3CPU,
    BranchPredictor,
    Root,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel
from .mcpat_cpu_btb_power_model import McPATCpuBtbPowerModel
from .mcpat_cpu_decode_power_model import McPATCpuDecodePowerModel
from .mcpat_cpu_tournament_bp_power_model import McPATCpuTournamentBpPowerModel


class McPATCpuFetchPowerModel(BaseMcPATPowerModel):
    def __init__(
        self,
        cpu: BaseCPU,
        act_energies,
        pipeline_act_factor: float,
        ifu_act_factor: float,
    ):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuFetchPowerModel"

        """ Below is to ensure that our PM doesn't panic if user hasn't given a BP """
        self._has_predictor = False
        for desc in self._simobj.descendants():
            if isinstance(desc, BranchPredictor):
                self._has_predictor = True

        self._bp = McPATCpuTournamentBpPowerModel(
            cpu=cpu,
            act_energies=act_energies,
            has_predictor=self._has_predictor,
        )
        self._btb = McPATCpuBtbPowerModel(
            cpu=cpu,
            act_energies=act_energies,
            has_predictor=self._has_predictor,
        )
        self._decode = McPATCpuDecodePowerModel(
            cpu=cpu, act_energies=act_energies
        )

        """ The Activity Factor of the Inst. Fetch Unit (default: 0.9): """
        self._ifu_act_factor = ifu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 4.0

        if isinstance(self._simobj, BaseO3CPU):
            self._num_units = 5.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def print_mcpat(self, indent):
        total_power = (
            self._decode.dynamic_power()
            + self._btb.dynamic_power()
            + self._bp.dynamic_power()
            + self.convert_to_watts(self.pipeline_energy())
        )
        print(" " * indent + f"Instruction Fetch Unit")
        print(" " * (indent + 2) + f"Runtime Dynamic = {total_power} W\n")
        self._btb.print_mcpat(indent + 4)
        self._bp.print_mcpat(indent + 4)
        self._decode.print_mcpat(indent + 4)

    def dynamic_power(self) -> float:
        if not self._has_predictor:
            return 0.0
        total_power = (
            self._decode.dynamic_power()
            + self._btb.dynamic_power()
            + self._bp.dynamic_power()
            + self.convert_to_watts(self.pipeline_energy())
        )
        return total_power

    def static_power(self) -> float:
        """Returns static power in Watts"""
        return 1.0

    def pipeline_energy(self) -> float:
        cycles = self.get_stat(
            "numCycles"
        ).total  # total number of cycles, idle or not
        rtp_pipeline_coe = (
            cycles * self._ifu_act_factor * self._pipeline_act_factor
        )
        total_pipeline_cost = (
            rtp_pipeline_coe * self._num_pipelines / self._num_units
        )
        return total_pipeline_cost * self._act_energies["Pipeline"]
