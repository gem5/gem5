from m5.objects import (
    BaseO3CPU,
    BranchPredictor,
    Root,
)

# Never use `import *`
from .base_power_model import AbstractPowerModel
from .mcpat_btb_power_model import McPATBtbPower
from .mcpat_power_model import McPATPowerModel
from .mcpat_tournament_bp_power_model import McPATTournamentBPPower
from .o3_mcpat_decode_power_model import O3McPATDecodePower


class O3McPATFetchPower(McPATPowerModel):
    STATIC_UNITS = ("InstBuffer",)
    STATIC_CHILDREN = ("_bp", "_btb", "_decode")
    STATIC_PIPELINE_SHARE = True

    # avoid the use of default values
    def __init__(
        self,
        cpu: BaseO3CPU,
        act_energies,
        pipeline_act_factor,
        ifu_act_factor,
        interval=0,
        interval_ticks=0,
        *,
        machine_model,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "O3McPATFetchPower"

        """ Below is to ensure that our PM doesn't panic if user hasn't given a BP """
        self._has_predictor = False
        for desc in self._simobj.descendants():
            if isinstance(desc, BranchPredictor):
                self._has_predictor = True

        self._bp = McPATTournamentBPPower(
            cpu,
            act_energies,
            self._has_predictor,
            interval,
            interval_ticks,
            machine_model,
        )
        self._btb = McPATBtbPower(
            cpu,
            act_energies,
            self._has_predictor,
            interval,
            interval_ticks,
            machine_model,
        )
        self._decode = O3McPATDecodePower(
            cpu, act_energies, interval, interval_ticks, machine_model
        )

        """ The Activity Factor of the Inst. Fetch Unit (default: 0.9): """
        self._ifu_act_factor = ifu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 5.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def set_sample_mode(self, enable: bool):
        super().set_sample_mode(enable)
        self._bp.set_sample_mode(enable)
        self._btb.set_sample_mode(enable)
        self._decode.set_sample_mode(enable)

    def reset_stats_dict(self):
        super().reset_stats_dict()
        self._bp.reset_stats_dict()
        self._btb.reset_stats_dict()
        self._decode.reset_stats_dict()

    def clear_sample_state(self):
        super().clear_sample_state()
        for sub in [self._bp, self._btb, self._decode]:
            if hasattr(sub, "clear_sample_state"):
                sub.clear_sample_state()
            else:
                sub.reset_stats_dict()

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
        total_power = (
            self._decode.dynamic_power()
            + self._btb.dynamic_power()
            + self._bp.dynamic_power()
            + self.convert_to_watts(self.pipeline_energy())
        )
        return total_power

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
