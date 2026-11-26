from m5.objects import (
    BaseO3CPU,
    Root,
)

from .base_power_model import AbstractPowerModel
from .mcpat_power_model import McPATPowerModel
from .o3_mcpat_alu_power_model import O3McPATAluPower
from .o3_mcpat_inst_scheduler_power_model import (
    O3McPATInstructionSchedulerPower,
)
from .o3_mcpat_rf_power_model import O3McPATRfPower


class O3McPATExecutePower(McPATPowerModel):
    STATIC_UNITS = (
        "IntBypass",
        "IntTagBypass",
        "MulBypass",
        "MulTagBypass",
        "FpBypass",
        "FpTagBypass",
    )
    STATIC_CHILDREN = ("_alu", "_rf", "_inst_scheduler")
    STATIC_PIPELINE_SHARE = True

    # avoid the use of default values
    def __init__(
        self,
        cpu: BaseO3CPU,
        act_energies,
        pipeline_act_factor,
        exu_act_factor,
        interval=0,
        interval_ticks=0,
        isEmbedded=True,
        *,
        machine_model,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "O3McPATExecutePower"
        self._alu = O3McPATAluPower(
            cpu,
            act_energies,
            interval,
            interval_ticks,
            isEmbedded,
            machine_model,
        )
        self._rf = O3McPATRfPower(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self._inst_scheduler = O3McPATInstructionSchedulerPower(
            cpu, act_energies, interval, interval_ticks, machine_model
        )

        """ The Activity Factor of the Execution Unit (default: 0.76): """
        self._exu_act_factor = exu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 5.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def set_stat_aliases(self, aliases):
        super().set_stat_aliases(aliases)
        for sub in (self._alu, self._rf, self._inst_scheduler):
            sub.set_stat_aliases(aliases)

    def set_sample_mode(self, enable: bool):
        super().set_sample_mode(enable)
        self._alu.set_sample_mode(enable)
        self._rf.set_sample_mode(enable)
        self._inst_scheduler.set_sample_mode(enable)

    def reset_stats_dict(self):
        super().reset_stats_dict()
        self._alu.reset_stats_dict()
        self._rf.reset_stats_dict()
        self._inst_scheduler.reset_stats_dict()

    def clear_sample_state(self):
        super().clear_sample_state()
        for sub in [self._alu, self._rf, self._inst_scheduler]:
            if hasattr(sub, "clear_sample_state"):
                sub.clear_sample_state()
            else:
                sub.reset_stats_dict()

    def print_mcpat(self, indent):
        cdb_energy = self.bypass_energy()
        total_energy = (
            self._alu.dynamic_power()
            + self._rf.dynamic_power()
            + self._inst_scheduler.dynamic_power()
            + self.convert_to_watts(self.bypass_energy())
            + self.convert_to_watts(self.pipeline_energy())
        )
        print(" " * indent + f"Execution Unit:")
        print(" " * (indent + 2) + f"Runtime Dynamic = {total_energy} W\n")
        self._rf.print_mcpat(indent + 4)
        self._inst_scheduler.print_mcpat(indent + 4)
        self._alu.print_mcpat(indent + 4)
        print(" " * (indent + 4) + f"Results Broadcast Bus:")
        print(
            " " * (indent + 6)
            + f"Runtime Dynamic = {self.convert_to_watts(cdb_energy)} W\n"
        )

    def dynamic_power(self) -> float:
        energy = (
            self._alu.dynamic_power()
            + self._rf.dynamic_power()
            + self._inst_scheduler.dynamic_power()
            + self.convert_to_watts(self.bypass_energy())
            + self.convert_to_watts(self.pipeline_energy())
        )
        return energy

    def bypass_energy(self) -> float:
        issued_insts = self.get_stat("issuedInstType")
        fp_adds = issued_insts.value[issued_insts.ysubnames.index("FloatAdd")]
        fp_mults = issued_insts.value[
            issued_insts.ysubnames.index("FloatMult")
        ]
        fp_maccs = issued_insts.value[
            issued_insts.ysubnames.index("FloatMultAcc")
        ]
        fp_divs = issued_insts.value[issued_insts.ysubnames.index("FloatDiv")]
        fp_misc = issued_insts.value[issued_insts.ysubnames.index("FloatMisc")]
        fp_cmp = issued_insts.value[issued_insts.ysubnames.index("FloatCmp")]
        fp_accesses = (
            fp_adds + fp_mults + fp_maccs + fp_divs + fp_misc + fp_cmp
        )

        int_mults = issued_insts.value[issued_insts.ysubnames.index("IntMult")]
        int_divs = issued_insts.value[issued_insts.ysubnames.index("IntDiv")]
        mul_accesses = int_mults + int_divs
        int_accesses = issued_insts.value[
            issued_insts.ysubnames.index("IntAlu")
        ]

        energy = (
            self._act_energies["IntBypass"] * int_accesses
            + self._act_energies["IntTagBypass"] * int_accesses
            + self._act_energies["FpBypass"] * fp_accesses
            + self._act_energies["FpTagBypass"] * fp_accesses
        )
        # McPAT builds the Mul bypass only when MUL_per_core > 0
        # (core.cc:1194/1235/1273, 3770-3809); a model built with no MUL
        # (e.g. MUL_per_core=0) emits no MulBypass key.
        if "MulBypass" in self._act_energies:
            # Two separate adds keep McPAT's left-to-right summation order.
            energy += self._act_energies["MulBypass"] * mul_accesses
            energy += self._act_energies["MulTagBypass"] * mul_accesses
        return energy

    def pipeline_energy(self) -> float:
        cycles = self.get_stat(
            "numCycles"
        ).total  # total number of cycles, idle or not
        rtp_pipeline_coe = (
            cycles * self._exu_act_factor * self._pipeline_act_factor
        )
        total_pipeline_cost = (
            rtp_pipeline_coe * self._num_pipelines / self._num_units
        )
        return total_pipeline_cost * self._act_energies["Pipeline"]
