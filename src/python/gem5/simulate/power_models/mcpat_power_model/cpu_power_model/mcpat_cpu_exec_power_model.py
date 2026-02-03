from m5.objects import (
    BaseCPU,
    BaseO3CPU,
)

from ..base_mcpat_power_model import BaseMcPATPowerModel
from .mcpat_cpu_alu_power_model import McPATCpuAluPowerModel
from .mcpat_cpu_inst_scheduler_power_model import (
    McPATCpuInstructionSchedulerPowerModel,
)
from .mcpat_cpu_rf_power_model import McPATCpuRfPowerModel


class McPATCpuExecutePowerModel(BaseMcPATPowerModel):
    # avoid the use of default values
    def __init__(
        self, cpu: BaseCPU, act_energies, pipeline_act_factor, exu_act_factor
    ):
        super().__init__(cpu, act_energies)
        self.name = "McPATCpuExecutePowerModel"
        self._alu = McPATCpuAluPowerModel(cpu=cpu, act_energies=act_energies)
        self._rf = McPATCpuRfPowerModel(cpu=cpu, act_energies=act_energies)
        self._inst_scheduler = McPATCpuInstructionSchedulerPowerModel(
            cpu=cpu, act_energies=act_energies
        )

        """ The Activity Factor of the Execution Unit (default: 0.76): """
        self._exu_act_factor = exu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any CPUs in McPAT: """
        self._num_units = 4.0
        if isinstance(cpu, BaseO3CPU):
            self._num_units = 5.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def print_mcpat(self, indent):
        cdb_energy = self.bypass_energy()
        total_energy = (
            self._alu.dynamic_power()
            + self._rf.dynamic_power()
            + self.convert_to_watts(cdb_energy)
            + self.convert_to_watts(self.pipeline_energy())
        )
        if isinstance(self._simobj, BaseO3CPU):
            total_energy += self._inst_scheduler.dynamic_power()

        print(" " * indent + f"Execution Unit:")
        print(" " * (indent + 2) + f"Runtime Dynamic = {total_energy} W\n")
        self._rf.print_mcpat(indent + 4)
        if isinstance(self._simobj, BaseO3CPU):
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
            + self.convert_to_watts(self.bypass_energy())
            + self.convert_to_watts(self.pipeline_energy())
        )
        if isinstance(self._simobj, BaseO3CPU):
            energy += self._inst_scheduler.dynamic_power()
        return energy

    def static_power(self) -> float:
        return 1.0

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
        fp_accesses = fp_adds + fp_mults + fp_maccs + fp_divs + fp_misc

        int_mults = issued_insts.value[issued_insts.ysubnames.index("IntMult")]
        int_divs = issued_insts.value[issued_insts.ysubnames.index("IntDiv")]
        mul_accesses = int_mults + int_divs
        int_accesses = issued_insts.value[
            issued_insts.ysubnames.index("IntAlu")
        ]

        return (
            self._act_energies["IntBypass"] * int_accesses
            + self._act_energies["IntTagBypass"] * int_accesses
            + self._act_energies["FpBypass"] * fp_accesses
            + self._act_energies["FpTagBypass"] * fp_accesses
            + self._act_energies["MulBypass"] * mul_accesses
            + self._act_energies["MulTagBypass"] * mul_accesses
        )

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
