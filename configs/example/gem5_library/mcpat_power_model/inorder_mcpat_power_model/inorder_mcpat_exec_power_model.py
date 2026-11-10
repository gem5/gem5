from m5.objects import (
    BaseCPU,
    Root,
)

from .base_power_model import AbstractPowerModel
from .inorder_mcpat_alu_power_model import InorderMcPATAluPower
from .inorder_mcpat_inst_scheduler_power_model import (
    InorderMcPATInstructionSchedulerPower,
)
from .inorder_mcpat_rf_power_model import InorderMcPATRfPower
from .mcpat_power_model import McPATPowerModel


class InorderMcPATExecutePower(McPATPowerModel):
    STATIC_UNITS = (
        "IntBypass",
        "IntTagBypass",
        "MulBypass",
        "MulTagBypass",
        "FpBypass",
        "FpTagBypass",
        "SelLogic",
    )
    STATIC_CHILDREN = ("_alu", "_rf", "_inst_scheduler")
    STATIC_PIPELINE_SHARE = True

    # avoid the use of default values
    def __init__(
        self,
        cpu: BaseCPU,
        act_energies,
        pipeline_act_factor,
        exu_act_factor,
        interval=0,
        interval_ticks=0,
        num_threads=None,
        *,
        machine_model,
    ):
        super().__init__(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self.name = "InorderMcPATExecutePower"
        self._alu = InorderMcPATAluPower(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        self._rf = InorderMcPATRfPower(
            cpu, act_energies, interval, interval_ticks, machine_model
        )
        # SchedulerU (the unified instruction window) is built by McPAT only
        # for a multithreaded inorder core, and nested under the Execution
        # Unit -- same placement as O3McPATExecutePower. num_threads defaults
        # to the live cpu.numThreads (online model); validate_daxpy_power.py
        # passes it explicitly to model the N-thread structure without gem5
        # running N threads.
        if num_threads is None:
            num_threads = int(cpu.numThreads)
        self._inst_scheduler = (
            InorderMcPATInstructionSchedulerPower(
                cpu, act_energies, interval, interval_ticks, machine_model
            )
            if num_threads > 1
            else None
        )

        """ The Activity Factor of the Execution Unit (default: 0.76): """
        self._exu_act_factor = exu_act_factor

        """ The Activity Factor of the Pipeline itself (default: 1.0): """
        self._pipeline_act_factor = pipeline_act_factor

        """ Number of Pipeline Stages for any Inorder CPU in McPAT: """
        self._num_units = 4.0

        """ The number of pipelines our CPU has (assume 1): """
        self._num_pipelines = 1.0

    def set_sample_mode(self, enable: bool):
        super().set_sample_mode(enable)
        self._alu.set_sample_mode(enable)
        self._rf.set_sample_mode(enable)
        if self._inst_scheduler is not None:
            self._inst_scheduler.set_sample_mode(enable)

    def reset_stats_dict(self):
        super().reset_stats_dict()
        self._alu.reset_stats_dict()
        self._rf.reset_stats_dict()
        if self._inst_scheduler is not None:
            self._inst_scheduler.reset_stats_dict()

    def clear_sample_state(self):
        super().clear_sample_state()
        subs = [self._alu, self._rf]
        if self._inst_scheduler is not None:
            subs.append(self._inst_scheduler)
        for sub in subs:
            if hasattr(sub, "clear_sample_state"):
                sub.clear_sample_state()
            else:
                sub.reset_stats_dict()

    def _scheduler_energy(self) -> float:
        if self._inst_scheduler is None:
            return 0.0
        return self._inst_scheduler.dynamic_power()

    def print_mcpat(self, indent):
        cdb_energy = self.bypass_energy()
        total_energy = (
            self._alu.dynamic_power()
            + self._rf.dynamic_power()
            + self._scheduler_energy()
            + self.convert_to_watts(self.bypass_energy())
            + self.convert_to_watts(self.pipeline_energy())
        )
        print(" " * indent + f"Execution Unit:")
        print(" " * (indent + 2) + f"Runtime Dynamic = {total_energy} W\n")
        self._rf.print_mcpat(indent + 4)
        if self._inst_scheduler is not None:
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
            + self._scheduler_energy()
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
