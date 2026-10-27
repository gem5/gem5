import dataclasses


@dataclasses.dataclass(frozen=True)
class StageDutyCycles:
    """McPAT per-stage pipeline duty cycles (core.cc:3984-4082).

    The MMU stage uses `lsu`, as McPAT's MMU pipeline term reads
    LSU_duty_cycle.
    """

    pipeline: float = 1.0
    ifu: float = 0.9
    lsu: float = 0.71
    alu: float = 0.76

    @classmethod
    def from_mcpat_stats(cls, duty):
        """Builds from `mcpat_xml.McpatXmlParams.duty_cycles`."""
        return cls(
            duty["pipeline_duty_cycle"],
            duty["IFU_duty_cycle"],
            duty["LSU_duty_cycle"],
            duty["ALU_duty_cycle"],
        )


# On ARM every scalar FP operand lives in a vector register, so a
# scalar-FP workload reads its FP counts from the vec* counters instead.
ARM_SCALAR_FP_STAT_ALIASES = {
    "rename.fpLookups": "rename.vecLookups",
    "executeStats0.numFpRegReads": "executeStats0.numVecRegReads",
    "executeStats0.numFpRegWrites": "executeStats0.numVecRegWrites",
    "fpInstQueueReads": "vecInstQueueReads",
    "fpInstQueueWrites": "vecInstQueueWrites",
    "fpInstQueueWakeupAccesses": "vecInstQueueWakeupAccesses",
}
