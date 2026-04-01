from m5.objects.BaseKvmCPU import BaseKvmCPU
from m5.objects.RiscvCPU import RiscvCPU
from m5.objects.RiscvISA import RiscvISA
from m5.objects.RiscvMMU import RiscvMMU


class RiscvKvmCPU(BaseKvmCPU, RiscvCPU):
    type = "RiscvKvmCPU"
    cxx_header = "arch/riscv/kvm/riscv_cpu.hh"
    cxx_class = "gem5::RiscvKvmCPU"

    mmu = RiscvMMU()
    # KVM exposes the host's vector register size. Since gem5 does not
    # currently auto-discover host VLEN before ISA construction, keep RVV off
    # by default for portable KVM configs. Users can still opt in by
    # overriding the ISA parameters with a host-matching VLEN/ELEN.
    isa = [RiscvISA(enable_rvv=False)]
    # gem5 uses perf cycle counters to translate KVM run time into simulated
    # ticks. RISC-V host guest-cycle accounting is not reliable enough to use
    # as the default time source, so fall back to the generic wall-clock path.
    usePerf = False
