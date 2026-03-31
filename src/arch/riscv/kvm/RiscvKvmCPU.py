from m5.objects.BaseKvmCPU import BaseKvmCPU
from m5.objects.RiscvCPU import RiscvCPU
from m5.objects.RiscvMMU import RiscvMMU


class RiscvKvmCPU(BaseKvmCPU, RiscvCPU):
    type = "RiscvKvmCPU"
    cxx_header = "arch/riscv/kvm/riscv_cpu.hh"
    cxx_class = "gem5::RiscvKvmCPU"

    mmu = RiscvMMU()
    # RISC-V perf overflow delivery is not reliable enough to drive KVM exits.
    # Use the POSIX timer path by default so WFI/timer wakeups do not depend
    # on the next MMIO exit.
    usePerf = False
