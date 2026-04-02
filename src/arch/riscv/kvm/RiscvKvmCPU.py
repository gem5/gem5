from m5.objects.BaseKvmCPU import BaseKvmCPU
from m5.objects.RiscvCPU import RiscvCPU
from m5.objects.RiscvISA import RiscvISA
from m5.objects.RiscvMMU import RiscvMMU


class RiscvKvmCPU(BaseKvmCPU, RiscvCPU):
    type = "RiscvKvmCPU"
    cxx_header = "arch/riscv/kvm/riscv_cpu.hh"
    cxx_class = "gem5::RiscvKvmCPU"

    mmu = RiscvMMU()
    # Keep RVV opt-in for KVM configs. If users enable RVV, the board validates
    # the requested VLEN against KVM before instantiation.
    isa = [RiscvISA(enable_rvv=False)]
