# src/cpu/kvm/CMakeLists.txt
# Converted from src/cpu/kvm/SConscript
# Conditional on CONF_USE_KVM (guarded by parent)

gem5_add_source(base.cc)
gem5_add_source(device.cc)
gem5_add_source(vm.cc)
gem5_add_source(perfevent.cc)
gem5_add_source(timer.cc)

gem5_add_debug_flag(Kvm "Basic KVM Functionality")
gem5_add_debug_flag(KvmContext "KVM/gem5 context synchronization")
gem5_add_debug_flag(KvmIO "KVM MMIO diagnostics")
gem5_add_debug_flag(KvmInt "KVM Interrupt handling")
gem5_add_debug_flag(KvmRun "KvmRun entry/exit diagnostics")
gem5_add_debug_flag(KvmTimer "KVM timing")

gem5_add_debug_flag(KvmAll "All KVM debug flags"
    COMPONENTS Kvm KvmContext KvmRun KvmIO KvmInt KvmTimer)

gem5_add_simobject(KvmVM.py
    SIM_OBJECTS KvmVM)
gem5_add_simobject(BaseKvmCPU.py
    SIM_OBJECTS BaseKvmCPU)
