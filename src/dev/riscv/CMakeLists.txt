# src/dev/riscv/CMakeLists.txt
# Converted from src/dev/riscv/SConscript
# Conditional on CONF_USE_RISCV_ISA (guarded by parent CMakeLists.txt)

gem5_add_source(pci_host.cc)
gem5_add_source(hifive.cc)
gem5_add_source(lupv.cc)
gem5_add_source(clint.cc)
gem5_add_source(plic_device.cc)
gem5_add_source(plic.cc)
gem5_add_source(rtc.cc)
gem5_add_source(vio_mmio.cc)

gem5_add_debug_flag(Clint "Clint debugging")
gem5_add_debug_flag(Plic "Plic debugging")
gem5_add_debug_flag(VirtIOMMIO "VirtIOMMIO debugging")

gem5_add_simobject(HiFive.py SIM_OBJECTS HiFiveBase GenericRiscvPciHost)
gem5_add_simobject(LupV.py SIM_OBJECTS LupV)
gem5_add_simobject(Clint.py SIM_OBJECTS Clint)
gem5_add_simobject(PlicDevice.py SIM_OBJECTS PlicIntDevice)
gem5_add_simobject(Plic.py SIM_OBJECTS PlicBase Plic)
gem5_add_simobject(RTC.py SIM_OBJECTS RiscvRTC)
gem5_add_simobject(RiscvVirtIOMMIO.py SIM_OBJECTS RiscvMmioVirtIO)
