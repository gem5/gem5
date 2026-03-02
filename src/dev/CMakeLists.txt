# src/dev/CMakeLists.txt
# Converted from src/dev/SConscript

# --- Sources ---
gem5_add_source(io_device.cc)
gem5_add_source(isa_fake.cc)
gem5_add_source(dma_device.cc)
gem5_add_source(dma_virt_device.cc)
gem5_add_source(platform.cc)
gem5_add_source(baddev.cc)
gem5_add_source(intel_8254_timer.cc)
gem5_add_source(mc146818.cc)
gem5_add_source(pixelpump.cc)

# --- Debug flags ---
gem5_add_debug_flag(IntPin "IntPin debugging")
gem5_add_debug_flag(IsaFake "IsaFake debugging")
gem5_add_debug_flag(DMA "DMA debugging")
gem5_add_debug_flag(Intel8254Timer "Intel8254Timer debugging")
gem5_add_debug_flag(MC146818 "MC146818 debugging")

# --- SimObjects ---
gem5_add_simobject(Device.py SIM_OBJECTS
    PioDevice BasicPioDevice DmaDevice DmaVirtDevice IsaFake)
gem5_add_simobject(Platform.py SIM_OBJECTS Platform)
gem5_add_simobject(BadDevice.py SIM_OBJECTS BadDevice)

# --- Subdirectories ---
add_subdirectory(pci)
add_subdirectory(net)
add_subdirectory(storage)
add_subdirectory(serial)
add_subdirectory(virtio)
add_subdirectory(i2c)
add_subdirectory(ps2)
add_subdirectory(qemu)

# ISA-conditional subdirectories
if(CONF_USE_ARM_ISA)
    add_subdirectory(arm)
endif()
if(CONF_USE_X86_ISA)
    add_subdirectory(x86)
endif()
if(CONF_USE_MIPS_ISA)
    add_subdirectory(mips)
endif()
if(CONF_USE_SPARC_ISA)
    add_subdirectory(sparc)
endif()
if(CONF_USE_RISCV_ISA)
    add_subdirectory(riscv)
    add_subdirectory(lupio)
endif()
if(CONF_BUILD_GPU)
    add_subdirectory(hsa)
    if(CONF_USE_X86_ISA)
        add_subdirectory(amdgpu)
    endif()
endif()

# --- Unit tests ---
gem5_add_test_source(reg_bank.test.cc)
