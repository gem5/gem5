# src/dev/amdgpu/CMakeLists.txt
# Converted from src/dev/amdgpu/SConscript
# Conditional on CONF_BUILD_GPU AND CONF_USE_X86_ISA (guarded by parent)

gem5_add_source(amdgpu_device.cc)
gem5_add_source(amdgpu_gfx.cc)
gem5_add_source(amdgpu_nbio.cc)
gem5_add_source(amdgpu_smu.cc)
gem5_add_source(amdgpu_vm.cc)
gem5_add_source(interrupt_handler.cc)
gem5_add_source(memory_manager.cc)
gem5_add_source(mmio_reader.cc)
gem5_add_source(pm4_packet_processor.cc)
gem5_add_source(sdma_engine.cc)
gem5_add_source(system_hub.cc)

gem5_add_debug_flag(AMDGPUDevice "AMDGPUDevice debugging")
gem5_add_debug_flag(AMDGPUMem "AMDGPUMem debugging")
gem5_add_debug_flag(AMDGPUSystemHub "AMDGPUSystemHub debugging")
gem5_add_debug_flag(PM4PacketProcessor "PM4PacketProcessor debugging")
gem5_add_debug_flag(SDMAEngine "SDMAEngine debugging")
gem5_add_debug_flag(SDMAData "SDMAData debugging")

gem5_add_simobject(AMDGPU.py SIM_OBJECTS
    AMDGPUDevice AMDGPUInterruptHandler AMDGPUMemoryManager
    AMDGPUSystemHub SDMAEngine PM4PacketProcessor)
