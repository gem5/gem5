# src/dev/hsa/CMakeLists.txt
# Converted from src/dev/hsa/SConscript
# Conditional on CONF_BUILD_GPU (guarded by parent CMakeLists.txt)

gem5_add_source(hsa_packet_processor.cc)
gem5_add_source(hw_scheduler.cc)

gem5_add_debug_flag(HSAPacketProcessor "HSAPacketProcessor debugging")

gem5_add_simobject(HSADevice.py SIM_OBJECTS HSAPacketProcessor)
