# src/cpu/trace/CMakeLists.txt
# Converted from src/cpu/trace/SConscript
# Conditional on HAVE_PROTOBUF (guarded by parent)

gem5_add_source(trace_cpu.cc)

gem5_add_debug_flag(TraceCPUData "TraceCPUData debugging")
gem5_add_debug_flag(TraceCPUInst "TraceCPUInst debugging")

gem5_add_simobject(TraceCPU.py
    SIM_OBJECTS TraceCPU)
