# src/cpu/checker/CMakeLists.txt
# Converted from src/cpu/checker/SConscript

gem5_add_source(cpu.cc)

gem5_add_simobject(CheckerCPU.py
    SIM_OBJECTS CheckerCPU)
