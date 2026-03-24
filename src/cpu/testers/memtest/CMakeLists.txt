# src/cpu/testers/memtest/CMakeLists.txt
# Converted from src/cpu/testers/memtest/SConscript

gem5_add_source(memtest.cc)

gem5_add_debug_flag(MemTest "MemTest debugging")

gem5_add_simobject(MemTest.py
    SIM_OBJECTS MemTest)
