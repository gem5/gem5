# src/cpu/testers/spatter_gen/CMakeLists.txt
# Converted from src/cpu/testers/spatter_gen/SConscript

gem5_add_source(spatter_gen.cc)

gem5_add_debug_flag(SpatterGen "SpatterGen debugging")
gem5_add_debug_flag(SpatterKernel "SpatterKernel debugging")

gem5_add_simobject(SpatterGen.py
    SIM_OBJECTS SpatterGen
    ENUMS SpatterKernelType SpatterProcessingMode)
