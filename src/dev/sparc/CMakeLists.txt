# src/dev/sparc/CMakeLists.txt
# Converted from src/dev/sparc/SConscript
# Conditional on CONF_USE_SPARC_ISA (guarded by parent CMakeLists.txt)

gem5_add_source(dtod.cc)
gem5_add_source(iob.cc)
gem5_add_source(t1000.cc)
gem5_add_source(mm_disk.cc)

gem5_add_debug_flag(Iob "Iob debugging")

gem5_add_simobject(T1000.py SIM_OBJECTS MmDisk DumbTOD Iob T1000)
