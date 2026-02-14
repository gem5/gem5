# src/dev/mips/CMakeLists.txt
# Converted from src/dev/mips/SConscript
# Conditional on CONF_USE_MIPS_ISA (guarded by parent CMakeLists.txt)

gem5_add_source(malta.cc)
gem5_add_source(malta_cchip.cc)
gem5_add_source(malta_io.cc)

gem5_add_debug_flag(Malta "Malta debugging")

gem5_add_simobject(Malta.py SIM_OBJECTS MaltaCChip MaltaIO Malta)
