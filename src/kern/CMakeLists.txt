# src/kern/CMakeLists.txt
# Converted from src/kern/SConscript

# --- Sources ---
gem5_add_source(linux/events.cc)
gem5_add_source(linux/linux.cc)
gem5_add_source(linux/helpers.cc)
gem5_add_source(linux/printk.cc)
gem5_add_source(freebsd/events.cc)
gem5_add_source(operatingsystem.cc)
gem5_add_source(system_events.cc)

# --- Debug flags ---
gem5_add_debug_flag(DebugPrintf "DebugPrintf debugging")
gem5_add_debug_flag(Printf "Printf debugging")
