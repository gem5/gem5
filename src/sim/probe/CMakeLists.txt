# src/sim/probe/CMakeLists.txt
# Converted from src/sim/probe/SConscript

gem5_add_source(probe.cc)
gem5_add_source(probe_listener_object.cc)

gem5_add_debug_flag(ProbeVerbose "ProbeVerbose debugging")

gem5_add_simobject(Probe.py SIM_OBJECTS ProbeListenerObject)
