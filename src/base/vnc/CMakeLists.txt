# src/base/vnc/CMakeLists.txt
# Converted from src/base/vnc/SConscript

gem5_add_source(vncinput.cc)
gem5_add_source(vncserver.cc)

gem5_add_debug_flag(VNC "VNC server debugging")

gem5_add_simobject(Vnc.py SIM_OBJECTS VncInput VncServer)
