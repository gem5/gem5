# src/dev/ps2/CMakeLists.txt
# Converted from src/dev/ps2/SConscript

gem5_add_source(device.cc)
gem5_add_source(keyboard.cc)
gem5_add_source(mouse.cc)
gem5_add_source(touchkit.cc)
gem5_add_source(types.cc)

gem5_add_debug_flag(PS2 "PS2 debugging")

gem5_add_simobject(PS2.py SIM_OBJECTS PS2Device PS2Keyboard PS2Mouse PS2TouchKit)
