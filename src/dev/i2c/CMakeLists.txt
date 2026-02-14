# src/dev/i2c/CMakeLists.txt
# Converted from src/dev/i2c/SConscript

gem5_add_source(bus.cc)

gem5_add_simobject(I2C.py SIM_OBJECTS I2CDevice I2CBus)
