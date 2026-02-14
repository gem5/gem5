# src/cpu/testers/garnet_synthetic_traffic/CMakeLists.txt
# Converted from src/cpu/testers/garnet_synthetic_traffic/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(GarnetSyntheticTraffic.cc)

gem5_add_debug_flag(GarnetSyntheticTraffic "GarnetSyntheticTraffic debugging")

gem5_add_simobject(GarnetSyntheticTraffic.py
    SIM_OBJECTS GarnetSyntheticTraffic)
