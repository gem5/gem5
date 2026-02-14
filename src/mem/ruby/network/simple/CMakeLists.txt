# src/mem/ruby/network/simple/CMakeLists.txt
# Converted from src/mem/ruby/network/simple/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(PerfectSwitch.cc)
gem5_add_source(SimpleLink.cc)
gem5_add_source(SimpleNetwork.cc)
gem5_add_source(Switch.cc)
gem5_add_source(Throttle.cc)

gem5_add_source(routing/WeightBased.cc)

# SimObjects
gem5_add_simobject(SimpleLink.py SIM_OBJECTS SimpleExtLink SimpleIntLink)
gem5_add_simobject(SimpleNetwork.py
    SIM_OBJECTS SimpleNetwork BaseRoutingUnit WeightBased Switch)
