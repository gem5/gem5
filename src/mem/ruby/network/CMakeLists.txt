# src/mem/ruby/network/CMakeLists.txt
# Converted from src/mem/ruby/network/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(BasicLink.cc)
gem5_add_source(BasicRouter.cc)
gem5_add_source(MessageBuffer.cc)
gem5_add_source(Network.cc)
gem5_add_source(Topology.cc)

add_subdirectory(fault_model)
add_subdirectory(garnet)
add_subdirectory(simple)

# SimObjects
gem5_add_simobject(BasicLink.py
    SIM_OBJECTS BasicLink BasicExtLink BasicIntLink)
gem5_add_simobject(BasicRouter.py SIM_OBJECTS BasicRouter)
gem5_add_simobject(MessageBuffer.py
    SIM_OBJECTS MessageBuffer ENUMS MessageRandomization)
gem5_add_simobject(Network.py SIM_OBJECTS RubyNetwork)
