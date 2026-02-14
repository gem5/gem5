# src/mem/ruby/network/fault_model/CMakeLists.txt
# Converted from src/mem/ruby/network/fault_model/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(FaultModel.cc)

# SimObjects
gem5_add_simobject(FaultModel.py SIM_OBJECTS FaultModel)
