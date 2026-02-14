# src/mem/ruby/protocol/chi/generic/CMakeLists.txt
# Converted from src/mem/ruby/protocol/chi/generic/SConscript
# Conditional on CONF_RUBY_PROTOCOL_CHI (guarded by parent)

gem5_add_source(CHIGenericController.cc)

gem5_add_debug_flag(RubyCHIGeneric "RubyCHIGeneric debugging")
gem5_add_debug_flag(RubyCHIGenericVerbose "RubyCHIGenericVerbose debugging")

# SimObjects
gem5_add_simobject(CHIGeneric.py SIM_OBJECTS CHIGenericController)
