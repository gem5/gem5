# src/mem/ruby/slicc_interface/CMakeLists.txt
# Converted from src/mem/ruby/slicc_interface/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(AbstractCacheEntry.cc)
gem5_add_source(AbstractController.cc)
gem5_add_source(RubyRequest.cc)

# SimObjects
gem5_add_simobject(Controller.py SIM_OBJECTS RubyController)
