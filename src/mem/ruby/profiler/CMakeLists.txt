# src/mem/ruby/profiler/CMakeLists.txt
# Converted from src/mem/ruby/profiler/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(AccessTraceForAddress.cc)
gem5_add_source(AddressProfiler.cc)
gem5_add_source(Profiler.cc)
gem5_add_source(StoreTrace.cc)
