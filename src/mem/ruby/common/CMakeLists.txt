# src/mem/ruby/common/CMakeLists.txt
# Converted from src/mem/ruby/common/SConscript
# Conditional on CONF_RUBY (guarded by parent)

# NUMBER_BITS_PER_SET compile definition from Kconfig
add_definitions(-DNUMBER_BITS_PER_SET=${CONF_NUMBER_BITS_PER_SET})

gem5_add_source(Address.cc)
gem5_add_source(BoolVec.cc)
gem5_add_source(Consumer.cc)
gem5_add_source(DataBlock.cc)
gem5_add_source(Histogram.cc)
gem5_add_source(IntVec.cc)
gem5_add_source(NetDest.cc)
gem5_add_source(SubBlock.cc)
gem5_add_source(WriteMask.cc)
