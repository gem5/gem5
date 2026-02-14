# src/base/filters/CMakeLists.txt
# Converted from src/base/filters/SConscript

gem5_add_source(block_bloom_filter.cc)
gem5_add_source(bulk_bloom_filter.cc)
gem5_add_source(h3_bloom_filter.cc)
gem5_add_source(multi_bit_sel_bloom_filter.cc)
gem5_add_source(multi_bloom_filter.cc)
gem5_add_source(perfect_bloom_filter.cc)

gem5_add_simobject(BloomFilters.py SIM_OBJECTS
    BloomFilterBase BloomFilterBlock BloomFilterMultiBitSel
    BloomFilterBulk BloomFilterH3 BloomFilterMulti BloomFilterPerfect)
