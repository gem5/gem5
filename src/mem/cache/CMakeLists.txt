# src/mem/cache/CMakeLists.txt
# Converted from src/mem/cache/SConscript

gem5_add_source(base.cc)
gem5_add_source(cache.cc)
gem5_add_source(cache_blk.cc)
gem5_add_source(mshr.cc)
gem5_add_source(mshr_queue.cc)
gem5_add_source(noncoherent_cache.cc)
gem5_add_source(write_queue.cc)
gem5_add_source(write_queue_entry.cc)

gem5_add_debug_flag(Cache "Cache debugging")
gem5_add_debug_flag(CacheComp "CacheComp debugging")
gem5_add_debug_flag(CachePort "CachePort debugging")
gem5_add_debug_flag(CacheRepl "CacheRepl debugging")
gem5_add_debug_flag(CacheTags "CacheTags debugging")
gem5_add_debug_flag(CacheVerbose "CacheVerbose debugging")
gem5_add_debug_flag(HWPrefetch "HWPrefetch debugging")
gem5_add_debug_flag(HWPrefetchQueue "HWPrefetchQueue debugging")
gem5_add_debug_flag(MSHR "MSHR debugging")
gem5_add_debug_flag(PartitionPolicy "PartitionPolicy debugging")

gem5_add_debug_flag(CacheAll "CacheAll compound flag"
    COMPONENTS Cache CacheComp CachePort CacheRepl CacheVerbose
              HWPrefetch MSHR PartitionPolicy)

add_subdirectory(compressors)
add_subdirectory(prefetch)
add_subdirectory(replacement_policies)
add_subdirectory(tags)

# SimObjects
gem5_add_simobject(Cache.py
    SIM_OBJECTS WriteAllocator BaseCache Cache NoncoherentCache
    ENUMS Clusivity)
