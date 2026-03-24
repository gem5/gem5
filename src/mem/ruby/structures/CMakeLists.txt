# src/mem/ruby/structures/CMakeLists.txt
# Converted from src/mem/ruby/structures/SConscript
# Conditional on CONF_RUBY (guarded by parent)

gem5_add_source(ALUFreeListArray.cc)
gem5_add_source(BankedArray.cc)
gem5_add_source(CacheMemory.cc)
gem5_add_source(DirectoryMemory.cc)
gem5_add_source(PersistentTable.cc)
gem5_add_source(RubyPrefetcher.cc)
gem5_add_source(RubyPrefetcherProxy.cc)
gem5_add_source(TBEStorage.cc)
gem5_add_source(TimerTable.cc)
gem5_add_source(WireBuffer.cc)

# CHI protocol specific
if(CONF_RUBY_PROTOCOL_CHI)
    gem5_add_source(MN_TBETable.cc)
endif()

# SimObjects
gem5_add_simobject(RubyCache.py SIM_OBJECTS RubyCache)
gem5_add_simobject(DirectoryMemory.py SIM_OBJECTS RubyDirectoryMemory)
gem5_add_simobject(RubyPrefetcher.py SIM_OBJECTS RubyPrefetcher)
gem5_add_simobject(WireBuffer.py SIM_OBJECTS RubyWireBuffer)
