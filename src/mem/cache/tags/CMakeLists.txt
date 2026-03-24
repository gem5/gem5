# src/mem/cache/tags/CMakeLists.txt
# Converted from src/mem/cache/tags/SConscript

gem5_add_source(base.cc)
gem5_add_source(base_set_assoc.cc)
gem5_add_source(compressed_tags.cc)
gem5_add_source(dueling.cc)
gem5_add_source(fa_lru.cc)
gem5_add_source(sector_blk.cc)
gem5_add_source(sector_tags.cc)
gem5_add_source(super_blk.cc)

add_subdirectory(indexing_policies)
add_subdirectory(partitioning_policies)

# SimObjects
gem5_add_simobject(Tags.py
    SIM_OBJECTS
        BaseTags BaseSetAssoc SectorTags CompressedTags FALRU
        TaggedIndexingPolicy TaggedSetAssociative)

# --- Unit tests ---
gem5_add_test_source(dueling.test.cc)
