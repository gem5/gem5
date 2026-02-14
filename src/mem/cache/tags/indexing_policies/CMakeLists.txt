# src/mem/cache/tags/indexing_policies/CMakeLists.txt
# Converted from src/mem/cache/tags/indexing_policies/SConscript

gem5_add_source(set_associative.cc)
gem5_add_source(skewed_associative.cc)

# SimObjects
gem5_add_simobject(IndexingPolicies.py
    SIM_OBJECTS BaseIndexingPolicy SetAssociative SkewedAssociative)
