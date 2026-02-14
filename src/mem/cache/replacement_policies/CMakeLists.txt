# src/mem/cache/replacement_policies/CMakeLists.txt
# Converted from src/mem/cache/replacement_policies/SConscript

gem5_add_source(bip_rp.cc)
gem5_add_source(brrip_rp.cc)
gem5_add_source(dueling_rp.cc)
gem5_add_source(fifo_rp.cc)
gem5_add_source(lfu_rp.cc)
gem5_add_source(lru_rp.cc)
gem5_add_source(mru_rp.cc)
gem5_add_source(random_rp.cc)
gem5_add_source(second_chance_rp.cc)
gem5_add_source(ship_rp.cc)
gem5_add_source(tree_plru_rp.cc)
gem5_add_source(weighted_lru_rp.cc)

# SimObjects
gem5_add_simobject(ReplacementPolicies.py
    SIM_OBJECTS
        BaseReplacementPolicy DuelingRP FIFORP SecondChanceRP
        LFURP LRURP BIPRP MRURP RandomRP BRRIPRP SHiPRP
        SHiPMemRP SHiPPCRP TreePLRURP WeightedLRURP)
