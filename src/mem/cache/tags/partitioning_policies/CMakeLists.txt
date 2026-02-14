# src/mem/cache/tags/partitioning_policies/CMakeLists.txt
# Converted from src/mem/cache/tags/partitioning_policies/SConscript

gem5_add_source(base_pp.cc)
gem5_add_source(max_capacity_pp.cc)
gem5_add_source(partition_manager.cc)
gem5_add_source(way_allocation.cc)
gem5_add_source(way_pp.cc)

# SimObjects
gem5_add_simobject(PartitioningPolicies.py
    SIM_OBJECTS
        PartitionManager BasePartitioningPolicy
        MaxCapacityPartitioningPolicy WayPolicyAllocation
        WayPartitioningPolicy)
