# src/cpu/probes/CMakeLists.txt
# Converted from src/cpu/probes/SConscript

gem5_add_source(pc_count_tracker.cc)
gem5_add_source(pc_count_tracker_manager.cc)
gem5_add_source(inst_tracker.cc)

gem5_add_debug_flag(PcCountTracker "PcCountTracker debugging")
gem5_add_debug_flag(InstTracker "InstTracker debugging")

gem5_add_simobject(PcCountTracker.py
    SIM_OBJECTS PcCountTracker PcCountTrackerManager)
gem5_add_simobject(InstTracker.py
    SIM_OBJECTS GlobalInstTracker LocalInstTracker)
