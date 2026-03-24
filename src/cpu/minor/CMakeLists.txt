# src/cpu/minor/CMakeLists.txt
# Converted from src/cpu/minor/SConscript
# Conditional on CONF_BUILD_ISA (guarded by parent)

gem5_add_source(activity.cc)
gem5_add_source(cpu.cc)
gem5_add_source(decode.cc)
gem5_add_source(dyn_inst.cc)
gem5_add_source(execute.cc)
gem5_add_source(fetch1.cc)
gem5_add_source(fetch2.cc)
gem5_add_source(func_unit.cc)
gem5_add_source(lsq.cc)
gem5_add_source(pipe_data.cc)
gem5_add_source(pipeline.cc)
gem5_add_source(scoreboard.cc)
gem5_add_source(stats.cc)

gem5_add_debug_flag(MinorCPU "Minor CPU-level events")
gem5_add_debug_flag(MinorExecute "Minor Execute stage")
gem5_add_debug_flag(MinorInterrupt "Minor interrupt handling")
gem5_add_debug_flag(MinorMem "Minor memory accesses")
gem5_add_debug_flag(MinorScoreboard "Minor Execute register scoreboard")
gem5_add_debug_flag(MinorTrace "MinorTrace cycle-by-cycle state trace")
gem5_add_debug_flag(MinorTiming "Extra timing for instructions")

gem5_add_debug_flag(Minor "All Minor CPU debugging flags"
    COMPONENTS MinorCPU MinorExecute MinorInterrupt MinorMem MinorScoreboard)

gem5_add_simobject(BaseMinorCPU.py
    SIM_OBJECTS MinorOpClass MinorOpClassSet MinorFUTiming MinorFU
                MinorFUPool BaseMinorCPU
    ENUMS ThreadPolicy)
# For backwards compatibility (empty sim_objects, ISA-tagged)
gem5_add_simobject(MinorCPU.py)
