# src/cpu/o3/CMakeLists.txt
# Converted from src/cpu/o3/SConscript
# Conditional on CONF_BUILD_ISA (guarded by parent)

gem5_add_source(bac.cc)
gem5_add_source(commit.cc)
gem5_add_source(cpu.cc)
gem5_add_source(decode.cc)
gem5_add_source(dyn_inst.cc)
gem5_add_source(fetch.cc)
gem5_add_source(free_list.cc)
gem5_add_source(ftq.cc)
gem5_add_source(fu_pool.cc)
gem5_add_source(iew.cc)
gem5_add_source(inst_queue.cc)
gem5_add_source(lsq.cc)
gem5_add_source(lsq_unit.cc)
gem5_add_source(mem_dep_unit.cc)
gem5_add_source(regfile.cc)
gem5_add_source(rename.cc)
gem5_add_source(rename_map.cc)
gem5_add_source(rob.cc)
gem5_add_source(scoreboard.cc)
gem5_add_source(store_set.cc)
gem5_add_source(thread_context.cc)
gem5_add_source(thread_state.cc)
gem5_add_source(checker.cc)

gem5_add_debug_flag(BAC "BAC debugging")
gem5_add_debug_flag(CommitRate "CommitRate debugging")
gem5_add_debug_flag(FTQ "FTQ debugging")
gem5_add_debug_flag(IEW "IEW debugging")
gem5_add_debug_flag(IQ "IQ debugging")
gem5_add_debug_flag(LSQ "LSQ debugging")
gem5_add_debug_flag(LSQUnit "LSQUnit debugging")
gem5_add_debug_flag(MemDepUnit "MemDepUnit debugging")
gem5_add_debug_flag(O3CPU "O3CPU debugging")
gem5_add_debug_flag(ROB "ROB debugging")
gem5_add_debug_flag(Rename "Rename debugging")
gem5_add_debug_flag(Scoreboard "Scoreboard debugging")
gem5_add_debug_flag(StoreSet "StoreSet debugging")
gem5_add_debug_flag(Writeback "Writeback debugging")

gem5_add_debug_flag(O3CPUAll "All O3 CPU debugging flags"
    COMPONENTS BAC FTQ Fetch Decode Rename IEW Commit IQ ROB FreeList
               LSQ LSQUnit StoreSet MemDepUnit DynInst O3CPU Activity
               Scoreboard Writeback)

add_subdirectory(probe)

gem5_add_simobject(FUPool.py
    SIM_OBJECTS FUPool)
gem5_add_simobject(FuncUnitConfig.py)
gem5_add_simobject(BaseO3CPU.py
    SIM_OBJECTS BaseO3CPU)
gem5_add_simobject(IQUnit.py
    SIM_OBJECTS IQUnit)
gem5_add_simobject(SMT.py
    ENUMS SMTFetchPolicy SMTQueuePolicy CommitPolicy)
gem5_add_simobject(BaseO3Checker.py
    SIM_OBJECTS BaseO3Checker)
# For backwards compatibility (empty sim_objects, ISA-tagged)
gem5_add_simobject(O3CPU.py)
gem5_add_simobject(O3Checker.py)
