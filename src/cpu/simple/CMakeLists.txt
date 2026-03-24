# src/cpu/simple/CMakeLists.txt
# Converted from src/cpu/simple/SConscript
# Conditional on CONF_BUILD_ISA (guarded by parent)

gem5_add_source(atomic.cc)
gem5_add_source(noncaching.cc)
gem5_add_source(timing.cc)
gem5_add_source(base.cc)

gem5_add_debug_flag(SimpleCPU "SimpleCPU debugging")

add_subdirectory(probes)

gem5_add_simobject(BaseAtomicSimpleCPU.py
    SIM_OBJECTS BaseAtomicSimpleCPU)
gem5_add_simobject(BaseNonCachingSimpleCPU.py
    SIM_OBJECTS BaseNonCachingSimpleCPU)
gem5_add_simobject(BaseTimingSimpleCPU.py
    SIM_OBJECTS BaseTimingSimpleCPU)
gem5_add_simobject(BaseSimpleCPU.py
    SIM_OBJECTS BaseSimpleCPU)
# For backwards compatibility (empty sim_objects, ISA-tagged)
gem5_add_simobject(AtomicSimpleCPU.py)
gem5_add_simobject(NonCachingSimpleCPU.py)
gem5_add_simobject(TimingSimpleCPU.py)
