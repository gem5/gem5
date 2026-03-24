# src/cpu/testers/traffic_gen/CMakeLists.txt
# Converted from src/cpu/testers/traffic_gen/SConscript

gem5_add_source(base.cc)
gem5_add_source(base_gen.cc)
gem5_add_source(dram_gen.cc)
gem5_add_source(dram_rot_gen.cc)
gem5_add_source(exit_gen.cc)
gem5_add_source(gups_gen.cc)
gem5_add_source(hybrid_gen.cc)
gem5_add_source(idle_gen.cc)
gem5_add_source(linear_gen.cc)
gem5_add_source(nvm_gen.cc)
gem5_add_source(random_gen.cc)
gem5_add_source(stream_gen.cc)
gem5_add_source(strided_gen.cc)

gem5_add_debug_flag(TrafficGen "TrafficGen debugging")
gem5_add_debug_flag(GUPSGen "GUPSGen debugging")

# Conditional: Python traffic generator
if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_source(pygen.cc)
endif()

# Conditional: Protobuf trace generator
if(HAVE_PROTOBUF)
    gem5_add_source(trace_gen.cc)
    gem5_add_source(traffic_gen.cc)
endif()

gem5_add_simobject(BaseTrafficGen.py
    SIM_OBJECTS BaseTrafficGen
    ENUMS StreamGenType)
gem5_add_simobject(GUPSGen.py
    SIM_OBJECTS GUPSGen)

# Conditional: Python traffic generator
if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_simobject(PyTrafficGen.py
        SIM_OBJECTS PyTrafficGen)
endif()

# Conditional: Protobuf trace generator
if(HAVE_PROTOBUF)
    gem5_add_simobject(TrafficGen.py
        SIM_OBJECTS TrafficGen)
endif()
