# src/cpu/o3/probe/CMakeLists.txt
# Converted from src/cpu/o3/probe/SConscript

gem5_add_source(simple_trace.cc)
gem5_add_debug_flag(SimpleTrace "SimpleTrace debugging")

if(HAVE_PROTOBUF)
    gem5_add_source(elastic_trace.cc)
    gem5_add_debug_flag(ElasticTrace "ElasticTrace debugging")
endif()

gem5_add_simobject(SimpleTrace.py
    SIM_OBJECTS SimpleTrace)

if(HAVE_PROTOBUF)
    gem5_add_simobject(ElasticTrace.py
        SIM_OBJECTS ElasticTrace)
endif()
