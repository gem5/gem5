# src/mem/probes/CMakeLists.txt
# Converted from src/mem/probes/SConscript

gem5_add_source(base.cc)
gem5_add_source(mem_footprint.cc)
gem5_add_source(stack_dist.cc)

# Conditional: protobuf trace
if(HAVE_PROTOBUF)
    gem5_add_source(mem_trace.cc)
endif()

# SimObjects
gem5_add_simobject(BaseMemProbe.py SIM_OBJECTS BaseMemProbe)
gem5_add_simobject(StackDistProbe.py SIM_OBJECTS StackDistProbe)
gem5_add_simobject(MemFootprintProbe.py SIM_OBJECTS MemFootprintProbe)

if(HAVE_PROTOBUF)
    gem5_add_simobject(MemTraceProbe.py SIM_OBJECTS MemTraceProbe)
endif()
