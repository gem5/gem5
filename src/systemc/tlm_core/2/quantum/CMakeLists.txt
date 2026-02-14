# src/systemc/tlm_core/2/quantum/CMakeLists.txt
# Converted from src/systemc/tlm_core/2/quantum/SConscript

gem5_add_source(global_quantum.cc)

if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_source(global_quantum_python.cc)
endif()
