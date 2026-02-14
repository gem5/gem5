# src/systemc/CMakeLists.txt
# Converted from src/systemc/SConscript
# Conditional on CONF_USE_SYSTEMC (guarded by parent)

gem5_add_simobject(Tlm.py)

add_subdirectory(core)
add_subdirectory(channel)
add_subdirectory(dt)
add_subdirectory(tlm_bridge)
add_subdirectory(tlm_core)
add_subdirectory(tlm_utils)
add_subdirectory(utils)

if(NOT GEM5_WITHOUT_PYTHON)
    add_subdirectory(python)
endif()

option(GEM5_WITH_SYSTEMC_TESTS "Build SystemC regression tests" OFF)
if(GEM5_WITH_SYSTEMC_TESTS)
    add_subdirectory(tests)
endif()
