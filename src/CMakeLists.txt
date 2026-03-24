# src/CMakeLists.txt
# Top-level source directory for gem5.
# Subdirectories are added incrementally as they are converted from SCons.

add_subdirectory(arch)
add_subdirectory(base)
add_subdirectory(cpu)
add_subdirectory(dev)
add_subdirectory(mem)
add_subdirectory(kern)
add_subdirectory(proto)
add_subdirectory(python)
add_subdirectory(sim)
add_subdirectory(learning_gem5)
add_subdirectory(sst)

if(CONF_BUILD_GPU)
    add_subdirectory(gpu-compute)
endif()

if(CONF_USE_SYSTEMC)
    add_subdirectory(systemc)
endif()

if(CONF_USE_TEST_OBJECTS)
    add_subdirectory(test_objects)
endif()
