# DRAMsim3 is optional: only available if the DRAMsim3/ subdirectory has been
# cloned into ext/dramsim3/.
#
# Unlike DRAMSim2, DRAMsim3 has its own CMake build system.  The SConscript
# links against a pre-built libdramsim3, so we use add_subdirectory() here to
# build it as part of the gem5 build if the subdir is present.

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/DRAMsim3")
    set(HAVE_DRAMSIM3 OFF PARENT_SCOPE)
    message(STATUS "ext/dramsim3: DRAMsim3/ not found, skipping")
    return()
endif()

set(HAVE_DRAMSIM3 ON PARENT_SCOPE)
message(STATUS "ext/dramsim3: DRAMsim3/ found, building")

# DRAMsim3 has its own CMakeLists.txt -- build it as a subdirectory.
# If it provides a CMake target we can link against, use that.
# Otherwise fall back to an imported library pointing at the built artifact.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/DRAMsim3/CMakeLists.txt")
    add_subdirectory(DRAMsim3 DRAMsim3_build)
    # Wrap the DRAMsim3 target with gem5 conventions
    if(NOT TARGET gem5_ext_dramsim3)
        add_library(gem5_ext_dramsim3 INTERFACE)
        target_link_libraries(gem5_ext_dramsim3 INTERFACE dramsim3)
        target_include_directories(gem5_ext_dramsim3 SYSTEM INTERFACE
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/DRAMsim3/src
        )
        find_package(OpenMP)
        if(OpenMP_CXX_FOUND)
            target_link_libraries(gem5_ext_dramsim3 INTERFACE OpenMP::OpenMP_CXX)
        endif()
    endif()
else()
    # No CMake build -- expect a pre-built library in DRAMsim3/
    add_library(gem5_ext_dramsim3 INTERFACE)
    target_include_directories(gem5_ext_dramsim3 SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/DRAMsim3/src
    )
    target_link_directories(gem5_ext_dramsim3 INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/DRAMsim3
    )
    target_link_libraries(gem5_ext_dramsim3 INTERFACE dramsim3)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(gem5_ext_dramsim3 INTERFACE OpenMP::OpenMP_CXX)
    endif()
endif()
