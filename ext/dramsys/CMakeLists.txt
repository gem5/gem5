# DRAMSys is optional: only built if the DRAMSys/ subdirectory has been
# cloned into ext/dramsys/.
#
# DRAMSys has its own CMake build system.  The original SConscript invoked
# cmake as an external process; here we integrate it natively via
# add_subdirectory() since we are already in a CMake build.

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/DRAMSys")
    set(HAVE_DRAMSYS OFF PARENT_SCOPE)
    message(STATUS "ext/dramsys: DRAMSys/ not found, skipping")
    return()
endif()

set(HAVE_DRAMSYS ON PARENT_SCOPE)
message(STATUS "ext/dramsys: DRAMSys/ found, building")

# DRAMSys build options
set(BUILD_SHARED_LIBS OFF)
set(DRAMSYS_BUILD_CLI OFF CACHE BOOL "" FORCE)
set(DRAMSYS_USE_FETCH_CONTENT ON CACHE BOOL "" FORCE)
set(DRAMSYS_USE_FETCH_CONTENT_INTERNAL ON CACHE BOOL "" FORCE)
set(DRAMSYS_USE_FETCH_CONTENT_SYSTEMC OFF CACHE BOOL "" FORCE)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

add_subdirectory(DRAMSys)

# Wrap DRAMSys targets with gem5 conventions
add_library(gem5_ext_dramsys INTERFACE)

# Link against DRAMSys libraries
if(TARGET dramsys)
    target_link_libraries(gem5_ext_dramsys INTERFACE dramsys)
endif()
if(TARGET DRAMPower)
    target_link_libraries(gem5_ext_dramsys INTERFACE DRAMPower)
endif()
if(TARGET sqlite3)
    target_link_libraries(gem5_ext_dramsys INTERFACE sqlite3)
endif()

target_include_directories(gem5_ext_dramsys INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/DRAMSys/src
)

# nlohmann_json include path (fetched by DRAMSys)
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/_deps/nlohmann_json-src/include")
    target_include_directories(gem5_ext_dramsys INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}/_deps/nlohmann_json-src/include
    )
endif()

# dramutils include path (fetched by DRAMSys)
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/_deps/dramutils-src/include")
    target_include_directories(gem5_ext_dramsys INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}/_deps/dramutils-src/include
    )
endif()

# DRAMPower include path (fetched by DRAMSys)
if(EXISTS "${CMAKE_CURRENT_BINARY_DIR}/_deps/drampower-src/src/DRAMPower")
    target_include_directories(gem5_ext_dramsys INTERFACE
        ${CMAKE_CURRENT_BINARY_DIR}/_deps/drampower-src/src/DRAMPower
    )
endif()

target_compile_definitions(gem5_ext_dramsys INTERFACE
    SYSTEMC_VERSION=20191203
)
