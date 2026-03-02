# cmake/Gem5Targets.cmake
# Final link targets: gem5 executable, static/shared libraries, unit tests.
#
# This module must be included AFTER all add_subdirectory() calls and
# gem5_build_gem5py_m5(), so that all global source properties are populated.

include(CTest)

# ---------------------------------------------------------------------------
# Collect OBJECT library sources
# ---------------------------------------------------------------------------
# Each src/ subdirectory creates its own OBJECT library via gem5_add_source().
# Collect all their compiled objects for composing the final link targets.
gem5_get_all_object_sources(_all_obj_sources)

# Generated sources (debug flags, SimObject params, etc.) are compiled
# centrally in a single OBJECT target.
get_property(_gen_srcs GLOBAL PROPERTY GEM5_GENERATED_SOURCES)
if(_gen_srcs)
    add_library(gem5_generated_objs OBJECT ${_gen_srcs})
    target_link_libraries(gem5_generated_objs PRIVATE gem5_deps)
    target_compile_options(gem5_generated_objs PRIVATE ${GEM5_WERROR_FLAGS})
    set_target_properties(gem5_generated_objs PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# PySource object library (breaks gem5py_m5 dependency cycle)
# ---------------------------------------------------------------------------
# PySource .cc files are compiled separately so that gem5_all can depend on
# SimObject codegen targets (which depend on gem5py_m5) without creating a
# dependency cycle through shared source files.
#
# When building without Python (GEM5_WITHOUT_PYTHON=ON), the embedded Python
# bytecode sources are excluded from the library. The SCons build achieves
# this by filtering out sources tagged 'python' from the library filter.
if(NOT GEM5_WITHOUT_PYTHON)
    get_property(_py_srcs GLOBAL PROPERTY GEM5_PYSOURCES)
    add_library(gem5_pysources OBJECT ${_py_srcs})
    target_link_libraries(gem5_pysources PRIVATE gem5_deps pybind11::pybind11)
    target_compile_options(gem5_pysources PRIVATE ${GEM5_WERROR_FLAGS})
    set_target_properties(gem5_pysources PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# Common dependencies: link libraries into gem5_deps
# ---------------------------------------------------------------------------
# gem5_deps INTERFACE library is created in the top-level CMakeLists.txt
# (with include directories and warning flags). Here we add link libraries
# that depend on ext/ targets created by add_subdirectory(ext).

# Always-built ext libraries
target_link_libraries(gem5_deps INTERFACE
    gem5_ext_elf
    gem5_ext_fputils
    gem5_ext_libfdt
    gem5_ext_nomali
    gem5_ext_drampower
    gem5_ext_iostream3
    gem5_ext_magic_enum
    gem5_ext_softfloat
)

# SystemC is only linked when enabled; integrations like
# gem5_within_systemc and util/tlm link an external libsystemc
# and require the internal one to be absent.
if(CONF_USE_SYSTEMC)
    target_link_libraries(gem5_deps INTERFACE gem5_ext_systemc)
endif()

# Optional ext libraries
if(HAVE_DRAMSIM)
    target_link_libraries(gem5_deps INTERFACE gem5_ext_dramsim2)
endif()
if(HAVE_DRAMSIM3)
    target_link_libraries(gem5_deps INTERFACE gem5_ext_dramsim3)
endif()
if(HAVE_DRAMSYS)
    target_link_libraries(gem5_deps INTERFACE gem5_ext_dramsys)
endif()

# System / found libraries
target_link_libraries(gem5_deps INTERFACE
    ZLIB::ZLIB
    Threads::Threads
)
# Python3 and pybind11 are only needed when building with Python support.
# Without Python, gem5 uses C++-based configuration (--with-cxx-config).
if(NOT GEM5_WITHOUT_PYTHON)
    target_link_libraries(gem5_deps INTERFACE
        Python3::Python
        pybind11::embed
    )
endif()

if(HAVE_PROTOBUF)
    target_link_libraries(gem5_deps INTERFACE protobuf::libprotobuf)
endif()
if(HAVE_PNG)
    target_link_libraries(gem5_deps INTERFACE PNG::PNG)
endif()
if(HAVE_HDF5)
    target_link_libraries(gem5_deps INTERFACE ${HDF5_CXX_LIBRARIES})
    target_include_directories(gem5_deps INTERFACE ${HDF5_CXX_INCLUDE_DIRS})
endif()
if(HAVE_TCMALLOC)
    target_link_libraries(gem5_deps INTERFACE tcmalloc::tcmalloc)
endif()
if(HAVE_CAPSTONE)
    target_link_libraries(gem5_deps INTERFACE Capstone::Capstone)
endif()
if(HAVE_LIBRT)
    target_link_libraries(gem5_deps INTERFACE rt)
endif()

# Additional libraries registered by subsystems (e.g., TLM armtlmchi)
get_property(_extra_libs GLOBAL PROPERTY GEM5_LINK_LIBRARIES)
if(_extra_libs)
    target_link_libraries(gem5_deps INTERFACE ${_extra_libs})
endif()

# -rdynamic so Python extensions can see gem5 symbols
if(NOT WIN32)
    target_link_options(gem5_deps INTERFACE -rdynamic)
endif()

# ---------------------------------------------------------------------------
# Code generation target dependencies
# ---------------------------------------------------------------------------
# Phase 1 targets: debug flags, ISA parser (no gem5py_m5 dependency)
get_property(_codegen_phase1 GLOBAL PROPERTY GEM5_CODEGEN_TARGETS_PHASE1)

# ---------------------------------------------------------------------------
# Propagate codegen dependencies to all OBJECT libraries
# ---------------------------------------------------------------------------
# OBJECT targets need generated headers (debug flags, ISA decoder, SimObject
# params) to exist before compilation.  These targets are only known after all
# subdirectories have been processed, so we add the deps retroactively.
get_property(_obj_libs GLOBAL PROPERTY GEM5_OBJECT_LIBS)
get_property(_slicc_targets GLOBAL PROPERTY GEM5_SLICC_TARGETS)

foreach(_obj ${_obj_libs})
    if(_codegen_phase1)
        add_dependencies(${_obj} ${_codegen_phase1})
    endif()
    if(_slicc_targets)
        add_dependencies(${_obj} ${_slicc_targets})
    endif()
endforeach()

if(TARGET gem5_generated_objs)
    if(_codegen_phase1)
        add_dependencies(gem5_generated_objs ${_codegen_phase1})
    endif()
    if(_slicc_targets)
        add_dependencies(gem5_generated_objs ${_slicc_targets})
    endif()
endif()

# ---------------------------------------------------------------------------
# Static library: libgem5_all
# ---------------------------------------------------------------------------
# Composed from per-directory OBJECT libraries + generated objects + date.cc.
# date.cc embeds the compile timestamp and is compiled directly by gem5_all.
set(_gem5_all_sources
    ${_all_obj_sources}
    "${CMAKE_SOURCE_DIR}/src/base/date.cc"
)
if(TARGET gem5_generated_objs)
    list(APPEND _gem5_all_sources "$<TARGET_OBJECTS:gem5_generated_objs>")
endif()

add_library(gem5_all STATIC ${_gem5_all_sources})
target_link_libraries(gem5_all PUBLIC gem5_deps)
target_compile_options(gem5_all PRIVATE ${GEM5_WERROR_FLAGS})

# Link PySource objects into gem5_all (only with Python support)
if(NOT GEM5_WITHOUT_PYTHON)
    target_sources(gem5_all PRIVATE $<TARGET_OBJECTS:gem5_pysources>)
endif()

set_target_properties(gem5_all PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# gem5 executable
# ---------------------------------------------------------------------------
add_executable(gem5 "${CMAKE_SOURCE_DIR}/src/sim/main.cc")
gem5_target_link_whole_archive(gem5 PRIVATE gem5_all)

# Stripped binary
add_custom_command(TARGET gem5 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:gem5>" "$<TARGET_FILE:gem5>.stripped"
    COMMAND strip "$<TARGET_FILE:gem5>.stripped"
    COMMENT "Creating stripped gem5 binary"
)

# ---------------------------------------------------------------------------
# Shared library: libgem5_shared
# ---------------------------------------------------------------------------
# Uses the same OBJECT targets as gem5_all (all compiled with -fPIC).
set(_gem5_shared_sources
    ${_all_obj_sources}
    "${CMAKE_SOURCE_DIR}/src/base/date.cc"
)
if(TARGET gem5_generated_objs)
    list(APPEND _gem5_shared_sources "$<TARGET_OBJECTS:gem5_generated_objs>")
endif()

add_library(gem5_shared SHARED ${_gem5_shared_sources})
target_link_libraries(gem5_shared PUBLIC gem5_deps)
target_compile_options(gem5_shared PRIVATE ${GEM5_WERROR_FLAGS})
if(NOT GEM5_WITHOUT_PYTHON)
    target_sources(gem5_shared PRIVATE $<TARGET_OBJECTS:gem5_pysources>)
endif()

# ---------------------------------------------------------------------------
# Unit tests (GTest)
# ---------------------------------------------------------------------------
# Discover all .test.cc files under src/ and create test executables.
# Each test links against the full gem5 static library + gtest.

# gem5 GTest logging support: provides gtestLogOutput used by many tests.
add_library(gem5_gtest_logging STATIC
    "${CMAKE_SOURCE_DIR}/src/base/gtest/logging.cc"
)
# gem5_ext_gtest propagates googletest/googlemock include paths as SYSTEM PUBLIC,
# so no explicit target_include_directories is needed here.
target_link_libraries(gem5_gtest_logging PUBLIC gem5_deps gem5_ext_gtest)
set_target_properties(gem5_gtest_logging PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# gem5 GTest mock loggers: replaces Logger::getPanic()/getWarn()/etc. with
# mock versions that redirect output to gtestLogOutput instead of stderr.
# Must be linked with --whole-archive BEFORE gem5_all.
add_library(gem5_gtest_mock STATIC
    "${CMAKE_SOURCE_DIR}/src/base/gtest/logging_mock.cc"
)
# Include paths are inherited transitively through gem5_gtest_logging -> gem5_ext_gtest.
target_link_libraries(gem5_gtest_mock PUBLIC gem5_gtest_logging)
add_dependencies(gem5_gtest_mock gem5_all)
set_target_properties(gem5_gtest_mock PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

# Collect explicitly registered test sources (ISA/GPU filtering is handled
# by CONDITION parameters in each gem5_add_test_source() call).
get_property(_test_sources GLOBAL PROPERTY GEM5_TEST_CC_SOURCES)

# Safety check: detect .test.cc files on disk that have no gem5_add_test_source()
# call at all. GEM5_TEST_CC_ALL_SOURCES includes every registration regardless
# of CONDITION, so condition-excluded tests do not produce false positives.
get_property(_all_registered_sources GLOBAL PROPERTY GEM5_TEST_CC_ALL_SOURCES)
file(GLOB_RECURSE _glob_test_sources "${CMAKE_SOURCE_DIR}/src/*.test.cc")
foreach(_glob_src ${_glob_test_sources})
    if(NOT "${_glob_src}" IN_LIST _all_registered_sources)
        message(WARNING
            "Test file not registered: ${_glob_src}\n"
            "Add gem5_add_test_source() for this file in its CMakeLists.txt.")
    endif()
endforeach()

set(_all_test_names "")
foreach(_test_src ${_test_sources})
    # Derive test name from path: src/base/bitfield.test.cc -> test_base_bitfield
    file(RELATIVE_PATH _rel "${CMAKE_SOURCE_DIR}/src" "${_test_src}")
    string(REPLACE "/" "_" _test_name "${_rel}")
    string(REPLACE ".test.cc" "" _test_name "${_test_name}")
    set(_test_name "test_${_test_name}")
    list(APPEND _all_test_names ${_test_name})

    add_executable(${_test_name} EXCLUDE_FROM_ALL "${_test_src}")

    # The logging test uses skip_lib=True in SCons: it tests the REAL
    # Logger (which writes to cerr) rather than the mock (which redirects
    # to gtestLogOutput).  Link it without the mock library.
    if("${_test_name}" STREQUAL "test_base_logging")
        target_link_libraries(${_test_name} PRIVATE
            gem5_gtest_logging
            gem5_all
            gem5_ext_gmock
            gem5_ext_gtest
        )
    else()
        gem5_target_link_whole_archive(${_test_name} PRIVATE gem5_gtest_mock)
        target_link_libraries(${_test_name} PRIVATE
            gem5_all
            gem5_ext_gmock
            gem5_ext_gtest
        )
    endif()
    # googletest/googlemock include paths are propagated transitively through
    # gem5_ext_gtest and gem5_ext_gmock (both declare them SYSTEM PUBLIC).
    # Ensure all generated headers (enums, params from phase 2 codegen) are
    # available before compiling test sources.  The simplest way is to wait
    # for gem5_all to finish: it already depends on all codegen targets, and
    # tests link against it anyway.
    add_dependencies(${_test_name} gem5_all)
    add_test(NAME ${_test_name} COMMAND ${_test_name})
endforeach()

# Convenience target: build all unit tests
add_custom_target(gem5_tests)
if(_all_test_names)
    add_dependencies(gem5_tests ${_all_test_names})
endif()

# ---------------------------------------------------------------------------
# Install targets
# ---------------------------------------------------------------------------
install(TARGETS gem5 RUNTIME DESTINATION bin)
install(TARGETS gem5_shared LIBRARY DESTINATION lib)
install(TARGETS gem5_all ARCHIVE DESTINATION lib)
