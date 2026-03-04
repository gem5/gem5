# cmake/Gem5Targets.cmake
# Final link targets: gem5 executable, static/shared libraries, unit tests.
#
# This module must be included AFTER all add_subdirectory() calls and
# gem5_build_gem5py_m5(), so that all global source properties are populated.

include(CTest)

# Generated sources (debug flags, SimObject params, etc.) are compiled
# centrally in a STATIC library. Using STATIC (not OBJECT) allows
# individual whole-archive linking into the gem5 executable.
get_property(_gen_srcs GLOBAL PROPERTY GEM5_GENERATED_SOURCES)
if(_gen_srcs)
    add_library(gem5_generated STATIC ${_gen_srcs})
    target_link_libraries(gem5_generated PRIVATE gem5_deps)
    target_compile_options(gem5_generated PRIVATE ${GEM5_WERROR_FLAGS})
    set_target_properties(gem5_generated PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# ---------------------------------------------------------------------------
# PySource object library (breaks gem5py_m5 dependency cycle)
# ---------------------------------------------------------------------------
# PySource .cc files are compiled separately so that gem5_all can depend on
# SimObject codegen targets (which depend on gem5py_m5) without creating a
# dependency cycle through shared source files.
#
# Using STATIC (not OBJECT) allows individual whole-archive linking into
# the gem5 executable, preserving EmbeddedPython global constructor
# registrations.
#
# When building without Python (GEM5_WITHOUT_PYTHON=ON), the embedded Python
# bytecode sources are excluded from the library. The SCons build achieves
# this by filtering out sources tagged 'python' from the library filter.
if(NOT GEM5_WITHOUT_PYTHON)
    get_property(_py_srcs GLOBAL PROPERTY GEM5_PYSOURCES)
    add_library(gem5_pysources STATIC ${_py_srcs})
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

# ---------------------------------------------------------------------------
# ext/ library include-path propagation (scoped link deps)
# ---------------------------------------------------------------------------
# Scoped ext/ libraries are linked ONLY by their owning subsystem (defined
# in Gem5Subsystems.cmake), NOT globally via gem5_deps. However, OBJECT
# targets still need their SYSTEM include directories at compile time.
#
# _gem5_propagate_ext_includes() extracts INTERFACE_INCLUDE_DIRECTORIES
# from an ext/ library target and adds them to gem5_deps as SYSTEM INTERFACE
# includes -- without adding the link dependency itself.
function(_gem5_propagate_ext_includes lib)
    if(NOT TARGET "${lib}")
        return()
    endif()
    get_target_property(_inc_dirs "${lib}" INTERFACE_INCLUDE_DIRECTORIES)
    if(_inc_dirs)
        target_include_directories(gem5_deps SYSTEM INTERFACE ${_inc_dirs})
    endif()
endfunction()

# Scoped ext/ libraries: include dirs propagated globally, link deps scoped
# to owning subsystems (see Gem5Subsystems.cmake LINK_DEPS).
_gem5_propagate_ext_includes(gem5_ext_elf)        # -> gem5_base
_gem5_propagate_ext_includes(gem5_ext_drampower)   # -> gem5_mem
_gem5_propagate_ext_includes(gem5_ext_libfdt)      # -> gem5_dev
_gem5_propagate_ext_includes(gem5_ext_nomali)      # -> gem5_dev

# Global ext/ libraries: linked by gem5_deps because they are used
# pervasively across subsystems or carry critical INTERFACE include dirs.
target_link_libraries(gem5_deps INTERFACE
    gem5_ext_fputils
    gem5_ext_iostream3
    gem5_ext_magic_enum
    gem5_ext_softfloat
)

# SystemC is only linked when enabled; integrations like
# gem5_within_systemc and util/tlm link an external libsystemc
# and require the internal one to be absent.
if(CONF_USE_SYSTEMC)
    _gem5_propagate_ext_includes(gem5_ext_systemc) # -> gem5_systemc
endif()

# Conditional ext/ libraries: include dirs propagated, link scoped to gem5_mem
if(HAVE_DRAMSIM)
    _gem5_propagate_ext_includes(gem5_ext_dramsim2)  # -> gem5_mem
endif()
if(HAVE_DRAMSIM3)
    _gem5_propagate_ext_includes(gem5_ext_dramsim3)  # -> gem5_mem
endif()
if(HAVE_DRAMSYS)
    _gem5_propagate_ext_includes(gem5_ext_dramsys)   # -> gem5_mem
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

# Combine all codegen deps into a single list for cleaner propagation.
set(_codegen_deps ${_codegen_phase1} ${_slicc_targets})
if(_codegen_deps)
    foreach(_obj ${_obj_libs})
        add_dependencies(${_obj} ${_codegen_deps})
    endforeach()
    if(TARGET gem5_generated)
        add_dependencies(gem5_generated ${_codegen_deps})
    endif()
endif()

# ---------------------------------------------------------------------------
# Subsystem STATIC libraries
# ---------------------------------------------------------------------------
# Define subsystem STATIC libraries from per-directory OBJECT library manifests.
# Must happen after codegen deps are propagated (subsystems need codegen deps)
# and before final target creation (gem5 executable links subsystems).
include(Gem5Subsystems)

# ---------------------------------------------------------------------------
# Static library: libgem5_all (thin subsystem aggregation)
# ---------------------------------------------------------------------------
# gem5_all is a thin aggregation target that links all subsystem STATIC
# libraries, gem5_generated, gem5_pysources, and gem5_deps. Unit tests
# and downstream consumers link gem5_all for convenience (no need for
# per-subsystem linking). Normal linking (not whole-archive) is sufficient
# for unit tests because tests explicitly reference the symbols they need.
#
# CMake 3.24+ path: gem5_all compiles only date.cc and links subsystem
# STATIC libraries using $<LINK_GROUP:RESCAN,...> (--start-group /
# --end-group) to resolve circular inter-subsystem dependencies.
#
# CMake < 3.24 fallback: gem5_all includes all OBJECT sources directly
# (no circular dependency issue since all objects are in one archive).
gem5_get_subsystem_libs(_subsystem_libs)

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24" AND NOT APPLE)
    # CMake 3.24+: thin archive with subsystem STATICs linked via RESCAN
    # (--start-group / --end-group) to resolve circular dependencies
    # (e.g., gem5_base references gem5_sim::print_backtrace and vice versa).
    # Apple's linker does not support --start-group/--end-group, so macOS
    # always uses the OBJECT-sources fallback path below.
    add_library(gem5_all STATIC
        "${CMAKE_SOURCE_DIR}/src/base/date.cc"
    )
    set(_gem5_all_link_libs ${_subsystem_libs})
    if(TARGET gem5_generated)
        list(APPEND _gem5_all_link_libs gem5_generated)
    endif()
    if(NOT GEM5_WITHOUT_PYTHON)
        list(APPEND _gem5_all_link_libs gem5_pysources)
    endif()
    target_link_libraries(gem5_all PUBLIC
        "$<LINK_GROUP:RESCAN,${_gem5_all_link_libs}>"
    )
else()
    # CMake < 3.24 fallback: compose from raw OBJECT sources (no circular
    # dependency issue since all objects are in one archive) and link
    # subsystem STATICs for their scoped ext/ library dependencies.
    gem5_get_all_object_sources(_all_obj_sources)
    add_library(gem5_all STATIC
        ${_all_obj_sources}
        "${CMAKE_SOURCE_DIR}/src/base/date.cc"
    )
    target_link_libraries(gem5_all PUBLIC ${_subsystem_libs})
    if(TARGET gem5_generated)
        target_link_libraries(gem5_all PUBLIC gem5_generated)
    endif()
    if(NOT GEM5_WITHOUT_PYTHON)
        target_link_libraries(gem5_all PUBLIC gem5_pysources)
    endif()
endif()
target_link_libraries(gem5_all PUBLIC gem5_deps)
target_compile_options(gem5_all PRIVATE ${GEM5_WERROR_FLAGS})
set_target_properties(gem5_all PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ---------------------------------------------------------------------------
# Helper: whole-archive-link all subsystems + codegen to a target
# ---------------------------------------------------------------------------
# Reusable helper that links every subsystem STATIC library, gem5_generated,
# and gem5_pysources into a target using --whole-archive so that SimObject
# factory and EmbeddedPython global constructors survive linking.
function(_gem5_link_all_whole_archive target visibility)
    foreach(_subsys ${_subsystem_libs})
        gem5_target_link_whole_archive(${target} ${visibility} ${_subsys})
    endforeach()
    if(TARGET gem5_generated)
        gem5_target_link_whole_archive(${target} ${visibility} gem5_generated)
    endif()
    if(NOT GEM5_WITHOUT_PYTHON)
        gem5_target_link_whole_archive(${target} ${visibility} gem5_pysources)
    endif()
endfunction()

# ---------------------------------------------------------------------------
# gem5 executable (per-subsystem whole-archive linking)
# ---------------------------------------------------------------------------
# Each subsystem STATIC library is individually whole-archive-linked to
# preserve SimObject factory registrations and EmbeddedPython global
# constructors. Simply whole-archiving gem5_all would NOT force-include
# objects from nested STATIC libraries (gem5_base, gem5_sim, etc.).
#
# NOTE: We do NOT link gem5_all here -- it would duplicate every object
# already present in the subsystem STATIC libraries.
add_executable(gem5
    "${CMAKE_SOURCE_DIR}/src/sim/main.cc"
    "${CMAKE_SOURCE_DIR}/src/base/date.cc"
)
_gem5_link_all_whole_archive(gem5 PRIVATE)
target_link_libraries(gem5 PRIVATE gem5_deps)

# Stripped binary
add_custom_command(TARGET gem5 POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:gem5>" "$<TARGET_FILE:gem5>.stripped"
    COMMAND strip "$<TARGET_FILE:gem5>.stripped"
    COMMENT "Creating stripped gem5 binary"
)

# ---------------------------------------------------------------------------
# Shared library: libgem5_shared
# ---------------------------------------------------------------------------
# Links subsystem STATIC libraries + gem5_generated + gem5_pysources.
# Uses whole-archive to preserve global constructors (SimObject factories,
# EmbeddedPython registrations).
add_library(gem5_shared SHARED "${CMAKE_SOURCE_DIR}/src/base/date.cc")
target_link_libraries(gem5_shared PUBLIC gem5_deps)
target_compile_options(gem5_shared PRIVATE ${GEM5_WERROR_FLAGS})
_gem5_link_all_whole_archive(gem5_shared PUBLIC)

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
# Constructor-registration retention test
# ---------------------------------------------------------------------------
# Verify that SimObject factory and EmbeddedPython global constructors
# survive the per-subsystem whole-archive linking. Checks for known symbols
# in the gem5 binary using nm.
find_program(NM_EXECUTABLE nm)
if(NM_EXECUTABLE)
    add_test(
        NAME check_constructor_retention
        COMMAND ${CMAKE_COMMAND}
            -DNM=${NM_EXECUTABLE}
            -DGEM5_BINARY=$<TARGET_FILE:gem5>
            -P "${CMAKE_SOURCE_DIR}/cmake/CheckConstructorRetention.cmake"
    )
    set_tests_properties(check_constructor_retention PROPERTIES
        LABELS "link;smoke"
    )
endif()

# ---------------------------------------------------------------------------
# Install targets
# ---------------------------------------------------------------------------
install(TARGETS gem5 RUNTIME DESTINATION bin)
install(TARGETS gem5_shared LIBRARY DESTINATION lib)
install(TARGETS gem5_all ARCHIVE DESTINATION lib)
