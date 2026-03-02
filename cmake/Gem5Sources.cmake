# cmake/Gem5Sources.cmake
# Helper CMake functions for registering gem5 sources, python files,
# SimObjects, and debug flags.
#
# In the SCons build, each SConscript calls functions like Source(),
# SimObject(), PySource(), DebugFlag() etc.  These CMake equivalents
# collect sources into global properties that are later consumed when
# building the gem5 library target.

# ---------------------------------------------------------------------------
# Global source collectors (CMake global properties)
# ---------------------------------------------------------------------------

# We use GLOBAL properties to accumulate data across subdirectories.
# Each property stores a list of target names, absolute file paths, or
# generator expressions.

# Per-directory OBJECT libraries (replacing the old flat GEM5_SOURCES list).
# Each src/ subdirectory that registers sources gets its own OBJECT target.
define_property(GLOBAL PROPERTY GEM5_OBJECT_LIBS
    BRIEF_DOCS "OBJECT library target names"
    FULL_DOCS "Accumulated list of per-directory OBJECT library targets for gem5")
set_property(GLOBAL PROPERTY GEM5_OBJECT_LIBS "")

define_property(GLOBAL PROPERTY GEM5_GENERATED_SOURCES
    BRIEF_DOCS "Generated C++ sources for the gem5 library"
    FULL_DOCS "Accumulated list of generated C++ source files (debug flags, etc.)")
set_property(GLOBAL PROPERTY GEM5_GENERATED_SOURCES "")

define_property(GLOBAL PROPERTY GEM5_PYSOURCES
    BRIEF_DOCS "Python source embedding .cc files"
    FULL_DOCS "Accumulated list of marshal-generated .cc files for Python embedding")
set_property(GLOBAL PROPERTY GEM5_PYSOURCES "")

# Parallel lists for deferred PySource custom command creation.
# Custom commands are created centrally at the top level (not in subdirectories)
# to avoid CMake/Ninja cross-directory custom command stub issues.
define_property(GLOBAL PROPERTY GEM5_PYSOURCE_INPUTS
    BRIEF_DOCS "Input .py files for PySource generation"
    FULL_DOCS "Parallel list with GEM5_PYSOURCES: input .py file paths")
set_property(GLOBAL PROPERTY GEM5_PYSOURCE_INPUTS "")

define_property(GLOBAL PROPERTY GEM5_PYSOURCE_MODPATHS
    BRIEF_DOCS "Module paths for PySource generation"
    FULL_DOCS "Parallel list with GEM5_PYSOURCES: Python module paths")
set_property(GLOBAL PROPERTY GEM5_PYSOURCE_MODPATHS "")

define_property(GLOBAL PROPERTY GEM5_PYSOURCE_ABSPATHS
    BRIEF_DOCS "Absolute paths for PySource generation"
    FULL_DOCS "Parallel list with GEM5_PYSOURCES: filesystem-style import paths")
set_property(GLOBAL PROPERTY GEM5_PYSOURCE_ABSPATHS "")

define_property(GLOBAL PROPERTY GEM5_DEBUG_FLAG_HEADERS
    BRIEF_DOCS "Generated debug flag .hh files"
    FULL_DOCS "Headers for debug flags, needed as dependencies")
set_property(GLOBAL PROPERTY GEM5_DEBUG_FLAG_HEADERS "")

# Phase 1 codegen targets: debug flags, ISA parser, blobs.
# These do NOT depend on gem5py_m5.
define_property(GLOBAL PROPERTY GEM5_CODEGEN_TARGETS_PHASE1
    BRIEF_DOCS "Codegen targets that do not depend on gem5py_m5"
    FULL_DOCS "Custom target names for debug flags, ISA parser, etc.")
set_property(GLOBAL PROPERTY GEM5_CODEGEN_TARGETS_PHASE1 "")

# Phase 2 codegen: deferred SimObject param/enum custom command data.
# Same pattern as PySource: custom commands created centrally at top level.
define_property(GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_NAMES
    BRIEF_DOCS "SimObject param names for deferred codegen"
    FULL_DOCS "Parallel list: SimObject class names for param struct generation")
set_property(GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_NAMES "")

define_property(GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_MODPATHS
    BRIEF_DOCS "SimObject param modpaths"
    FULL_DOCS "Parallel list: Python module paths for SimObject params")
set_property(GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_MODPATHS "")

define_property(GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_PYFILES
    BRIEF_DOCS "SimObject param source .py files"
    FULL_DOCS "Parallel list: .py source files for SimObject params")
set_property(GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_PYFILES "")

define_property(GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_NAMES
    BRIEF_DOCS "Enum names for deferred codegen"
    FULL_DOCS "Parallel list: enum names for enum header/source generation")
set_property(GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_NAMES "")

define_property(GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_MODPATHS
    BRIEF_DOCS "Enum modpaths"
    FULL_DOCS "Parallel list: Python module paths for enum generation")
set_property(GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_MODPATHS "")

define_property(GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_PYFILES
    BRIEF_DOCS "Enum source .py files"
    FULL_DOCS "Parallel list: .py source files for enum generation")
set_property(GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_PYFILES "")

# Phase 2 codegen (SimObject params/enums) depends on gem5py_m5.
# Dependency is tracked implicitly: each add_custom_command for param/enum
# .cc files lists gem5py_m5 in DEPENDS, so CMake resolves the ordering.
# No explicit custom-target wrappers are created for phase 2 outputs.

# Deferred protobuf custom command data (same cross-directory stub workaround).
define_property(GLOBAL PROPERTY GEM5_PROTO_FILES
    BRIEF_DOCS "Protobuf .proto source files"
    FULL_DOCS "Absolute paths to .proto files for deferred protoc invocation")
set_property(GLOBAL PROPERTY GEM5_PROTO_FILES "")

define_property(GLOBAL PROPERTY GEM5_PROTO_SUPPRESS_FLAGS
    BRIEF_DOCS "Compile flags for generated protobuf code"
    FULL_DOCS "Warning suppression flags applied to generated .pb.cc files")
set_property(GLOBAL PROPERTY GEM5_PROTO_SUPPRESS_FLAGS "")

define_property(GLOBAL PROPERTY GEM5_TEST_CC_SOURCES
    BRIEF_DOCS "GTest .test.cc source file paths"
    FULL_DOCS "Accumulated list of absolute paths to .test.cc unit test files")
set_property(GLOBAL PROPERTY GEM5_TEST_CC_SOURCES "")

# Tracks all .test.cc registrations regardless of CONDITION, so that the
# safety check in Gem5Targets.cmake can compare against the on-disk GLOB
# without false positives from condition-excluded tests.
define_property(GLOBAL PROPERTY GEM5_TEST_CC_ALL_SOURCES
    BRIEF_DOCS "All registered .test.cc paths (unconditional)"
    FULL_DOCS "Accumulated list of all .test.cc paths passed to gem5_add_test_source(), regardless of CONDITION")
set_property(GLOBAL PROPERTY GEM5_TEST_CC_ALL_SOURCES "")

define_property(GLOBAL PROPERTY GEM5_LINK_LIBRARIES
    BRIEF_DOCS "Additional libraries to link into the gem5 target"
    FULL_DOCS "Accumulated list of library targets/names for the gem5 executable")
set_property(GLOBAL PROPERTY GEM5_LINK_LIBRARIES "")

# Central generated output directory
set(GEM5_GEN_DIR "${CMAKE_BINARY_DIR}/generated" CACHE PATH
    "Directory for generated source files")
file(MAKE_DIRECTORY "${GEM5_GEN_DIR}")

# build_tools/ directory
set(GEM5_BUILD_TOOLS_DIR "${CMAKE_SOURCE_DIR}/build_tools" CACHE PATH
    "Directory containing build tool Python scripts")

# ---------------------------------------------------------------------------
# OBJECT library helpers
# ---------------------------------------------------------------------------
# Each src/ subdirectory that registers sources gets its own OBJECT library.
# This follows modern CMake practice (per ripopov's review) while preserving
# gem5's cross-directory header coupling through the gem5_deps INTERFACE.
#
# Naming: src/base -> gem5_obj_base, src/mem/cache -> gem5_obj_mem_cache.

# Derive a deterministic OBJECT target name from the calling directory.
function(_gem5_object_target_name out_var)
    file(RELATIVE_PATH _rel "${CMAKE_SOURCE_DIR}/src" "${CMAKE_CURRENT_SOURCE_DIR}")
    if(_rel STREQUAL "." OR _rel STREQUAL "")
        message(FATAL_ERROR
            "gem5_add_source() called from src/ root directory. "
            "Sources must be registered from a subdirectory of src/.")
    endif()
    string(REPLACE "/" "_" _name "${_rel}")
    set(${out_var} "gem5_obj_${_name}" PARENT_SCOPE)
endfunction()

# Get or create the OBJECT library for the calling directory.
# Created lazily on first source registration; subsequent calls reuse it.
function(_gem5_ensure_object_target out_var)
    _gem5_object_target_name(_tgt)
    if(NOT TARGET "${_tgt}")
        # OBJECT libraries compile sources but produce no archive; their
        # .o files are consumed via $<TARGET_OBJECTS:...> in Gem5Targets.cmake.
        add_library("${_tgt}" OBJECT)
        target_link_libraries("${_tgt}" PRIVATE gem5_deps)
        target_compile_options("${_tgt}" PRIVATE ${GEM5_WERROR_FLAGS})
        set_target_properties("${_tgt}" PROPERTIES POSITION_INDEPENDENT_CODE ON)
        set_property(GLOBAL APPEND PROPERTY GEM5_OBJECT_LIBS "${_tgt}")
    endif()
    set(${out_var} "${_tgt}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_source(<file> [CONDITION <cond>] [APPEND_FLAGS <flags...>])
#
# Register a C++ source file for compilation into the gem5 library.
# The source is added to the OBJECT library for the calling directory.
# If CONDITION is specified, the source is only added when the condition
# evaluates to TRUE.
# ---------------------------------------------------------------------------
function(gem5_add_source file)
    cmake_parse_arguments(ARG "" "CONDITION" "APPEND_FLAGS" ${ARGN})

    if(ARG_CONDITION)
        if(NOT ${ARG_CONDITION})
            return()
        endif()
    endif()

    if(NOT IS_ABSOLUTE "${file}")
        set(file "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
    endif()

    _gem5_ensure_object_target(_obj_tgt)
    target_sources("${_obj_tgt}" PRIVATE "${file}")

    if(ARG_APPEND_FLAGS)
        # Source file properties are directory-scoped in CMake.  Since the
        # OBJECT target is created in the same directory, the property is
        # correctly visible during compilation.
        set_source_files_properties("${file}" APPEND PROPERTY
            COMPILE_OPTIONS "${ARG_APPEND_FLAGS}")
    endif()
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_sources(<file1> [file2 ...] [CONDITION <cond>])
#
# Register multiple C++ source files at once.
# ---------------------------------------------------------------------------
function(gem5_add_sources)
    cmake_parse_arguments(ARG "" "CONDITION" "" ${ARGN})

    if(ARG_CONDITION)
        if(NOT ${ARG_CONDITION})
            return()
        endif()
    endif()

    _gem5_ensure_object_target(_obj_tgt)
    foreach(file ${ARG_UNPARSED_ARGUMENTS})
        if(NOT IS_ABSOLUTE "${file}")
            set(file "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
        endif()
        target_sources("${_obj_tgt}" PRIVATE "${file}")
    endforeach()
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_generated_source(<file>)
#
# Register a generated C++ source file (already an absolute path in the
# build tree).  Generated sources are compiled centrally via the
# gem5_generated_objs OBJECT target in Gem5Targets.cmake.
# ---------------------------------------------------------------------------
function(gem5_add_generated_source file)
    set_property(GLOBAL APPEND PROPERTY GEM5_GENERATED_SOURCES "${file}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_test_source(<file> [CONDITION <cond>])
#
# Register a .test.cc file for GTest unit testing.
# If CONDITION is specified, the test is only registered when the condition
# evaluates to TRUE.  Test executables are created centrally in
# Gem5Targets.cmake from the accumulated list.
# ---------------------------------------------------------------------------
function(gem5_add_test_source file)
    cmake_parse_arguments(ARG "" "CONDITION" "" ${ARGN})

    if(NOT IS_ABSOLUTE "${file}")
        set(file "${CMAKE_CURRENT_SOURCE_DIR}/${file}")
    endif()

    # Always track the registration regardless of condition, so the safety
    # check in Gem5Targets.cmake can detect genuinely unregistered files
    # without false positives from condition-excluded tests.
    set_property(GLOBAL APPEND PROPERTY GEM5_TEST_CC_ALL_SOURCES "${file}")

    if(ARG_CONDITION)
        if(NOT ${ARG_CONDITION})
            return()
        endif()
    endif()

    set_property(GLOBAL APPEND PROPERTY GEM5_TEST_CC_SOURCES "${file}")
endfunction()

# ---------------------------------------------------------------------------
# Helper: collect $<TARGET_OBJECTS:...> from all registered OBJECT libraries
# ---------------------------------------------------------------------------
function(gem5_get_all_object_sources out_var)
    get_property(_obj_libs GLOBAL PROPERTY GEM5_OBJECT_LIBS)
    set(_objs "")
    foreach(_lib ${_obj_libs})
        list(APPEND _objs "$<TARGET_OBJECTS:${_lib}>")
    endforeach()
    # Note: GEM5_PYSOURCES are NOT included here. They are compiled into a
    # separate OBJECT library (gem5_pysources) to avoid a dependency cycle:
    #   gem5_all -> SimObject codegen -> gem5py_m5 -> PySource .cc -> gem5_all
    # By separating them, gem5_all can safely depend on SimObject codegen.
    set(${out_var} ${_objs} PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# gem5_target_link_whole_archive(<target> <visibility> <library>)
#
# Link a static library with --whole-archive (Linux) or -force_load (macOS).
# Needed for libraries containing objects only referenced via global
# constructors (EmbeddedPython registrations, SimObject factories, GTest
# mock loggers, etc.).
# ---------------------------------------------------------------------------
function(gem5_target_link_whole_archive target visibility library)
    if(APPLE)
        target_link_libraries(${target} ${visibility} -Wl,-force_load ${library})
    else()
        target_link_libraries(${target} ${visibility}
            -Wl,--whole-archive ${library} -Wl,--no-whole-archive)
    endif()
endfunction()
