# src/mem/ruby/protocol/CMakeLists.txt
# Converted from src/mem/ruby/protocol/SConscript
# Conditional on CONF_RUBY (guarded by parent)
#
# Runs the SLICC compiler at configure time to generate protocol code.
# SLICC produces .cc, .hh, and .py files from .sm state machine descriptions.
# Generated .cc files are registered as gem5 sources; .py files are registered
# as SimObjects (which also registers them as PySources for embedding).

set(_slicc_output_dir "${GEM5_GEN_DIR}/mem/ruby/protocol")
set(_run_slicc "${CMAKE_SOURCE_DIR}/cmake/run_slicc.py")
set(_protocol_base "${CMAKE_CURRENT_SOURCE_DIR}")
set(_slicc_includes "mem/ruby/slicc_interface/RubySlicc_includes.hh")
set(_ruby_dir "${CMAKE_SOURCE_DIR}/src/mem/ruby")

file(MAKE_DIRECTORY "${_slicc_output_dir}")

# ---------------------------------------------------------------------------
# Generate forwarding headers for external types referenced by SLICC.
# SLICC-generated code includes external types as "mem/ruby/protocol/<Type>.hh"
# but the actual headers live in subdirectories (common/, structures/, etc.).
# The SCons build uses MakeInclude() to create redirect headers in the protocol
# output directory. We replicate that here at configure time.
# ---------------------------------------------------------------------------
function(_ruby_make_include _subpath)
    get_filename_component(_basename "${_subpath}" NAME)
    set(_target "${_slicc_output_dir}/${_basename}")
    # Use source-tree relative include so it works from the generated dir
    file(WRITE "${_target}" "#include \"mem/ruby/${_subpath}\"\n")
endfunction()

# slicc_interface types
_ruby_make_include("slicc_interface/AbstractCacheEntry.hh")
_ruby_make_include("slicc_interface/Message.hh")
_ruby_make_include("slicc_interface/RubyRequest.hh")

# common types
_ruby_make_include("common/Address.hh")
_ruby_make_include("common/BoolVec.hh")
_ruby_make_include("common/DataBlock.hh")
_ruby_make_include("common/ExpectedMap.hh")
_ruby_make_include("common/IntVec.hh")
_ruby_make_include("common/MachineID.hh")
_ruby_make_include("common/NetDest.hh")
_ruby_make_include("common/TriggerQueue.hh")
_ruby_make_include("common/Set.hh")
_ruby_make_include("common/WriteMask.hh")

# network types
_ruby_make_include("network/MessageBuffer.hh")

# structures types
_ruby_make_include("structures/CacheMemory.hh")
_ruby_make_include("structures/DirectoryMemory.hh")
_ruby_make_include("structures/PerfectCacheMemory.hh")
_ruby_make_include("structures/PersistentTable.hh")
_ruby_make_include("structures/RubyPrefetcher.hh")
_ruby_make_include("structures/RubyPrefetcherProxy.hh")
_ruby_make_include("structures/TBEStorage.hh")
_ruby_make_include("structures/TBETable.hh")
_ruby_make_include("structures/TimerTable.hh")
_ruby_make_include("structures/WireBuffer.hh")

# system types
_ruby_make_include("system/DMASequencer.hh")
_ruby_make_include("system/Sequencer.hh")
_ruby_make_include("system/GPUCoalescer.hh")
_ruby_make_include("system/HTMSequencer.hh")
_ruby_make_include("system/VIPERCoalescer.hh")
_ruby_make_include("system/VIPERSequencer.hh")

# CHI-specific types
if(CONF_RUBY_PROTOCOL_CHI)
    _ruby_make_include("structures/MN_TBEStorage.hh")
    _ruby_make_include("structures/MN_TBETable.hh")
endif()

# Gather enabled protocols from Kconfig CONF_RUBY_PROTOCOL_* variables
set(_enabled_protocols "")
get_cmake_property(_all_vars VARIABLES)
foreach(_var ${_all_vars})
    if(_var MATCHES "^CONF_RUBY_PROTOCOL_(.+)$" AND ${_var})
        list(APPEND _enabled_protocols "${CMAKE_MATCH_1}")
    endif()
endforeach()

if(NOT _enabled_protocols)
    message(WARNING "Ruby is enabled but no protocols are selected")
    return()
endif()

message(STATUS "SLICC protocols: ${_enabled_protocols}")

# Additional directories where SLICC protocol files may reside.
# The SCons build used PROTOCOL_DIRS for this; we replicate it here.
set(_slicc_extra_dirs
    "${CMAKE_SOURCE_DIR}/src/learning_gem5/part3"
)
# Include GEM5_EXTRAS paths so external projects can provide custom
# Ruby protocols via the EXTRAS mechanism.
if(GEM5_EXTRAS)
    foreach(_extras_dir IN LISTS GEM5_EXTRAS)
        get_filename_component(_extras_abs "${_extras_dir}" ABSOLUTE)
        list(APPEND _slicc_extra_dirs "${_extras_abs}")
    endforeach()
endif()

# Add configure-time dependency on .sm and .slicc files so CMake re-runs
# if protocol definitions change.
file(GLOB_RECURSE _slicc_dep_files
    "${CMAKE_CURRENT_SOURCE_DIR}/*.sm"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.slicc"
)
foreach(_extra_dir ${_slicc_extra_dirs})
    file(GLOB_RECURSE _extra_dep_files
        "${_extra_dir}/*.sm"
        "${_extra_dir}/*.slicc"
    )
    list(APPEND _slicc_dep_files ${_extra_dep_files})
endforeach()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_slicc_dep_files})

# Collect all SLICC Python module dependencies for the stamp target
file(GLOB_RECURSE _slicc_py_deps "${CMAKE_SOURCE_DIR}/src/mem/slicc/*.py")

# Track all generated .cc files across protocols (for deduplication)
set(_all_slicc_cc_files "")

foreach(_protocol ${_enabled_protocols})
    set(_slicc_file "${CMAKE_CURRENT_SOURCE_DIR}/${_protocol}.slicc")
    if(NOT EXISTS "${_slicc_file}")
        # Some protocols (e.g. CHI) live in a lowercase subdirectory
        string(TOLOWER "${_protocol}" _proto_lower)
        set(_slicc_file "${CMAKE_CURRENT_SOURCE_DIR}/${_proto_lower}/${_protocol}.slicc")
    endif()
    if(NOT EXISTS "${_slicc_file}")
        # Search additional SLICC protocol directories (e.g. learning_gem5/part3 for MSI)
        foreach(_extra_dir ${_slicc_extra_dirs})
            if(EXISTS "${_extra_dir}/${_protocol}.slicc")
                set(_slicc_file "${_extra_dir}/${_protocol}.slicc")
                break()
            endif()
        endforeach()
    endif()
    if(NOT EXISTS "${_slicc_file}")
        message(FATAL_ERROR "SLICC file not found for protocol ${_protocol}")
    endif()

    set(_manifest "${_slicc_output_dir}/.slicc_manifest_${_protocol}")

    # Run SLICC at configure time to generate code and file manifest
    message(STATUS "Running SLICC for protocol: ${_protocol}")
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" "${_run_slicc}"
                "${_slicc_file}" "${_slicc_output_dir}" "${CMAKE_SOURCE_DIR}"
                "--includes" "${_slicc_includes}"
                "--protocol-base" "${_protocol_base}"
                "--file-manifest" "${_manifest}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE _slicc_result
        OUTPUT_VARIABLE _slicc_output
        ERROR_VARIABLE _slicc_error
    )
    if(NOT _slicc_result EQUAL 0)
        message(FATAL_ERROR "SLICC failed for ${_protocol}:\n${_slicc_error}")
    endif()

    # Read the file manifest to discover generated files
    if(NOT EXISTS "${_manifest}")
        message(FATAL_ERROR "SLICC manifest not found: ${_manifest}")
    endif()
    file(STRINGS "${_manifest}" _generated_files)

    foreach(_file ${_generated_files})
        set(_abs_file "${_slicc_output_dir}/${_file}")

        if(_file MATCHES "\\.cc$")
            # Avoid registering the same .cc file twice (shared files across protocols)
            if(NOT "${_abs_file}" IN_LIST _all_slicc_cc_files)
                list(APPEND _all_slicc_cc_files "${_abs_file}")
                gem5_add_generated_source("${_abs_file}")

                # Clang warning suppression for SLICC-generated code
                if(GEM5_CLANG)
                    set_source_files_properties("${_abs_file}" PROPERTIES
                        COMPILE_OPTIONS "-Wno-parentheses")
                endif()
            endif()
        elseif(_file MATCHES "_Controller\\.py$")
            # Register as SimObject (also registers as PySource in m5.objects)
            get_filename_component(_simobj_name "${_file}" NAME_WE)
            gem5_add_simobject("${_abs_file}"
                SIM_OBJECTS "${_simobj_name}")
        endif()
    endforeach()

    # Create a stamp-based custom target for incremental builds.
    # If .sm files change between cmake reconfigures, the stamp triggers rebuild.
    set(_stamp "${_slicc_output_dir}/.slicc_stamp_${_protocol}")
    add_custom_command(
        OUTPUT "${_stamp}"
        COMMAND "${Python3_EXECUTABLE}" "${_run_slicc}"
                "${_slicc_file}" "${_slicc_output_dir}" "${CMAKE_SOURCE_DIR}"
                "--includes" "${_slicc_includes}"
                "--protocol-base" "${_protocol_base}"
        COMMAND "${CMAKE_COMMAND}" -E touch "${_stamp}"
        DEPENDS ${_slicc_dep_files} ${_slicc_py_deps}
                "${_run_slicc}"
                "${CMAKE_SOURCE_DIR}/build_tools/code_formatter.py"
                "${CMAKE_SOURCE_DIR}/build_tools/grammar.py"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Regenerating SLICC protocol: ${_protocol}"
        VERBATIM
    )
    add_custom_target(slicc_${_protocol} DEPENDS "${_stamp}")
    set_property(GLOBAL APPEND PROPERTY GEM5_SLICC_TARGETS slicc_${_protocol})
endforeach()

message(STATUS "SLICC generated ${_all_slicc_cc_files}" )

# CHI protocol extensions
if(CONF_RUBY_PROTOCOL_CHI)
    add_subdirectory(chi)
endif()
