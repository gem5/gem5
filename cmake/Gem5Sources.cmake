# One OBJECT library per source directory, created lazily on first
# gem5_add_source() call. Using OBJECT libraries (rather than one monolithic
# library) preserves per-directory granularity for link-order and avoids
# re-archiving the entire gem5 codebase on incremental rebuilds.
# The GLOBAL property survives function scopes and is consumed by
# Gem5Targets (PR 3) to assemble the final executable.
define_property(GLOBAL PROPERTY GEM5_OBJECT_LIBS
    BRIEF_DOCS "List of all per-directory gem5 OBJECT targets"
    FULL_DOCS  "Accumulated by gem5_add_source(); consumed by Gem5Targets in PR 3")

function(_gem5_ensure_object_target out_var)
    # Derive target name from directory path relative to src/, replacing
    # path separators with underscores: src/cpu/o3 -> gem5_obj_cpu_o3
    file(RELATIVE_PATH _rel
        "${PROJECT_SOURCE_DIR}/src"
        "${CMAKE_CURRENT_SOURCE_DIR}")
    if(_rel STREQUAL "")
        message(FATAL_ERROR
            "gem5_add_source() called from src/ directly. "
            "Source files must be registered from a subdirectory CMakeLists.txt.")
    endif()
    string(REPLACE "/" "_" _suffix "${_rel}")
    set(_target "gem5_obj_${_suffix}")

    if(NOT TARGET ${_target})
        add_library(${_target} OBJECT)
        target_link_libraries(${_target} PRIVATE gem5_deps)
        set_property(GLOBAL APPEND PROPERTY GEM5_OBJECT_LIBS ${_target})
    endif()

    set(${out_var} "${_target}" PARENT_SCOPE)
endfunction()

# gem5_add_source(<file> [CONDITION <genex>] [COMPILE_OPTIONS <flags...>])
#
# CONDITION must be a generator expression that evaluates to 0 or 1, e.g.:
#   CONDITION "$<BOOL:${USE_X86_ISA}>"
#   CONDITION "$<AND:$<BOOL:${RUBY}>,$<BOOL:${USE_X86_ISA}>>"
# Do NOT pass a plain variable — pass $<BOOL:${VAR}> so unset vars become 0.
function(gem5_add_source file)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "CONDITION" "COMPILE_OPTIONS")

    _gem5_ensure_object_target(_target)

    if(ARG_CONDITION)
        target_sources(${_target} PRIVATE "$<${ARG_CONDITION}:${file}>")
    else()
        target_sources(${_target} PRIVATE "${file}")
    endif()

    if(ARG_COMPILE_OPTIONS)
        set_source_files_properties("${file}" PROPERTIES
            COMPILE_OPTIONS "${ARG_COMPILE_OPTIONS}")
    endif()
endfunction()

# gem5_add_sources(<file...> [CONDITION <genex>])
# Calls gem5_add_source() for each file. Same CONDITION rules apply.
function(gem5_add_sources)
    cmake_parse_arguments(PARSE_ARGV 0 ARG "" "CONDITION" "")
    foreach(_file ${ARG_UNPARSED_ARGUMENTS})
        if(ARG_CONDITION)
            gem5_add_source("${_file}" CONDITION "${ARG_CONDITION}")
        else()
            gem5_add_source("${_file}")
        endif()
    endforeach()
endfunction()

function(gem5_get_all_object_sources out_var)
    get_property(_libs GLOBAL PROPERTY GEM5_OBJECT_LIBS)
    set(_objects "")
    foreach(_lib ${_libs})
        list(APPEND _objects "$<TARGET_OBJECTS:${_lib}>")
    endforeach()
    set(${out_var} "${_objects}" PARENT_SCOPE)
endfunction()
