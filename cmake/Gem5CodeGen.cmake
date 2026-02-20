# cmake/Gem5CodeGen.cmake
# Wrapper functions around add_custom_command() for gem5 code generators.
#
# Each function invokes a build_tools/*.py script to produce generated
# C++ sources/headers during the build phase.  The generated files are
# automatically registered via gem5_add_generated_source() so they end
# up in the final gem5 library.

# ---------------------------------------------------------------------------
# gem5_write_if_unchanged(<filepath> <content>)
#
# Write <content> to <filepath> only if the file does not already exist
# or its current content differs.  This preserves mtime on unchanged
# configure-time generated files, allowing ninja's restat optimization
# to skip unnecessary downstream recompilation.
# ---------------------------------------------------------------------------
function(gem5_write_if_unchanged filepath content)
    if(EXISTS "${filepath}")
        file(READ "${filepath}" _existing)
        if(_existing STREQUAL content)
            return()
        endif()
    endif()
    file(WRITE "${filepath}" "${content}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_debug_flag(<name> <description>
#     [FORMAT]
#     [COMPONENTS comp1 comp2 ...])
#
# Generate debug flag header + source using debugflaghh.py / debugflagcc.py.
#
# Simple flag:   gem5_add_debug_flag(Activity "Pipeline activity")
# Format flag:   gem5_add_debug_flag(FmtFlag "..." FORMAT)
# Compound flag: gem5_add_debug_flag(GDBAll "..." COMPONENTS GDBMisc GDBAcc ...)
# ---------------------------------------------------------------------------
function(gem5_add_debug_flag name description)
    cmake_parse_arguments(ARG "FORMAT" "" "COMPONENTS" ${ARGN})

    set(_hh "${GEM5_GEN_DIR}/debug/${name}.hh")
    set(_cc "${GEM5_GEN_DIR}/debug/${name}.cc")

    # Format flag argument: "True" or "False"
    if(ARG_FORMAT)
        set(_fmt "True")
    else()
        set(_fmt "False")
    endif()

    # Components: joined with ':' for the script, empty string if none
    if(ARG_COMPONENTS)
        string(REPLACE ";" ":" _components "${ARG_COMPONENTS}")
    else()
        set(_components "")
    endif()

    # Collect component header dependencies
    set(_component_deps "")
    foreach(_comp ${ARG_COMPONENTS})
        list(APPEND _component_deps "${GEM5_GEN_DIR}/debug/${_comp}.hh")
    endforeach()

    # Generate .hh
    add_custom_command(
        OUTPUT "${_hh}"
        COMMAND "${Python3_EXECUTABLE}"
                "${GEM5_BUILD_TOOLS_DIR}/debugflaghh.py"
                "${_hh}" "${name}" "${description}" "${_fmt}" "${_components}"
        DEPENDS "${GEM5_BUILD_TOOLS_DIR}/debugflaghh.py"
                "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                ${_component_deps}
        WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
        COMMENT "Generating debug flag header: debug/${name}.hh"
        VERBATIM
    )

    # Generate .cc
    add_custom_command(
        OUTPUT "${_cc}"
        COMMAND "${Python3_EXECUTABLE}"
                "${GEM5_BUILD_TOOLS_DIR}/debugflagcc.py"
                "${_cc}" "${name}"
        DEPENDS "${GEM5_BUILD_TOOLS_DIR}/debugflagcc.py"
                "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                "${_hh}"
        WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
        COMMENT "Generating debug flag source: debug/${name}.cc"
        VERBATIM
    )

    # Register generated files
    gem5_add_generated_source("${_cc}")
    set_property(GLOBAL APPEND PROPERTY GEM5_DEBUG_FLAG_HEADERS "${_hh}")

    # Create a custom target in this directory scope so Ninja can resolve
    # the dependency (custom commands are directory-scoped in Ninja).
    # Debug flags use Python3 directly (not gem5py_m5), so they're Phase 1.
    add_custom_target(gen_debug_${name} DEPENDS "${_hh}" "${_cc}")
    set_property(GLOBAL APPEND PROPERTY GEM5_CODEGEN_TARGETS_PHASE1 "gen_debug_${name}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_blob(<symbol> <input_file> <output_cc> [OUTPUT_HH <hh_path>])
#
# Embed a binary blob into C++ source and header files using generate_blob.py.
#
# Generates:
#   <output_cc>  - C++ source with the blob data array and length
#   <hh_path>    - C++ header declaring the blob (defaults to same dir as cc)
#
# The generated header is at:
#   <dir_of_output_cc>/<symbol>.hh  (default)
#   or <hh_path> if OUTPUT_HH is specified
# ---------------------------------------------------------------------------
function(gem5_add_blob symbol input_file output_cc)
    cmake_parse_arguments(ARG "" "OUTPUT_HH" "" ${ARGN})

    if(NOT IS_ABSOLUTE "${input_file}")
        set(input_file "${CMAKE_CURRENT_SOURCE_DIR}/${input_file}")
    endif()

    # Compute header output path
    get_filename_component(_cc_dir "${output_cc}" DIRECTORY)
    if(ARG_OUTPUT_HH)
        set(_output_hh "${ARG_OUTPUT_HH}")
    else()
        set(_output_hh "${_cc_dir}/${symbol}.hh")
    endif()

    # Compute include path relative to the generated dir
    file(RELATIVE_PATH _include_path "${GEM5_GEN_DIR}" "${_output_hh}")

    add_custom_command(
        OUTPUT "${output_cc}" "${_output_hh}"
        COMMAND "${Python3_EXECUTABLE}"
                "${GEM5_BUILD_TOOLS_DIR}/generate_blob.py"
                "${symbol}" "${input_file}"
                "${output_cc}" "${_output_hh}"
                "${_include_path}"
        DEPENDS "${GEM5_BUILD_TOOLS_DIR}/generate_blob.py"
                "${GEM5_BUILD_TOOLS_DIR}/blob.py"
                "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                "${input_file}"
        WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
        COMMENT "Generating blob: ${symbol}"
        VERBATIM
    )

    gem5_add_generated_source("${output_cc}")

    # Create a custom target so Ninja can resolve cross-directory dependencies
    # (custom commands are directory-scoped in Ninja). Blob generation uses
    # Python3 directly (not gem5py_m5), so it belongs in Phase 1.
    add_custom_target(gen_blob_${symbol} DEPENDS "${output_cc}" "${_output_hh}")
    set_property(GLOBAL APPEND PROPERTY GEM5_CODEGEN_TARGETS_PHASE1 "gen_blob_${symbol}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_pysource(<modpath> <pyfile> [ABSPATH <abspath>])
#
# Embed a Python source file into C++ using marshal.py.
# The generated .cc registers the bytecode with the EmbeddedPython system.
#
# modpath: Python package path (e.g., "m5.objects"), or "" for root
# pyfile:  Path to the .py file (relative to CMAKE_CURRENT_SOURCE_DIR)
# abspath: Optional absolute import path override (filesystem-style,
#          e.g., "m5/objects/SimObject.py")
# ---------------------------------------------------------------------------
function(gem5_add_pysource package pyfile)
    cmake_parse_arguments(ARG "" "ABSPATH" "" ${ARGN})

    if(NOT IS_ABSOLUTE "${pyfile}")
        set(pyfile "${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}")
    endif()

    get_filename_component(_basename "${pyfile}" NAME)
    get_filename_component(_modname "${pyfile}" NAME_WE)

    # Compute full Python module path from package + filename, matching SCons:
    #   PySource('gem5', 'gem5/__init__.py')  -> modpath = "gem5"
    #   PySource('gem5', 'gem5/runtime.py')   -> modpath = "gem5.runtime"
    if(_modname STREQUAL "__init__")
        # __init__.py: modpath is just the package name
        set(_modpath "${package}")
    elseif(package STREQUAL "")
        # Root-level non-init file
        set(_modpath "${_modname}")
    else()
        # Non-init file in a package: package.modname
        set(_modpath "${package}.${_modname}")
    endif()

    # Convert dotted package to directory path for output file location
    string(REPLACE "." "/" _modpath_dir "${package}")

    if(_modpath_dir)
        set(_output "${GEM5_GEN_DIR}/python/${_modpath_dir}/${_basename}.cc")
    else()
        set(_output "${GEM5_GEN_DIR}/python/${_basename}.cc")
    endif()

    # Compute abspath: filesystem-style path (e.g., "m5/objects/__init__.py")
    if(ARG_ABSPATH)
        set(_abspath "${ARG_ABSPATH}")
    elseif(_modpath_dir)
        set(_abspath "${_modpath_dir}/${_basename}")
    else()
        set(_abspath "${_basename}")
    endif()

    # Store metadata for deferred custom command creation.
    # Custom commands are created centrally via gem5_create_pysource_commands()
    # at the top level to avoid CMake/Ninja cross-directory stub issues.
    # Use sentinel for empty strings (CMake lists silently drop empty elements).
    if(_modpath STREQUAL "")
        set(_modpath_store "_GEM5_EMPTY_")
    else()
        set(_modpath_store "${_modpath}")
    endif()

    set_property(GLOBAL APPEND PROPERTY GEM5_PYSOURCES "${_output}")
    set_property(GLOBAL APPEND PROPERTY GEM5_PYSOURCE_INPUTS "${pyfile}")
    set_property(GLOBAL APPEND PROPERTY GEM5_PYSOURCE_MODPATHS "${_modpath_store}")
    set_property(GLOBAL APPEND PROPERTY GEM5_PYSOURCE_ABSPATHS "${_abspath}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_simobject(<pyfile>
#     [SIM_OBJECTS obj1 obj2 ...]
#     [ENUMS enum1 enum2 ...])
#
# Generate SimObject param structures, pybind11 bindings, and enums.
#
# This function requires the gem5py_m5 helper executable, which has all
# gem5 Python modules (m5.SimObject, etc.) embedded.
# The GEM5PY_M5 variable must point to the gem5py_m5 binary.
#
# For each SIM_OBJECT, generates:
#   - ${GEM5_GEN_DIR}/params/${obj}.hh
#   - ${GEM5_GEN_DIR}/python/_m5/param_${obj}.cc
#
# For each ENUM, generates:
#   - ${GEM5_GEN_DIR}/enums/${enum}.hh
#   - ${GEM5_GEN_DIR}/enums/${enum}.cc
# ---------------------------------------------------------------------------
function(gem5_add_simobject pyfile)
    cmake_parse_arguments(ARG "" "" "SIM_OBJECTS;ENUMS" ${ARGN})

    if(NOT IS_ABSOLUTE "${pyfile}")
        set(pyfile "${CMAKE_CURRENT_SOURCE_DIR}/${pyfile}")
    endif()

    # SimObject .py files are always in the m5.objects package (matching SCons).
    # In SCons, SimObject extends PySource and calls PySource.__init__('m5.objects', source).
    # We must also register as a PySource so gem5py_m5 can import the module.
    get_filename_component(_basename "${pyfile}" NAME_WE)
    set(_modpath "m5.objects.${_basename}")

    # Register as PySource in m5.objects package (SimObject extends PySource in SCons)
    gem5_add_pysource("m5.objects" "${pyfile}")

    # Store metadata for deferred custom command creation.
    # Custom commands are created centrally via gem5_create_simobject_commands()
    # at the top level to avoid CMake/Ninja cross-directory stub issues.
    foreach(_obj ${ARG_SIM_OBJECTS})
        set_property(GLOBAL APPEND PROPERTY GEM5_SIMOBJ_PARAM_NAMES "${_obj}")
        set_property(GLOBAL APPEND PROPERTY GEM5_SIMOBJ_PARAM_MODPATHS "${_modpath}")
        set_property(GLOBAL APPEND PROPERTY GEM5_SIMOBJ_PARAM_PYFILES "${pyfile}")

        # Register generated .cc as source (will be created by deferred commands)
        gem5_add_generated_source("${GEM5_GEN_DIR}/python/_m5/param_${_obj}.cc")
    endforeach()

    foreach(_enum ${ARG_ENUMS})
        set_property(GLOBAL APPEND PROPERTY GEM5_SIMOBJ_ENUM_NAMES "${_enum}")
        set_property(GLOBAL APPEND PROPERTY GEM5_SIMOBJ_ENUM_MODPATHS "${_modpath}")
        set_property(GLOBAL APPEND PROPERTY GEM5_SIMOBJ_ENUM_PYFILES "${pyfile}")

        # Register generated .cc as source
        gem5_add_generated_source("${GEM5_GEN_DIR}/enums/${_enum}.cc")
    endforeach()
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_cxx_config(<sim_object>)
#
# Generate C++ config header/source for --without-python builds.
# Uses cxx_config_hh.py and cxx_config_cc.py.
# ---------------------------------------------------------------------------
function(gem5_add_cxx_config sim_object modpath)
    set(_hh "${GEM5_GEN_DIR}/cxx_config/${sim_object}.hh")
    set(_cc "${GEM5_GEN_DIR}/cxx_config/${sim_object}.cc")

    # cxx_config_hh.py args: modpath cxx_config_hh
    add_custom_command(
        OUTPUT "${_hh}"
        COMMAND "${GEM5PY_M5}" "${GEM5_BUILD_TOOLS_DIR}/cxx_config_hh.py"
                "${modpath}" "${_hh}"
        DEPENDS gem5py_m5
                "${GEM5_BUILD_TOOLS_DIR}/cxx_config_hh.py"
                "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
        WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
        COMMENT "Generating cxx_config header: ${sim_object}.hh"
        VERBATIM
    )

    # cxx_config_cc.py args: modpath cxx_config_cc
    add_custom_command(
        OUTPUT "${_cc}"
        COMMAND "${GEM5PY_M5}" "${GEM5_BUILD_TOOLS_DIR}/cxx_config_cc.py"
                "${modpath}" "${_cc}"
        DEPENDS gem5py_m5 "${_hh}"
                "${GEM5_BUILD_TOOLS_DIR}/cxx_config_cc.py"
                "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
        WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
        COMMENT "Generating cxx_config source: ${sim_object}.cc"
        VERBATIM
    )

    gem5_add_generated_source("${_cc}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_isa_parser(<isa> <isa_file> <output_dir>
#     [DECODER_SPLITS <n>]
#     [EXEC_SPLITS <n>])
#
# Invoke the ISA parser to generate decoder .cc/.hh files.
# ---------------------------------------------------------------------------
function(gem5_isa_parser isa isa_file output_dir)
    cmake_parse_arguments(ARG "" "DECODER_SPLITS;EXEC_SPLITS" "" ${ARGN})

    if(NOT ARG_DECODER_SPLITS)
        set(ARG_DECODER_SPLITS 1)
    endif()
    if(NOT ARG_EXEC_SPLITS)
        set(ARG_EXEC_SPLITS 1)
    endif()

    set(_isa_parser "${CMAKE_SOURCE_DIR}/cmake/run_isa_parser.py")

    # Compute output files -- must match what isa_parser.py actually generates.
    # See src/arch/SConscript ISADesc() for the canonical list.
    set(_outputs
        "${output_dir}/decoder.hh"
        "${output_dir}/decoder.cc"
        # Include files generated by the ISA parser
        "${output_dir}/decoder-g.cc.inc"
        "${output_dir}/decoder-ns.cc.inc"
        "${output_dir}/decode-method.cc.inc"
        "${output_dir}/decoder-g.hh.inc"
        "${output_dir}/decoder-ns.hh.inc"
        "${output_dir}/exec-g.cc.inc"
        "${output_dir}/exec-ns.cc.inc"
    )

    # inst-constrs splits: no suffix when splits==1
    if(ARG_DECODER_SPLITS GREATER 1)
        foreach(_i RANGE 1 ${ARG_DECODER_SPLITS})
            list(APPEND _outputs "${output_dir}/inst-constrs-${_i}.cc")
        endforeach()
    else()
        list(APPEND _outputs "${output_dir}/inst-constrs.cc")
    endif()

    # generic_cpu_exec splits: no suffix when splits==1
    if(ARG_EXEC_SPLITS GREATER 1)
        foreach(_i RANGE 1 ${ARG_EXEC_SPLITS})
            list(APPEND _outputs "${output_dir}/generic_cpu_exec_${_i}.cc")
        endforeach()
    else()
        list(APPEND _outputs "${output_dir}/generic_cpu_exec.cc")
    endif()

    # Collect .isa file dependencies
    get_filename_component(_isa_dir "${isa_file}" DIRECTORY)
    file(GLOB_RECURSE _isa_deps "${_isa_dir}/*.isa")

    add_custom_command(
        OUTPUT ${_outputs}
        COMMAND "${Python3_EXECUTABLE}" "${_isa_parser}" "${isa_file}" "${output_dir}"
        DEPENDS ${_isa_deps}
                "${_isa_parser}"
                "${CMAKE_SOURCE_DIR}/src/arch/isa_parser/isa_parser.py"
                "${CMAKE_SOURCE_DIR}/src/arch/isa_parser/operand_list.py"
                "${CMAKE_SOURCE_DIR}/src/arch/isa_parser/operand_types.py"
                "${CMAKE_SOURCE_DIR}/src/arch/isa_parser/util.py"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Generating ISA decoder for ${isa}"
        VERBATIM
    )

    foreach(_out ${_outputs})
        if(_out MATCHES "\\.cc$")
            gem5_add_generated_source("${_out}")
        endif()
    endforeach()

    # Custom target for ISA parser outputs (needed for cross-directory deps).
    # ISA parser uses Python3 directly (not gem5py_m5), so it's Phase 1.
    add_custom_target(gen_isa_${isa} DEPENDS ${_outputs})
    set_property(GLOBAL APPEND PROPERTY GEM5_CODEGEN_TARGETS_PHASE1 "gen_isa_${isa}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_slicc_protocol(<protocol> <slicc_file> <output_dir>
#     [PROTOCOL_BASE <dir>]
#     [INCLUDES inc1 inc2 ...])
#
# Invoke the SLICC parser to generate Ruby protocol code.
#
# SLICC generates .cc, .hh, and .py files dynamically (the exact set of
# output files depends on the protocol). Because the output file list is
# not known until SLICC runs, we use a stamp-file + custom-target approach:
#   - A custom command runs SLICC and touches a stamp file
#   - A custom target (slicc_<protocol>) depends on the stamp
#   - Other targets that use SLICC outputs depend on slicc_<protocol>
#
# INCLUDES: C++ #include paths added to generated code (e.g.,
#   "mem/ruby/slicc_interface/RubySlicc_includes.hh")
# PROTOCOL_BASE: Base directory for protocol files (default:
#   src/mem/ruby/protocol)
# ---------------------------------------------------------------------------
function(gem5_slicc_protocol protocol slicc_file output_dir)
    cmake_parse_arguments(ARG "" "PROTOCOL_BASE" "INCLUDES" ${ARGN})

    set(_run_slicc "${CMAKE_SOURCE_DIR}/cmake/run_slicc.py")
    set(_stamp "${output_dir}/.slicc_stamp")
    set(_manifest "${output_dir}/.slicc_manifest")

    # Build includes argument (comma-separated)
    set(_includes_arg "")
    if(ARG_INCLUDES)
        string(REPLACE ";" "," _includes_csv "${ARG_INCLUDES}")
        set(_includes_arg "--includes" "${_includes_csv}")
    endif()

    # Protocol base argument
    set(_protocol_base_arg "")
    if(ARG_PROTOCOL_BASE)
        set(_protocol_base_arg "--protocol-base" "${ARG_PROTOCOL_BASE}")
    endif()

    # Get all .sm and .slicc input dependencies
    get_filename_component(_slicc_dir "${slicc_file}" DIRECTORY)
    file(GLOB_RECURSE _slicc_deps
        "${_slicc_dir}/*.sm"
        "${_slicc_dir}/*.slicc"
    )

    # Get SLICC Python module dependencies
    file(GLOB_RECURSE _slicc_py_deps
        "${CMAKE_SOURCE_DIR}/src/mem/slicc/*.py"
    )

    # Build-time: run SLICC to generate all output files + file manifest
    add_custom_command(
        OUTPUT "${_stamp}"
        COMMAND "${Python3_EXECUTABLE}" "${_run_slicc}"
                "${slicc_file}" "${output_dir}" "${CMAKE_SOURCE_DIR}"
                ${_includes_arg} ${_protocol_base_arg}
                "--file-manifest" "${_manifest}"
        COMMAND "${CMAKE_COMMAND}" -E touch "${_stamp}"
        DEPENDS ${_slicc_deps} ${_slicc_py_deps}
                "${_run_slicc}"
                "${CMAKE_SOURCE_DIR}/build_tools/code_formatter.py"
                "${CMAKE_SOURCE_DIR}/build_tools/file_utils.py"
                "${CMAKE_SOURCE_DIR}/build_tools/grammar.py"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Generating SLICC protocol: ${protocol}"
        VERBATIM
    )

    # Create a custom target so other targets can depend on SLICC generation
    add_custom_target(slicc_${protocol} DEPENDS "${_stamp}")
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_switching_header(<header_path> <target_header>)
#
# Generate a switching header that redirects to an ISA-specific header.
# For example: arch/decoder.hh -> arch/x86/decoder.hh
# ---------------------------------------------------------------------------
function(gem5_add_switching_header header_path target_header)
    set(_output "${GEM5_GEN_DIR}/${header_path}")
    get_filename_component(_output_dir "${_output}" DIRECTORY)
    file(MAKE_DIRECTORY "${_output_dir}")

    gem5_write_if_unchanged("${_output}" "#include \"${target_header}\"\n")
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_config_header(<name> <value>)
#
# Generate a config/<name>.hh header file from a Kconfig variable.
# ---------------------------------------------------------------------------
function(gem5_add_config_header name value)
    set(_output "${GEM5_GEN_DIR}/config/${name}.hh")
    set(_guard "__CONFIG_${name}_HH__")

    string(TOUPPER "${_guard}" _guard)
    string(REPLACE "/" "_" _guard "${_guard}")

    if(value STREQUAL "TRUE" OR value STREQUAL "ON" OR value STREQUAL "y")
        set(_value_line "static constexpr bool value = true;")
    elseif(value STREQUAL "FALSE" OR value STREQUAL "OFF" OR value STREQUAL "n")
        set(_value_line "static constexpr bool value = false;")
    elseif(value MATCHES "^[0-9]+$")
        set(_value_line "static constexpr auto value = ${value};")
    else()
        set(_value_line "static constexpr auto value = \"${value}\";")
    endif()

    gem5_write_if_unchanged("${_output}"
        "#ifndef ${_guard}\n#define ${_guard}\n\nnamespace gem5 {\nnamespace config {\n${_value_line}\n} // namespace config\n} // namespace gem5\n\n#endif // ${_guard}\n"
    )
endfunction()

# ---------------------------------------------------------------------------
# gem5_add_info_py()
#
# Generate the python/m5/info.py file with build information.
# Uses infopy.py.
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# gem5_generate_defines_py()
#
# Generate m5/defines.py with the buildEnv dictionary.
# This Python module is imported by SimObject .py files to check build
# configuration (e.g., `from m5.defines import buildEnv`).
#
# Must be called after Kconfig processing (CONF_* variables are available).
# Registers the file as a PySource in the m5 package.
# ---------------------------------------------------------------------------
function(gem5_generate_defines_py)
    set(_output "${GEM5_GEN_DIR}/python/m5/defines.py")
    file(MAKE_DIRECTORY "${GEM5_GEN_DIR}/python/m5")

    # Collect all CONF_* variables and build the Python dict
    get_cmake_property(_all_vars VARIABLES)
    set(_dict_entries "")
    foreach(_var ${_all_vars})
        if(_var MATCHES "^CONF_(.+)")
            set(_key "${CMAKE_MATCH_1}")
            set(_val "${${_var}}")
            # Convert CMake values to Python values
            if(_val STREQUAL "TRUE" OR _val STREQUAL "ON")
                string(APPEND _dict_entries "    '${_key}': True,\n")
            elseif(_val STREQUAL "FALSE" OR _val STREQUAL "OFF")
                string(APPEND _dict_entries "    '${_key}': False,\n")
            elseif(_val MATCHES "^[0-9]+$")
                string(APPEND _dict_entries "    '${_key}': ${_val},\n")
            else()
                string(APPEND _dict_entries "    '${_key}': '${_val}',\n")
            endif()
        endif()
    endforeach()

    gem5_write_if_unchanged("${_output}"
        "# Auto-generated by CMake - do not edit\nbuildEnv = {\n${_dict_entries}}\n"
    )

    # Register as PySource in m5 package
    gem5_add_pysource("m5" "${_output}" ABSPATH "m5/defines.py")
endfunction()

# ---------------------------------------------------------------------------
# gem5_generate_info_py()
#
# Generate m5/info.py containing COPYING, LICENSE, README.md text.
# Uses build_tools/infopy.py at configure time.
# Registers the file as a PySource in the m5 package.
# ---------------------------------------------------------------------------
function(gem5_generate_info_py)
    set(_output "${GEM5_GEN_DIR}/python/m5/info.py")
    file(MAKE_DIRECTORY "${GEM5_GEN_DIR}/python/m5")

    execute_process(
        COMMAND "${Python3_EXECUTABLE}"
                "${GEM5_BUILD_TOOLS_DIR}/infopy.py"
                "${_output}"
                "${CMAKE_SOURCE_DIR}/COPYING"
                "${CMAKE_SOURCE_DIR}/LICENSE"
                "${CMAKE_SOURCE_DIR}/README.md"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        RESULT_VARIABLE _result
    )
    if(NOT _result EQUAL 0)
        message(FATAL_ERROR "Failed to generate m5/info.py")
    endif()

    gem5_add_pysource("m5" "${_output}" ABSPATH "m5/info.py")
endfunction()

# ---------------------------------------------------------------------------
# gem5_create_pysource_commands()
#
# Create all PySource custom commands in the current directory scope.
# Must be called from the top-level CMakeLists.txt AFTER all
# gem5_add_pysource() registrations and BEFORE gem5_build_gem5py_m5().
#
# This deferred approach is necessary because CMake's Ninja generator
# requires custom commands to be in the same directory scope as the targets
# that consume their outputs. Since gem5py_m5 and gem5_pysources are both
# defined at the top level, all PySource custom commands must also be
# created at the top level.
# ---------------------------------------------------------------------------
function(gem5_create_pysource_commands)
    get_property(_outputs GLOBAL PROPERTY GEM5_PYSOURCES)
    get_property(_inputs GLOBAL PROPERTY GEM5_PYSOURCE_INPUTS)
    get_property(_modpaths GLOBAL PROPERTY GEM5_PYSOURCE_MODPATHS)
    get_property(_abspaths GLOBAL PROPERTY GEM5_PYSOURCE_ABSPATHS)

    list(LENGTH _outputs _count)
    if(_count EQUAL 0)
        return()
    endif()

    math(EXPR _last "${_count} - 1")

    foreach(_i RANGE 0 ${_last})
        list(GET _outputs ${_i} _output)
        list(GET _inputs ${_i} _pyfile)
        list(GET _modpaths ${_i} _modpath)
        list(GET _abspaths ${_i} _abspath)

        # Restore empty strings from sentinel
        if(_modpath STREQUAL "_GEM5_EMPTY_")
            set(_modpath "")
        endif()

        get_filename_component(_basename "${_pyfile}" NAME)

        add_custom_command(
            OUTPUT "${_output}"
            COMMAND "${Python3_EXECUTABLE}"
                    "${GEM5_BUILD_TOOLS_DIR}/marshal.py"
                    "${_output}" "${_pyfile}" "${_modpath}" "${_abspath}"
            DEPENDS "${GEM5_BUILD_TOOLS_DIR}/marshal.py"
                    "${GEM5_BUILD_TOOLS_DIR}/blob.py"
                    "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                    "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                    "${_pyfile}"
            WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
            COMMENT "Embedding Python: ${_modpath}/${_basename}"
            VERBATIM
        )
    endforeach()

    message(STATUS "Created ${_count} PySource custom commands")
endfunction()

# ---------------------------------------------------------------------------
# gem5_create_simobject_commands()
#
# Create all SimObject param/enum custom commands in the current directory
# scope. Must be called from the top-level CMakeLists.txt AFTER all
# gem5_add_simobject() registrations and AFTER gem5_build_gem5py_m5().
#
# Same rationale as gem5_create_pysource_commands(): Ninja requires custom
# commands to be in the same directory scope as the consuming targets.
# ---------------------------------------------------------------------------
function(gem5_create_simobject_commands)
    # Determine USE_PYTHON argument for .cc generators
    if(GEM5_WITHOUT_PYTHON)
        set(_use_python "False")
    else()
        set(_use_python "True")
    endif()

    # --- SimObject params ---
    get_property(_names GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_NAMES)
    get_property(_modpaths GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_MODPATHS)
    get_property(_pyfiles GLOBAL PROPERTY GEM5_SIMOBJ_PARAM_PYFILES)

    list(LENGTH _names _count)
    if(_count GREATER 0)
        math(EXPR _last "${_count} - 1")
        foreach(_i RANGE 0 ${_last})
            list(GET _names ${_i} _obj)
            list(GET _modpaths ${_i} _modpath)
            list(GET _pyfiles ${_i} _pyfile)

            set(_param_hh "${GEM5_GEN_DIR}/params/${_obj}.hh")
            set(_param_cc "${GEM5_GEN_DIR}/python/_m5/param_${_obj}.cc")

            add_custom_command(
                OUTPUT "${_param_hh}"
                COMMAND "${GEM5PY_M5}" "${GEM5_BUILD_TOOLS_DIR}/sim_object_param_struct_hh.py"
                        "${_modpath}" "${_param_hh}"
                DEPENDS gem5py_m5 "${_pyfile}"
                        "${GEM5_BUILD_TOOLS_DIR}/sim_object_param_struct_hh.py"
                        "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                        "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
                COMMENT "Generating params header: params/${_obj}.hh"
                VERBATIM
            )

            add_custom_command(
                OUTPUT "${_param_cc}"
                COMMAND "${GEM5PY_M5}" "${GEM5_BUILD_TOOLS_DIR}/sim_object_param_struct_cc.py"
                        "${_modpath}" "${_param_cc}" "${_use_python}"
                DEPENDS gem5py_m5 "${_pyfile}" "${_param_hh}"
                        "${GEM5_BUILD_TOOLS_DIR}/sim_object_param_struct_cc.py"
                        "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                        "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
                COMMENT "Generating param bindings: python/_m5/param_${_obj}.cc"
                VERBATIM
            )

            # Generate cxx_config header/source when C++ config is enabled.
            # In SCons, cxx_config .hh/.cc are always generated but the .cc
            # is only compiled when --with-cxx-config is set. Here we only
            # generate them when the option is ON.
            if(GEM5_WITH_CXX_CONFIG)
                gem5_add_cxx_config("${_obj}" "${_modpath}")
            endif()
        endforeach()
    endif()

    message(STATUS "Created ${_count} SimObject param custom commands")
    if(GEM5_WITH_CXX_CONFIG)
        message(STATUS "  (with C++ config generation enabled)")
    endif()

    # --- Enums ---
    get_property(_enames GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_NAMES)
    get_property(_emodpaths GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_MODPATHS)
    get_property(_epyfiles GLOBAL PROPERTY GEM5_SIMOBJ_ENUM_PYFILES)

    list(LENGTH _enames _ecount)
    if(_ecount GREATER 0)
        math(EXPR _elast "${_ecount} - 1")
        foreach(_i RANGE 0 ${_elast})
            list(GET _enames ${_i} _enum)
            list(GET _emodpaths ${_i} _modpath)
            list(GET _epyfiles ${_i} _pyfile)

            set(_enum_hh "${GEM5_GEN_DIR}/enums/${_enum}.hh")
            set(_enum_cc "${GEM5_GEN_DIR}/enums/${_enum}.cc")

            add_custom_command(
                OUTPUT "${_enum_hh}"
                COMMAND "${GEM5PY_M5}" "${GEM5_BUILD_TOOLS_DIR}/enum_hh.py"
                        "${_modpath}" "${_enum_hh}"
                DEPENDS gem5py_m5 "${_pyfile}"
                        "${GEM5_BUILD_TOOLS_DIR}/enum_hh.py"
                        "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                        "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
                COMMENT "Generating enum header: enums/${_enum}.hh"
                VERBATIM
            )

            add_custom_command(
                OUTPUT "${_enum_cc}"
                COMMAND "${GEM5PY_M5}" "${GEM5_BUILD_TOOLS_DIR}/enum_cc.py"
                        "${_modpath}" "${_enum_cc}" "${_use_python}"
                DEPENDS gem5py_m5 "${_pyfile}" "${_enum_hh}"
                        "${GEM5_BUILD_TOOLS_DIR}/enum_cc.py"
                        "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                        "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
                WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
                COMMENT "Generating enum source: enums/${_enum}.cc"
                VERBATIM
            )
        endforeach()
    endif()

    message(STATUS "Created ${_ecount} SimObject enum custom commands")
endfunction()

# ---------------------------------------------------------------------------
# gem5_create_proto_commands()
#
# Create all protobuf custom commands in the current directory scope.
# Must be called from the top-level CMakeLists.txt AFTER all
# add_subdirectory() calls that register .proto files.
#
# Same rationale as gem5_create_pysource_commands(): Ninja requires custom
# commands to be in the same directory scope as the consuming targets.
# ---------------------------------------------------------------------------
function(gem5_create_proto_commands)
    get_property(_proto_files GLOBAL PROPERTY GEM5_PROTO_FILES)
    get_property(_suppress_flags GLOBAL PROPERTY GEM5_PROTO_SUPPRESS_FLAGS)

    list(LENGTH _proto_files _count)
    if(_count EQUAL 0)
        return()
    endif()

    set(_proto_out "${GEM5_GEN_DIR}/proto")
    file(MAKE_DIRECTORY "${_proto_out}")

    foreach(_proto_file ${_proto_files})
        get_filename_component(_name "${_proto_file}" NAME_WE)
        get_filename_component(_proto_dir "${_proto_file}" DIRECTORY)

        set(_pb_cc "${_proto_out}/${_name}.pb.cc")
        set(_pb_h "${_proto_out}/${_name}.pb.h")

        add_custom_command(
            OUTPUT "${_pb_cc}" "${_pb_h}"
            COMMAND "${Protobuf_PROTOC_EXECUTABLE}"
                    "--cpp_out=${_proto_out}"
                    "--proto_path=${_proto_dir}"
                    "${_proto_file}"
            DEPENDS "${_proto_file}"
            COMMENT "Generating protobuf: ${_name}.proto"
            VERBATIM
        )

        # Apply warning suppression flags to generated .pb.cc
        if(_suppress_flags)
            set_source_files_properties("${_pb_cc}" PROPERTIES
                COMPILE_OPTIONS "${_suppress_flags}")
        endif()
    endforeach()

    message(STATUS "Created ${_count} protobuf custom commands")
endfunction()
