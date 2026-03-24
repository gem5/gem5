# cmake/Gem5Py.cmake
#
# Build the gem5py and gem5py_m5 helper executables used during build-time
# code generation.
#
# gem5py:    Simple embedded Python interpreter (pybind11).
#            Used by build scripts that don't need m5.SimObject imports.
#            In this CMake build, most such scripts use Python3 directly,
#            but gem5py provides the exact same Python version as gem5.
#
# gem5py_m5: Full embedded Python interpreter with all gem5 Python modules
#            baked in (embedded.cc + importer + marshalled .py files).
#            Used by SimObject/enum code generation scripts that need to
#            `import m5.SimObject` and friends.
#
# Build flow:
#   1. System Python3 generates m5ImporterCode blob from importer.py
#   2. System Python3 marshals PySource .py files -> .py.cc (gem5_add_pysource)
#   3. gem5py is built from gem5py.cc + pybind11::embed
#   4. gem5py_m5 is built from gem5py_m5.cc + embedded.cc + importer.cc +
#      m5ImporterCode.cc + all .py.cc files + pybind11::embed + zlib
#      (must be called after all gem5_add_pysource() registrations)

# ---------------------------------------------------------------------------
# gem5_build_gem5py()
#
# Build the simple gem5py interpreter. Call this early in the build
# (after Python3 and pybind11 are available).
#
# Sets cache variable GEM5PY to the executable path (generator expression).
# ---------------------------------------------------------------------------
function(gem5_build_gem5py)
    add_executable(gem5py "${CMAKE_SOURCE_DIR}/src/python/gem5py.cc")

    target_link_libraries(gem5py PRIVATE pybind11::embed)

    target_include_directories(gem5py PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
    )

    # -rdynamic: make symbols visible to dynamically loaded Python extensions
    if(NOT WIN32)
        target_link_options(gem5py PRIVATE -rdynamic)
    endif()

    # Export path for use in add_custom_command
    set(GEM5PY "$<TARGET_FILE:gem5py>" CACHE INTERNAL
        "Path to gem5py executable")
endfunction()

# ---------------------------------------------------------------------------
# gem5_generate_importer_blob(<out_cc_var> <out_hh_var>)
#
# Generate m5ImporterCode.{cc,hh} from src/python/importer.py.
#
# The importer is a core piece of gem5's embedded Python infrastructure:
# it provides a custom import hook that allows Python code to be loaded
# from compressed bytecode blobs compiled into the binary.
#
# importer.cc includes "python/m5ImporterCode.hh" to access the blob.
#
# Arguments:
#   out_cc_var - variable to receive the path to the generated .cc file
#   out_hh_var - variable to receive the path to the generated .hh file
# ---------------------------------------------------------------------------
function(gem5_generate_importer_blob out_cc_var out_hh_var)
    set(_importer_py "${CMAKE_SOURCE_DIR}/src/python/importer.py")
    set(_blob_hh "${GEM5_GEN_DIR}/python/m5ImporterCode.hh")
    set(_blob_cc "${GEM5_GEN_DIR}/python/m5ImporterCode.cc")

    file(MAKE_DIRECTORY "${GEM5_GEN_DIR}/python")

    add_custom_command(
        OUTPUT "${_blob_hh}" "${_blob_cc}"
        COMMAND "${Python3_EXECUTABLE}"
                "${GEM5_BUILD_TOOLS_DIR}/generate_blob.py"
                "m5ImporterCode"
                "${_importer_py}"
                "${_blob_cc}"
                "${_blob_hh}"
                "python/m5ImporterCode.hh"
        DEPENDS "${_importer_py}"
                "${GEM5_BUILD_TOOLS_DIR}/generate_blob.py"
                "${GEM5_BUILD_TOOLS_DIR}/blob.py"
                "${GEM5_BUILD_TOOLS_DIR}/code_formatter.py"
                "${GEM5_BUILD_TOOLS_DIR}/file_utils.py"
        WORKING_DIRECTORY "${GEM5_BUILD_TOOLS_DIR}"
        COMMENT "Generating importer blob: m5ImporterCode"
        VERBATIM
    )

    # Register the blob .cc as a generated source for the main gem5 library too.
    # importer.cc (now in gem5_all) needs the blob header, and the blob .cc
    # provides the data array.
    gem5_add_generated_source("${_blob_cc}")

    set(${out_cc_var} "${_blob_cc}" PARENT_SCOPE)
    set(${out_hh_var} "${_blob_hh}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# gem5_build_gem5py_m5()
#
# Build the full gem5py_m5 interpreter with all embedded Python modules.
#
# IMPORTANT: This must be called AFTER all gem5_add_pysource() registrations
# are complete (typically at the end of the top-level CMakeLists.txt, after
# all add_subdirectory() calls that register Python sources).
#
# gem5py_m5 is used by code generation scripts that need full m5 imports:
#   - sim_object_param_struct_hh.py  (SimObject param header)
#   - sim_object_param_struct_cc.py  (SimObject pybind11 bindings)
#   - enum_hh.py / enum_cc.py       (enum header/source)
#   - cxx_config_hh.py / cxx_config_cc.py (C++ config)
#
# Sets cache variable GEM5PY_M5 to the executable path (generator expression).
# ---------------------------------------------------------------------------
function(gem5_build_gem5py_m5)
    # Generate the importer blob (m5ImporterCode.{cc,hh})
    gem5_generate_importer_blob(_blob_cc _blob_hh)

    # Get all accumulated Python source .cc files
    get_property(_pysources GLOBAL PROPERTY GEM5_PYSOURCES)

    add_executable(gem5py_m5
        "${CMAKE_SOURCE_DIR}/src/python/gem5py_m5.cc"
        "${CMAKE_SOURCE_DIR}/src/python/embedded.cc"
        "${CMAKE_SOURCE_DIR}/src/python/importer.cc"
        "${_blob_cc}"
        # Include the blob header so CMake knows importer.cc depends on it.
        # Headers listed as sources are not compiled but tracked for deps.
        "${_blob_hh}"
        ${_pysources}
    )

    target_link_libraries(gem5py_m5 PRIVATE
        pybind11::embed
        ZLIB::ZLIB
    )

    target_include_directories(gem5py_m5 PRIVATE
        "${CMAKE_SOURCE_DIR}/src"
        "${GEM5_GEN_DIR}"
    )

    # -rdynamic: make symbols visible to dynamically loaded Python extensions
    if(NOT WIN32)
        target_link_options(gem5py_m5 PRIVATE -rdynamic)
    endif()

    # Export path for use in add_custom_command
    set(GEM5PY_M5 "$<TARGET_FILE:gem5py_m5>" CACHE INTERNAL
        "Path to gem5py_m5 executable (with embedded m5 modules)")
endfunction()
