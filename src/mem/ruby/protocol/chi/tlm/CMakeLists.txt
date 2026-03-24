# src/mem/ruby/protocol/chi/tlm/CMakeLists.txt
# Converted from src/mem/ruby/protocol/chi/tlm/SConscript
# Conditional on CONF_BUILD_TLM (guarded by parent)

# ---------------------------------------------------------------------------
# Find external TLM library (libarmtlmchi.so)
# ---------------------------------------------------------------------------
# TLM_PATH is set by the top-level CMake option (or env var).
# Validate that the library exists at the expected location.
set(_tlm_include "${GEM5_TLM_PATH}/include")
set(_tlm_lib_dir "${GEM5_TLM_PATH}/lib")

find_library(TLM_ARMTLMCHI_LIB armtlmchi
    PATHS "${_tlm_lib_dir}"
    NO_DEFAULT_PATH
    NO_CMAKE_FIND_ROOT_PATH
)

if(NOT TLM_ARMTLMCHI_LIB)
    message(FATAL_ERROR
        "Cannot find libarmtlmchi.so in TLM directory.\n"
        "TLM path: ${GEM5_TLM_PATH}\n"
        "Expected library at: ${_tlm_lib_dir}/libarmtlmchi.so\n"
        "Set -DGEM5_TLM_PATH=<path> or TLM_PATH environment variable.")
endif()

# Create an imported target for the TLM library
add_library(gem5_tlm_armtlmchi SHARED IMPORTED)
set_target_properties(gem5_tlm_armtlmchi PROPERTIES
    IMPORTED_LOCATION "${TLM_ARMTLMCHI_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${_tlm_include}"
)

# Register the TLM library for linking into gem5
set_property(GLOBAL APPEND PROPERTY GEM5_LINK_LIBRARIES gem5_tlm_armtlmchi)

# ---------------------------------------------------------------------------
# Source files
# ---------------------------------------------------------------------------
gem5_add_source(controller.cc)
gem5_add_source(generator.cc)
gem5_add_source(utils.cc)
# tlm_chi.cc and tlm_chi_gen.cc are tagged 'python' in SCons and only
# included in builds with Python support.
if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_source(tlm_chi.cc)
    gem5_add_source(tlm_chi_gen.cc)
endif()

# ---------------------------------------------------------------------------
# Debug flags
# ---------------------------------------------------------------------------
gem5_add_debug_flag(TLM "TLM debugging")
gem5_add_debug_flag(TLMPort "TLMPort debugging")

# ---------------------------------------------------------------------------
# Python sources (m5.tlm_chi package)
# ---------------------------------------------------------------------------
if(NOT GEM5_WITHOUT_PYTHON)
    gem5_add_pysource("m5.tlm_chi" "${CMAKE_CURRENT_SOURCE_DIR}/python/__init__.py")
    gem5_add_pysource("m5.tlm_chi" "${CMAKE_CURRENT_SOURCE_DIR}/python/port.py")
    gem5_add_pysource("m5.tlm_chi" "${CMAKE_CURRENT_SOURCE_DIR}/python/utils.py")
endif()

# ---------------------------------------------------------------------------
# SimObjects
# ---------------------------------------------------------------------------
gem5_add_simobject(TlmController.py SIM_OBJECTS TlmController)
gem5_add_simobject(TlmGenerator.py SIM_OBJECTS TlmGenerator)

message(STATUS "BUILD_TLM set: Building TLM integration with libarmtlmchi.so from '${_tlm_lib_dir}'")
