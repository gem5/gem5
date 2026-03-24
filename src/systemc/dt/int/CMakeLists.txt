# src/systemc/dt/int/CMakeLists.txt
# Converted from src/systemc/dt/int/SConscript

gem5_add_sources(
    messages.cc
    sc_int_base.cc
    sc_int_mask.cc
    sc_length_param.cc
    sc_nbexterns.cc
    sc_nbutils.cc
    sc_uint_base.cc
)

# Disable false-positive warnings for sc_signed.cc and sc_unsigned.cc
# (GCC 10+ -Warray-bounds, -Wstringop-overflow)
gem5_add_source(sc_signed.cc APPEND_FLAGS
    "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_GREATER_EQUAL:$<CXX_COMPILER_VERSION>,10.0>>:-Wno-array-bounds;-Wno-stringop-overflow>")
gem5_add_source(sc_unsigned.cc APPEND_FLAGS
    "$<$<AND:$<CXX_COMPILER_ID:GNU>,$<VERSION_GREATER_EQUAL:$<CXX_COMPILER_VERSION>,10.0>>:-Wno-array-bounds;-Wno-stringop-overflow>")
