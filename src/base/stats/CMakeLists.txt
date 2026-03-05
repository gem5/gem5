# src/base/stats/CMakeLists.txt
# Converted from src/base/stats/SConscript

gem5_add_source(group.cc)
gem5_add_source(info.cc)
gem5_add_source(storage.cc)
gem5_add_source(text.cc)

# HDF5 stats output (conditional).
# -Wno-deprecated-copy for GCC is applied globally in GEM5_WARNING_FLAGS.
if(CONF_HAVE_HDF5)
    gem5_add_source(hdf5.cc)
endif()

# --- Unit tests ---
gem5_add_test_source(group.test.cc)
gem5_add_test_source(info.test.cc)
gem5_add_test_source(storage.test.cc)
gem5_add_test_source(units.test.cc)
