# src/base/stats/CMakeLists.txt
# Converted from src/base/stats/SConscript

gem5_add_source(group.cc)
gem5_add_source(info.cc)
gem5_add_source(storage.cc)
gem5_add_source(text.cc)

# HDF5 stats output (conditional)
if(CONF_HAVE_HDF5)
    if(GEM5_GCC)
        gem5_add_source(hdf5.cc APPEND_FLAGS -Wno-deprecated-copy)
    else()
        gem5_add_source(hdf5.cc)
    endif()
endif()
