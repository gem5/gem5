add_library(gem5_ext_nomali STATIC
    lib/gpu.cc
    lib/gpublock.cc
    lib/gpucontrol.cc
    lib/jobcontrol.cc
    lib/jobslot.cc
    lib/mali_midgard.cc
    lib/mali_t6xx.cc
    lib/mali_t7xx.cc
    lib/addrspace.cc
    lib/mmu.cc
    lib/nomali_api.cc
)

target_include_directories(gem5_ext_nomali SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(gem5_ext_nomali PRIVATE
    -Wno-ignored-qualifiers
)

set_target_properties(gem5_ext_nomali PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
