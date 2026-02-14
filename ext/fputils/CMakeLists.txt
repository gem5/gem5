add_library(gem5_ext_fputils STATIC
    fp64.c
    fp80.c
)

target_include_directories(gem5_ext_fputils PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(gem5_ext_fputils PRIVATE
    -std=c99
)

set_target_properties(gem5_ext_fputils PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
