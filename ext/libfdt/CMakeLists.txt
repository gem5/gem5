add_library(gem5_ext_libfdt STATIC
    fdt.c
    fdt_ro.c
    fdt_rw.c
    fdt_sw.c
    fdt_wip.c
    fdt_empty_tree.c
    fdt_strerror.c
)

target_include_directories(gem5_ext_libfdt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(gem5_ext_libfdt PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
