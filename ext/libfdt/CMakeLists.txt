add_library(gem5_ext_libfdt STATIC
    fdt.c
    fdt_ro.c
    fdt_rw.c
    fdt_sw.c
    fdt_wip.c
    fdt_empty_tree.c
    fdt_strerror.c
)

target_include_directories(gem5_ext_libfdt SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Suppress Clang warning from vendored pointer overflow check.
target_compile_options(gem5_ext_libfdt PRIVATE
    -Wno-tautological-compare
)

set_target_properties(gem5_ext_libfdt PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
