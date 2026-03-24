add_library(gem5_ext_magic_enum INTERFACE)

target_include_directories(gem5_ext_magic_enum INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(gem5_ext_magic_enum INTERFACE
    MAGIC_ENUM_RANGE_MAX=0x100
)
