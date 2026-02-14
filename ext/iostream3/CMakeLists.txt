find_package(ZLIB REQUIRED)

add_library(gem5_ext_iostream3 STATIC
    zfstream.cc
)

target_include_directories(gem5_ext_iostream3 PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(gem5_ext_iostream3 PUBLIC
    ZLIB::ZLIB
)

set_target_properties(gem5_ext_iostream3 PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
