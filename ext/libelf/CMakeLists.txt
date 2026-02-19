# M4 macro processor for generated sources
find_program(M4_EXECUTABLE NAMES gm4 m4 REQUIRED)

# Generate C files from M4 macros
set(_m4_generated_sources
    libelf_convert.c
    libelf_fsize.c
    libelf_msize.c
)

foreach(_m4_output ${_m4_generated_sources})
    string(REPLACE ".c" ".m4" _m4_input "${_m4_output}")
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_m4_output}
        COMMAND ${M4_EXECUTABLE}
            -DSRCDIR=${CMAKE_CURRENT_SOURCE_DIR}
            ${CMAKE_CURRENT_SOURCE_DIR}/elf_types.m4
            ${CMAKE_CURRENT_SOURCE_DIR}/${_m4_input}
            > ${CMAKE_CURRENT_BINARY_DIR}/${_m4_output}
        DEPENDS
            ${CMAKE_CURRENT_SOURCE_DIR}/elf_types.m4
            ${CMAKE_CURRENT_SOURCE_DIR}/${_m4_input}
        COMMENT "Generating ${_m4_output} with M4"
    )
endforeach()

# Generate native-elf-format.h
add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/native-elf-format.h
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/native-elf-format
        > ${CMAKE_CURRENT_BINARY_DIR}/native-elf-format.h
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/native-elf-format
    COMMENT "Generating native-elf-format.h"
)

add_library(gem5_ext_elf STATIC
    elf.c
    elf_begin.c
    elf_cntl.c
    elf_data.c
    elf_end.c
    elf_errmsg.c
    elf_errno.c
    elf_fill.c
    elf_flag.c
    elf_getarhdr.c
    elf_getarsym.c
    elf_getbase.c
    elf_getident.c
    elf_hash.c
    elf_kind.c
    elf_memory.c
    elf_next.c
    elf_open.c
    elf_phnum.c
    elf_rand.c
    elf_rawfile.c
    elf_scn.c
    elf_shnum.c
    elf_shstrndx.c
    elf_strptr.c
    elf_update.c
    elf_version.c
    gelf_cap.c
    gelf_checksum.c
    gelf_dyn.c
    gelf_ehdr.c
    gelf_fsize.c
    gelf_getclass.c
    gelf_move.c
    gelf_phdr.c
    gelf_rel.c
    gelf_rela.c
    gelf_shdr.c
    gelf_sym.c
    gelf_syminfo.c
    gelf_symshndx.c
    gelf_xlate.c
    libelf.c
    libelf_align.c
    libelf_allocate.c
    libelf_ar.c
    libelf_ar_util.c
    libelf_checksum.c
    libelf_data.c
    libelf_ehdr.c
    libelf_extended.c
    libelf_memory.c
    libelf_open.c
    libelf_phdr.c
    libelf_shdr.c
    libelf_xlate.c
    # M4-generated sources
    ${CMAKE_CURRENT_BINARY_DIR}/libelf_convert.c
    ${CMAKE_CURRENT_BINARY_DIR}/libelf_fsize.c
    ${CMAKE_CURRENT_BINARY_DIR}/libelf_msize.c
    # Generated header (listed as source so CMake knows about the dependency)
    ${CMAKE_CURRENT_BINARY_DIR}/native-elf-format.h
)

target_include_directories(gem5_ext_elf SYSTEM PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(gem5_ext_elf PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_compile_options(gem5_ext_elf PRIVATE
    -Wno-implicit
    -Wno-undef
    $<$<C_COMPILER_ID:GNU>:
        -Wno-pointer-sign
        -Wno-unused-but-set-variable
        -Wno-implicit-function-declaration
        -Wno-override-init
    >
    $<$<C_COMPILER_ID:Clang>:
        -Wno-initializer-overrides
        -Wno-pointer-sign
        -std=gnu89
    >
    $<$<C_COMPILER_ID:AppleClang>:
        -Wno-initializer-overrides
        -Wno-pointer-sign
        -std=gnu89
    >
)

set_target_properties(gem5_ext_elf PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
