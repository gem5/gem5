load("@rules_cc//cc:defs.bzl", "cc_library")

genrule(
    name = "libelf_convert_c",
    srcs = ["elf_types.m4", "libelf_convert.m4"],
    outs = ["libelf_convert.c"],
    cmd = "m4 -DSRCDIR=$$(dirname $(location elf_types.m4)) $(location elf_types.m4) $(location libelf_convert.m4) > $@",
)

genrule(
    name = "libelf_fsize_c",
    srcs = ["elf_types.m4", "libelf_fsize.m4"],
    outs = ["libelf_fsize.c"],
    cmd = "m4 -DSRCDIR=$$(dirname $(location elf_types.m4)) $(location elf_types.m4) $(location libelf_fsize.m4) > $@",
)

genrule(
    name = "libelf_msize_c",
    srcs = ["elf_types.m4", "libelf_msize.m4"],
    outs = ["libelf_msize.c"],
    cmd = "m4 -DSRCDIR=$$(dirname $(location elf_types.m4)) $(location elf_types.m4) $(location libelf_msize.m4) > $@",
)

genrule(
    name = "native_elf_format_h",
    srcs = ["native-elf-format"],
    outs = ["native-elf-format.h"],
    cmd = """
if command -v readelf >/dev/null 2>&1; then
    ./$(location native-elf-format) > $@
else
    # Fallback for Mac or systems without readelf
    ARCH=$$(uname -m)
    if [ "$$ARCH" = "arm64" ]; then
        echo "#define ELFTC_CLASS ELFCLASS64" > $@
        echo "#define ELFTC_ARCH EM_AARCH64" >> $@
        echo "#define ELFTC_BYTEORDER ELFDATA2LSB" >> $@
    elif [ "$$ARCH" = "x86_64" ]; then
        echo "#define ELFTC_CLASS ELFCLASS64" > $@
        echo "#define ELFTC_ARCH EM_X86_64" >> $@
        echo "#define ELFTC_BYTEORDER ELFDATA2LSB" >> $@
    else
        echo "#error Unknown architecture for libelf fallback" > $@
    fi
fi
""",
)

cc_library(
    name = "libelf",
    srcs = [
        "elf.c",
        "elf_begin.c",
        "elf_cntl.c",
        "elf_data.c",
        "elf_end.c",
        "elf_errmsg.c",
        "elf_errno.c",
        "elf_fill.c",
        "elf_flag.c",
        "elf_getarhdr.c",
        "elf_getarsym.c",
        "elf_getbase.c",
        "elf_getident.c",
        "elf_hash.c",
        "elf_kind.c",
        "elf_memory.c",
        "elf_next.c",
        "elf_open.c",
        "elf_phnum.c",
        "elf_rand.c",
        "elf_rawfile.c",
        "elf_scn.c",
        "elf_shnum.c",
        "elf_shstrndx.c",
        "elf_strptr.c",
        "elf_update.c",
        "elf_version.c",
        "gelf_cap.c",
        "gelf_checksum.c",
        "gelf_dyn.c",
        "gelf_ehdr.c",
        "gelf_fsize.c",
        "gelf_getclass.c",
        "gelf_move.c",
        "gelf_phdr.c",
        "gelf_rel.c",
        "gelf_rela.c",
        "gelf_shdr.c",
        "gelf_sym.c",
        "gelf_syminfo.c",
        "gelf_symshndx.c",
        "gelf_xlate.c",
        "libelf.c",
        "libelf_align.c",
        "libelf_allocate.c",
        "libelf_ar.c",
        "libelf_ar_util.c",
        "libelf_checksum.c",
        "libelf_data.c",
        "libelf_ehdr.c",
        "libelf_extended.c",
        "libelf_memory.c",
        "libelf_open.c",
        "libelf_phdr.c",
        "libelf_shdr.c",
        "libelf_xlate.c",
        ":libelf_convert_c",
        ":libelf_fsize_c",
        ":libelf_msize_c",
    ],
    hdrs = glob(["*.h"]) + [":native_elf_format_h"],
    includes = ["."],
    visibility = ["//visibility:public"],
    copts = [
        "-Wno-pointer-sign",
        "-Wno-unused-but-set-variable",
        "-Wno-implicit-function-declaration",
        "-Wno-override-init",
        "-std=gnu89",
    ],
)
