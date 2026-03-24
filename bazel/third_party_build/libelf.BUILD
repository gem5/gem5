load("@rules_cc//cc:defs.bzl", "cc_library")

# M4-generated source files
genrule(
    name = "gen_libelf_convert",
    srcs = ["libelf_convert.m4", "elf_types.m4"],
    outs = ["libelf_convert.c"],
    cmd = "m4 -DSRCDIR=$$(dirname $(location libelf_convert.m4)) $(location elf_types.m4) $(location libelf_convert.m4) > $@",
)

genrule(
    name = "gen_libelf_fsize",
    srcs = ["libelf_fsize.m4", "elf_types.m4"],
    outs = ["libelf_fsize.c"],
    cmd = "m4 -DSRCDIR=$$(dirname $(location libelf_fsize.m4)) $(location elf_types.m4) $(location libelf_fsize.m4) > $@",
)

genrule(
    name = "gen_libelf_msize",
    srcs = ["libelf_msize.m4", "elf_types.m4"],
    outs = ["libelf_msize.c"],
    cmd = "m4 -DSRCDIR=$$(dirname $(location libelf_msize.m4)) $(location elf_types.m4) $(location libelf_msize.m4) > $@",
)

# Generate native-elf-format.h from the native-elf-format script
genrule(
    name = "gen_native_elf_format",
    srcs = ["native-elf-format"],
    outs = ["native-elf-format.h"],
    cmd = "cp $(location native-elf-format) $(@D)/_native_elf_format && chmod +x $(@D)/_native_elf_format && $(@D)/_native_elf_format > $@",
)

cc_library(
    name = "libelf",
    srcs = glob(
        ["*.c"],
        exclude = [
            "libelf_convert.c",
            "libelf_fsize.c",
            "libelf_msize.c",
        ],
    ) + [
        ":gen_libelf_convert",
        ":gen_libelf_fsize",
        ":gen_libelf_msize",
    ],
    hdrs = glob(["*.h"]) + [":gen_native_elf_format"],
    includes = ["."],
    copts = [
        "-Wno-implicit",
        "-Wno-undef",
        "-Wno-pointer-sign",
        "-Wno-unused-but-set-variable",
        "-Wno-implicit-function-declaration",
        "-Wno-override-init",
        "-Wno-unused-parameter",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hdrs",
    hdrs = ["elfdefinitions.h", "gelf.h", "libelf.h"],
    includes = ["."],
    visibility = ["//visibility:public"],
)
