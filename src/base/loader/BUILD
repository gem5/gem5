load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_debug_flag")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "elf_object",
    srcs = ["elf_object.cc"],
    hdrs = ["elf_object.hh"],
    deps = [
        ":debug_Loader",
        ":image_file",
        ":object_file",
        ":symtab",
        "//ext/libelf:libelf",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:byteswap",
    ],
)

gem5_debug_flag(
    name = "debug_Loader",
    desc = "Enable debug output for elf_object.hh",
    flag = "Loader",
)

gem5_cc_library(
    name = "object_file_hh",
    hdrs = ["object_file.hh"],
    deps = [
        ":image_file",
        ":image_file_data",
        ":memory_image",
        ":symtab",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:enum_ByteOrder_hh",
    ],
)

gem5_cc_library(
    name = "object_file",
    srcs = ["object_file.cc"],
    hdrs = ["object_file.hh"],
    deps = [
        ":image_file",
        ":image_file_data",
        ":memory_image",
        ":raw_image",
        ":symtab",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:enum_ByteOrder_hh",
    ],
)

gem5_cc_library(
    name = "image_file",
    hdrs = ["image_file.hh"],
    deps = [
        ":image_file_data",
        ":memory_image",
        "//src/base:compiler",
    ],
)

# TODO(jlowepower): The port proxy dependency is bad
gem5_cc_library(
    name = "memory_image",
    srcs = ["memory_image.cc"],
    hdrs = ["memory_image.hh"],
    deps = [
        ":image_file_data",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
        "//src/mem:port_proxy",
    ],
)

gem5_cc_library(
    name = "image_file_data",
    srcs = ["image_file_data.cc"],
    hdrs = ["image_file_data.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:logging",
        "@zlib//:zlib",
    ],
)

# TODO(jlowepower): Lots of test failures.
# gem5_cc_test(
#     name = "image_file_data_test",
#     srcs = [
#         "image_file_data.test.cc",
#         "small_image_file.test.hh",
#     ],
#     deps = [
#         ":image_file_data",
#         "//ext/googletest:gtest_main",
#         "//ext/googletest:gtest",
#     ],
# )

gem5_cc_library(
    name = "symtab",
    srcs = ["symtab.cc"],
    hdrs = ["symtab.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:serialize",
    ],
)

gem5_cc_test(
    name = "symtab_test",
    srcs = ["symtab.test.cc"],
    deps = [
        ":symtab",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
        "//src/base/gtest:gtest",
    ],
)

gem5_cc_library(
    name = "raw_image",
    hdrs = ["raw_image.hh"],
    deps = [
        ":object_file_hh",
        "//src/base:compiler",
    ],
)

gem5_cc_library(
    name = "symtab_hh",
    hdrs = ["symtab.hh"],
    deps = [
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:serialize",
        "//src/sim:serialize_handlers_hh",
    ],
)

gem5_cc_library(
    name = "memory_image_hh",
    hdrs = ["memory_image.hh"],
    deps = [
        ":image_file_data",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "dtb_file",
    srcs = ["dtb_file.cc"],
    hdrs = ["dtb_file.hh"],
    deps = [
        ":image_file",
        ":image_file_data",
        ":symtab",
        "//ext/libfdt",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:byteswap",
    ],
)
