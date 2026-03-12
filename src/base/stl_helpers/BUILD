load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "ostream_helpers",
    hdrs = ["ostream_helpers.hh"],
    deps = [
        "//ext/magic_enum",
        "//src/base:type_traits",
    ],
)

gem5_cc_test(
    name = "ostream_helpers_test",
    srcs = ["ostream_helpers.test.cc"],
    deps = [
        ":ostream_helpers",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "hash_helpers",
    hdrs = ["hash_helpers.hh"],
    deps = ["//src/base:type_traits"],
)

gem5_cc_test(
    name = "hash_helpers_test",
    srcs = ["hash_helpers.test.cc"],
    deps = [
        ":hash_helpers",
        "//ext/googletest:gtest_main",
    ],
)
