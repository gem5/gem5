load("//bazel:rules.bzl", "gem5_cc_library")

package(
    default_visibility = ["//visibility:public"],
)

gem5_cc_library(
    name = "associative_cache",
    hdrs = ["associative_cache.hh"],
    deps = [
        ":cache_entry",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:named",
        "//src/base:types",
        "//src/mem/cache/replacement_policies:base",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
    ],
)

gem5_cc_library(
    name = "cache_entry",
    hdrs = ["cache_entry.hh"],
    deps = [
        "//src/base:cprintf",
        "//src/base:types",
        "//src/mem/cache/replacement_policies:replaceable_entry",
        "//src/mem/cache/tags/indexing_policies:base",
    ],
)
