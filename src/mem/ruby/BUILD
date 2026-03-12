load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_debug_flag")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "ruby_init",
    srcs = ["__init__.py"],
)

py_library(
    name = "ruby_all_libs",
    deps = [
        ":ruby_init",
        "//src/mem/ruby/common:common_init",
        "//src/mem/ruby/network:network_init",
        "//src/mem/ruby/profiler:profiler_init",
        "//src/mem/ruby/protocol:protocol_init",
        "//src/mem/ruby/slicc_interface:slicc_interface_init",
        "//src/mem/ruby/structures:structures_init",
        "//src/mem/ruby/system:system_init",
    ],
)

gem5_debug_flag(
    name = "debug_ProtocolTrace",
    flag = "ProtocolTrace",
)

gem5_debug_flag(
    name = "debug_RubyCache",
    flag = "RubyCache",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyCacheTrace",
    flag = "RubyCacheTrace",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyDma",
    flag = "RubyDma",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyGenerated",
    flag = "RubyGenerated",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyNetwork",
    flag = "RubyNetwork",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyPort",
    flag = "RubyPort",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyPrefetcher",
    flag = "RubyPrefetcher",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyQueue",
    flag = "RubyQueue",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubySequencer",
    flag = "RubySequencer",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubySlicc",
    flag = "RubySlicc",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubySystem",
    flag = "RubySystem",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyTester",
    flag = "RubyTester",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyStats",
    flag = "RubyStats",
)

gem5_debug_flag(
    name = "debug_RubyResourceStalls",
    flag = "RubyResourceStalls",
)

gem5_debug_flag(
    name = "debug_RubyProtocol",
    flag = "RubyProtocol",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_RubyHitMiss",
    flag = "RubyHitMiss",
    included_by = [":debug_Ruby"],
)

gem5_debug_flag(
    name = "debug_Ruby",
    components = [
        ":debug_RubyQueue",
        ":debug_RubyNetwork",
        ":debug_RubyTester",
        ":debug_RubyGenerated",
        ":debug_RubySlicc",
        ":debug_RubySystem",
        ":debug_RubyCache",
        ":debug_RubyDma",
        ":debug_RubyPort",
        ":debug_RubySequencer",
        ":debug_RubyCacheTrace",
        ":debug_RubyPrefetcher",
        ":debug_RubyProtocol",
        ":debug_RubyHitMiss",
    ],
    flag = "Ruby",
)
