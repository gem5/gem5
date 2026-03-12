load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

py_library(
    name = "rubytest_init",
    srcs = ["__init__.py"],
    deps = [
        ":RubyTester_py",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_RubyTest",
    flag = "RubyTest",
)

py_library(
    name = "RubyTester_py",
    srcs = ["RubyTester.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ruby_tester_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
    sim_object_header = ":RubyTester",
    sim_object_module = "src.cpu.testers.rubytest.RubyTester",
    sim_object_name = "RubyTester",
    sim_object_py_lib = ":RubyTester_py",
)

gem5_cc_library(
    name = "RubyTester",
    srcs = ["RubyTester.cc"],
    hdrs = ["RubyTester.hh"],
    deps = [
        ":Check",
        ":CheckTable",
        ":debug_RubyTest",
        ":ruby_tester_sim_object",
        "//src/base:logging",
        "//src/base:trace",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem/ruby/common",
        "//src/sim:clocked_object",
        "//src/sim:sim_exit",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "RubyTester_hh",
    hdrs = ["RubyTester.hh"],
    deps = [
        ":CheckTable_hh",
        ":ruby_tester_sim_object",
        "//src/mem:packet",
        "//src/mem:port",
        "//src/mem/ruby/common",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "Check",
    srcs = ["Check.cc"],
    hdrs = ["Check.hh"],
    deps = [
        ":RubyTester_hh",
        ":debug_RubyTest",
        "//src/base:random",
        "//src/base:trace",
        "//src/mem/ruby/common",
        "//src/mem/ruby/protocol:RubyAccessMode_hh",
        "//src/mem/ruby/protocol:TesterStatus_hh",
    ],
)

gem5_cc_library(
    name = "CheckTable",
    srcs = ["CheckTable.cc"],
    hdrs = ["CheckTable.hh"],
    deps = [
        ":Check",
        ":debug_RubyTest",
        "//src/base:intmath",
        "//src/base:random",
        "//src/base:trace",
        "//src/mem/ruby/common",
    ],
)

gem5_cc_library(
    name = "CheckTable_hh",
    hdrs = ["CheckTable.hh"],
    deps = [
        "//src/base:random",
        "//src/mem/ruby/common",
    ],
)
