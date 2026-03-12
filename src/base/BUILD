load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "base_init",
    srcs = ["__init__.py"],
    deps = [
        ":Graphics_py",
        "//src/base/vnc:vnc_init",
    ],
)

gem5_debug_flag(
    name = "debug_VNC",
    desc = "VNC server debugging",
    flag = "VNC",
)

gem5_debug_flag(
    name = "debug_Annotate",
    desc = "State machine annotation debugging",
    flag = "Annotate",
    included_by = [":debug_AnnotateAll"],
)

gem5_debug_flag(
    name = "debug_AnnotateQ",
    desc = "State machine annotation queue debugging",
    flag = "AnnotateQ",
    included_by = [":debug_AnnotateAll"],
)

gem5_debug_flag(
    name = "debug_AnnotateVerbose",
    desc = "Dump all state machine annotation details",
    flag = "AnnotateVerbose",
    included_by = [":debug_AnnotateAll"],
)

gem5_debug_flag(
    name = "debug_AnnotateAll",
    components = [
        ":debug_Annotate",
        ":debug_AnnotateQ",
        ":debug_AnnotateVerbose",
    ],
    desc = "All Annotation flags",
    flag = "AnnotateAll",
)

gem5_debug_flag(
    name = "debug_FmtFlag",
    desc = "Show the --debug-flag that enabled each debug message",
    flag = "FmtFlag",
    fmt = True,
)

gem5_debug_flag(
    name = "debug_FmtStackTrace",
    desc = "Show the stack trace when a debug message is emitted",
    flag = "FmtStackTrace",
    fmt = True,
)

gem5_debug_flag(
    name = "debug_FmtTicksOff",
    desc = "Don't show the tick count on debug messages",
    flag = "FmtTicksOff",
    fmt = True,
    included_by = ["//src/cpu:debug_ExecNoTicks"],
)

gem5_debug_flag(
    name = "debug_GDBAcc",
    desc = "Remote debugger accesses",
    flag = "GDBAcc",
    included_by = [":debug_GDBAll"],
)

gem5_debug_flag(
    name = "debug_GDBMisc",
    desc = "Breakpoints, traps, watchpoints, etc.",
    flag = "GDBMisc",
    included_by = [":debug_GDBAll"],
)

gem5_debug_flag(
    name = "debug_GDBRecv",
    flag = "GDBRecv",
    included_by = [":debug_GDBAll"],
)

gem5_debug_flag(
    name = "debug_GDBSend",
    flag = "GDBSend",
    included_by = [":debug_GDBAll"],
)

gem5_debug_flag(
    name = "debug_GDBExtra",
    flag = "GDBExtra",
    included_by = [":debug_GDBAll"],
)

gem5_debug_flag(
    name = "debug_GDBRead",
    flag = "GDBRead",
    included_by = [":debug_GDBAll"],
)

gem5_debug_flag(
    name = "debug_GDBWrite",
    flag = "GDBWrite",
    included_by = [":debug_GDBAll"],
)

gem5_debug_flag(
    name = "debug_GDBAll",
    components = [
        ":debug_GDBAcc",
        ":debug_GDBExtra",
        ":debug_GDBMisc",
        ":debug_GDBRead",
        ":debug_GDBRecv",
        ":debug_GDBSend",
        ":debug_GDBWrite",
    ],
    desc = "All Remote debugging flags",
    flag = "GDBAll",
)

gem5_debug_flag(
    name = "debug_SQL",
    desc = "SQL queries sent to the server",
    flag = "SQL",
)

gem5_debug_flag(
    name = "debug_Stats",
    desc = "Statistics management",
    flag = "Stats",
)

gem5_debug_flag(
    name = "debug_StatEvents",
    desc = "Statistics event tracking",
    flag = "StatEvents",
)

gem5_cc_library(
    name = "addr_range",
    hdrs = [
        "addr_range.hh",
    ],
    deps = [
        ":bitfield",
        ":cprintf",
        ":logging",
        ":types",
    ],
)

gem5_cc_test(
    name = "addr_range_test",
    srcs = [
        "addr_range.test.cc",
    ],
    deps = [
        ":addr_range",
        ":bitfield",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "addr_range_map",
    hdrs = [
        "addr_range_map.hh",
    ],
    deps = [
        ":addr_range",
        ":types",
    ],
)

gem5_cc_test(
    name = "addr_range_map_test",
    srcs = [
        "addr_range_map.test.cc",
    ],
    deps = [
        ":addr_range_map",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "inet_hh",
    hdrs = [
        "inet.hh",
    ],
    deps = [
        ":compiler",
        ":cprintf",
        ":logging",
        ":types",
        "//ext/dnet",
        "//src/dev/net:etherpkt_hh",
    ],
)

# Circular dependency on src/dev/net/etherpkt.hh
gem5_cc_library(
    name = "inet",
    srcs = [
        "inet.cc",
    ],
    deps = [
        ":compiler",
        ":cprintf",
        ":inet_hh",
        ":logging",
        ":types",
        "//ext/dnet",
    ],
)

gem5_cc_library(
    name = "named",
    hdrs = [
        "named.hh",
    ],
)

gem5_cc_library(
    name = "uncontended_mutex",
    hdrs = [
        "uncontended_mutex.hh",
    ],
)

gem5_cc_test(
    name = "uncontended_mutex_test",
    srcs = [
        "uncontended_mutex.test.cc",
    ],
    deps = [
        ":uncontended_mutex",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "compiler",
    hdrs = [
        "compiler.hh",
    ],
    deps = [
        "//bazel/generated/config",
    ],
)

gem5_cc_library(
    name = "flags",
    hdrs = [
        "flags.hh",
    ],
)

gem5_cc_test(
    name = "flags_test",
    srcs = [
        "flags.test.cc",
    ],
    deps = [
        ":flags",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

# TODO(jlowepower): logging -> hostinfo -> str -> panic (in logging)
gem5_cc_library(
    name = "logging",
    srcs = [
        "hostinfo.cc",
        "logging.cc",
        "str.cc",
    ],
    hdrs = [
        "hostinfo.hh",
        "logging.hh",
        "str.hh",
    ],
    deps = [
        ":compiler",
        ":cprintf",
    ],
)

# TODO(jlowepower): Lots of tests failing here.
# gem5_cc_test(
#     name = "logging_test",
#     srcs = [
#         "logging.test.cc",
#     ],
#     copts = [""],
#     deps = [
#         ":logging",
#         "//ext/googletest:gtest_main",
#         "//ext/googletest:gtest",
#     ],
# )

# TODO(jlowepower): Test fails because of exception thrown. The exception is expected.
# StrTest.ToNumber8BitIntStringOutOfRange
# gem5_cc_test(
#     name = "str_test",
#     srcs = [
#         "str.test.cc",
#     ],
#     copts = [""],
#     deps = [
#         ":logging",
#         "//ext/googletest:gtest_main",
#     ],
# )

gem5_cc_library(
    name = "stl_helpers",
    hdrs = [
        "stl_helpers.hh",
    ],
    deps = [
        ":type_traits",
        "//src/base/stl_helpers:hash_helpers",
        "//src/base/stl_helpers:ostream_helpers",
    ],
)

gem5_cc_library(
    name = "trace",
    srcs = [
        "trace.cc",
    ],
    hdrs = [
        "trace.hh",
    ],
    deps = [
        ":atomicio",
        ":compiler",
        ":cprintf",
        ":debug",
        ":debug_FmtFlag",
        ":debug_FmtStackTrace",
        ":debug_FmtTicksOff",
        ":logging",
        ":match",
        ":types",
        "//src/sim:backtrace",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_test(
    name = "trace_test",
    srcs = [
        "trace.test.cc",
    ],
    deps = [
        ":named",
        ":trace",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_library(
    name = "atomicio",
    srcs = [
        "atomicio.cc",
    ],
    hdrs = [
        "atomicio.hh",
    ],
)

gem5_cc_test(
    name = "atomicio_test",
    srcs = [
        "atomicio.test.cc",
    ],
    deps = [
        ":atomicio",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "match",
    srcs = [
        "match.cc",
    ],
    hdrs = [
        "match.hh",
    ],
    deps = [
        ":logging",
    ],
)

gem5_cc_test(
    name = "match_test",
    srcs = [
        "match.test.cc",
    ],
    deps = [
        ":match",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "cast",
    hdrs = [
        "cast.hh",
    ],
    deps = [
        ":logging",
    ],
)

gem5_cc_library(
    name = "type_traits",
    hdrs = [
        "type_traits.hh",
    ],
    deps = [
        ":compiler",
    ],
)

gem5_cc_library(
    name = "inifile",
    srcs = [
        "inifile.cc",
    ],
    hdrs = [
        "inifile.hh",
    ],
    deps = [
        ":logging",
    ],
)

gem5_cc_test(
    name = "inifile_test",
    srcs = [
        "inifile.test.cc",
    ],
    deps = [
        ":inifile",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "debug",
    srcs = [
        "debug.cc",
    ],
    hdrs = [
        "debug.hh",
    ],
    deps = [
        ":compiler",
        ":cprintf",
        ":logging",
    ],
)

gem5_cc_test(
    name = "debug_test",
    srcs = [
        "debug.test.cc",
    ],
    deps = [
        ":debug",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_library(
    name = "types",
    srcs = [
        "types.cc",
    ],
    hdrs = [
        "types.hh",
    ],
    copts = [
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
)

gem5_cc_test(
    name = "types_test",
    srcs = [
        "types.test.cc",
    ],
    deps = [
        ":types",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "cprintf",
    srcs = [
        "cprintf.cc",
    ],
    hdrs = [
        "cprintf.hh",
        "cprintf_formats.hh",
    ],
    deps = [
        ":compiler",
        ":stl_helpers",
        ":type_traits",
    ],
)

gem5_cc_test(
    name = "cprintf_test",
    srcs = [
        "cprintf.test.cc",
    ],
    deps = [
        ":cprintf",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_test(
    name = "named_test",
    srcs = [
        "named.test.cc",
    ],
    deps = [
        ":named",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

# Circular dependency statistics -> root_hh -> statistics
gem5_cc_library(
    name = "statistics_hh",
    hdrs = [
        "statistics.hh",
    ],
    deps = [
        ":cast",
        ":compiler",
        ":cprintf",
        ":intmath",
        ":logging",
        ":time",
        ":types",
        "//src/base/stats:group",
        "//src/base/stats:info",
        "//src/base/stats:output",
        "//src/base/stats:storage",
        "//src/base/stats:types",
        "//src/base/stats:units",
    ],
)

gem5_cc_library(
    name = "statistics",
    srcs = [
        "statistics.cc",
    ],
    hdrs = [
        "statistics.hh",
    ],
    deps = [
        ":callback",
        ":cast",
        ":compiler",
        ":cprintf",
        ":intmath",
        ":logging",
        ":time",
        ":types",
        "//src/base/stats:group",
        "//src/base/stats:info",
        "//src/base/stats:output",
        "//src/base/stats:storage",
        "//src/base/stats:types",
        "//src/base/stats:units",
        "//src/sim:root",
    ],
)

gem5_cc_library(
    name = "intmath",
    hdrs = [
        "intmath.hh",
    ],
    deps = [
        ":bitfield",
    ],
)

gem5_cc_test(
    name = "intmath_test",
    srcs = [
        "intmath.test.cc",
    ],
    deps = [
        ":intmath",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "bitfield",
    srcs = [
        "bitfield.cc",
    ],
    hdrs = [
        "bitfield.hh",
    ],
    deps = [
    ],
)

gem5_cc_test(
    name = "bitfield_test",
    srcs = [
        "bitfield.test.cc",
    ],
    deps = [
        ":bitfield",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "callback",
    hdrs = [
        "callback.hh",
    ],
    deps = [
    ],
)

gem5_cc_library(
    name = "time",
    srcs = [
        "time.cc",
    ],
    hdrs = [
        "time.hh",
    ],
    copts = [
        "",
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
    deps = [
        ":logging",
        ":types",
        "//bazel/generated/config",
        "//src/sim:core",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "output",
    srcs = [
        "output.cc",
    ],
    hdrs = [
        "output.hh",
    ],
    deps = [
        ":compiler",
        ":logging",
        "//ext/iostream3:zfstream",
    ],
)

gem5_cc_library(
    name = "barrier",
    hdrs = [
        "barrier.hh",
    ],
    deps = [
    ],
)

gem5_cc_library(
    name = "random",
    srcs = [
        "random.cc",
    ],
    hdrs = [
        "random.hh",
    ],
    deps = [
        ":compiler",
        ":logging",
        ":types",
    ],
)

gem5_cc_library(
    name = "socket",
    srcs = [
        "socket.cc",
    ],
    hdrs = [
        "socket.hh",
    ],
    deps = [
        ":logging",
        ":named",
        ":output",
        ":types",
        "//src/sim:byteswap",
    ],
)

gem5_cc_test(
    name = "random_test",
    srcs = [
        "random.test.cc",
    ],
    deps = [
        ":random",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_library(
    name = "temperature",
    srcs = [
        "temperature.cc",
    ],
    hdrs = [
        "temperature.hh",
    ],
    deps = [
    ],
)

gem5_cc_test(
    name = "temperature_test",
    srcs = [
        "temperature.test.cc",
    ],
    deps = [
        ":temperature",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "extensible",
    hdrs = [
        "extensible.hh",
    ],
    deps = [
    ],
)

gem5_cc_test(
    name = "extensible_test",
    srcs = [
        "extensible.test.cc",
    ],
    deps = [
        ":extensible",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "amo",
    hdrs = [
        "amo.hh",
    ],
    deps = [
    ],
)

gem5_cc_test(
    name = "amo_test",
    srcs = [
        "amo.test.cc",
    ],
    deps = [
        ":amo",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "printable",
    hdrs = [
        "printable.hh",
    ],
    deps = [
    ],
)

gem5_cc_library(
    name = "chunk_generator",
    hdrs = [
        "chunk_generator.hh",
    ],
    deps = [
        ":intmath",
        ":types",
    ],
)

gem5_cc_test(
    name = "chunk_generator_test",
    srcs = [
        "chunk_generator.test.cc",
    ],
    deps = [
        ":chunk_generator",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "date",
    srcs = [
        "date.cc",
    ],
    deps = [
        ":compiler",
    ],
)

gem5_cc_library(
    name = "version",
    srcs = [
        "version.cc",
    ],
)

gem5_cc_library(
    name = "pollevent",
    srcs = [
        "pollevent.cc",
    ],
    hdrs = [
        "pollevent.hh",
    ],
    deps = [
        ":logging",
        ":types",
        "//src/sim:async",
        "//src/sim:eventq",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "refcnt",
    hdrs = [
        "refcnt.hh",
    ],
    deps = [
        ":compiler",
    ],
)

gem5_cc_test(
    name = "refcnt_test",
    srcs = [
        "refcnt.test.cc",
    ],
    deps = [
        ":refcnt",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "free_list",
    hdrs = [
        "free_list.hh",
    ],
    deps = [
        ":logging",
    ],
)

gem5_cc_test(
    name = "free_list_test",
    srcs = [
        "free_list.test.cc",
    ],
    deps = [
        ":free_list",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_library(
    name = "remote_gdb",
    srcs = [
        "remote_gdb.cc",
    ],
    hdrs = [
        "remote_gdb.hh",
    ],
    copts = [
        "",
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
    deps = [
        ":cprintf",
        ":debug_GDBAll",
        ":intmath",
        ":logging",
        ":pollevent",
        ":socket",
        ":trace",
        ":types",
        "//ext/gdbremote:signals",
        "//src/arch/generic:pcstate",
        "//src/cpu:base",
        "//src/cpu:pc_event",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context",
        "//src/mem:port",
        "//src/mem:port_proxy",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
        "//src/sim:debug",
        "//src/sim:debug_Event",
        "//src/sim:eventq",
        "//src/sim:full_system",
        "//src/sim:process",
        "//src/sim:sim_events",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "sat_counter",
    hdrs = [
        "sat_counter.hh",
    ],
    deps = [
        ":logging",
        ":types",
    ],
)

gem5_cc_test(
    name = "sat_counter_test",
    srcs = [
        "sat_counter.test.cc",
    ],
    deps = [
        ":sat_counter",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_library(
    name = "circular_queue",
    hdrs = [
        "circular_queue.hh",
    ],
    deps = [
    ],
)

gem5_cc_test(
    name = "circular_queue_test",
    srcs = [
        "circular_queue.test.cc",
    ],
    deps = [
        ":circular_queue",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "condcodes",
    hdrs = [
        "condcodes.hh",
    ],
    deps = [
        ":bitfield",
    ],
)

gem5_cc_test(
    name = "condcodes_test",
    srcs = [
        "condcodes.test.cc",
    ],
    deps = [
        ":condcodes",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "bitunion",
    hdrs = [
        "bitunion.hh",
    ],
    deps = [
        ":bitfield",
        ":compiler",
        "//src/sim:serialize_handlers",
    ],
)

gem5_cc_test(
    name = "bitunion_test",
    srcs = [
        "bitunion.test.cc",
    ],
    deps = [
        ":bitunion",
        ":cprintf",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "memoizer",
    hdrs = [
        "memoizer.hh",
    ],
)

gem5_cc_test(
    name = "memoizer_test",
    srcs = [
        "memoizer.test.cc",
    ],
    deps = [
        ":memoizer",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "circlebuf",
    hdrs = [
        "circlebuf.hh",
    ],
    deps = [
        ":logging",
        "//src/sim:serialize",
    ],
)

gem5_cc_test(
    name = "circlebuf_test",
    srcs = [
        "circlebuf.test.cc",
    ],
    deps = [
        ":circlebuf",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "coroutine",
    hdrs = [
        "coroutine.hh",
    ],
    deps = [
        ":compiler",
        ":fiber",
    ],
)

# TODO(jlowepower): Test fails due to heap leak in coroutine/fiber context switching.
# gem5_cc_test(
#     name = "coroutine_test",
#     srcs = [
#         "coroutine.test.cc",
#     ],
#     deps = [
#         ":coroutine",
#         "//ext/googletest:gtest_main",
#     ],
# )

gem5_cc_library(
    name = "fiber",
    srcs = [
        "fiber.cc",
    ],
    hdrs = [
        "fiber.hh",
    ],
    deps = [
        ":logging",
        "//bazel/generated/config",
    ],
)

gem5_cc_test(
    name = "fiber_test",
    srcs = [
        "fiber.test.cc",
    ],
    deps = [
        ":fiber",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "crc",
    hdrs = [
        "crc.hh",
    ],
    deps = [
        ":bitfield",
    ],
)

gem5_cc_library(
    name = "fenv",
    srcs = [
        "fenv.cc",
    ],
    hdrs = [
        "fenv.hh",
    ],
    deps = [
        ":bitfield",
        "//bazel/generated/config",
    ],
)

py_library(
    name = "Graphics_py",
    srcs = ["Graphics.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_ImageFormat",
    enum_module = "src.base.Graphics",
    enum_name = "ImageFormat",
    enum_py_lib = ":Graphics_py",
)

gem5_cc_library(
    name = "pixel",
    srcs = ["pixel.cc"],
    hdrs = ["pixel.hh"],
    deps = [
        ":bitfield",
        ":compiler",
        ":cprintf",
        ":logging",
        ":types",
        "//src/sim:byteswap",
    ],
)

gem5_cc_test(
    name = "pixel_test",
    srcs = ["pixel.test.cc"],
    deps = [
        ":pixel",
        "//src/base/gtest:gtest",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_library(
    name = "framebuffer",
    srcs = ["framebuffer.cc"],
    hdrs = ["framebuffer.hh"],
    deps = [
        ":compiler",
        ":cprintf",
        ":logging",
        ":pixel",
        ":types",
        "//src/base:bitfield",
        "//src/sim:serialize",
        "@zlib//:zlib",
    ],
)

gem5_cc_library(
    name = "imgwriter_hh",
    hdrs = ["imgwriter.hh"],
    deps = [
        ":compiler",
        ":enum_ImageFormat_hh",
        ":framebuffer",
    ],
)

# gem5_cc_library(
#     name = "pngwriter",
#     srcs = ["pngwriter.cc"],
#     hdrs = ["pngwriter.hh"],
#     deps = [
#         ":compiler",
#         ":framebuffer",
#         ":imgwriter_hh",
#         ":logging",
#     ],
# )

gem5_cc_library(
    name = "bmpwriter",
    srcs = ["bmpwriter.cc"],
    hdrs = ["bmpwriter.hh"],
    deps = [
        ":compiler",
        ":framebuffer",
        ":imgwriter_hh",
        ":logging",
    ],
)

gem5_cc_library(
    name = "imgwriter",
    srcs = [
        "imgwriter.cc",
    ],
    deps = [
        ":bmpwriter",
        ":compiler",
        ":enum_ImageFormat_hh",
        ":imgwriter_hh",
        ":logging",
        "//ext/iostream3:zfstream",
        "//bazel/generated/config",
        "//src/base:framebuffer",
    ],
)


# VNC targets moved to src/base/vnc/BUILD
