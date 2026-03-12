load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_debug_flag", "gem5_enum_generator", "gem5_pybind_library", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "sim_init",
    srcs = ["__init__.py"],
    deps = [
        ":ClockDomain_py",
        ":ClockedObject_py",
        ":DVFSHandler_py",
        ":InstTracer_py",
        ":PowerDomain_py",
        ":PowerState_py",
        ":Process_py",
        ":RedirectPath_py",
        ":Root_py",
        ":SubSystem_py",
        ":System_py",
        ":TickedObject_py",
        ":VoltageDomain_py",
        ":Workload_py",
        "//src/sim/guest_abi:guest_abi_init",
        "//src/sim/power:power_init",
        "//src/sim/probe:probe_init",
    ],
)

# ==============================================================================
# Debug Flags
# ==============================================================================
gem5_debug_flag(
    name = "debug_Checkpoint",
    flag = "Checkpoint",
)

gem5_debug_flag(
    name = "debug_Config",
    flag = "Config",
)

gem5_debug_flag(
    name = "debug_CxxConfig",
    flag = "CxxConfig",
)

gem5_debug_flag(
    name = "debug_Drain",
    flag = "Drain",
)

gem5_debug_flag(
    name = "debug_Event",
    flag = "Event",
)

gem5_debug_flag(
    name = "debug_ExternalSignal",
    flag = "ExternalSignal",
)

gem5_debug_flag(
    name = "debug_Flow",
    flag = "Flow",
)

gem5_debug_flag(
    name = "debug_IPI",
    flag = "IPI",
)

gem5_debug_flag(
    name = "debug_IPR",
    flag = "IPR",
)

gem5_debug_flag(
    name = "debug_Interrupt",
    flag = "Interrupt",
)

gem5_debug_flag(
    name = "debug_Loader",
    flag = "Loader",
)

gem5_debug_flag(
    name = "debug_PseudoInst",
    flag = "PseudoInst",
)

gem5_debug_flag(
    name = "debug_Stack",
    flag = "Stack",
)

gem5_debug_flag(
    name = "debug_SyscallBase",
    flag = "SyscallBase",
    included_by = [":debug_SyscallAll"],
)

gem5_debug_flag(
    name = "debug_SyscallVerbose",
    flag = "SyscallVerbose",
    included_by = [":debug_SyscallAll"],
)

gem5_debug_flag(
    name = "debug_TimeSync",
    flag = "TimeSync",
)

gem5_debug_flag(
    name = "debug_Thread",
    flag = "Thread",
)

gem5_debug_flag(
    name = "debug_Timer",
    flag = "Timer",
)

gem5_debug_flag(
    name = "debug_VtoPhys",
    flag = "VtoPhys",
)

gem5_debug_flag(
    name = "debug_WorkItems",
    flag = "WorkItems",
)

gem5_debug_flag(
    name = "debug_ClockDomain",
    flag = "ClockDomain",
)

gem5_debug_flag(
    name = "debug_VoltageDomain",
    flag = "VoltageDomain",
)

gem5_debug_flag(
    name = "debug_DVFS",
    flag = "DVFS",
)

gem5_debug_flag(
    name = "debug_Vma",
    flag = "Vma",
)

gem5_debug_flag(
    name = "debug_PowerDomain",
    flag = "PowerDomain",
)

gem5_debug_flag(
    name = "debug_EnteringEventQueue",
    flag = "EnteringEventQueue",
)

gem5_debug_flag(
    name = "debug_SyscallAll",
    components = [
        ":debug_SyscallBase",
        ":debug_SyscallVerbose",
    ],
    flag = "SyscallAll",
)

# ==============================================================================
# Enum Generators
# ==============================================================================
# ByteOrder is a special case. It's definition is in params, but it is the only
# enum that is specified there.
# TODO(jlowepower): Think about moving this.
gem5_enum_generator(
    name = "enum_ByteOrder",
    enum_module = "src.python.m5.params",
    enum_name = "ByteOrder",
    enum_py_lib = "//src/python/m5/params",
)

gem5_enum_generator(
    name = "enum_PwrState",
    enum_module = "src.sim.PowerState",
    enum_name = "PwrState",
    enum_py_lib = ":PowerState_py",
)

# ==============================================================================
# Core/Utility Libraries
# ==============================================================================

gem5_cc_library(
    name = "async",
    srcs = [
        "async.cc",
    ],
    hdrs = [
        "async.hh",
    ],
    deps = [
    ],
)

# Note: Exported to base/ for trace
# TODO(jlowepower): Move to base/
# TODO(jlowepower): Make this optional if not TRACING_ON (see src/sim/backtrace_none.cc)
gem5_cc_library(
    name = "backtrace",
    srcs = [
        "backtrace_glibc.cc",
    ],
    hdrs = [
        "backtrace.hh",
    ],
    deps = [
        "//src/base:atomicio",
    ],
)

gem5_cc_library(
    name = "bufval",
    srcs = [
        "bufval.cc",
    ],
    hdrs = [
        "bufval.hh",
    ],
    deps = [
        ":byteswap",
        ":enum_ByteOrder_hh",
        "//src/base:intmath",
        "//src/base:logging",
    ],
)

gem5_cc_library(
    name = "byteswap",
    hdrs = [
        "byteswap.hh",
    ],
    deps = [
        ":enum_ByteOrder_hh",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "core",
    srcs = [
        "core.cc",
    ],
    hdrs = [
        "core.hh",
    ],
    deps = [
        "//src/base:callback",
        "//src/base:compiler",
        "//src/base:cprintf",
        "//src/base:logging",
        "//src/base:output",
        "//src/base:types",
    ],
)

# Note: Exported to base/
gem5_cc_library(
    name = "cur_tick",
    srcs = [
        "cur_tick.cc",
    ],
    hdrs = [
        "cur_tick.hh",
    ],
    copts = [
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
    deps = [
        "//src/base:types",
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
        ":eventq",
        ":global_event",
        ":sim_events",
        ":sim_exit",
        "//src/base:debug",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "drain",
    srcs = [
        "drain.cc",
    ],
    hdrs = [
        "drain.hh",
    ],
    deps = [
        ":debug_Drain",
        ":sim_events",
        ":sim_exit",
        "//src/base:logging",
        "//src/base:named",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "drain_hh",
    hdrs = [
        "drain.hh",
    ],
    deps = [
    ],
)

gem5_cc_library(
    name = "eventq",
    srcs = [
        "eventq.cc",
    ],
    hdrs = [
        "eventq.hh",
    ],
    deps = [
        ":cur_tick",
        ":debug_Checkpoint",
        ":debug_Event",
        ":serialize",
        "//src/base:debug",
        "//src/base:flags",
        "//src/base:logging",
        "//src/base:named",
        "//src/base:trace",
        "//src/base:type_traits",
        "//src/base:types",
        "//src/base:uncontended_mutex",
        "//src/cpu:smt",
    ],
)

gem5_cc_library(
    name = "full_system",
    hdrs = [
        "full_system.hh",
    ],
    deps = [
    ],
)

gem5_cc_library(
    name = "global_event",
    srcs = [
        "global_event.cc",
    ],
    hdrs = [
        "global_event.hh",
    ],
    deps = [
        ":cur_tick",
        ":eventq",
        "//src/base:barrier",
    ],
)

gem5_cc_library(
    name = "globals",
    srcs = [
        "globals.cc",
    ],
    hdrs = [
        "globals.hh",
    ],
    deps = [
        ":cur_tick",
        ":serialize",
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "init_signals",
    srcs = [
        "init_signals.cc",
    ],
    hdrs = [
        "init_signals.hh",
    ],
    deps = [
        ":async",
        ":backtrace",
        ":debug_ExternalSignal",
        ":eventq",
        ":sim_exit",
        "//src/base:atomicio",
        "//src/base:cprintf",
        "//src/base:logging",
    ],
)

gem5_cc_library(
    name = "linear_solver",
    srcs = [
        "linear_solver.cc",
    ],
    hdrs = [
        "linear_solver.hh",
    ],
    deps = [
    ],
)

gem5_cc_library(
    name = "port",
    srcs = [
        "port.cc",
    ],
    hdrs = [
        "port.hh",
    ],
    deps = [
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "serialize",
    srcs = [
        "serialize.cc",
    ],
    hdrs = [
        "serialize.hh",
    ],
    deps = [
        ":debug_Checkpoint",
        ":serialize_handlers",
        "//src/base:inifile",
        "//src/base:logging",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "serialize_handlers",
    hdrs = [
        "serialize_handlers.hh",
    ],
    deps = [
        "//src/base:logging",
    ],
)

gem5_cc_library(
    name = "serialize_handlers_hh",
    hdrs = [
        "serialize_handlers.hh",
    ],
    deps = [
        "//src/base:logging",
    ],
)

gem5_cc_library(
    name = "redirect_path_hh",
    hdrs = [
        "redirect_path.hh",
    ],
    deps = [
        ":redirect_path_sim_object",
        ":sim_object_hh",
    ],
)

gem5_cc_library(
    name = "redirect_path",
    srcs = [
        "redirect_path.cc",
    ],
    hdrs = [
        "redirect_path.hh",
    ],
    deps = [
        ":redirect_path_hh",
        ":redirect_path_sim_object",
        ":sim_object",
        "//src/base:logging",
    ],
)

gem5_sim_object(
    name = "redirect_path_sim_object",
    extra_param_deps = [
        ":sim_object_sim_object",
        ":sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":redirect_path",
    sim_object_module = "src.sim.RedirectPath",
    sim_object_name = "RedirectPath",
    sim_object_py_lib = ":RedirectPath_py",
)

gem5_cc_library(
    name = "sim_events",
    srcs = [
        "sim_events.cc",
    ],
    hdrs = [
        "sim_events.hh",
    ],
    deps = [
        ":eventq",
        ":global_event",
        ":serialize",
        ":sim_exit",
        ":stats",
        "//src/base:callback",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "sim_exit",
    hdrs = [
        "sim_exit.hh",
    ],
    deps = [
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "simulate",
    srcs = [
        "simulate.cc",
    ],
    hdrs = [
        "simulate.hh",
    ],
    deps = [
        ":async",
        ":debug_EnteringEventQueue",
        ":eventq",
        ":init_signals",
        ":sim_events",
        ":sim_exit",
        ":stat_control",
        "//src/base:logging",
        "//src/base:pollevent",
        "//src/base:trace",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "stat_control",
    srcs = [
        "stat_control.cc",
    ],
    hdrs = [
        "stat_control.hh",
    ],
    deps = [
        ":cur_tick",
        ":global_event",
        "//src/base:callback",
        "//src/base:compiler",
        "//src/base:statistics_hh",
        "//src/base:time",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "stat_register",
    srcs = [
        "stat_register.cc",
    ],
    hdrs = [
        "stat_register.hh",
    ],
    deps = [
        "//src/base:compiler",
        "//src/base:statistics_hh",
    ],
)

gem5_cc_library(
    name = "stats",
    srcs = [
        "stats.cc",
    ],
    hdrs = [
        "stats.hh",
    ],
    deps = [
        ":root_hh",
        "//src/base:statistics_hh",
    ],
)

gem5_cc_library(
    name = "system_hh",
    hdrs = ["system.hh"],
    deps = [
        "enum_MemoryMode_hh",
        ":futex_map_hh",
        ":redirect_path_hh",
        ":se_signal",
        ":sim_object",
        ":system_sim_object",
        ":workload_hh",
        "//src/base:statistics",
        "//src/base/loader:memory_image_hh",
        "//src/base/loader:symtab_hh",
        "//src/cpu:pc_event_hh",
        "//src/mem:mem_requestor",
        "//src/mem:physical_hh",
        "//src/mem:port_hh",
        "//src/mem:port_proxy",
    ],
)

gem5_cc_library(
    name = "system",
    srcs = ["system.cc"],
    hdrs = ["system.hh"],
    deps = [
        "enum_MemoryMode_hh",
        ":byteswap",
        ":debug",
        ":debug_WorkItems",
        ":futex_map_hh",
        ":redirect_path",
        ":redirect_path_hh",
        ":se_signal",
        ":serialize_handlers",
        ":sim_object",
        ":system_hh",
        ":system_sim_object",
        ":workload_hh",
        "//src/base:compiler",
        "//src/base:cprintf",
        "//src/base:debug",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base/loader:debug_Loader",
        "//src/base/loader:memory_image",
        "//src/base/loader:object_file",
        "//src/base/loader:symtab_hh",
        "//src/cpu:base",
        "//src/cpu:debug_Quiesce",
        "//src/cpu:pc_event_hh",
        "//src/cpu:thread_context",
        "//src/mem:abstract_mem_hh",
        "//src/mem:mem_requestor",
        "//src/mem:physical",
        "//src/mem:port_hh",
        "//src/mem:port_proxy",
    ],
)

gem5_pybind_library(
    name = "init",
    srcs = [
        "init.cc",
    ],
    hdrs = [
        "init.hh",
    ],
    deps = [
        "//src/base:cprintf",
        "//src/python:pybind_init",
        "//src/python/pybind11:debug",
        "//src/python/pybind11:event",
        "//src/python/pybind11:gem5_pybind11_core",
        "//src/python/pybind11:port",
        "//src/python/pybind11:pybind",
    ],
)

# ==============================================================================
# ClockDomain
# ==============================================================================

py_library(
    name = "ClockDomain_py",
    srcs = ["ClockDomain.py"],
    deps = [
        ":VoltageDomain_py",
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "clock_domain_hh",
    hdrs = ["clock_domain.hh"],
    deps = [
        ":clock_domain_sim_object",
        ":derived_clock_domain_sim_object",
        ":sim_object_hh",
        ":src_clock_domain_sim_object",
        "//src/base:statistics_hh",
    ],
)

gem5_cc_library(
    name = "clock_domain",
    srcs = ["clock_domain.cc"],
    hdrs = ["clock_domain.hh"],
    deps = [
        ":clock_domain_hh",
        ":clock_domain_sim_object",
        ":clocked_object_hh",
        ":debug_ClockDomain",
        ":derived_clock_domain_sim_object",
        ":serialize",
        ":sim_object_hh",
        ":src_clock_domain_sim_object",
        ":voltage_domain",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:statistics_hh",
        "//src/base:trace",
    ],
)

# Generator for ClockDomain
gem5_sim_object(
    name = "clock_domain_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":sim_object_sim_object",
    ],
    sim_object_header = ":clock_domain",
    sim_object_module = "src.sim.ClockDomain",
    sim_object_name = "ClockDomain",
    sim_object_py_lib = ":ClockDomain_py",
)

# Generator for DerivedClockDomain
gem5_sim_object(
    name = "derived_clock_domain_sim_object",
    extra_deps = [
        ":sim_object_hh",
    ],
    extra_param_deps = [
        ":clock_domain_sim_object",
        ":sim_object_sim_object",
        ":voltage_domain_sim_object",
    ],
    extra_tool_deps = [":VoltageDomain_py"],
    sim_object_header = ":clock_domain",
    sim_object_module = "src.sim.ClockDomain",
    sim_object_name = "DerivedClockDomain",
    sim_object_py_lib = ":ClockDomain_py",
)

# Generator for SrcClockDomain
gem5_sim_object(
    name = "src_clock_domain_sim_object",
    extra_deps = [
        ":sim_object_hh",
        ":voltage_domain_hh",
    ],
    extra_param_deps = [
        ":voltage_domain_sim_object",
        ":clock_domain_sim_object",
        ":sim_object_sim_object",
    ],
    extra_tool_deps = ["//src/python/m5/objects:VoltageDomain"],
    sim_object_header = ":clock_domain",
    sim_object_module = "src.sim.ClockDomain",
    sim_object_name = "SrcClockDomain",
    sim_object_py_lib = ":ClockDomain_py",
)

# ==============================================================================
# ClockedObject
# ==============================================================================

py_library(
    name = "ClockedObject_py",
    srcs = ["ClockedObject.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:PowerModel",
        "//src/python/m5/objects:PowerState",
        "//src/python/m5/objects:SubSystem",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "clocked_object_hh",
    hdrs = ["clocked_object.hh"],
    deps = [
        ":clock_domain_hh",
        ":clocked_object_sim_object",
        ":core",
        ":power_state_hh",
        ":sim_object_hh",
    ],
)

gem5_cc_library(
    name = "clocked_object",
    srcs = ["clocked_object.cc"],
    hdrs = ["clocked_object.hh"],
    deps = [
        ":clock_domain",
        ":clocked_object_sim_object",
        ":core",
        ":power_state",
        ":sim_object_hh",
        "//src/base:logging",
        "//src/sim/power:power_model",
    ],
)

# generator for ClockedObject
gem5_sim_object(
    name = "clocked_object_sim_object",
    extra_deps = [
        ":sim_object_hh",
        ":clock_domain_hh",
        "//src/sim/power:power_model_hh",
        ":power_state_hh",
    ],
    extra_param_deps = [
        ":clock_domain_sim_object",
        ":sim_object_sim_object",
        "//src/sim/power:power_model_sim_object",
        "//src/sim:power_state_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PowerState",
        "//src/python/m5/objects:PowerModel",
        "//src/python/m5/objects:ClockDomain",
    ],
    sim_object_header = ":clocked_object",
    sim_object_module = "src.sim.ClockedObject",
    sim_object_name = "ClockedObject",
    sim_object_py_lib = ":ClockedObject_py",
)

# ==============================================================================
# TickedObject
# ==============================================================================

py_library(
    name = "TickedObject_py",
    srcs = ["TickedObject.py"],
    deps = [
        ":ClockedObject_py",
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "ticked_object_hh",
    hdrs = ["ticked_object.hh"],
    deps = [
        ":clocked_object_hh",
        ":ticked_object_sim_object",
    ],
)

gem5_cc_library(
    name = "ticked_object",
    srcs = ["ticked_object.cc"],
    hdrs = ["ticked_object.hh"],
    deps = [
        ":clocked_object",
        ":ticked_object_sim_object",
    ],
)

# generator for TickedObject
gem5_sim_object(
    name = "ticked_object_sim_object",
    extra_deps = [
        ":clocked_object_hh",
    ],
    extra_param_deps = [
        ":clocked_object_sim_object",
    ],
    sim_object_header = ":ticked_object",
    sim_object_module = "src.sim.TickedObject",
    sim_object_name = "TickedObject",
    sim_object_py_lib = ":TickedObject_py",
)

# ==============================================================================
# PowerDomain
# ==============================================================================

py_library(
    name = "PowerDomain_py",
    srcs = ["PowerDomain.py"],
    deps = [
        ":PowerState_py",
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "power_domain_hh",
    hdrs = ["power_domain.hh"],
    deps = [
        ":clocked_object_hh",
        ":power_domain_sim_object",
        ":power_state_hh",
        "//src/base:statistics_hh",
    ],
)

gem5_cc_library(
    name = "power_domain",
    srcs = ["power_domain.cc"],
    hdrs = ["power_domain.hh"],
    deps = [
        ":clocked_object_hh",
        ":debug_PowerDomain",
        ":power_domain_hh",
        ":power_domain_sim_object",
        ":power_state_hh",
        "//src/base:statistics",
        "//src/base:trace",
    ],
)

# Generator for PowerDomain
gem5_sim_object(
    name = "power_domain_sim_object",
    extra_deps = [
        ":sim_object_hh",
    ],
    extra_param_deps = [
        "//src/base:temperature",
        ":enum_PwrState_hh",
        ":power_state_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PowerState",
    ],
    sim_object_header = ":power_domain",
    sim_object_module = "src.sim.PowerDomain",
    sim_object_name = "PowerDomain",
    sim_object_py_lib = ":PowerDomain_py",
)

# ==============================================================================
# PowerState
# ==============================================================================

py_library(
    name = "PowerState_py",
    srcs = ["PowerState.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "power_state_hh",
    hdrs = ["power_state.hh"],
    deps = [
        ":enum_PwrState_hh",
        ":power_state_sim_object",
        ":sim_object_hh",
        "//src/base:callback",
        "//src/base:statistics_hh",
    ],
)

gem5_cc_library(
    name = "power_state",
    srcs = ["power_state.cc"],
    hdrs = ["power_state.hh"],
    deps = [
        ":cur_tick",
        ":debug_PowerDomain",
        ":enum_PwrState_hh",
        ":power_domain",
        ":power_domain_sim_object",
        ":power_state_hh",
        ":power_state_sim_object",
        ":serialize",
        ":sim_object_hh",
        "//src/base:callback",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:trace",
    ],
)

# SimObject generator for PowerState
gem5_sim_object(
    name = "power_state_sim_object",
    extra_deps = [
        ":sim_object_hh",
    ],
    extra_param_deps = [
        ":enum_PwrState_hh",
        ":sim_object_sim_object",
    ],
    sim_object_header = ":power_state",
    sim_object_module = "src.sim.PowerState",
    sim_object_name = "PowerState",
    sim_object_py_lib = ":PowerState_py",
)

# ==============================================================================
# Root
# ==============================================================================

py_library(
    name = "Root_py",
    srcs = ["Root.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
        "//src/python/m5/util",
    ],
)

gem5_cc_library(
    name = "root_hh",
    hdrs = [
        "root.hh",
    ],
    deps = [
        ":eventq",
        ":globals",
        ":root_sim_object",
        ":sim_object_hh",
        "//src/base:statistics_hh",
        "//src/base:time",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "root",
    srcs = [
        "root.cc",
    ],
    hdrs = [
        "root.hh",
    ],
    deps = [
        ":core",
        ":cur_tick",
        ":debug_TimeSync",
        ":eventq",
        ":full_system",
        ":globals",
        ":root_sim_object",
        ":sim_object",
        "//bazel/generated:tags",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:socket",
        "//src/base:statistics_hh",
        "//src/base:time",
        "//src/base:trace",
        "//src/base:types",
    ],
)

gem5_sim_object(
    name = "root_sim_object",
    extra_deps = [
        ":sim_object_hh",
    ],
    extra_param_deps = [
        ":sim_object_sim_object",
    ],
    sim_object_header = ":root_hh",
    sim_object_module = "src.sim.Root",
    sim_object_name = "Root",
    sim_object_py_lib = ":Root_py",
)

# ==============================================================================
# SimObject
# ==============================================================================

gem5_cc_library(
    name = "sim_object_hh",
    hdrs = [
        "sim_object.hh",
    ],
    deps = [
        ":drain_hh",
        ":eventq",
        ":port",
        ":serialize",
        ":param_sim_object_hh",
        "//src/base:named",
        "//src/base/stats:group",
    ],
)

gem5_cc_library(
    name = "sim_object",
    srcs = [
        "sim_object.cc",
    ],
    hdrs = [
        "sim_object.hh",
    ],
    deps = [
        ":debug_Checkpoint",
        ":drain",
        ":eventq",
        ":port",
        ":serialize",
        ":sim_object_hh",
        ":sim_object_sim_object",
        "//src/base:logging",
        "//src/base:match",
        "//src/base:named",
        "//src/base:trace",
        "//src/base/stats:group",
        "//src/sim/probe",
    ],
)

# Note: SimObject is a weird one since the python module is in src/python/m5, not with with cc/hh
# files.
gem5_sim_object(
    name = "sim_object_sim_object",
    sim_object_header = ":sim_object_hh",
    sim_object_module = "src.python.m5.SimObject",
    sim_object_name = "SimObject",
    sim_object_py_lib = "//src/python/m5:SimObject",
)

# ==============================================================================
# SubSystem
# ==============================================================================

py_library(
    name = "SubSystem_py",
    srcs = ["SubSystem.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

# Circular thermal_domain -> sub_system -> thermal_domain
gem5_cc_library(
    name = "sub_system_hh",
    hdrs = ["sub_system.hh"],
    deps = [
        ":sim_object_hh",
        ":sub_system_sim_object",
    ],
)

gem5_cc_library(
    name = "sub_system",
    srcs = ["sub_system.cc"],
    hdrs = ["sub_system.hh"],
    deps = [
        ":sim_object_hh",
        ":sub_system_sim_object",
        "//src/sim/power:power_model",
        "//src/sim/power:thermal_domain",
    ],
)

# Generator for SubSystem
gem5_sim_object(
    name = "sub_system_sim_object",
    extra_deps = [
        ":sim_object_hh",
        "//src/sim/power:thermal_domain",
    ],
    extra_param_deps = [
        ":sim_object_sim_object",
        "//src/base:temperature",
        "//src/sim/power:thermal_domain_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ThermalDomain",
    ],
    sim_object_header = ":sub_system",
    sim_object_module = "src.sim.SubSystem",
    sim_object_name = "SubSystem",
    sim_object_py_lib = ":SubSystem_py",
)

# ==============================================================================
# VoltageDomain
# ==============================================================================

py_library(
    name = "VoltageDomain_py",
    srcs = ["VoltageDomain.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "voltage_domain_hh",
    hdrs = ["voltage_domain.hh"],
    deps = [
        ":clock_domain_hh",
        ":sim_object_hh",
        ":voltage_domain_sim_object",
        "//src/base:statistics",
    ],
)

gem5_cc_library(
    name = "voltage_domain",
    srcs = ["voltage_domain.cc"],
    hdrs = ["voltage_domain.hh"],
    deps = [
        ":clock_domain_hh",
        ":debug_VoltageDomain",  # Debug flag
        ":serialize",
        ":sim_object_hh",
        ":voltage_domain_sim_object",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:statistics_hh",
        "//src/base:trace",
    ],
)

# Generator for VoltageDomain
gem5_sim_object(
    name = "voltage_domain_sim_object",
    extra_deps = [
        ":sim_object_hh",
    ],
    extra_param_deps = [
        ":sim_object_sim_object",
    ],
    sim_object_header = ":voltage_domain",
    sim_object_module = "src.sim.VoltageDomain",
    sim_object_name = "VoltageDomain",
    sim_object_py_lib = ":VoltageDomain_py",
)

# ==============================================================================
# C++ Tests
# ==============================================================================

gem5_cc_test(
    name = "bufval_test",
    srcs = [
        "bufval.test.cc",
    ],
    deps = [
        ":bufval",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_test(
    name = "byteswap_test",
    srcs = [
        "byteswap.test.cc",
    ],
    deps = [
        ":byteswap",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_test(
    name = "globals_test",
    srcs = [
        "globals.test.cc",
    ],
    deps = [
        ":globals",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_test(
    name = "port_test",
    srcs = [
        "port.test.cc",
    ],
    deps = [
        ":port",
        "//ext/googletest:gtest_main",
        "//ext/googletest:gtest",
    ],
)

gem5_cc_library(
    name = "faults_hh",
    hdrs = ["faults.hh"],
    deps = [
        ":stats",
        "//src/base:types",
        "//src/cpu:null_static_inst",
        "//src/cpu:static_inst_fwd",
        "//src/mem:htm",
    ],
)

gem5_cc_library(
    name = "faults",
    srcs = ["faults.cc"],
    hdrs = ["faults.hh"],
    deps = [
        ":full_system",
        ":process",
        ":stats",
        ":system_hh",
        "//src/arch:debug_Faults",
        "//src/arch/generic:decoder",
        "//src/base:logging",
        "//src/base:types",
        "//src/cpu:base_hh",
        "//src/cpu:null_static_inst",
        "//src/cpu:static_inst_fwd",
        "//src/cpu:thread_context",
        "//src/mem:htm",
        "//src/mem:page_table",
    ],
)

# TODO(jlowepower): I think the test is not quite right, especially for EXPECT_FALSE
# gem5_cc_test(
#     name = "serialize_handlers_test",
#     srcs = [
#         "serialize_handlers.test.cc",
#     ],
#     copts = ["-Ithird_party.gem5.develop.src.],
#     deps = [
#         ":serialize_handlers",
#         "//ext/googletest:gtest_main",
#     ],
# )

gem5_cc_library(
    name = "kernel_workload",
    srcs = [
        "kernel_workload.cc",
    ],
    hdrs = [
        "kernel_workload.hh",
    ],
    deps = [
        ":kernel_workload_sim_object",
        ":system",
        ":workload",
        "//src/base:types",
        "//src/base/loader:debug_Loader",
        "//src/base/loader:object_file",
        "//src/base/loader:symtab",
    ],
)

py_library(
    name = "System_py",
    srcs = ["System.py"],
    deps = [
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/objects:DVFSHandler",
        "//src/python/m5/objects:RedirectPath",
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:SimpleMemory",
        "//src/python/m5/objects:ThermalModel",
        "//src/python/m5/objects:Workload",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "system_sim_object",
    extra_deps = [
        ":sim_object_hh",
        ":redirect_path",
        "//src/sim/power:thermal_model",
        "//src/mem:abstract_mem",
        "//src/sim:workload",
        ":sub_system",
    ],
    extra_param_deps = [
        ":sim_object_sim_object",
        "//src/mem:abstract_memory_sim_object",
        "//src/base:addr_range",
        "enum_MemoryMode_hh",
        ":redirect_path_sim_object",
        "//src/sim/power:thermal_model_sim_object",
        ":workload_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/objects:Workload",
        "//src/python/m5/objects:RedirectPath",
        "//src/python/m5/objects:PowerState",
        "//src/python/m5/objects:ThermalDomain",
    ],
    sim_object_header = ":system",
    sim_object_module = "src.sim.System",
    sim_object_name = "System",
    sim_object_py_lib = ":System_py",
)

py_library(
    name = "DVFSHandler_py",
    srcs = ["DVFSHandler.py"],
    deps = [
        "//src/python/m5/objects:ClockDomain",
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "dvfs_handler_sim_object",
    extra_deps = [
        ":sim_object_hh",
        ":clock_domain",
    ],
    extra_param_deps = [
        ":sim_object_sim_object",
        ":src_clock_domain_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:ClockDomain",
    ],
    sim_object_header = ":dvfs_handler",
    sim_object_module = "src.sim.DVFSHandler",
    sim_object_name = "DVFSHandler",
    sim_object_py_lib = ":DVFSHandler_py",
)

gem5_cc_library(
    name = "dvfs_handler",
    srcs = ["dvfs_handler.cc"],
    hdrs = ["dvfs_handler.hh"],
    deps = [
        ":clock_domain",
        ":debug_DVFS",
        ":dvfs_handler_sim_object",
        ":eventq",
        ":serialize",
        ":sim_object",
        ":stat_control",
        ":voltage_domain",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
    ],
)

py_library(
    name = "RedirectPath_py",
    srcs = ["RedirectPath.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "Workload_py",
    srcs = ["Workload.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:SimpleMemory",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_MemoryMode",
    enum_module = "src.sim.System",
    enum_name = "MemoryMode",
    enum_py_lib = ":System_py",
)

gem5_enum_generator(
    name = "enum_KernelPanicOopsBehaviour",
    enum_module = "src.sim.Workload",
    enum_name = "KernelPanicOopsBehaviour",
    enum_py_lib = ":Workload_py",
    extra_tool_deps = [
        "//src/python/m5/objects:SimpleMemory",
        "//src/python/m5/objects:SimObject",
    ],
)

gem5_cc_library(
    name = "futex_map_hh",
    hdrs = ["futex_map.hh"],
    deps = [
        "//src/cpu:thread_context_hh",
    ],
)

gem5_cc_library(
    name = "futex_map",
    srcs = ["futex_map.cc"],
    hdrs = ["futex_map.hh"],
    deps = [
        ":futex_map_hh",
        "//src/cpu:thread_context",
    ],
)

gem5_cc_library(
    name = "se_signal",
    srcs = ["se_signal.cc"],
    hdrs = ["se_signal.hh"],
    deps = [
    ],
)

gem5_cc_library(
    name = "workload_hh",
    hdrs = ["workload.hh"],
    deps = [
        ":enum_ByteOrder_hh",
        ":sim_object_hh",
        ":stats_hh",
        ":stub_workload_sim_object",
        ":workload_sim_object",
        "//ext/gdbremote:signals",
        "//src/base/loader:object_file_hh",
        "//src/base/loader:symtab_hh",
    ],
)

gem5_cc_library(
    name = "workload",
    srcs = ["workload.cc"],
    hdrs = ["workload.hh"],
    deps = [
        ":debug",
        ":enum_ByteOrder_hh",
        ":sim_object_hh",
        ":stats_hh",
        ":stub_workload_sim_object",
        ":system",
        ":workload_hh",
        ":workload_sim_object",
        "//ext/gdbremote:signals",
        "//src/base:remote_gdb",
        "//src/base/loader:object_file",
        "//src/base/loader:symtab",
        "//src/cpu:thread_context",
    ],
)

# Generator for Workload
gem5_sim_object(
    name = "workload_sim_object",
    extra_deps = [
        ":sim_object_hh",
    ],
    extra_param_deps = [
        ":sim_object_sim_object",
        "//src/base:socket",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":workload",
    sim_object_module = "src.sim.Workload",
    sim_object_name = "Workload",
    sim_object_py_lib = ":Workload_py",
)

# Generator for StubWorkload
gem5_sim_object(
    name = "stub_workload_sim_object",
    extra_deps = [
        ":sim_object_hh",
    ],
    extra_param_deps = [
        ":workload_sim_object",
        "enum_ByteOrder_hh",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":workload",
    sim_object_module = "src.sim.Workload",
    sim_object_name = "StubWorkload",
    sim_object_py_lib = ":Workload_py",
)

# Generator for SEWorkload
gem5_sim_object(
    name = "se_workload_sim_object",
    extra_param_deps = [
        ":sim_object_sim_object",
        ":workload_sim_object",
        ":sim_object_hh",
        "//src/python/pybind11:object_file",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":se_workload",
    sim_object_module = "src.sim.Workload",
    sim_object_name = "SEWorkload",
    sim_object_py_lib = ":Workload_py",
)

gem5_sim_object(
    name = "kernel_workload_sim_object",
    extra_param_deps = [
        ":sim_object_sim_object",
        ":workload_sim_object",
        ":sim_object_hh",
        ":enum_KernelPanicOopsBehaviour_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":kernel_workload",
    sim_object_module = "src.sim.Workload",
    sim_object_name = "KernelWorkload",
    sim_object_py_lib = ":Workload_py",
)

gem5_cc_library(
    name = "se_workload",
    srcs = ["se_workload.cc"],
    hdrs = ["se_workload.hh"],
    deps = [
        ":mem_pool",
        ":process_hh",
        ":se_workload_sim_object",
        ":system_hh",
        ":workload_hh",
        "//src/cpu:thread_context_hh",
    ],
)

gem5_cc_library(
    name = "mem_pool",
    srcs = ["mem_pool.cc"],
    hdrs = ["mem_pool.hh"],
    deps = [
        ":serialize",
        "//src/base:addr_range",
        "//src/base:free_list",
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "stats_hh",
    hdrs = ["stats.hh"],
    deps = [
        ":root_hh",
        "//src/base:statistics_hh",
    ],
)

gem5_cc_library(
    name = "signal",
    hdrs = ["signal.hh"],
    deps = [
        ":port",
        "//src/base:logging",
    ],
)

py_library(
    name = "InstTracer_py",
    srcs = ["InstTracer.py"],
    deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "inst_tracer_sim_object",
    extra_param_deps = [
        ":sim_object_sim_object",
        ":inst_disassembler_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":insttracer",
    sim_object_module = "src.sim.InstTracer",
    sim_object_name = "InstTracer",
    sim_object_py_lib = ":InstTracer_py",
)

gem5_sim_object(
    name = "inst_disassembler_sim_object",
    extra_param_deps = [
        ":sim_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":insttracer",
    sim_object_module = "src.sim.InstTracer",
    sim_object_name = "InstDisassembler",
    sim_object_py_lib = ":InstTracer_py",
)

gem5_cc_library(
    name = "insttracer",
    hdrs = ["insttracer.hh"],
    deps = [
        ":inst_tracer_sim_object",
        ":sim_object_hh",
        "//src/arch/generic:pcstate",
        "//src/base:types",
        "//src/cpu:inst_res",
        "//src/cpu:inst_seq",
        "//src/cpu:static_inst",
    ],
)

py_library(
    name = "Process_py",
    srcs = ["Process.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "process_sim_object",
    extra_param_deps = [
        ":sim_object_sim_object",
        ":system_sim_object",
        ":emulated_driver_sim_object",
        ":sim_object_hh",
        ":emul_driver",
        ":system_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":process",
    sim_object_module = "src.sim.Process",
    sim_object_name = "Process",
    sim_object_py_lib = ":Process_py",
)

gem5_sim_object(
    name = "emulated_driver_sim_object",
    extra_param_deps = [
        ":sim_object_hh",
        ":sim_object_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":emul_driver",
    sim_object_module = "src.sim.Process",
    sim_object_name = "EmulatedDriver",
    sim_object_py_lib = ":Process_py",
)

gem5_cc_library(
    name = "process",
    srcs = ["process.cc"],
    hdrs = ["process.hh"],
    deps = [
        ":emul_driver",
        ":fd_array",
        ":fd_entry",
        ":mem_state",
        ":process_sim_object",
        ":redirect_path_hh",
        ":se_workload",
        ":sim_object_hh",
        ":syscall_desc",
        ":system_hh",
        "//src/base:intmath",
        "//src/base:statistics",
        "//src/base:types",
        "//src/base/loader:memory_image",
        "//src/base/loader:object_file",
        "//src/base/loader:symtab_hh",
        "//src/cpu:thread_context",
        "//src/mem:page_table",
        "//src/mem:se_translating_port_proxy",
    ],
)

gem5_cc_library(
    name = "process_hh",
    hdrs = ["process.hh"],
    deps = [
        ":fd_array_hh",
        ":fd_entry",
        ":mem_state_hh",
        ":sim_object_hh",
        "//src/base:statistics",
        "//src/base:types",
        "//src/base/loader:memory_image",
    ],
)

gem5_cc_library(
    name = "fd_entry",
    srcs = ["fd_entry.cc"],
    hdrs = ["fd_entry.hh"],
    deps = [
        ":serialize",
    ],
)

gem5_cc_library(
    name = "fd_array_hh",
    hdrs = ["fd_array.hh"],
    deps = [
        ":fd_entry",
        ":serialize",
    ],
)

gem5_cc_library(
    name = "fd_array",
    srcs = ["fd_array.cc"],
    hdrs = ["fd_array.hh"],
    deps = [
        ":fd_array_hh",
        ":fd_entry",
        ":process_hh",
        ":process_sim_object",
        ":serialize",
        "//src/base:logging",
        "//src/base:output",
    ],
)

gem5_cc_library(
    name = "vma",
    srcs = ["vma.cc"],
    hdrs = ["vma.hh"],
    deps = [
        ":debug_Vma",
        "//src/base:addr_range",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:se_translating_port_proxy_hh",
    ],
)

gem5_cc_library(
    name = "syscall_debug_macros",
    hdrs = ["syscall_debug_macros.hh"],
    deps = [
        ":debug_SyscallAll",
        ":debug_SyscallBase",
        ":debug_SyscallVerbose",
        "//src/base:cprintf",
        "//src/cpu:base_hh",
        "//src/cpu:thread_context_hh",
    ],
)

gem5_cc_library(
    name = "mem_state_hh",
    hdrs = ["mem_state.hh"],
    deps = [
        ":debug_Vma",
        ":serialize",
        ":vma",
        "//src/mem:page_table_hh",
        "//src/mem:se_translating_port_proxy_hh",
    ],
)

gem5_cc_library(
    name = "mem_state",
    srcs = ["mem_state.cc"],
    hdrs = ["mem_state.hh"],
    deps = [
        ":debug_Vma",
        ":process_hh",
        ":serialize",
        ":syscall_debug_macros",
        ":system_hh",
        ":vma",
        "//src/arch/generic:mmu",
        "//src/mem:page_table",
        "//src/mem:se_translating_port_proxy",
    ],
)

gem5_cc_library(
    name = "emul_driver",
    hdrs = ["emul_driver.hh"],
    deps = [
        ":emulated_driver_sim_object",
        ":sim_object_hh",
    ],
)

gem5_cc_library(
    name = "syscall_desc",
    srcs = ["syscall_desc.cc"],
    hdrs = ["syscall_desc.hh"],
    deps = [
        ":eventq",
        ":guest_abi",
        ":process_hh",
        ":syscall_debug_macros",
        ":syscall_return",
        "//src/base:logging",
        "//src/base:types",
        "//src/cpu:thread_context_hh",
    ],
)

gem5_cc_library(
    name = "guest_abi",
    hdrs = ["guest_abi.hh"],
    deps = [
        "//src/base:types",
        "//src/sim/guest_abi",
    ],
)

gem5_cc_test(
    name = "guest_abi_test",
    srcs = ["guest_abi.test.cc"],
    deps = [
        ":guest_abi",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "syscall_return",
    hdrs = ["syscall_return.hh"],
    deps = [
    ],
)

gem5_pybind_library(
    name = "python_py",
    srcs = ["python.cc"],
    deps = [
        ":init",
        ":port",
    ],
)

gem5_cc_library(
    name = "pseudo_inst",
    srcs = ["pseudo_inst.cc"],
    hdrs = ["pseudo_inst.hh"],
    deps = [
        ":debug_Loader",
        ":debug_PseudoInst",
        ":debug_WorkItems",
        ":full_system",
        ":guest_abi",
        ":process_hh",
        ":serialize",
        ":sim_events",
        ":sim_exit",
        ":stat_control",
        ":stats_hh",
        ":system_hh",
        "//include:gem5_asm_generic",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:debug",
        "//src/base:logging",
        "//src/base:output",
        "//src/base:trace",
        "//src/base:types",
        "//src/cpu:base_cpu_sim_object",
        "//src/cpu:base_hh",
        "//src/cpu:debug_Quiesce",
        "//src/cpu:static_inst",
        "//src/cpu:thread_context_hh",
        "//src/dev/net:dist_iface",
        "//src/mem:se_translating_port_proxy",
        "//src/mem:translating_port_proxy",
    ],
)

gem5_cc_library(
    name = "proxy_ptr",
    hdrs = ["proxy_ptr.hh"],
    deps = [
        ":guest_abi",
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_test(
    name = "proxy_ptr_test",
    srcs = ["proxy_ptr.test.cc"],
    deps = [
        ":proxy_ptr",
        "//ext/googletest:gtest_main",
    ],
)

gem5_cc_library(
    name = "syscall_abi",
    hdrs = ["syscall_abi.hh"],
    deps = [
        ":guest_abi",
        ":pseudo_inst",
        ":syscall_return",
        "//src/base:bitfield",
        "//src/base:types",
        "//src/cpu:thread_context_hh",
    ],
)

gem5_cc_library(
    name = "syscall_emul",
    srcs = ["syscall_emul.cc"],
    hdrs = ["syscall_emul.hh"],
    deps = [
        ":byteswap",
        ":emul_driver",
        ":futex_map",
        ":guest_abi",
        ":process",
        ":process_sim_object",
        ":proxy_ptr",
        ":sim_exit",
        ":syscall_debug_macros",
        ":syscall_desc",
        ":syscall_emul_buf",
        ":syscall_return",
        ":system",
        "//src/arch/generic:tlb",
        "//src/base:chunk_generator",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:trace",
        "//src/base:types",
        "//src/base/loader:object_file",
        "//src/cpu:base_hh",
        "//src/cpu:thread_context",
        "//src/dev/net:dist_iface",
        "//src/kern/linux",
        "//src/mem:page_table",
        "//src/mem:se_translating_port_proxy",
    ],
)

gem5_cc_library(
    name = "syscall_emul_buf",
    hdrs = ["syscall_emul_buf.hh"],
    deps = [
        "//src/base:types",
        "//src/mem:se_translating_port_proxy",
    ],
)

gem5_cc_library(
    name = "aux_vector",
    hdrs = ["aux_vector.hh"],
    deps = [
        ":byteswap",
        "//src/base:compiler",
    ],
)

gem5_cc_library(
    name = "process_impl",
    hdrs = ["process_impl.hh"],
    deps = [
        "//src/mem:se_translating_port_proxy",
    ],
)
