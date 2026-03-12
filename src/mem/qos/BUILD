load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_QOS",
    flag = "QOS",
)

# --- Python SimObject Definitions ---

py_library(
    name = "QoSMemCtrl_py",
    srcs = ["QoSMemCtrl.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:MemInterface",
        "//src/python/m5/objects:QoSPolicy",
        "//src/python/m5/objects:QoSTurnaround",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_QoSQPolicy",
    enum_module = "src.mem.qos.QoSMemCtrl",
    enum_name = "QoSQPolicy",
    enum_py_lib = ":QoSMemCtrl_py",
)

gem5_sim_object(
    name = "qo_s_mem_ctrl_sim_object",
    extra_deps = [
        "//src/mem/qos:policy",
        "//src/mem/qos:turnaround_policy",
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        "//src/mem:mem_interface_sim_object",
        ":enum_QoSQPolicy_hh",
        "//src/sim:sim_object_hh",
        ":qo_s_policy_sim_object",
        ":qo_s_turnaround_policy_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:clocked_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:MemInterface",
        "//src/python/m5/objects:QoSPolicy",
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":mem_ctrl_qos_hh",
    sim_object_module = "src.mem.qos.QoSMemCtrl",
    sim_object_name = "QoSMemCtrl",
    sim_object_py_lib = ":QoSMemCtrl_py",
)

py_library(
    name = "QoSMemSinkCtrl_py",
    srcs = ["QoSMemSinkCtrl.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "qo_s_mem_sink_ctrl_sim_object",
    extra_param_deps = [
        "//src/mem:mem_ctrl_sim_object",
        "//src/sim:sim_object_hh",
        ":qo_s_mem_sink_interface_sim_object",
        ":qo_s_mem_ctrl_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/objects:QoSMemSinkInterface",
    ],
    sim_object_header = ":mem_sink_qos",
    sim_object_module = "src.mem.qos.QoSMemSinkCtrl",
    sim_object_name = "QoSMemSinkCtrl",
    sim_object_py_lib = ":QoSMemSinkCtrl_py",
)

py_library(
    name = "QoSMemSinkInterface_py",
    srcs = ["QoSMemSinkInterface.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:MemInterface",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "qo_s_mem_sink_interface_sim_object",
    extra_param_deps = [
        "//src/mem:mem_interface_sim_object",
        "//src/sim:sim_object_hh",
        "//src/mem:abstract_memory_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:MemInterface",
    ],
    sim_object_header = ":mem_sink_qos",
    sim_object_module = "src.mem.qos.QoSMemSinkInterface",
    sim_object_name = "QoSMemSinkInterface",
    sim_object_py_lib = ":QoSMemSinkInterface_py",
)

py_library(
    name = "QoSPolicy_py",
    srcs = ["QoSPolicy.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "qo_s_policy_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":policy",
    sim_object_module = "src.mem.qos.QoSPolicy",
    sim_object_name = "QoSPolicy",
    sim_object_py_lib = ":QoSPolicy_py",
)

gem5_sim_object(
    name = "qo_s_fixed_priority_policy_sim_object",
    extra_param_deps = [
        ":qo_s_policy_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:QoSPolicy",
    ],
    sim_object_header = ":policy_fixed_prio",
    sim_object_module = "src.mem.qos.QoSPolicy",
    sim_object_name = "QoSFixedPriorityPolicy",
    sim_object_py_lib = ":QoSPolicy_py",
)

gem5_sim_object(
    name = "qo_s_prop_fair_policy_sim_object",
    extra_param_deps = [
        ":qo_s_policy_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:QoSPolicy",
    ],
    sim_object_header = ":policy_pf",
    sim_object_module = "src.mem.qos.QoSPolicy",
    sim_object_name = "QoSPropFairPolicy",
    sim_object_py_lib = ":QoSPolicy_py",
)

py_library(
    name = "QoSTurnaround_py",
    srcs = ["QoSTurnaround.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "qo_s_turnaround_policy_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_hh",
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":turnaround_policy",
    sim_object_module = "src.mem.qos.QoSTurnaround",
    sim_object_name = "QoSTurnaroundPolicy",
    sim_object_py_lib = ":QoSTurnaround_py",
)

gem5_sim_object(
    name = "qo_s_turnaround_policy_ideal_sim_object",
    extra_param_deps = [
        ":qo_s_turnaround_policy_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:QoSTurnaround",
    ],
    sim_object_header = ":turnaround_policy_ideal",
    sim_object_module = "src.mem.qos.QoSTurnaround",
    sim_object_name = "QoSTurnaroundPolicyIdeal",
    sim_object_py_lib = ":QoSTurnaround_py",
)

# --- C++ Library Definitions ---

gem5_cc_library(
    name = "policy",
    srcs = ["policy.cc"],
    hdrs = ["policy.hh"],
    deps = [
        ":debug_QOS",
        ":mem_ctrl_qos_hh",
        ":q_policy",
        ":qo_s_policy_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:mem_requestor",
        "//src/mem:packet",
        "//src/mem:request",
        "//src/sim:clocked_object_hh",
        "//src/sim:stats",
    ],
)

gem5_cc_library(
    name = "policy_fixed_prio",
    srcs = ["policy_fixed_prio.cc"],
    hdrs = ["policy_fixed_prio.hh"],
    deps = [
        ":debug_QOS",
        ":policy",
        ":qo_s_fixed_priority_policy_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "policy_pf",
    srcs = ["policy_pf.cc"],
    hdrs = ["policy_pf.hh"],
    deps = [
        ":policy",
        ":qo_s_prop_fair_policy_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:statistics_hh",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "turnaround_policy_ideal",
    srcs = ["turnaround_policy_ideal.cc"],
    hdrs = ["turnaround_policy_ideal.hh"],
    deps = [
        ":qo_s_turnaround_policy_ideal_sim_object",
        ":turnaround_policy",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "turnaround_policy",
    hdrs = ["turnaround_policy.hh"],
    deps = [
        ":mem_ctrl_qos_hh",
        ":qo_s_turnaround_policy_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "q_policy",
    srcs = ["q_policy.cc"],
    hdrs = ["q_policy.hh"],
    deps = [
        ":debug_QOS",
        ":enum_QoSQPolicy_hh",
        ":mem_ctrl_qos_hh",
        ":qo_s_mem_ctrl_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet",
        "//src/mem:request",
    ],
)

gem5_cc_library(
    name = "mem_ctrl_qos_hh",
    hdrs = ["mem_ctrl.hh"],
    deps = [
        ":debug_QOS",
        ":qo_s_mem_ctrl_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:packet_hh",
        "//src/mem:request",
        "//src/sim:clocked_object_hh",
        "//src/sim:cur_tick",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "mem_ctrl_qos",
    srcs = ["mem_ctrl.cc"],
    hdrs = ["mem_ctrl.hh"],
    deps = [
        ":mem_ctrl_qos_hh",
        ":policy",
        ":q_policy",
        ":qo_s_mem_ctrl_sim_object",
        ":turnaround_policy",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:mem_ctrl_hh",
        "//src/mem:mem_interface",
        "//src/mem:packet_hh",
        "//src/mem:request",
        "//src/mem/qos:debug_QOS",
        "//src/sim:clocked_object",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "mem_sink_qos",
    srcs = ["mem_sink.cc"],
    hdrs = ["mem_sink.hh"],
    deps = [
        ":debug_QOS",
        ":mem_ctrl_qos_hh",
        ":q_policy",
        ":qo_s_mem_sink_ctrl_sim_object",
        ":qo_s_mem_sink_interface_sim_object",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:abstract_mem_hh",
        "//src/mem:port",
        "//src/mem:qport",
        "//src/mem:request",
        "//src/sim:debug_Drain",
        "//src/sim:eventq",
        "//src/sim:sim_object",
    ],
)
