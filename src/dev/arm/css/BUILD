load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_MHU",
    desc = "",
    flag = "MHU",
)

gem5_debug_flag(
    name = "debug_SCMI",
    desc = "",
    flag = "SCMI",
)

gem5_cc_library(
    name = "scp",
    hdrs = ["scp.hh"],
    deps = [
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "mhu",
    srcs = ["mhu.cc"],
    hdrs = ["mhu.hh"],
    deps = [
        ":ap2_scp_doorbell_sim_object",
        ":debug_MHU",
        ":mhu_sim_object",
        ":scp",
        ":scp2_ap_doorbell_sim_object",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/dev/arm:amba_device_hh",
        "//src/dev/arm:base_gic",
        "//src/dev/arm:doorbell",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "scmi_protocols_hh",
    hdrs = ["scmi_protocols.hh"],
    deps = [
        "//src/base:compiler",
    ],
)

gem5_cc_library(
    name = "scmi_protocols",
    srcs = ["scmi_protocols.cc"],
    deps = [
        ":debug_SCMI",
        ":scmi_platform_hh",
        ":scmi_protocols_hh",
        "//src/base:bitunion",
        "//src/base:trace",
    ],
)

gem5_cc_library(
    name = "scmi_platform_hh",
    hdrs = ["scmi_platform.hh"],
    deps = [
        ":scmi_platform_sim_object",
        ":scmi_protocols_hh",
        ":scp",
        "//src/base:bitunion",
        "//src/dev:dma_device",
    ],
)

gem5_cc_library(
    name = "scmi_platform",
    srcs = ["scmi_platform.cc"],
    deps = [
        ":debug_SCMI",
        ":scmi_agent_channel_sim_object",
        ":scmi_channel_sim_object",
        ":scmi_communication_sim_object",
        ":scmi_platform_channel_sim_object",
        ":scmi_platform_hh",
        ":scmi_protocols",
        "//src/base:trace",
        "//src/dev/arm:doorbell",
        "//src/mem:packet_access",
        "//src/mem:port",
    ],
)

py_library(
    name = "Scp_py",
    srcs = ["Scp.py"],
    deps = [
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "MHU_py",
    srcs = ["MHU.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Doorbell",
        "//src/python/m5/objects:Gic",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

py_library(
    name = "Scmi_py",
    srcs = ["Scmi.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Doorbell",
        "//src/python/m5/objects:Scp",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_sim_object(
    name = "scp_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockedObject",
    ],
    sim_object_header = ":scp",
    sim_object_module = "src.dev.arm.css.Scp",
    sim_object_name = "Scp",
    sim_object_py_lib = ":Scp_py",
)

gem5_sim_object(
    name = "mhu_sim_object",
    extra_deps = [
        ":scp",
    ],
    extra_param_deps = [
        ":scp2_ap_doorbell_sim_object",
        ":ap2_scp_doorbell_sim_object",
        "//src/dev:basic_pio_device_sim_object",
        "//src/dev/arm/css:scp_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Scp",
    ],
    sim_object_header = ":mhu",
    sim_object_module = "src.dev.arm.css.MHU",
    sim_object_name = "MHU",
    sim_object_py_lib = ":MHU_py",
)

gem5_sim_object(
    name = "mhu_doorbell_sim_object",
    extra_param_deps = [
        "//src/dev/arm:doorbell_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Doorbell",
    ],
    sim_object_header = ":mhu",
    sim_object_module = "src.dev.arm.css.MHU",
    sim_object_name = "MhuDoorbell",
    sim_object_py_lib = ":MHU_py",
)

gem5_sim_object(
    name = "scp2_ap_doorbell_sim_object",
    extra_deps = [
        "//src/dev/arm:base_gic",
    ],
    extra_param_deps = [
        ":mhu_doorbell_sim_object",
        "//src/dev/arm:arm_interrupt_pin_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Doorbell",
        "//src/python/m5/objects:Gic",
    ],
    sim_object_header = ":mhu",
    sim_object_module = "src.dev.arm.css.MHU",
    sim_object_name = "Scp2ApDoorbell",
    sim_object_py_lib = ":MHU_py",
)

gem5_sim_object(
    name = "ap2_scp_doorbell_sim_object",
    extra_param_deps = [
        ":mhu_doorbell_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Doorbell",
    ],
    sim_object_header = ":mhu",
    sim_object_module = "src.dev.arm.css.MHU",
    sim_object_name = "Ap2ScpDoorbell",
    sim_object_py_lib = ":MHU_py",
)

gem5_sim_object(
    name = "scmi_communication_sim_object",
    extra_deps = [
        ":scmi_platform_hh",
    ],
    extra_param_deps = [
        ":scmi_agent_channel_sim_object",
        ":scmi_platform_channel_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":scmi_platform",
    sim_object_module = "src.dev.arm.css.Scmi",
    sim_object_name = "ScmiCommunication",
    sim_object_py_lib = ":Scmi_py",
)

gem5_sim_object(
    name = "scmi_platform_sim_object",
    extra_deps = [
        ":scmi_platform_hh",
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        ":scmi_communication_sim_object",
        ":scp_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Scp",
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":scmi_platform",
    sim_object_module = "src.dev.arm.css.Scmi",
    sim_object_name = "ScmiPlatform",
    sim_object_py_lib = ":Scmi_py",
)

gem5_sim_object(
    name = "scmi_channel_sim_object",
    extra_deps = [
        ":scmi_platform_hh",
        "//src/dev/arm:doorbell",
    ],
    extra_param_deps = [
        "//src/base:addr_range",
        "//src/dev/arm:doorbell_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Doorbell",
    ],
    sim_object_header = ":scmi_platform",
    sim_object_module = "src.dev.arm.css.Scmi",
    sim_object_name = "ScmiChannel",
    sim_object_py_lib = ":Scmi_py",
)

gem5_sim_object(
    name = "scmi_agent_channel_sim_object",
    extra_deps = [
        ":scmi_platform_hh",
    ],
    extra_param_deps = [
        ":scmi_channel_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":scmi_platform",
    sim_object_module = "src.dev.arm.css.Scmi",
    sim_object_name = "ScmiAgentChannel",
    sim_object_py_lib = ":Scmi_py",
)

gem5_sim_object(
    name = "scmi_platform_channel_sim_object",
    extra_deps = [
        ":scmi_platform_hh",
    ],
    extra_param_deps = [
        ":scmi_channel_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":scmi_platform",
    sim_object_module = "src.dev.arm.css.Scmi",
    sim_object_name = "ScmiPlatformChannel",
    sim_object_py_lib = ":Scmi_py",
)
