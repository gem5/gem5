load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_PciDevice",
    desc = "",
    flag = "PciDevice",
)

gem5_debug_flag(
    name = "debug_PciHost",
    desc = "",
    flag = "PciHost",
)

gem5_debug_flag(
    name = "debug_PciUpstream",
    desc = "",
    flag = "PciUpstream",
)

gem5_cc_library(
    name = "pcireg_h",
    hdrs = ["pcireg.h"],
    deps = [
        "//src/base:bitfield",
        "//src/base:bitunion",
        "//src/base:compiler",
    ],
)

gem5_cc_library(
    name = "types",
    hdrs = ["types.hh"],
    deps = [
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "host",
    srcs = ["host.cc"],
    hdrs = ["host.hh"],
    deps = [
        ":debug_PciHost",
        ":device",
        ":generic_pci_host_sim_object",
        ":pci_host_sim_object",
        ":pcireg_h",
        ":types",
        ":upstream",
        ":upstream_hh",
        "//src/base:addr_range",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/dev:platform",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "device",
    srcs = ["device.cc"],
    hdrs = ["device.hh"],
    deps = [
        ":debug_PciDevice",
        ":pci_bar_none_sim_object",
        ":pci_bar_sim_object",
        ":pci_device_sim_object",
        ":pci_endpoint_sim_object",
        ":pci_io_bar_sim_object",
        ":pci_legacy_io_bar_sim_object",
        ":pci_mem_bar_sim_object",
        ":pci_mem_upper_bar_sim_object",
        ":pci_type1_device_sim_object",
        ":pcireg_h",
        ":types",
        ":upstream_hh",
        "//src/base:inifile",
        "//src/base:intmath",
        "//src/base:logging",
        "//src/base:trace",
        "//src/dev:dma_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:byteswap",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "bus",
    srcs = ["bus.cc"],
    hdrs = ["bus.hh"],
    deps = [
        ":pci_bus_sim_object",
        ":pcireg_h",
        "//src/base:trace",
        "//src/mem:debug_AddrRanges",
        "//src/mem:noncoherent_xbar",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "upstream_hh",
    hdrs = ["upstream.hh"],
    deps = [
        ":pci_config_error_sim_object",
        ":pci_upstream_sim_object",
        ":types",
        ":up_down_bridge",
        "//src/base:addr_range",
        "//src/dev:isa_fake",
        "//src/sim:clocked_object",
    ],
)

gem5_cc_library(
    name = "upstream",
    srcs = ["upstream.cc"],
    deps = [
        ":debug_PciUpstream",
        ":device",
        ":pci_config_error_sim_object",
        ":pci_upstream_sim_object",
        ":pcireg_h",
        ":types",
        ":upstream_hh",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
    ],
)

gem5_cc_library(
    name = "up_down_bridge",
    srcs = ["up_down_bridge.cc"],
    hdrs = ["up_down_bridge.hh"],
    deps = [
        ":debug_PciUpstream",
        ":pci_up_down_bridge_sim_object",
        "//src/base:addr_range",
        "//src/base:logging",
        "//src/mem:bridge",
    ],
)

py_library(
    name = "PciHost_py",
    srcs = ["PciHost.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:PciUpstream",
        "//src/python/m5/objects:Platform",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "PciDevice_py",
    srcs = ["PciDevice.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:PciUpstream",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "PciUpstream_py",
    srcs = ["PciUpstream.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Bridge",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:XBar",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "generic_pci_host_sim_object",
    extra_deps = [
        "//src/dev:platform",
    ],
    extra_param_deps = [
        ":pci_host_sim_object",
        "//src/dev:platform_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:Platform",
    ],
    sim_object_header = ":host",
    sim_object_module = "src.dev.pci.PciHost",
    sim_object_name = "GenericPciHost",
    sim_object_py_lib = ":PciHost_py",
)

gem5_sim_object(
    name = "pci_host_sim_object",
    extra_param_deps = [
        "//src/dev/pci:pci_upstream_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciUpstream",
    ],
    sim_object_header = ":host",
    sim_object_module = "src.dev.pci.PciHost",
    sim_object_name = "PciHost",
    sim_object_py_lib = ":PciHost_py",
)

gem5_sim_object(
    name = "pci_device_sim_object",
    extra_deps = [
        ":upstream_hh",
    ],
    extra_param_deps = [
        "//src/dev:dma_device_sim_object",
        "//src/dev/pci:pci_upstream_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:PciUpstream",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciDevice",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_endpoint_sim_object",
    extra_param_deps = [
        ":pci_bar_sim_object",
        ":pci_device_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciEndpoint",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_type1_device_sim_object",
    extra_param_deps = [
        ":pci_bar_sim_object",
        ":pci_device_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciType1Device",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_bar_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciBar",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_bar_none_sim_object",
    extra_param_deps = [
        ":pci_bar_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciBarNone",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_io_bar_sim_object",
    extra_param_deps = [
        ":pci_bar_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciIoBar",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_legacy_io_bar_sim_object",
    extra_param_deps = [
        ":pci_io_bar_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciLegacyIoBar",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_mem_bar_sim_object",
    extra_param_deps = [
        ":pci_bar_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciMemBar",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_mem_upper_bar_sim_object",
    extra_param_deps = [
        ":pci_bar_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":device",
    sim_object_module = "src.dev.pci.PciDevice",
    sim_object_name = "PciMemUpperBar",
    sim_object_py_lib = ":PciDevice_py",
)

gem5_sim_object(
    name = "pci_bus_sim_object",
    extra_param_deps = [
        "//src/mem:noncoherent_x_bar_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:XBar",
    ],
    sim_object_header = ":bus",
    sim_object_module = "src.dev.pci.PciUpstream",
    sim_object_name = "PciBus",
    sim_object_py_lib = ":PciUpstream_py",
)

gem5_sim_object(
    name = "pci_upstream_sim_object",
    extra_deps = [
        ":up_down_bridge",
        ":upstream_hh",
        "//src/mem:bridge",
    ],
    extra_param_deps = [
        ":pci_config_error_sim_object",
        ":pci_up_down_bridge_sim_object",
        "//src/mem:bridge_base_sim_object",
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Bridge",
        "//src/python/m5/objects:ClockedObject",
    ],
    sim_object_header = ":upstream",
    sim_object_module = "src.dev.pci.PciUpstream",
    sim_object_name = "PciUpstream",
    sim_object_py_lib = ":PciUpstream_py",
)

gem5_sim_object(
    name = "pci_config_error_sim_object",
    extra_deps = [
        ":upstream_hh",
    ],
    extra_param_deps = [
        "//src/dev:isa_fake_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Device",
    ],
    sim_object_header = ":upstream",
    sim_object_module = "src.dev.pci.PciUpstream",
    sim_object_name = "PciConfigError",
    sim_object_py_lib = ":PciUpstream_py",
)

gem5_sim_object(
    name = "pci_up_down_bridge_sim_object",
    extra_param_deps = [
        "//src/mem:bridge_base_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Bridge",
    ],
    sim_object_header = ":up_down_bridge",
    sim_object_module = "src.dev.pci.PciUpstream",
    sim_object_name = "PciUpDownBridge",
    sim_object_py_lib = ":PciUpstream_py",
)
