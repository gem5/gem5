load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_DistEthernet",
    flag = "DistEthernet",
)

gem5_debug_flag(
    name = "debug_DistEthernetPkt",
    flag = "DistEthernetPkt",
)

gem5_debug_flag(
    name = "debug_DistEthernetCmd",
    flag = "DistEthernetCmd",
)

gem5_cc_library(
    name = "etherpkt",
    srcs = ["etherpkt.cc"],
    hdrs = ["etherpkt.hh"],
    deps = [
        ":etherpkt_hh",
        "//src/base:inet_hh",
        "//src/base:logging",
        "//src/base:types",
        "//src/sim:serialize",
    ],
)

# circular dependency on gem5/src/base:inet
gem5_cc_library(
    name = "etherpkt_hh",
    hdrs = ["etherpkt.hh"],
    deps = [
        "//src/base:types",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "dist_iface",
    srcs = ["dist_iface.cc"],
    hdrs = ["dist_iface.hh"],
    deps = [
        ":debug_DistEthernet",
        ":debug_DistEthernetPkt",
        ":dist_packet",
        ":etherpkt",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:trace",
        "//src/cpu:thread_context",
        "//src/sim:cur_tick",
        "//src/sim:drain",
        "//src/sim:global_event",
        "//src/sim:serialize",
        "//src/sim:sim_exit",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_cc_library(
    name = "dist_packet",
    hdrs = ["dist_packet.hh"],
    deps = [
        "//src/base:types",
    ],
)

gem5_cc_library(
    name = "tcp_iface",
    srcs = ["tcp_iface.cc"],
    hdrs = ["tcp_iface.hh"],
    deps = [
        ":debug_DistEthernet",
        ":debug_DistEthernetCmd",
        ":dist_iface",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:socket",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:serialize",
        "//src/sim:sim_exit",
    ],
)

# --- Added targets ---

gem5_debug_flag(
    name = "debug_Ethernet",
    desc = "",
    flag = "Ethernet",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetDesc",
    desc = "",
    flag = "EthernetDesc",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetIntr",
    desc = "",
    flag = "EthernetIntr",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetSM",
    desc = "",
    flag = "EthernetSM",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetCksum",
    desc = "",
    flag = "EthernetCksum",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetData",
    desc = "",
    flag = "EthernetData",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetAll",
    components = [
        ":debug_Ethernet",
        ":debug_EthernetCksum",
        ":debug_EthernetDMA",
        ":debug_EthernetData",
        ":debug_EthernetDesc",
        ":debug_EthernetEEPROM",
        ":debug_EthernetIntr",
        ":debug_EthernetPIO",
        ":debug_EthernetSM",
    ],
    desc = "",
    flag = "EthernetAll",
)

gem5_debug_flag(
    name = "debug_EthernetPIO",
    desc = "",
    flag = "EthernetPIO",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetDMA",
    desc = "",
    flag = "EthernetDMA",
    included_by = [":debug_EthernetAll"],
)

gem5_debug_flag(
    name = "debug_EthernetEEPROM",
    desc = "",
    flag = "EthernetEEPROM",
    included_by = [":debug_EthernetAll"],
)

gem5_cc_library(
    name = "etherint",
    srcs = ["etherint.cc"],
    hdrs = ["etherint.hh"],
    deps = [
        ":etherpkt",
        "//src/base:logging",
        "//src/mem:port",
        "//src/sim:port",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "etherdevice",
    srcs = ["etherdevice.cc"],
    hdrs = ["etherdevice.hh"],
    deps = [
        ":ether_dev_base_sim_object",
        ":ether_device_sim_object",
        ":etherint",
        ":etherpkt",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/dev/pci:device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:sim_object",
        "//src/sim:stats_hh",
    ],
)

gem5_cc_library(
    name = "ns_gige",
    srcs = ["ns_gige.cc"],
    hdrs = [
        "ns_gige.hh",
        "ns_gige_reg.h",
    ],
    deps = [
        ":debug_EthernetAll",
        ":etherdevice",
        ":etherdump",
        ":etherint",
        ":etherlink",
        ":etherpkt",
        ":ns_gig_e_sim_object",
        ":pktfifo",
        "//src/base:debug",
        "//src/base:inet",
        "//src/base:inet_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:eventq",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "i8254xGBe",
    srcs = ["i8254xGBe.cc"],
    hdrs = [
        "i8254xGBe.hh",
        "i8254xGBe_defs.hh",
    ],
    deps = [
        ":debug_Ethernet",
        ":debug_EthernetAll",
        ":debug_EthernetCksum",
        ":debug_EthernetDMA",
        ":debug_EthernetData",
        ":debug_EthernetDesc",
        ":debug_EthernetEEPROM",
        ":debug_EthernetIntr",
        ":debug_EthernetPIO",
        ":debug_EthernetSM",
        ":ether_dev_base_sim_object",
        ":etherdevice",
        ":etherint",
        ":etherpkt",
        ":i_gb_e_sim_object",
        ":pktfifo",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:inet",
        "//src/base:inet_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:io_device",
        "//src/dev/pci:device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:debug_Drain",
        "//src/sim:eventq",
        "//src/sim:serialize",
        "//src/sim:stats_hh",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "etherdump",
    srcs = ["etherdump.cc"],
    hdrs = ["etherdump.hh"],
    deps = [
        ":ether_dump_sim_object",
        ":etherpkt",
        "//src/base:logging",
        "//src/base:output",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "Ethernet_py",
    srcs = ["Ethernet.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:defines",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:PciDevice",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "ether_dump_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":etherdump",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherDump",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_sim_object(
    name = "ether_device_sim_object",
    extra_param_deps = [
        "//src/dev/pci:pci_endpoint_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciDevice",
    ],
    sim_object_header = ":etherdevice",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherDevice",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_sim_object(
    name = "ether_dev_base_sim_object",
    extra_param_deps = [
        ":ether_device_sim_object",
        "//src/base:inet_hh",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciDevice",
    ],
    sim_object_header = ":etherdevice",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherDevBase",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_sim_object(
    name = "i_gb_e_sim_object",
    extra_param_deps = [
        ":ether_device_sim_object",
        "//src/base:inet_hh",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciDevice",
    ],
    sim_object_header = ":i8254xGBe",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "IGbE",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_sim_object(
    name = "ns_gig_e_sim_object",
    extra_param_deps = [
        ":ether_dev_base_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciDevice",
    ],
    sim_object_header = ":ns_gige",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "NSGigE",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_sim_object(
    name = "ether_link_sim_object",
    extra_deps = [
        ":etherdump",
    ],
    extra_param_deps = [
        ":ether_dump_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":etherlink",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherLink",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_cc_library(
    name = "etherlink",
    srcs = ["etherlink.cc"],
    hdrs = ["etherlink.hh"],
    deps = [
        ":debug_Ethernet",
        ":debug_EthernetData",
        ":ether_link_sim_object",
        ":etherdump",
        ":etherint",
        ":etherpkt",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
        "//src/sim:serialize",
        "//src/sim:sim_object",
        "//src/sim:system",
    ],
)

gem5_sim_object(
    name = "dist_ether_link_sim_object",
    extra_deps = [
        ":etherdump",
    ],
    extra_param_deps = [
        ":ether_dump_sim_object",
        ":ether_link_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":dist_etherlink",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "DistEtherLink",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_cc_library(
    name = "dist_etherlink",
    srcs = ["dist_etherlink.cc"],
    hdrs = ["dist_etherlink.hh"],
    deps = [
        ":debug_DistEthernet",
        ":debug_DistEthernetPkt",
        ":debug_EthernetData",
        ":dist_ether_link_sim_object",
        ":dist_iface",
        ":ether_link_sim_object",
        ":etherdump",
        ":etherint",
        ":etherlink",
        ":etherpkt",
        ":tcp_iface",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:cur_tick",
        "//src/sim:serialize",
        "//src/sim:sim_object",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "sinic_sim_object",
    extra_param_deps = [
        ":ether_dev_base_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciDevice",
    ],
    sim_object_header = ":sinic",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "Sinic",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_cc_library(
    name = "sinic",
    srcs = ["sinic.cc"],
    hdrs = [
        "sinic.hh",
        "sinicreg.hh",
    ],
    deps = [
        ":debug_Ethernet",
        ":debug_EthernetAll",
        ":debug_EthernetCksum",
        ":debug_EthernetDMA",
        ":debug_EthernetData",
        ":debug_EthernetDesc",
        ":debug_EthernetIntr",
        ":debug_EthernetPIO",
        ":debug_EthernetSM",
        ":etherdevice",
        ":etherint",
        ":etherlink",
        ":etherpkt",
        ":pktfifo",
        ":sinic_sim_object",
        "//src/base:compiler",
        "//src/base:debug",
        "//src/base:inet",
        "//src/base:inet_hh",
        "//src/base:statistics_hh",
        "//src/base:trace",
        "//src/base:types",
        "//src/dev:io_device",
        "//src/dev/pci:device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:eventq",
        "//src/sim:stats_hh",
    ],
)

gem5_cc_library(
    name = "pktfifo",
    srcs = ["pktfifo.cc"],
    hdrs = ["pktfifo.hh"],
    deps = [
        ":etherpkt",
        "//src/base:logging",
        "//src/sim:serialize",
    ],
)

gem5_sim_object(
    name = "ether_bus_sim_object",
    extra_deps = [
        ":etherdump",
    ],
    extra_param_deps = [
        ":ether_dump_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":etherbus",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherBus",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_cc_library(
    name = "etherbus",
    srcs = ["etherbus.cc"],
    hdrs = ["etherbus.hh"],
    deps = [
        ":debug_Ethernet",
        ":debug_EthernetData",
        ":ether_bus_sim_object",
        ":etherdump",
        ":etherint",
        ":etherpkt",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "ether_switch_sim_object",
    extra_deps = [
        ":etherdump",
    ],
    extra_param_deps = [
        ":ether_dump_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":etherswitch",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherSwitch",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_cc_library(
    name = "etherswitch",
    srcs = ["etherswitch.cc"],
    hdrs = ["etherswitch.hh"],
    deps = [
        ":debug_Ethernet",
        ":debug_EthernetAll",
        ":ether_switch_sim_object",
        ":etherint",
        ":etherlink",
        ":etherpkt",
        ":pktfifo",
        "//src/base:inet_hh",
        "//src/base:logging",
        "//src/base:random",
        "//src/base:trace",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
        "//src/sim:serialize",
        "//src/sim:sim_object",
    ],
)

gem5_sim_object(
    name = "ether_tap_base_sim_object",
    extra_deps = [
        ":etherdump",
    ],
    extra_param_deps = [
        ":ether_dump_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":ethertap",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherTapBase",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_sim_object(
    name = "ether_tap_stub_sim_object",
    extra_param_deps = [
        ":ether_tap_base_sim_object",
        "//src/base:socket",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":ethertap",
    sim_object_module = "src.dev.net.Ethernet",
    sim_object_name = "EtherTapStub",
    sim_object_py_lib = ":Ethernet_py",
)

gem5_cc_library(
    name = "ethertap",
    srcs = ["ethertap.cc"],
    hdrs = ["ethertap.hh"],
    deps = [
        ":debug_Ethernet",
        ":debug_EthernetData",
        ":ether_tap_base_sim_object",
        ":ether_tap_stub_sim_object",
        ":etherdump",
        ":etherint",
        ":etherpkt",
        "//bazel/generated/config",
        "//src/base:logging",
        "//src/base:pollevent",
        "//src/base:socket",
        "//src/base:trace",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
        "//src/sim:sim_object",
    ],
)
