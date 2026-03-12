load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_cc_test", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

py_library(
    name = "mem_init",
    srcs = ["__init__.py"],
    deps = [
        ":AbstractMemory_py",
        ":AddrMapper_py",
        ":Bridge_py",
        ":CfiMemory_py",
        ":CommMonitor_py",
        ":DRAMInterface_py",
        ":HBMCtrl_py",
        ":MemCtrl_py",
        ":MemInterface_py",
        ":NVMInterface_py",
        ":PortTerminator_py",
        ":SimpleMemory_py",
        ":XBar_py",
        "//src/mem/cache:cache_init",
        "//src/mem/ruby:ruby_all_libs",
    ],
)

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_DRAM",
    flag = "DRAM",
)

gem5_debug_flag(
    name = "debug_DRAMPower",
    flag = "DRAMPower",
)

gem5_debug_flag(
    name = "debug_DRAMState",
    flag = "DRAMState",
)

gem5_debug_flag(
    name = "debug_LLSC",
    flag = "LLSC",
)

gem5_debug_flag(
    name = "debug_MemoryAccess",
    flag = "MemoryAccess",
)

gem5_debug_flag(
    name = "debug_HtmMem",
    desc = "Hardware Transactional Memory (Mem side)",
    flag = "HtmMem",
)

gem5_debug_flag(
    name = "debug_MemCtrl",
    flag = "MemCtrl",
)

gem5_debug_flag(
    name = "debug_NVM",
    flag = "NVM",
)

py_library(
    name = "DRAMInterface_py",
    srcs = ["DRAMInterface.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/objects:MemInterface",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_PageManage",
    enum_module = "src.mem.DRAMInterface",
    enum_name = "PageManage",
    enum_py_lib = ":DRAMInterface_py",
    extra_tool_deps = [
        "//src/python/m5/objects:MemCtrl",
    ],
)

py_library(
    name = "MemInterface_py",
    srcs = ["MemInterface.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_AddrMap",
    enum_module = "src.mem.MemInterface",
    enum_name = "AddrMap",
    enum_py_lib = ":MemInterface_py",
)

gem5_cc_library(
    name = "mem_interface_hh",
    hdrs = ["mem_interface.hh"],
    deps = [
        ":abstract_mem_hh",
        ":enum_AddrMap_hh",
        ":enum_PageManage_hh",
        ":mem_interface_sim_object",
        "//src/base:compiler",
        "//src/base:statistics",
        "//src/mem/qos:mem_ctrl_qos_hh",
        "//src/sim:eventq",
    ],
)

gem5_cc_library(
    name = "mem_interface",
    srcs = ["mem_interface.cc"],
    hdrs = ["mem_interface.hh"],
    deps = [
        ":abstract_mem_hh",
        ":enum_AddrMap_hh",
        ":enum_PageManage_hh",
        ":mem_ctrl_hh",
        ":mem_interface_hh",
        ":mem_interface_sim_object",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:cprintf",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/mem/qos:mem_ctrl_qos_hh",
        "//src/sim:eventq",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "mem_interface_sim_object",
    extra_param_deps = [
        "//src/mem:abstract_memory_sim_object",
        ":enum_AddrMap_hh",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:AbstractMemory",
    ],
    sim_object_header = ":mem_interface",
    sim_object_module = "src.mem.MemInterface",
    sim_object_name = "MemInterface",
    sim_object_py_lib = ":MemInterface_py",
)

gem5_cc_library(
    name = "drampower_hh",
    hdrs = ["drampower.hh"],
    deps = [
        ":dram_interface_sim_object",
        "//ext/drampower:libdrampower",
    ],
)

gem5_cc_library(
    name = "drampower",
    srcs = ["drampower.cc"],
    hdrs = ["drampower.hh"],
    deps = [
        ":dram_interface_sim_object",
        ":drampower_hh",
        "//ext/drampower:libdrampower",
        "//src/base:intmath",
        "//src/sim:core",
    ],
)

gem5_cc_library(
    name = "dram_interface_hh",
    hdrs = ["dram_interface.hh"],
    deps = [
        ":dram_interface_sim_object",
        ":drampower_hh",
        ":mem_ctrl_hh",
        ":mem_interface_hh",
    ],
)

gem5_cc_library(
    name = "dram_interface",
    srcs = ["dram_interface.cc"],
    hdrs = ["dram_interface.hh"],
    deps = [
        ":debug_DRAM",
        ":debug_DRAMPower",
        ":debug_DRAMState",
        ":dram_interface_hh",
        ":dram_interface_sim_object",
        ":drampower_hh",
        ":mem_ctrl_hh",
        ":mem_interface_hh",
        "//src/base:bitfield",
        "//src/base:cprintf",
        "//src/base:trace",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "dram_interface_sim_object",
    extra_deps = [
        ":dram_interface",
        ":drampower",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":enum_PageManage_hh",
        ":mem_interface_sim_object",
    ],
    extra_tool_deps = [
        ":MemCtrl_py",
        ":MemInterface_py",
        "//src/python/m5/params",
        "//src/python/m5:proxy",
    ],
    sim_object_header = ":dram_interface_hh",
    sim_object_module = "src.mem.DRAMInterface",
    sim_object_name = "DRAMInterface",
    sim_object_py_lib = ":DRAMInterface_py",
)

gem5_cc_library(
    name = "port_proxy",
    srcs = ["port_proxy.cc"],
    hdrs = ["port_proxy.hh"],
    deps = [
        ":port",
        "//src/base:chunk_generator",
        "//src/cpu:thread_context_hh",
        "//src/mem/protocol:functional",
        "//src/sim:byteswap",
    ],
)

py_library(
    name = "PortTerminator_py",
    srcs = ["PortTerminator.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "port_terminator_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":port_terminator",
    sim_object_module = "src.mem.PortTerminator",
    sim_object_name = "PortTerminator",
    sim_object_py_lib = ":PortTerminator_py",
)

gem5_cc_library(
    name = "port_terminator",
    srcs = ["port_terminator.cc"],
    hdrs = ["port_terminator.hh"],
    deps = [
        ":port_hh",
        ":port_terminator_sim_object",
        "//src/sim:sim_object_hh",
    ],
)

py_library(
    name = "Bridge_py",
    srcs = ["Bridge.py"],
    deps = [
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "bridge_base_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockedObject",
    ],
    sim_object_header = ":bridge",
    sim_object_module = "src.mem.Bridge",
    sim_object_name = "BridgeBase",
    sim_object_py_lib = ":Bridge_py",
)

gem5_sim_object(
    name = "bridge_sim_object",
    extra_param_deps = [
        ":bridge_base_sim_object",
        "//src/base:addr_range",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":bridge",
    sim_object_module = "src.mem.Bridge",
    sim_object_name = "Bridge",
    sim_object_py_lib = ":Bridge_py",
)

gem5_cc_library(
    name = "bridge",
    srcs = ["bridge.cc"],
    hdrs = ["bridge.hh"],
    deps = [
        ":bridge_base_sim_object",
        ":bridge_sim_object",
        ":debug_Bridge",
        ":port_hh",
        "//src/base:addr_range",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:clocked_object_hh",
    ],
)

gem5_debug_flag(
    name = "debug_Bridge",
    flag = "Bridge",
)

gem5_cc_library(
    name = "packet_hh",
    hdrs = ["packet.hh"],
    deps = [
        ":htm",
        ":request",
        "//src/base:addr_range",
        "//src/base:cast",
        "//src/base:compiler",
        "//src/base:extensible",
        "//src/base:flags",
        "//src/base:logging",
        "//src/base:printable",
        "//src/base:types",
        "//src/sim:byteswap",
    ],
)

gem5_cc_library(
    name = "packet",
    srcs = ["packet.cc"],
    hdrs = ["packet.hh"],
    deps = [
        ":htm",
        ":packet_access",
        ":request",
        "//src/base:addr_range",
        "//src/base:cast",
        "//src/base:compiler",
        "//src/base:cprintf",
        "//src/base:extensible",
        "//src/base:flags",
        "//src/base:logging",
        "//src/base:printable",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:bufval",
        "//src/sim:byteswap",
    ],
)

gem5_cc_library(
    name = "packet_access",
    hdrs = ["packet_access.hh"],
    deps = [
        ":packet_hh",
        "//src/sim:byteswap",
    ],
)

gem5_cc_library(
    name = "request",
    hdrs = ["request.hh"],
    deps = [
        ":htm",
        "//src/base:amo",
        "//src/base:compiler",
        "//src/base:extensible",
        "//src/base:flags",
        "//src/base:types",
        "//src/cpu:inst_seq",
        "//src/sim:cur_tick",
    ],
)

gem5_cc_library(
    name = "htm",
    srcs = ["htm.cc"],
    hdrs = ["htm.hh"],
    deps = [
    ],
)

gem5_cc_library(
    name = "htm_hh",
    hdrs = ["htm.hh"],
    deps = [
    ],
)

gem5_cc_library(
    name = "backdoor",
    hdrs = ["backdoor.hh"],
    deps = [
        "//src/base:addr_range",
        "//src/base:callback",
    ],
)

gem5_debug_flag(
    name = "debug_PortTrace",
    desc = "None",
    flag = "PortTrace",
)

gem5_debug_flag(
    name = "debug_ResponsePort",
    desc = "None",
    flag = "ResponsePort",
)

gem5_debug_flag(
    name = "debug_PacketQueue",
    desc = "None",
    flag = "PacketQueue",
)

gem5_debug_flag(
    name = "debug_SnoopFilter",
    desc = "None",
    flag = "SnoopFilter",
    included_by = [":debug_XBar"],
)

gem5_debug_flag(
    name = "debug_AddrRanges",
    flag = "AddrRanges",
)

gem5_debug_flag(
    name = "debug_BaseXBar",
    flag = "BaseXBar",
    included_by = [":debug_XBar"],
)

gem5_debug_flag(
    name = "debug_CoherentXBar",
    flag = "CoherentXBar",
    included_by = [":debug_XBar"],
)

gem5_debug_flag(
    name = "debug_NoncoherentXBar",
    flag = "NoncoherentXBar",
    included_by = [":debug_XBar"],
)

gem5_debug_flag(
    name = "debug_XBar",
    components = [
        ":debug_BaseXBar",
        ":debug_CoherentXBar",
        ":debug_NoncoherentXBar",
        ":debug_SnoopFilter",
    ],
    flag = "XBar",
)

gem5_debug_flag(
    name = "debug_MMU",
    flag = "MMU",
)

gem5_debug_flag(
    name = "debug_CFI",
    flag = "CFI",
)

gem5_debug_flag(
    name = "debug_CommMonitor",
    flag = "CommMonitor",
)

py_library(
    name = "CfiMemory_py",
    srcs = ["CfiMemory.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/params",
        "//src/python/m5/util:fdthelper",
    ],
)

gem5_cc_library(
    name = "cfi_mem_hh",
    hdrs = ["cfi_mem.hh"],
    deps = [
        ":abstract_mem_hh",
        ":cfi_memory_sim_object",
        "//src/base:random",
    ],
)

gem5_cc_library(
    name = "cfi_mem",
    srcs = ["cfi_mem.cc"],
    hdrs = ["cfi_mem.hh"],
    deps = [
        ":abstract_mem_hh",
        ":cfi_mem_hh",
        ":cfi_memory_sim_object",
        ":debug_CFI",
        "//src/base:intmath",
        "//src/base:random",
        "//src/base:trace",
        "//src/sim:debug_Drain",
    ],
)

gem5_sim_object(
    name = "cfi_memory_sim_object",
    extra_deps = [
        ":cfi_mem",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":abstract_memory_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:AbstractMemory",
    ],
    sim_object_header = ":cfi_mem_hh",
    sim_object_module = "src.mem.CfiMemory",
    sim_object_name = "CfiMemory",
    sim_object_py_lib = ":CfiMemory_py",
)

py_library(
    name = "CommMonitor_py",
    srcs = ["CommMonitor.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "comm_monitor_hh",
    hdrs = ["comm_monitor.hh"],
    deps = [
        ":comm_monitor_sim_object",
        ":port_hh",
        "//src/base:statistics",
        "//src/sim:sim_object_hh",
        "//src/sim/probe:mem",
    ],
)

gem5_cc_library(
    name = "comm_monitor",
    srcs = ["comm_monitor.cc"],
    hdrs = ["comm_monitor.hh"],
    deps = [
        ":comm_monitor_hh",
        ":comm_monitor_sim_object",
        ":debug_CommMonitor",
        ":port_hh",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/sim:core",
        "//src/sim:cur_tick",
        "//src/sim:sim_object_hh",
        "//src/sim:stats",
        "//src/sim/probe:mem",
    ],
)

gem5_sim_object(
    name = "comm_monitor_sim_object",
    extra_deps = [
        ":comm_monitor",
        "//src/sim:sim_object_hh",
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:system_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":comm_monitor_hh",
    sim_object_module = "src.mem.CommMonitor",
    sim_object_name = "CommMonitor",
    sim_object_py_lib = ":CommMonitor_py",
)

gem5_cc_library(
    name = "port",
    srcs = ["port.cc"],
    hdrs = ["port.hh"],
    copts = [
        "-Ithird_party/gem5/develop/src",
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
    deps = [
        ":debug_PortTrace",
        ":debug_ResponsePort",
        ":packet_hh",
        "//src/base:addr_range",
        "//src/base:trace",
        "//src/mem/protocol:atomic",
        "//src/mem/protocol:functional",
        "//src/mem/protocol:timing",
        "//src/sim:port",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "port_hh",
    hdrs = ["port.hh"],
    copts = [
        "-Ithird_party/gem5/develop/src",
        "-fexceptions",
    ],
    features = ["-use_header_modules"],
    deps = [
        ":debug_PortTrace",
        ":debug_ResponsePort",
        ":packet_hh",
        "//src/base:addr_range",
        "//src/base:trace",
        "//src/mem/protocol:atomic",
        "//src/mem/protocol:functional",
        "//src/mem/protocol:timing",
        "//src/sim:port",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "packet_queue",
    srcs = ["packet_queue.cc"],
    hdrs = ["packet_queue.hh"],
    deps = [
        ":debug_PacketQueue",
        ":port",
        "//src/base:trace",
        "//src/sim:debug_Drain",
        "//src/sim:drain",
        "//src/sim:eventq",
    ],
)

gem5_cc_library(
    name = "qport",
    hdrs = ["qport.hh"],
    deps = [
        ":packet_queue",
        ":port",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "tport",
    srcs = ["tport.cc"],
    hdrs = ["tport.hh"],
    deps = [
        ":qport",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "AbstractMemory_py",
    srcs = ["AbstractMemory.py"],
    deps = [
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "SimpleMemory_py",
    srcs = ["SimpleMemory.py"],
    deps = [
        "//src/python/m5/objects:AbstractMemory",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "simple_memory_sim_object",
    extra_deps = [
        "//src/sim:sim_object",
    ],
    extra_param_deps = [
        ":abstract_memory_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:AbstractMemory",
    ],
    sim_object_header = ":simple_mem",
    sim_object_module = "src.mem.SimpleMemory",
    sim_object_name = "SimpleMemory",
    sim_object_py_lib = ":SimpleMemory_py",
)

gem5_cc_library(
    name = "simple_mem",
    srcs = ["simple_mem.cc"],
    hdrs = ["simple_mem.hh"],
    deps = [
        ":abstract_mem",
        ":port",
        ":simple_memory_sim_object",
        "//src/base:random",
        "//src/base:trace",
        "//src/sim:debug_Drain",
    ],
)

gem5_cc_library(
    name = "abstract_mem_hh",
    hdrs = ["abstract_mem.hh"],
    deps = [
        ":abstract_memory_sim_object",
        ":backdoor",
        ":port_hh",
        "//src/sim:clocked_object_hh",
        "//src/sim:stats_hh",
    ],
)

gem5_cc_library(
    name = "abstract_mem",
    srcs = ["abstract_mem.cc"],
    hdrs = ["abstract_mem.hh"],
    deps = [
        ":abstract_mem_hh",
        ":abstract_memory_sim_object",
        ":backdoor",
        ":debug_LLSC",
        ":debug_MemoryAccess",
        ":packet_access",
        ":port",
        "//src/base/loader:memory_image",
        "//src/base/loader:object_file",
        "//src/cpu:thread_context",
        "//src/sim:clocked_object",
        "//src/sim:stats",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "abstract_memory_sim_object",
    extra_deps = [
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/base:addr_range",
        "//src/sim:clocked_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:PowerState",
    ],
    sim_object_header = ":abstract_mem",
    sim_object_module = "src.mem.AbstractMemory",
    sim_object_name = "AbstractMemory",
    sim_object_py_lib = ":AbstractMemory_py",
)

gem5_cc_library(
    name = "physical_hh",
    hdrs = ["physical.hh"],
    deps = [
        ":packet_hh",
        "//src/base:addr_range",
        "//src/base:addr_range_map",
        "//src/sim:serialize",
        "//src/sim:serialize_handlers_hh",
    ],
)

gem5_cc_library(
    name = "physical",
    srcs = ["physical.cc"],
    hdrs = ["physical.hh"],
    deps = [
        ":abstract_mem",
        ":debug_AddrRanges",
        ":packet",
        ":physical_hh",
        "//src/base:addr_range",
        "//src/base:addr_range_map",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/sim:debug_Checkpoint",
        "//src/sim:serialize",
        "//src/sim:serialize_handlers_hh",
        "//src/sim:sim_exit",
        "@zlib//:zlib",
    ],
)

gem5_cc_library(
    name = "mem_requestor",
    hdrs = ["mem_requestor.hh"],
    deps = [
        ":request",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "translation_gen",
    hdrs = ["translation_gen.hh"],
    deps = [
        ":request",
        "//src/base:logging",
        "//src/base:types",
    ],
)

gem5_cc_test(
    name = "translation_gen_test",
    srcs = ["translation_gen.test.cc"],
    deps = [
        ":translation_gen",
        "//ext/googletest:gtest_main",
        "//src/base:cprintf",
        "//ext/googletest:gtest",
    ],
)

py_library(
    name = "XBar_py",
    srcs = ["XBar.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:System",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "snoop_filter",
    srcs = ["snoop_filter.cc"],
    hdrs = ["snoop_filter.hh"],
    deps = [
        ":debug_SnoopFilter",
        ":packet_hh",
        ":port",
        ":qport",
        ":snoop_filter_sim_object",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:sim_object",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "snoop_filter_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object_hh",
        "//src/sim:system_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":snoop_filter",
    sim_object_module = "src.mem.XBar",
    sim_object_name = "SnoopFilter",
    sim_object_py_lib = ":XBar_py",
)

gem5_cc_library(
    name = "xbar",
    srcs = ["xbar.cc"],
    hdrs = ["xbar.hh"],
    deps = [
        ":base_x_bar_sim_object",
        ":debug_AddrRanges",
        ":debug_XBar",
        ":qport",
        "//src/base:addr_range_map",
        "//src/base:logging",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:clocked_object",
        "//src/sim:debug_Drain",
        "//src/sim:stats",
    ],
)

gem5_sim_object(
    name = "base_x_bar_sim_object",
    extra_param_deps = [
        "//src/sim:clocked_object_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:ClockedObject",
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":xbar",
    sim_object_module = "src.mem.XBar",
    sim_object_name = "BaseXBar",
    sim_object_py_lib = ":XBar_py",
)

gem5_cc_library(
    name = "noncoherent_xbar",
    srcs = ["noncoherent_xbar.cc"],
    hdrs = ["noncoherent_xbar.hh"],
    deps = [
        ":debug_NoncoherentXBar",
        ":debug_XBar",
        ":noncoherent_x_bar_sim_object",
        ":xbar",
        "//src/base:logging",
        "//src/base:trace",
    ],
)

gem5_sim_object(
    name = "noncoherent_x_bar_sim_object",
    extra_param_deps = [
        ":base_x_bar_sim_object",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":noncoherent_xbar",
    sim_object_module = "src.mem.XBar",
    sim_object_name = "NoncoherentXBar",
    sim_object_py_lib = ":XBar_py",
)

gem5_cc_library(
    name = "coherent_xbar",
    srcs = ["coherent_xbar.cc"],
    hdrs = ["coherent_xbar.hh"],
    deps = [
        "debug_AddrRanges",
        ":coherent_x_bar_sim_object",
        ":debug_CoherentXBar",
        ":snoop_filter",
        ":xbar",
        "//src/base:compiler",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "coherent_x_bar_sim_object",
    extra_param_deps = [
        ":base_x_bar_sim_object",
        ":snoop_filter_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object_hh",
        "//src/sim:system_hh",
        ":snoop_filter",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":coherent_xbar",
    sim_object_module = "src.mem.XBar",
    sim_object_name = "CoherentXBar",
    sim_object_py_lib = ":XBar_py",
)

gem5_cc_library(
    name = "page_table_hh",
    hdrs = ["page_table.hh"],
    deps = [
        ":request",
        ":translation_gen",
        "//src/base:bitfield",
        "//src/base:intmath",
        "//src/base:types",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "page_table",
    srcs = ["page_table.cc"],
    hdrs = ["page_table.hh"],
    deps = [
        ":debug_MMU",
        ":request",
        ":translation_gen",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:intmath",
        "//src/base:trace",
        "//src/base:types",
        "//src/sim:faults_hh",
        "//src/sim:serialize",
    ],
)

gem5_cc_library(
    name = "translating_port_proxy",
    srcs = ["translating_port_proxy.cc"],
    hdrs = ["translating_port_proxy.hh"],
    deps = [
        ":port_proxy",
        "//src/arch/generic:mmu",
        "//src/base:chunk_generator",
        "//src/cpu:base_hh",
        "//src/cpu:thread_context_hh",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "se_translating_port_proxy",
    srcs = ["se_translating_port_proxy.cc"],
    hdrs = ["se_translating_port_proxy.hh"],
    deps = [
        ":translating_port_proxy",
        "//src/sim:process_hh",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "se_translating_port_proxy_hh",
    hdrs = ["se_translating_port_proxy.hh"],
    deps = [
        ":translating_port_proxy",
    ],
)

py_library(
    name = "NVMInterface_py",
    srcs = ["NVMInterface.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:DRAMInterface",
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/objects:MemInterface",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "nvm_interface_hh",
    hdrs = ["nvm_interface.hh"],
    deps = [
        ":mem_interface_hh",
        ":nvm_interface_sim_object",
    ],
)

gem5_cc_library(
    name = "nvm_interface",
    srcs = ["nvm_interface.cc"],
    hdrs = ["nvm_interface.hh"],
    deps = [
        ":debug_NVM",
        ":mem_ctrl_hh",
        ":mem_interface_hh",
        ":nvm_interface_hh",
        ":nvm_interface_sim_object",
        "//src/base:bitfield",
        "//src/base:cprintf",
        "//src/base:trace",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "nvm_interface_sim_object",
    extra_deps = [
        ":nvm_interface",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":mem_interface_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:DRAMInterface",
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/objects:MemInterface",
    ],
    sim_object_header = ":nvm_interface_hh",
    sim_object_module = "src.mem.NVMInterface",
    sim_object_name = "NVMInterface",
    sim_object_py_lib = ":NVMInterface_py",
)

py_library(
    name = "MemCtrl_py",
    srcs = ["MemCtrl.py"],
    deps = [
        "//src/python/m5:citations",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:QoSMemCtrl",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_MemSched",
    enum_module = "src.mem.MemCtrl",
    enum_name = "MemSched",
    enum_py_lib = ":MemCtrl_py",
)

gem5_sim_object(
    name = "mem_ctrl_sim_object",
    extra_deps = [
        ":mem_interface_hh",
    ],
    extra_param_deps = [
        "//src/mem/qos:qo_s_mem_ctrl_sim_object",
        ":mem_interface_sim_object",
        ":enum_MemSched_hh",
        "//src/sim:sim_object_hh",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:QoSMemCtrl",
    ],
    sim_object_header = ":mem_ctrl",
    sim_object_module = "src.mem.MemCtrl",
    sim_object_name = "MemCtrl",
    sim_object_py_lib = ":MemCtrl_py",
)

gem5_cc_library(
    name = "mem_ctrl_hh",
    hdrs = ["mem_ctrl.hh"],
    deps = [
        ":enum_MemSched_hh",
        ":mem_ctrl_sim_object",
        ":mem_interface_hh",
        ":packet_hh",
        ":port_hh",
        ":qport",
        "//src/base:callback",
        "//src/base:statistics",
        "//src/mem/qos:mem_ctrl_qos_hh",
        "//src/sim:eventq",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "mem_ctrl",
    srcs = ["mem_ctrl.cc"],
    hdrs = ["mem_ctrl.hh"],
    deps = [
        ":debug_DRAM",
        ":debug_MemCtrl",
        ":debug_NVM",
        ":dram_interface_hh",
        ":enum_MemSched_hh",
        ":mem_ctrl_hh",
        ":mem_ctrl_sim_object",
        ":mem_interface_hh",
        ":nvm_interface_hh",
        ":packet_hh",
        ":qport",
        "//src/base:callback",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/mem/qos:debug_QOS",
        "//src/mem/qos:mem_ctrl_qos",
        "//src/sim:debug_Drain",
        "//src/sim:eventq",
        "//src/sim:stats",
        "//src/sim:system_hh",
    ],
)

py_library(
    name = "HBMCtrl_py",
    srcs = ["HBMCtrl.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:DRAMInterface",
        "//src/python/m5/objects:MemCtrl",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "hbm_ctrl_sim_object",
    extra_deps = [
        ":dram_interface_hh",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":dram_interface_sim_object",
        ":mem_ctrl_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:DRAMInterface",
        "//src/python/m5/objects:MemCtrl",
    ],
    sim_object_header = ":hbm_ctrl",
    sim_object_module = "src.mem.HBMCtrl",
    sim_object_name = "HBMCtrl",
    sim_object_py_lib = ":HBMCtrl_py",
)

gem5_cc_library(
    name = "hbm_ctrl_hh",
    hdrs = ["hbm_ctrl.hh"],
    deps = [
        ":hbm_ctrl_sim_object",
        ":mem_ctrl_hh",
    ],
)

gem5_cc_library(
    name = "hbm_ctrl",
    srcs = ["hbm_ctrl.cc"],
    hdrs = ["hbm_ctrl.hh"],
    deps = [
        ":debug_DRAM",
        ":debug_MemCtrl",
        ":dram_interface_hh",
        ":hbm_ctrl_hh",
        ":hbm_ctrl_sim_object",
        ":mem_ctrl_hh",
        ":mem_interface_hh",
        "//src/base:trace",
        "//src/mem/qos:debug_QOS",
        "//src/sim:debug_Drain",
        "//src/sim:system_hh",
    ],
)

gem5_cc_library(
    name = "backdoor_manager_hh",
    hdrs = ["backdoor_manager.hh"],
    deps = [
        ":backdoor",
        ":packet_hh",
        "//src/base:addr_range",
    ],
)

gem5_cc_library(
    name = "backdoor_manager",
    srcs = ["backdoor_manager.cc"],
    hdrs = ["backdoor_manager.hh"],
    deps = [
        ":backdoor",
        ":backdoor_manager_hh",
        ":packet_hh",
        "//src/base:logging",
    ],
)

gem5_cc_test(
    name = "backdoor_manager_test",
    srcs = ["backdoor_manager.test.cc"],
    deps = [
        ":backdoor",
        ":backdoor_manager",
        "//ext/googletest:gtest_main",
        "//src/base:addr_range",
        "//ext/googletest:gtest",
    ],
)

py_library(
    name = "AddrMapper_py",
    srcs = ["AddrMapper.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_cc_library(
    name = "addr_mapper_hh",
    hdrs = ["addr_mapper.hh"],
    deps = [
        ":addr_mapper_sim_object",
        ":backdoor_manager_hh",
        ":packet_hh",
        ":port_hh",
        ":range_addr_mapper_sim_object",
        "//src/base:addr_range",
        "//src/sim:sim_object_hh",
    ],
)

gem5_cc_library(
    name = "addr_mapper",
    srcs = ["addr_mapper.cc"],
    hdrs = ["addr_mapper.hh"],
    deps = [
        ":addr_mapper_hh",
        ":addr_mapper_sim_object",
        ":backdoor_manager",
        ":backdoor_manager_hh",
        ":packet_hh",
        ":port_hh",
        ":range_addr_mapper_sim_object",
        "//src/base:addr_range",
        "//src/sim:sim_object_hh",
    ],
)

gem5_sim_object(
    name = "addr_mapper_sim_object",
    extra_deps = [
        ":addr_mapper",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:SimObject",
    ],
    sim_object_header = ":addr_mapper_hh",
    sim_object_module = "src.mem.AddrMapper",
    sim_object_name = "AddrMapper",
    sim_object_py_lib = ":AddrMapper_py",
)

gem5_sim_object(
    name = "range_addr_mapper_sim_object",
    extra_deps = [
        ":addr_mapper",
        "//src/sim:sim_object_hh",
    ],
    extra_param_deps = [
        ":addr_mapper_sim_object",
        "//src/base:addr_range",
    ],
    extra_tool_deps = [
    ],
    sim_object_header = ":addr_mapper_hh",
    sim_object_module = "src.mem.AddrMapper",
    sim_object_name = "RangeAddrMapper",
    sim_object_py_lib = ":AddrMapper_py",
)
