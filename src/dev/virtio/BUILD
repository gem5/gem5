load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_VIO",
    desc = "",
    flag = "VIO",
)

py_library(
    name = "virtio_init",
    srcs = ["__init__.py"],
    deps = [
        ":VirtIO_py",
        ":virt_io9p",
        ":virt_io_block",
        ":virt_io_console",
        ":virt_io_rng",
    ],
)

gem5_debug_flag(
    name = "debug_VIOConsole",
    desc = "",
    flag = "VIOConsole",
)

gem5_debug_flag(
    name = "debug_VIORng",
    desc = "",
    flag = "VIORng",
)

gem5_debug_flag(
    name = "debug_VIO9P",
    desc = "",
    flag = "VIO9P",
)

gem5_debug_flag(
    name = "debug_VIO9PData",
    desc = "",
    flag = "VIO9PData",
)

gem5_debug_flag(
    name = "debug_VIOIface",
    desc = "",
    flag = "VIOIface",
)

gem5_debug_flag(
    name = "debug_VIOBlock",
    desc = "VirtIO Block device",
    flag = "VIOBlock",
)

gem5_cc_library(
    name = "virtio_ring",
    hdrs = ["virtio_ring.h"],
)

gem5_cc_library(
    name = "base",
    srcs = ["base.cc"],
    hdrs = ["base.hh"],
    deps = [
        ":debug_VIO",
        ":virt_io_device_base_sim_object",
        ":virt_io_dummy_device_sim_object",
        ":virtio_ring",
        "//src/base:bitunion",
        "//src/base:compiler",
        "//src/base:trace",
        "//src/base:types",
        "//src/mem:port_proxy",
        "//src/sim:serialize",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "pci",
    srcs = ["pci.cc"],
    hdrs = ["pci.hh"],
    deps = [
        ":base",
        ":debug_VIOIface",
        ":pci_virt_io_sim_object",
        "//src/base:bitfield",
        "//src/base:compiler",
        "//src/base:statistics_hh",
        "//src/dev/pci:device",
        "//src/mem:packet_access",
    ],
)

py_library(
    name = "VirtIO_py",
    srcs = ["VirtIO.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Device",
        "//src/python/m5/objects:PciDevice",
        "//src/python/m5/params",
    ],
)

gem5_sim_object(
    name = "virt_io_device_base_sim_object",
    extra_deps = [
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        "//src/sim:enum_ByteOrder_hh",
        "//src/sim:sim_object_sim_object",
        "//src/sim:system_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:System",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.dev.virtio.VirtIO",
    sim_object_name = "VirtIODeviceBase",
    sim_object_py_lib = ":VirtIO_py",
)

gem5_sim_object(
    name = "virt_io_dummy_device_sim_object",
    extra_param_deps = [
        ":virt_io_device_base_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":base",
    sim_object_module = "src.dev.virtio.VirtIO",
    sim_object_name = "VirtIODummyDevice",
    sim_object_py_lib = ":VirtIO_py",
)

gem5_sim_object(
    name = "pci_virt_io_sim_object",
    extra_deps = [
        ":base",
    ],
    extra_param_deps = [
        ":virt_io_device_base_sim_object",
        "//src/dev/pci:pci_endpoint_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciDevice",
    ],
    sim_object_header = ":pci",
    sim_object_module = "src.dev.virtio.VirtIO",
    sim_object_name = "PciVirtIO",
    sim_object_py_lib = ":VirtIO_py",
)

gem5_sim_object(
    name = "virt_io_block_sim_object",
    extra_deps = [
        "//src/dev/storage:disk_image",
    ],
    extra_param_deps = [
        ":virt_io_device_base_sim_object",
        "//src/dev/storage:disk_image_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:DiskImage",
        "//src/python/m5/objects:VirtIO",
    ],
    sim_object_header = ":block",
    sim_object_module = "src.dev.virtio.VirtIOBlock",
    sim_object_name = "VirtIOBlock",
    sim_object_py_lib = ":virt_io_block",
)

gem5_cc_library(
    name = "block",
    srcs = ["block.cc"],
    hdrs = ["block.hh"],
    deps = [
        ":base",
        ":debug_VIOBlock",
        ":virt_io_block_sim_object",
        "//src/base:compiler",
        "//src/dev/storage:disk_image",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "virt_io_console_sim_object",
    extra_deps = [
        "//src/sim:system_hh",
        "//src/dev/serial:serial",
    ],
    extra_param_deps = [
        ":virt_io_device_base_sim_object",
        "//src/dev/serial:serial_device_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:Serial",
        "//src/python/m5/objects:VirtIO",
    ],
    sim_object_header = ":console",
    sim_object_module = "src.dev.virtio.VirtIOConsole",
    sim_object_name = "VirtIOConsole",
    sim_object_py_lib = ":virt_io_console",
)

gem5_cc_library(
    name = "console_hh",
    hdrs = ["console.hh"],
    deps = [
        ":virt_io_console_sim_object",
        "//src/base:compiler",
        "//src/dev/serial",
        "//src/dev/virtio:base",
    ],
)

gem5_cc_library(
    name = "console",
    srcs = ["console.cc"],
    hdrs = ["console.hh"],
    deps = [
        ":console_hh",
        ":debug_VIOConsole",
        ":virt_io_console_sim_object",
        "//src/base:compiler",
        "//src/dev/serial",
        "//src/dev/serial:serial_device_sim_object",
        "//src/dev/virtio:base",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "virt_io_rng_sim_object",
    extra_deps = [
        "//src/sim:system_hh",
    ],
    extra_param_deps = [
        ":virt_io_device_base_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:VirtIO",
    ],
    sim_object_header = ":rng",
    sim_object_module = "src.dev.virtio.VirtIORng",
    sim_object_name = "VirtIORng",
    sim_object_py_lib = ":virt_io_rng",
)

gem5_cc_library(
    name = "rng_hh",
    hdrs = ["rng.hh"],
    deps = [
        ":virt_io_rng_sim_object",
        "//src/base:compiler",
        "//src/base:random",
        "//src/dev/virtio:base",
    ],
)

gem5_cc_library(
    name = "rng",
    srcs = ["rng.cc"],
    hdrs = ["rng.hh"],
    deps = [
        ":debug_VIORng",
        ":rng_hh",
        ":virt_io_rng_sim_object",
        "//src/base:compiler",
        "//src/base:random",
        "//src/dev/virtio:base",
        "//src/sim:core",
        "//src/sim:system_hh",
    ],
)

gem5_sim_object(
    name = "virt_io9_p_base_sim_object",
    extra_param_deps = [
        ":virt_io_device_base_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:VirtIO",
    ],
    sim_object_header = ":fs9p",
    sim_object_module = "src.dev.virtio.VirtIO9P",
    sim_object_name = "VirtIO9PBase",
    sim_object_py_lib = ":virt_io9p",
)

gem5_sim_object(
    name = "virt_io9_p_proxy_sim_object",
    extra_param_deps = [
        ":virt_io9_p_base_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":fs9p",
    sim_object_module = "src.dev.virtio.VirtIO9P",
    sim_object_name = "VirtIO9PProxy",
    sim_object_py_lib = ":virt_io9p",
)

gem5_sim_object(
    name = "virt_io9_p_diod_sim_object",
    extra_param_deps = [
        ":virt_io9_p_proxy_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":fs9p",
    sim_object_module = "src.dev.virtio.VirtIO9P",
    sim_object_name = "VirtIO9PDiod",
    sim_object_py_lib = ":virt_io9p",
)

gem5_sim_object(
    name = "virt_io9_p_socket_sim_object",
    extra_param_deps = [
        ":virt_io9_p_proxy_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":fs9p",
    sim_object_module = "src.dev.virtio.VirtIO9P",
    sim_object_name = "VirtIO9PSocket",
    sim_object_py_lib = ":virt_io9p",
)

gem5_cc_library(
    name = "fs9p_hh",
    hdrs = ["fs9p.hh"],
    deps = [
        ":virt_io9_p_base_sim_object",
        ":virt_io9_p_diod_sim_object",
        ":virt_io9_p_proxy_sim_object",
        ":virt_io9_p_socket_sim_object",
        "//src/base:compiler",
        "//src/base:pollevent",
        "//src/dev/virtio:base",
    ],
)

gem5_cc_library(
    name = "fs9p",
    srcs = ["fs9p.cc"],
    hdrs = ["fs9p.hh"],
    deps = [
        ":debug_VIO9P",
        ":debug_VIO9PData",
        ":fs9p_hh",
        ":virt_io9_p_base_sim_object",
        ":virt_io9_p_diod_sim_object",
        ":virt_io9_p_proxy_sim_object",
        ":virt_io9_p_socket_sim_object",
        "//src/base:callback",
        "//src/base:compiler",
        "//src/base:output",
        "//src/base:pollevent",
        "//src/dev/virtio:base",
        "//src/sim:core",
        "//src/sim:system_hh",
    ],
)

py_library(
    name = "virt_io9p",
    srcs = ["VirtIO9P.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:VirtIO",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "virt_io_console",
    srcs = ["VirtIOConsole.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:Serial",
        "//src/python/m5/objects:VirtIO",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "virt_io_rng",
    srcs = ["VirtIORng.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:VirtIO",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "virt_io_block",
    srcs = ["VirtIOBlock.py"],
    deps = [
        "//src/python/m5:proxy",
        "//src/python/m5/objects:VirtIO",
        "//src/python/m5/params",
    ],
)
