load("@rules_python//python:defs.bzl", "py_library")
load("//bazel:rules.bzl", "gem5_cc_library", "gem5_debug_flag", "gem5_enum_generator", "gem5_sim_object")

package(
    default_visibility = ["//visibility:public"],
)

gem5_debug_flag(
    name = "debug_IdeCtrl",
    desc = "",
    flag = "IdeCtrl",
)

gem5_debug_flag(
    name = "debug_IdeDisk",
    desc = "",
    flag = "IdeDisk",
)

gem5_debug_flag(
    name = "debug_DiskImage",
    desc = "",
    flag = "DiskImage",
)

gem5_debug_flag(
    name = "debug_DiskImageRead",
    desc = "",
    flag = "DiskImageRead",
)

gem5_debug_flag(
    name = "debug_DiskImageWrite",
    desc = "",
    flag = "DiskImageWrite",
)

gem5_cc_library(
    name = "ide_ctrl_hh",
    hdrs = [
        "ide_ctrl.hh",
    ],
    deps = [
        ":ide_controller_sim_object",
        "//src/base:bitunion",
        "//src/dev:io_device",
        "//src/dev:reg_bank",
        "//src/dev/pci:device",
    ],
)

gem5_cc_library(
    name = "ide_ctrl",
    srcs = ["ide_ctrl.cc"],
    hdrs = [
        "ide_ctrl.hh",
    ],
    deps = [
        ":debug_IdeCtrl",
        ":ide_controller_sim_object",
        ":ide_ctrl_hh",
        ":ide_disk",
        "//src/base:bitunion",
        "//src/base:cprintf",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/dev:reg_bank",
        "//src/dev/pci:device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:byteswap",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "ide_disk",
    srcs = ["ide_disk.cc"],
    hdrs = [
        "ide_atareg.h",
        "ide_disk.hh",
        "ide_wdcreg.h",
    ],
    deps = [
        ":debug_IdeDisk",
        ":disk_image",
        ":enum_IdeID_hh",
        ":ide_ctrl_hh",
        ":ide_disk_sim_object",
        "//src/base:bitfield",
        "//src/base:chunk_generator",
        "//src/base:compiler",
        "//src/base:cprintf",
        "//src/base:statistics",
        "//src/base:trace",
        "//src/dev:io_device",
        "//src/mem:packet",
        "//src/mem:packet_access",
        "//src/sim:cur_tick",
        "//src/sim:eventq",
        "//src/sim:sim_object",
    ],
)

gem5_cc_library(
    name = "disk_image",
    srcs = ["disk_image.cc"],
    hdrs = ["disk_image.hh"],
    deps = [
        ":cow_disk_image_sim_object",
        ":debug_DiskImage",
        ":debug_DiskImageRead",
        ":debug_DiskImageWrite",
        ":disk_image_sim_object",
        ":raw_disk_image_sim_object",
        "//src/base:callback",
        "//src/base:logging",
        "//src/base:trace",
        "//src/sim:byteswap",
        "//src/sim:serialize",
        "//src/sim:sim_exit",
        "//src/sim:sim_object",
    ],
)

py_library(
    name = "Ide_py",
    srcs = ["Ide.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5:proxy",
        "//src/python/m5/objects:DiskImage",
        "//src/python/m5/objects:PciDevice",
        "//src/python/m5/params",
    ],
)

py_library(
    name = "DiskImage_py",
    srcs = ["DiskImage.py"],
    deps = [
        "//src/python/m5:SimObject",
        "//src/python/m5/params",
    ],
)

gem5_enum_generator(
    name = "enum_IdeID",
    enum_module = "src.dev.storage.Ide",
    enum_name = "IdeID",
    enum_py_lib = ":Ide_py",
)

gem5_sim_object(
    name = "ide_controller_sim_object",
    extra_deps = [
        ":ide_disk",
    ],
    extra_param_deps = [
        ":ide_disk_sim_object",
        "//src/dev/pci:pci_endpoint_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:PciDevice",
    ],
    sim_object_header = ":ide_ctrl",
    sim_object_module = "src.dev.storage.Ide",
    sim_object_name = "IdeController",
    sim_object_py_lib = ":Ide_py",
)

gem5_sim_object(
    name = "ide_disk_sim_object",
    extra_deps = [
        ":disk_image",
    ],
    extra_param_deps = [
        ":enum_IdeID_hh",
        ":disk_image_sim_object",
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:DiskImage",
    ],
    sim_object_header = ":ide_disk",
    sim_object_module = "src.dev.storage.Ide",
    sim_object_name = "IdeDisk",
    sim_object_py_lib = ":Ide_py",
)

gem5_sim_object(
    name = "disk_image_sim_object",
    extra_param_deps = [
        "//src/sim:sim_object_sim_object",
        "//src/sim:sim_object",
    ],
    sim_object_header = ":disk_image",
    sim_object_module = "src.dev.storage.DiskImage",
    sim_object_name = "DiskImage",
    sim_object_py_lib = ":DiskImage_py",
)

gem5_sim_object(
    name = "raw_disk_image_sim_object",
    extra_param_deps = [
        ":disk_image_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:DiskImage",
    ],
    sim_object_header = ":disk_image",
    sim_object_module = "src.dev.storage.DiskImage",
    sim_object_name = "RawDiskImage",
    sim_object_py_lib = ":DiskImage_py",
)

gem5_sim_object(
    name = "cow_disk_image_sim_object",
    extra_param_deps = [
        ":disk_image_sim_object",
        "//src/sim:sim_object",
    ],
    extra_tool_deps = [
        "//src/python/m5/objects:DiskImage",
    ],
    sim_object_header = ":disk_image",
    sim_object_module = "src.dev.storage.DiskImage",
    sim_object_name = "CowDiskImage",
    sim_object_py_lib = ":DiskImage_py",
)
