# Copyright 2025 Google, Inc.
# All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Collection of constants.

This file is a collection of constants to provide information required by
various steps of the BUILD file generation.  Some of them are hardcoded simply
because automation are not yet implemented.  Some dependency handling logic
depends on the coding convention of libraries so are hardcoded here
case-by-case.
"""

BOOL_CONFIG_DECL = (
    'value = select({"//src/generated/flags:%s_is_true": "1",'
    ' "//conditions:default": "0"})'
)
config_headers = {
    "have_valgrind": BOOL_CONFIG_DECL % "have_valgrind",
    "have_fenv": BOOL_CONFIG_DECL % "have_fenv",
    "have_protobuf": BOOL_CONFIG_DECL % "have_protobuf",
    "build_gpu": BOOL_CONFIG_DECL % "build_gpu",
    "have_deprecated_namespace": BOOL_CONFIG_DECL
    % "have_deprecated_namespace",
    "use_posix_clock": BOOL_CONFIG_DECL % "use_posix_clock",
    "have_hdf5": BOOL_CONFIG_DECL % "have_hdf5",
    "have_tuntap": BOOL_CONFIG_DECL % "have_tuntap",
    "have_png": BOOL_CONFIG_DECL % "have_png",
    "use_kvm": BOOL_CONFIG_DECL % "use_kvm",
    "the_gpu_isa": 'value = "None", guard = True, def_name = "TheGpuISA"',
    "have_perf_attr_exclude_host": 'value = "0"',
    "kvm_isa": 'value = "x86"',
}


ext_lib_map = {
    "zfstream.": "@iostream3//:lib",
    "python/m5ImporterCode.hh": "//src/python:m5_importer_code_pyo",
    "fdt.": "@libfdt//:lib",
    "libfdt.": "@libfdt//:lib",
    "gelf.": "@libelf//:lib",
    "gem5/": "//include:hdr",
    "magic_enum/": "@magic_enum//:lib",
    "dnet/": "@dnet//:lib",
    "gdbremote/": "@gdbremote//:lib",
    "pybind11/": "@pybind11//:lib",
    "libdrampower/": "@drampower//:lib",
    "x11keysym/": "@x11keysym//:lib",
    "softfloat.h": "@softfloat//:lib",
}


config_setting_from_tag = {
    "riscv_isa": "use_riscv_isa_is_true",
    "x86_isa": "use_x86_isa_is_true",
    "x86_kvm": False,  # TODO(hchsiao): make a config_setting for this
    "kvm": "use_kvm_is_true",
    "mips_isa": "use_mips_isa_is_true",
    "power_isa": "use_power_isa_is_true",
    "arm_isa": "use_arm_isa_is_true",
    "sparc_isa": "use_sparc_isa_is_true",
    "protobuf": "have_protobuf_is_true",
    "fenv": "have_fenv_is_true",
    "png": "have_png_is_true",
    "hdf5": "have_hdf5_is_true",
    "python": True,
    "gem5_lib": True,
    "gem5_trace": True,
    "gem5_serialize": True,
    "gem5_simobject": True,
    "gem5_drain": True,
    "gem5_events": True,
    "main": True,
    "gtest_lib": True,
}

all_sim_object_targets = "\n".join(
    [
        '"//src/kern:sim_objects",',
        '"//src/proto:sim_objects",',
        '"//src/base:sim_objects",',
        '"//src/python:sim_objects",',
        '"//src/sst:sim_objects",',
        '"//src/mem:sim_objects",',
        '"//src/dev:sim_objects",',
        '"//src/cpu:sim_objects",',
        '"//src/sim:sim_objects",',
        '"//src/arch:sim_objects",',
        '"//src/learning_gem5:sim_objects",',
    ]
)

hardcoded_contents = {
    # TODO(hchsiao): remove this workaround
    (
        "src",
        "OBJS_GOES_HERE",
    ): """
"//src/learning_gem5:lib",
""",  # end of OBJS_GOES_HERE of src
    (
        "src",
        "TARGET_GOES_HERE",
    ): """
cc_binary(
    name = "gem5",
    deps = [
        ":lib",
        "//src/python/m5:sim_objects_lib",
        "//src/proto:manual_pb",
        "//src/generated/enums:lib",
        "//src/generated/params:lib",
        "//src/generated/debug:lib",
        "//src/generated/flags:lib",
        "//src/generated/config:lib",
        %s
    ],
    copts = COMMON_COPTS,
    linkopts = COMMON_LINKOPTS,
    visibility = ["//tests/bazel:__pkg__"],
)"""
    % all_sim_object_targets,  # end of TARGET_GOES_HERE of src
    ("src/base", "OBJS_GOES_HERE"): '":manual_date",',
    ("src/python", "OBJS_GOES_HERE"): '":m5_importer_code_pyo",',
    # TODO(hchsiao): use `proto_library` rule instead of system `protoc`
    (
        "src/proto",
        "TARGET_GOES_HERE",
    ): r"""
cc_library(
    name = "inst_pb",
    srcs = [":inst_proto_cc"],
    hdrs = [":inst_proto_h"],
    include_prefix = "proto",
    visibility = ["//build_tools/bazel/package_group:simulator"],
)
cc_library(
    name = "inst_dep_record_pb",
    srcs = [":inst_dep_record_proto_cc"],
    hdrs = [":inst_dep_record_proto_h"],
    include_prefix = "proto",
    visibility = ["//build_tools/bazel/package_group:simulator"],
)
cc_library(
    name = "packet_pb",
    srcs = [":packet_proto_cc"],
    hdrs = [":packet_proto_h"],
    include_prefix = "proto",
    visibility = ["//build_tools/bazel/package_group:simulator"],
)
genrule(
    name = "inst_proto_h",
    srcs = ["inst.proto"],
    outs = ["inst.pb.h"],
    cmd = " ".join([
        "protoc",
        "--cpp_out",
        "`dirname \"$@\"`",
        "--proto_path",
        "`dirname $(execpath :inst.proto)`",
        "\"$<\"",
    ]),
)
genrule(
    name = "inst_proto_cc",
    srcs = ["inst.proto"],
    outs = ["inst.pb.cc"],
    cmd = " ".join([
        "protoc",
        "--cpp_out",
        "`dirname \"$@\"`",
        "--proto_path",
        "`dirname $(execpath :inst.proto)`",
        "\"$<\"",
    ]),
)
genrule(
    name = "inst_dep_record_proto_h",
    srcs = ["inst_dep_record.proto"],
    outs = ["inst_dep_record.pb.h"],
    cmd = " ".join([
        "protoc",
        "--cpp_out",
        "`dirname \"$@\"`",
        "--proto_path",
        "`dirname $(execpath :inst_dep_record.proto)`",
        "\"$<\"",
    ]),
)
genrule(
    name = "inst_dep_record_proto_cc",
    srcs = ["inst_dep_record.proto"],
    outs = ["inst_dep_record.pb.cc"],
    cmd = " ".join([
        "protoc",
        "--cpp_out",
        "`dirname \"$@\"`",
        "--proto_path",
        "`dirname $(execpath :inst_dep_record.proto)`",
        "\"$<\"",
    ]),
)
genrule(
    name = "packet_proto_h",
    srcs = ["packet.proto"],
    outs = ["packet.pb.h"],
    cmd = " ".join([
        "protoc",
        "--cpp_out",
        "`dirname \"$@\"`",
        "--proto_path",
        "`dirname $(execpath :packet.proto)`",
        "\"$<\"",
    ]),
)
genrule(
    name = "packet_proto_cc",
    srcs = ["packet.proto"],
    outs = ["packet.pb.cc"],
    cmd = " ".join([
        "protoc",
        "--cpp_out",
        "`dirname \"$@\"`",
        "--proto_path",
        "`dirname $(execpath :packet.proto)`",
        "\"$<\"",
    ]),
)""",  # end of TARGET_GOES_HERE of src/proto
    (
        "src/sim",
        "TARGET_GOES_HERE",
    ): r"""
genrule(
    name = "gen_tags_cc",
    srcs = [],
    outs = ["tags.cc"],
    cmd = "$(execpath //build_tools/bazel/hermetic_wrapper:stub_tags_cc) > \"$@\"",
    tools = [
        "//build_tools/bazel/hermetic_wrapper:stub_tags_cc",
    ],
)""",  # end of TARGET_GOES_HERE of src/sim
    (
        "src/python",
        "TARGET_GOES_HERE",
    ): r"""
cc_binary(
	name = "gem5py",
    srcs = [
        "gem5py.cc",
    ],
    deps = ["@pybind11//:lib"],
    copts = COMMON_COPTS,
    linkopts = COMMON_LINKOPTS,
    visibility = ["//build_tools/bazel/package_group:simulator"],
)

cc_binary(
	name = "gem5py_m5",
    srcs = [
        "gem5py_m5.cc",
    ],
	deps = [
        "@pybind11//:lib",
        ":m5_importer_code_pyo",
        ":importer",
        "//src/python/m5:sim_objects_lib",
        %s
    ],
    copts = COMMON_COPTS,
    linkopts = COMMON_LINKOPTS,
    visibility = ["//build_tools/bazel/package_group:simulator"],
)

gem5_py_source(
    name_pyo = "importer_py_pyo",
    name_o = "importer_py_o",
    src = "importer.py",
    module_prefix = "",
)

genrule(
    name = "gen_m5_importer_code",
    srcs = ["importer.py"],
    outs = ["m5ImporterCode.cc", "m5ImporterCode.hh"],
    cmd = " ".join([
        "$(execpath //build_tools/bazel/hermetic_wrapper:scons_blob)",
        "\"$<\"",
        "m5ImporterCode",
        "\"$(RULEDIR)\"",
    ]) + " > /dev/null",
    tools = [
        "//build_tools/bazel/hermetic_wrapper:scons_blob",
        "//site_scons:modules",
        "//build_tools:modules",
        "//src/python/m5:modules",
    ],
)

cc_library(
    name = "m5_importer_code_pyo",
    srcs = ["m5ImporterCode.cc"],
    hdrs = ["m5ImporterCode.hh"],
    include_prefix = "python",
)"""
    % all_sim_object_targets,  # end of TARGET_GOES_HERE of src/python
    (
        "src/python/m5/objects",
        "TARGET_GOES_HERE",
    ): r"""
exports_files([
    "__init__.py",
    "SimObject.py",
])
py_sources = [
    macro_py_source('m5.objects', '__init__.py'),
]
cc_library(
    name = "sim_objects_lib",
    deps = py_sources + sim_objects,
    include_prefix = "python/m5/objects",
    visibility = ["//build_tools/bazel/package_group:sim_object_core"],
)""",  # end of TARGET_GOES_HERE of src/python/m5/objects
    # TODO(hchsiao): try writing rules to invoke `gem5py` or use real py rules
    (
        "src/python/m5",
        "TARGET_GOES_HERE",
    ): r"""
genrule(
    name = "gen_defines_py",
    srcs = [],
    outs = ["defines.py"],
    cmd = "$(execpath //build_tools/bazel/hermetic_wrapper:stub_defines_py) > \"$@\"",
    tools = [
        "//build_tools/bazel/hermetic_wrapper:stub_defines_py",
    ],
)

# Not using py_binary because python runtime is gem5py
# This is bad because *.py are now opaque to bazel
# And Python's own dependency management is weak
filegroup(
    name = "modules",
    srcs = [
        "defines.py",
        "info.py",
        "params.py",
        "main.py",
        "options.py",
        "__init__.py",
        "citations.py",
        "ticks.py",
        "event.py",
        "SimObject.py",
        "core.py",
        "debug.py",
        "trace.py",
        "proxy.py",
        "simulate.py",
        "stats/__init__.py",
        "stats/gem5stats.py",
        "internal/params.py",
        "internal/__init__.py",
        "//src/python/m5/objects:__init__.py",
        "//src/python/m5/objects:SimObject.py",
        "ext/pyfdt/__init__.py",
        "ext/pyfdt/pyfdt.py",
        "ext/__init__.py",
        "ext/pystats/storagetype.py",
        "ext/pystats/__init__.py",
        "ext/pystats/timeconversion.py",
        "ext/pystats/group.py",
        "ext/pystats/statistic.py",
        "ext/pystats/jsonloader.py",
        "ext/pystats/simstat.py",
        "ext/pystats/serializable_stat.py",
        "ext/pystats/abstract_stat.py",
        "util/__init__.py",
        "util/fdthelper.py",
        "util/dot_writer_ruby.py",
        "util/attrdict.py",
        "util/pybind.py",
        "util/multidict.py",
        "util/convert.py",
        "util/terminal.py",
        "util/terminal_formatter.py",
        "util/dot_writer.py",
    ],
    visibility = [
        "//build_tools/bazel/package_group:sim_object_core",
        "//build_tools/bazel/hermetic_wrapper:__pkg__",
    ],
)

genrule(
    name = "info_py",
    srcs = [
        "//:COPYING",
        "//:LICENSE",
        "//:README.md",
    ],
    outs = ["info.py"],
    cmd = " ".join([
        "PYTHONPATH=$(execpath //build_tools:infopy.py)/..",
        "$(execpath //src/python:gem5py)",
        "$(execpath //build_tools:infopy.py)",
        "\"$@\"",
        "$(SRCS)",
    ]),
    tools = [
        "//src/python:gem5py",
        "//build_tools:modules", # `import` dependencies
        "//build_tools:infopy.py", # For $(execpath ...)
    ],
    visibility = ["//build_tools/bazel/package_group:simulator"],
)

py_sources = [
    macro_py_source('m5', '__init__.py'),
    macro_py_source('m5', 'SimObject.py'),
    macro_py_source('m5', 'citations.py'),
    macro_py_source('m5', 'core.py'),
    macro_py_source('m5', 'debug.py'),
    macro_py_source('m5', 'event.py'),
    macro_py_source('m5', 'main.py'),
    macro_py_source('m5', 'options.py'),
    macro_py_source('m5', 'params.py'),
    macro_py_source('m5', 'proxy.py'),
    macro_py_source('m5', 'simulate.py'),
    macro_py_source('m5', 'ticks.py'),
    macro_py_source('m5', 'trace.py'),
    macro_py_source('m5', 'defines.py'),
    macro_py_source('m5', 'info.py'),

    macro_py_source('m5.stats', 'stats/__init__.py'),
    macro_py_source('m5.util', 'util/__init__.py'),
    macro_py_source('m5.util', 'util/attrdict.py'),
    macro_py_source('m5.util', 'util/convert.py'),
    macro_py_source('m5.util', 'util/dot_writer.py'),
    macro_py_source('m5.util', 'util/dot_writer_ruby.py'),
    macro_py_source('m5.util', 'util/fdthelper.py'),
    macro_py_source('m5.util', 'util/multidict.py'),
    macro_py_source('m5.util', 'util/pybind.py'),
    macro_py_source('m5.util', 'util/terminal.py'),
    macro_py_source('m5.util', 'util/terminal_formatter.py'),

    macro_py_source('m5.internal', 'internal/__init__.py'),
    macro_py_source('m5.internal', 'internal/params.py'),
    macro_py_source('m5.ext', 'ext/__init__.py'),
    macro_py_source('m5.ext.pyfdt', 'ext/pyfdt/pyfdt.py'),
    macro_py_source('m5.ext.pyfdt', 'ext/pyfdt/__init__.py'),

    macro_py_source('m5.ext.pystats', 'ext/pystats/__init__.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/serializable_stat.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/abstract_stat.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/group.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/simstat.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/statistic.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/storagetype.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/timeconversion.py'),
    macro_py_source('m5.ext.pystats', 'ext/pystats/jsonloader.py'),
    macro_py_source('m5.stats', 'stats/gem5stats.py'),
]

cc_library(
    name = "sim_objects_lib",
    deps = py_sources + [
        "//src/python/m5/objects:sim_objects_lib",
    ],
    include_prefix = "python/m5",
    visibility = ["//build_tools/bazel/package_group:sim_object_core"],
)""",  # end of TARGET_GOES_HERE of src/python/m5
}
