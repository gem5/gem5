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

load("@rules_cc//cc:defs.bzl", "cc_library")

BASIC_COPTS = [
    "-std=c++17",
    "-gz",
    "-pipe",
    "-fno-strict-aliasing",
    "-Wall",
    "-Wundef",
    "-Wextra",
    "-Wno-sign-compare",
    "-Wno-unused-parameter",
] + select({
    "//build_tools/bazel:python_3_11": ["-I/usr/include/python3.11"],
    "//build_tools/bazel:python_3_12": ["-I/usr/include/python3.12"],
    "//conditions:default": [],
})
COMMON_COPTS = BASIC_COPTS + [
    "-fno-builtin-malloc",
    "-fno-builtin-calloc",
    "-fno-builtin-realloc",
    "-fno-builtin-free",
    "-DPROTOBUF_INLINE_NOT_IN_HEADERS=0",
    "-ggdb3",
    "-DMAGIC_ENUM_RANGE_MAX=0x100",
    "-DNUMBER_BITS_PER_SET=64",
    "-DGEM5_DEBUG",
    "-DTRACING_ON=1",
]
COMMON_LINKOPTS = [
    # TODO(hchsiao): avoid explicit paths
    "-L/usr/lib/x86_64-linux-gnu",
    "-ldl",
    "-lm",
    "-lz",
    "-lprotobuf",
] + select({
    "//build_tools/bazel:python_3_11": [
        "-L/usr/lib/python3.11/config-3.11-x86_64-linux-gnu",
        "-lpython3.11",
    ],
    "//build_tools/bazel:python_3_12": [
        "-L/usr/lib/python3.12/config-3.12-x86_64-linux-gnu",
        "-lpython3.12",
    ],
    "//conditions:default": [],
})

def get_py_name(dot_py):
    if dot_py.startswith(":"):
        dot_py = dot_py[1:]
    if dot_py.startswith("//"):
        dot_py = dot_py.split(":")[1]
    if not dot_py.endswith(".py"):
        fail("%s should end with '.py'" % dot_py)
    return dot_py[:-3].replace("/", "_")

def _derive_module(prefix, dot_py):
    if not dot_py.endswith(".py"):
        fail("%s should end with '.py'" % dot_py)
    if prefix.endswith("."):
        prefix = prefix[:-1]
    py_file = dot_py.rsplit("/")[-1]
    if py_file == "__init__.py":
        return prefix
    if prefix:
        prefix += "."
    return prefix + py_file[:-3]

def py_cc(name, src, module_prefix):
    native.genrule(
        name = name,
        srcs = [src],
        outs = [src + ".cc"],
        cmd = " ".join([
            "PYTHONPATH=$(execpath //build_tools:marshal.py)/..",
            "$(execpath //src/python:gem5py)",
            "$(execpath //build_tools:marshal.py)",
            "\"$@\"",
            "\"$<\"",
            _derive_module(module_prefix, src),
            "`realpath \"$<\"`",
        ]),
        tools = [
            "//src/python:gem5py",
            "//build_tools:modules",
            "//build_tools:marshal.py",
        ],
    )

# TODO(hchsiao): write customized rule with multiple outputs
def gem5_py_source(name_pyo, name_o, src, module_prefix):
    py_name = get_py_name(src)
    name_cc = py_name + "_cc"
    py_cc(
        name = name_cc,
        src = src,
        module_prefix = module_prefix,
    )
    cc_library(
        name = name_pyo,
        deps = [
            "@pybind11//:lib",
            "//src/python:embedded",
        ],
        alwayslink = True,
        copts = BASIC_COPTS,
        srcs = [name_cc],
    )
    cc_library(
        name = name_o,  # TODO(hchsiao): can we remove this target?
        srcs = [name_cc],
        #copts = COMMON_COPTS,  # TODO(hchsiao): is this needed?
        #alwayslink = True,  # TODO(hchsiao): is this needed?
    )

def sim_object(name_o, name_pyo, src, sim_objects, enums):
    gem5_py_source(
        name_pyo = name_pyo,
        name_o = name_o,
        src = src,
        module_prefix = "m5.objects",
    )
    for enum in enums:
        name_hh = enum + "_enum_hh"
        name_cc = enum + "_enum_cc"
        out_hh = enum + ".hh"
        out_cc = enum + ".cc"
        native.genrule(
            name = name_hh,
            srcs = [src],
            outs = [out_hh],
            cmd = " ".join([
                "PYTHONPATH=$(execpath //build_tools:enum_hh.py)/..",
                "$(execpath //src/python:gem5py_m5)",
                "$(execpath //build_tools:enum_hh.py)",
                "m5.objects." + get_py_name(src),
                "\"$@\"",
            ]),
            tools = [
                "//src/python:gem5py_m5",
                "//build_tools:modules",  # `import` dependencies
                "//build_tools:enum_hh.py",  # For $(execpath ...)
            ],
            visibility = ["//build_tools/bazel/package_group:simulator"],
        )
        native.genrule(
            name = name_cc,
            srcs = [src],
            outs = [out_cc],
            cmd = " ".join([
                "PYTHONPATH=$(execpath //build_tools:enum_cc.py)/..",
                "$(execpath //src/python:gem5py_m5)",
                "$(execpath //build_tools:enum_cc.py)",
                "m5.objects." + get_py_name(src),
                "\"$@\"",
                "True",  # USE_PYTHON (assumed always true)
            ]),
            tools = [
                "//src/python:gem5py_m5",
                "//build_tools:modules",  # `import` dependencies
                "//build_tools:enum_cc.py",  # For $(execpath ...)
            ],
            visibility = ["//build_tools/bazel/package_group:simulator"],
        )
    for sim_object in sim_objects:
        name_hh = sim_object + "_param_hh"
        name_cc = sim_object + "_param_cc"
        out_hh = sim_object + ".hh"
        out_cc = "param_" + sim_object + ".cc"
        native.genrule(
            name = name_hh,
            srcs = [src],
            outs = [out_hh],
            cmd = " ".join([
                "PYTHONPATH=$(execpath //build_tools:sim_object_param_struct_hh.py)/..",
                "$(execpath //src/python:gem5py_m5)",
                "$(execpath //build_tools:sim_object_param_struct_hh.py)",
                "m5.objects." + get_py_name(src),
                "\"$@\"",
            ]),
            tools = [
                "//src/python:gem5py_m5",
                "//build_tools:modules",  # `import` dependencies
                "//build_tools:sim_object_param_struct_hh.py",  # For $(execpath ...)
            ],
            visibility = ["//build_tools/bazel/package_group:simulator"],
        )
        native.genrule(
            name = name_cc,
            srcs = [src],
            outs = [out_cc],
            cmd = " ".join([
                "PYTHONPATH=$(execpath //build_tools:sim_object_param_struct_cc.py)/..",
                "$(execpath //src/python:gem5py_m5)",
                "$(execpath //build_tools:sim_object_param_struct_cc.py)",
                "m5.objects." + get_py_name(src),
                "\"$@\"",
                "True",  # USE_PYTHON (assumed always true)
            ]),
            tools = [
                "//src/python:gem5py_m5",
                "//build_tools:modules",  # `import` dependencies
                "//build_tools:sim_object_param_struct_cc.py",  # For $(execpath ...)
            ],
            visibility = ["//build_tools/bazel/package_group:simulator"],
        )

def debug_flag(name, description, flags, is_fmt = False):
    native.genrule(
        name = name + "_hh",
        srcs = [],
        outs = [name + ".hh"],
        cmd = " ".join([
            "PYTHONPATH=$(execpath //build_tools:debugflaghh.py)/..",
            "$(execpath //src/python:gem5py)",
            "$(execpath //build_tools:debugflaghh.py)",
            "\"$@\"",
            "\"%s\"" % name,
            "\"%s\"" % description,
            "True" if is_fmt else "False",
            "\"%s\"" % ":".join(flags),
        ]),
        tools = [
            "//src/python:gem5py",
            "//build_tools:modules",  # `import` dependencies
            "//build_tools:debugflaghh.py",  # For $(execpath ...)
        ],
        visibility = ["//build_tools/bazel/package_group:simulator"],
    )
    native.genrule(
        name = name + "_cc",
        srcs = [],
        outs = [name + ".cc"],
        cmd = " ".join([
            "PYTHONPATH=$(execpath //build_tools:debugflagcc.py)/..",
            "$(execpath //src/python:gem5py)",
            "$(execpath //build_tools:debugflagcc.py)",
            "\"$@\"",
            "\"%s\"" % name,
        ]),
        tools = [
            "//src/python:gem5py",
            "//build_tools:modules",  # `import` dependencies
            "//build_tools:debugflagcc.py",  # For $(execpath ...)
        ],
        visibility = ["//build_tools/bazel/package_group:simulator"],
    )
    cc_library(
        name = name,
        srcs = [name + "_cc"],
        hdrs = [name + "_hh"],
        deps = ["//src/base:compiler", "//src/base:debug"] + ["//src/generated/debug:%s" % flag for flag in flags],
        copts = COMMON_COPTS,  # TODO(hchsiao): is this needed?
        #alwayslink = True, # TODO(hchsiao): is this needed?
        include_prefix = "debug",
        visibility = ["//build_tools/bazel/package_group:simulator"],
    )

# TODO(hchsiao): write a customized rule to make use of string_flag
def gen_config(name, value, guard = False, def_name = None):
    if not def_name:
        def_name = name
    if guard:
        cmd = "\n".join([
            "cat <<EOF > \"$@\"",
            "#ifndef %s" % def_name,
            "#define %s " % def_name,
        ]) + value + "\n" + "\n".join([
            "#endif // %s" % def_name,
            "EOF",
        ])
    else:
        cmd = ("echo \"#define %s" % def_name.upper() + " " +
               value + "\" > \"$@\"")
    native.genrule(
        name = name + "_gen",
        srcs = [],
        outs = [name + ".hh"],
        cmd = cmd,
    )
    cc_library(
        name = name,
        srcs = [],
        hdrs = [name + "_gen"],
        include_prefix = "config",
        visibility = ["//build_tools/bazel/package_group:simulator"],
    )
