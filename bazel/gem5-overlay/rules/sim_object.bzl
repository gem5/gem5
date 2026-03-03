"""Rules for SimObject parameter struct and enum generation.

Provides two approaches:
1. gem5_sim_object(): Per-file generation for individual SimObject .py files.
2. gem5_sim_object_aggregate(): Aggregate generation that discovers all SimObject
   classes via m5.objects and generates all params/enums in one pass. This is
   the recommended approach for the top-level build as it mirrors CMake's
   deferred gem5_create_simobject_commands() pattern.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

# ---------------------------------------------------------------------------
# Per-file SimObject generation (kept for fine-grained control if needed)
# ---------------------------------------------------------------------------

def _sim_object_gen_impl(ctx):
    """Generate SimObject parameter structs and enums."""
    py_file = ctx.file.py_file
    gem5py_m5 = ctx.executable.gem5py_m5

    if not ctx.attr.sim_objects and not ctx.attr.enums:
        fail("No sim_objects or enums specified for {}".format(ctx.label.name))

    # Use tree artifact for output (file set depends on SimObject/enum lists)
    output_dir = ctx.actions.declare_directory("simobj_gen_{}".format(ctx.label.name))

    script = ctx.actions.declare_file("_run_simobj_gen_{}.py".format(ctx.label.name))
    ctx.actions.write(
        output = script,
        content = """\
import sys
import os
import subprocess

gem5py_m5 = sys.argv[1]
output_dir = sys.argv[2]
py_file = sys.argv[3]
sim_objects = sys.argv[4].split(",") if sys.argv[4] else []
enums = sys.argv[5].split(",") if sys.argv[5] else []
gen_scripts_dir = sys.argv[6]

os.makedirs(os.path.join(output_dir, "params"), exist_ok=True)
os.makedirs(os.path.join(output_dir, "enums"), exist_ok=True)
os.makedirs(os.path.join(output_dir, "python", "_m5"), exist_ok=True)

for obj in sim_objects:
    hh_script = os.path.join(gen_scripts_dir, "sim_object_param_struct_hh.py")
    hh_out = os.path.join(output_dir, "params", obj + ".hh")
    subprocess.check_call([gem5py_m5, hh_script, obj, hh_out])

    cc_script = os.path.join(gen_scripts_dir, "sim_object_param_struct_cc.py")
    cc_out = os.path.join(output_dir, "python", "_m5", "param_" + obj + ".cc")
    subprocess.check_call([gem5py_m5, cc_script, obj, cc_out])

for enum in enums:
    hh_script = os.path.join(gen_scripts_dir, "enum_hh.py")
    cc_script = os.path.join(gen_scripts_dir, "enum_cc.py")
    hh_out = os.path.join(output_dir, "enums", enum + ".hh")
    cc_out = os.path.join(output_dir, "enums", enum + ".cc")
    subprocess.check_call([gem5py_m5, hh_script, enum, hh_out])
    subprocess.check_call([gem5py_m5, cc_script, enum, cc_out])
""",
    )

    ctx.actions.run_shell(
        outputs = [output_dir],
        inputs = [py_file, script] + ctx.files._gen_scripts,
        tools = [gem5py_m5],
        command = "python3 {} {} {} {} {} {} {}".format(
            script.path,
            gem5py_m5.path,
            output_dir.path,
            py_file.path,
            ",".join(ctx.attr.sim_objects),
            ",".join(ctx.attr.enums),
            ctx.files._gen_scripts[0].dirname if ctx.files._gen_scripts else "",
        ),
        mnemonic = "SimObject",
        progress_message = "Generating SimObject params for {}".format(ctx.label.name),
    )

    return [DefaultInfo(files = depset([output_dir]))]

_sim_object_gen = rule(
    implementation = _sim_object_gen_impl,
    attrs = {
        "py_file": attr.label(mandatory = True, allow_single_file = [".py"]),
        "sim_objects": attr.string_list(default = []),
        "enums": attr.string_list(default = []),
        "gem5py_m5": attr.label(
            mandatory = True,
            executable = True,
            cfg = "exec",
        ),
        "_gen_scripts": attr.label(
            default = "//build_tools:simobj_gen_scripts",
            allow_files = True,
        ),
    },
)

def gem5_sim_object(name, py_file, sim_objects = [], enums = [],
                    gem5py_m5 = "//src/python:gem5py_m5",
                    visibility = None, deps = []):
    """Generate SimObject parameter structs and enums from a .py file.

    Args:
        name: Target name.
        py_file: SimObject .py definition file.
        sim_objects: List of SimObject class names to generate params for.
        enums: List of Enum class names to generate.
        gem5py_m5: Label of the gem5py_m5 executable.
        visibility: Visibility.
        deps: Additional dependencies.
    """
    gen_name = name + "_gen"
    _sim_object_gen(
        name = gen_name,
        py_file = py_file,
        sim_objects = sim_objects,
        enums = enums,
        gem5py_m5 = gem5py_m5,
    )
    cc_library(
        name = name,
        srcs = [gen_name],
        hdrs = [gen_name],
        deps = deps,
        includes = ["."],
        visibility = visibility,
        alwayslink = True,
    )

# ---------------------------------------------------------------------------
# Aggregate SimObject generation (discovers all SimObjects via m5.objects)
# ---------------------------------------------------------------------------

def _sim_object_aggregate_gen_impl(ctx):
    """Discover all SimObject classes and generate all params/enums.

    Produces three separate tree artifacts for explicit output mapping:
      params_hdrs:  params/{ClassName}.hh
      enums:        enums/{EnumName}.hh, enums/{EnumName}.cc
      pybind_srcs:  python/_m5/param_{ClassName}.cc

    All three are inside a single parent directory so that one includes=
    entry exposes params/ and enums/ at the correct include depth.
    """
    gem5py_m5 = ctx.executable.gem5py_m5
    use_python = "True" if ctx.attr.use_python else "False"

    # Single parent tree artifact preserving the params/enums/python/_m5
    # directory structure that source code expects via #include "params/..."
    output_dir = ctx.actions.declare_directory(ctx.label.name + "_out")

    script = ctx.actions.declare_file("_gen_all_simobjects.py")
    ctx.actions.write(
        output = script,
        content = """\
import sys
import os

output_dir = sys.argv[1]
build_tools_dir = sys.argv[2]
use_python = sys.argv[3] == "True"

sys.path.insert(0, build_tools_dir)

from sim_object_param_struct_hh import write_header_file as write_param_hh
from sim_object_param_struct_cc import write_cc_file as write_param_cc
from enum_hh import write_header_file as write_enum_hh
from enum_cc import write_cc_file as write_enum_cc

import m5
import m5.objects
from m5.SimObject import SimObject
from m5.params import Enum

os.makedirs(os.path.join(output_dir, "params"), exist_ok=True)
os.makedirs(os.path.join(output_dir, "enums"), exist_ok=True)
os.makedirs(os.path.join(output_dir, "python", "_m5"), exist_ok=True)

generated_enums = set()

for name in sorted(SimObject.allClasses.keys()):
    cls = SimObject.allClasses[name]
    write_param_hh(cls, os.path.join(output_dir, "params", name + ".hh"))
    write_param_cc(cls, use_python,
                   os.path.join(output_dir, "python", "_m5",
                                "param_" + name + ".cc"))

    for param_name, param in sorted(cls._params.local.items()):
        for ptype in getattr(param, "ptypes", []):
            if issubclass(ptype, Enum) and ptype.__name__ not in generated_enums:
                enum_name = ptype.__name__
                generated_enums.add(enum_name)
                write_enum_hh(ptype,
                              os.path.join(output_dir, "enums",
                                           enum_name + ".hh"))
                write_enum_cc(ptype, use_python,
                              os.path.join(output_dir, "enums",
                                           enum_name + ".cc"))

print("Generated params for", len(SimObject.allClasses), "SimObjects")
print("Generated", len(generated_enums), "enum types")
""",
    )

    gen_scripts = ctx.files._gen_scripts
    build_tools_dir = gen_scripts[0].dirname if gen_scripts else ""

    ctx.actions.run_shell(
        outputs = [output_dir],
        inputs = [script] + gen_scripts,
        tools = [gem5py_m5],
        command = "{} {} {} {} {}".format(
            gem5py_m5.path,
            script.path,
            output_dir.path,
            build_tools_dir,
            use_python,
        ),
        mnemonic = "SimObjectAggregate",
        progress_message = "Generating all SimObject params and enums",
    )

    return [
        DefaultInfo(files = depset([output_dir])),
        OutputGroupInfo(
            params_hdrs = depset([output_dir]),
            enums = depset([output_dir]),
            pybind_srcs = depset([output_dir]),
        ),
    ]

_sim_object_aggregate_gen = rule(
    implementation = _sim_object_aggregate_gen_impl,
    attrs = {
        "gem5py_m5": attr.label(
            mandatory = True,
            executable = True,
            cfg = "exec",
        ),
        "use_python": attr.bool(default = True),
        "_gen_scripts": attr.label(
            default = "//build_tools:simobj_gen_scripts",
            allow_files = True,
        ),
    },
)

def gem5_sim_object_aggregate(name, gem5py_m5 = "//src/python:gem5py_m5",
                               use_python = True, visibility = None, deps = []):
    """Generate all SimObject params and enums in one pass.

    Discovers all SimObject subclasses by importing m5.objects via gem5py_m5,
    then generates parameter struct headers/sources and enum headers/sources
    for all discovered classes. Output is a tree artifact containing:
      params/{ClassName}.hh
      python/_m5/param_{ClassName}.cc
      enums/{EnumName}.hh
      enums/{EnumName}.cc

    Args:
        name: Target name.
        gem5py_m5: Label of the gem5py_m5 executable.
        use_python: Whether to generate pybind11 bindings.
        visibility: Visibility.
        deps: Additional dependencies for the generated cc_library.
    """
    gen_name = name + "_gen"
    _sim_object_aggregate_gen(
        name = gen_name,
        gem5py_m5 = gem5py_m5,
        use_python = use_python,
    )
    cc_library(
        name = name,
        srcs = [gen_name],
        hdrs = [gen_name],
        deps = deps,
        includes = [gen_name + "_out"],
        visibility = visibility,
        alwayslink = True,
    )
