"""Rule for SimObject parameter struct and enum generation.

Generates C++ parameter structs and enum definitions from SimObject .py files.
Uses the gem5py_m5 helper executable as interpreter. Outputs are collected
in a tree artifact since the exact file set depends on the SimObject/enum lists.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

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
