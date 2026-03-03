"""Rule for embedding Python source files into C++ via marshal.

Compiles .py files to bytecode, marshals, compresses with zlib, and generates
C++ source files that register as EmbeddedPython modules.

IMPORTANT: The marshal tool MUST run with the SAME Python version that gem5py_m5
links against (system Python). Using a different version (e.g., hermetic Python
from rules_python) produces incompatible bytecode that crashes at runtime.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _py_source_gen_impl(ctx):
    """Marshal a Python source file into a C++ embedding.

    Uses system python3 (not the hermetic rules_python interpreter) to ensure
    bytecode compatibility with gem5py_m5 which links against system Python.
    """
    py_file = ctx.file.src
    modpath = ctx.attr.module_path
    abspath = ctx.attr.abspath

    # Generate a safe output name from the module path
    safe_name = modpath.replace(".", "_").replace("/", "_")
    cc_file = ctx.actions.declare_file("python/{}.py.cc".format(safe_name))

    marshal_scripts = ctx.files._marshal_scripts
    marshal_py = None
    for f in marshal_scripts:
        if f.basename == "marshal.py":
            marshal_py = f
            break
    if not marshal_py:
        fail("marshal.py not found in _marshal_scripts")

    abs_arg = abspath if abspath else py_file.path

    ctx.actions.run_shell(
        outputs = [cc_file],
        inputs = [py_file] + marshal_scripts,
        command = "python3 {} {} {} {} {}".format(
            marshal_py.path,
            cc_file.path,
            py_file.path,
            modpath,
            abs_arg,
        ),
        mnemonic = "PyMarshal",
        progress_message = "Marshaling Python source {}".format(modpath),
    )

    return [DefaultInfo(files = depset([cc_file]))]

_py_source_gen = rule(
    implementation = _py_source_gen_impl,
    attrs = {
        "src": attr.label(mandatory = True, allow_single_file = [".py"]),
        "module_path": attr.string(mandatory = True),
        "abspath": attr.string(default = ""),
        "_marshal_scripts": attr.label(
            default = "//build_tools:build_tools_files",
            allow_files = True,
        ),
    },
)

def gem5_py_source(name, src, module_path, visibility = None, deps = [], **kwargs):
    """Embed a Python source file as C++ via marshal.

    Args:
        name: Target name.
        src: The .py file to embed.
        module_path: Python module path (e.g., "m5.objects.Root").
        visibility: Visibility.
        deps: Additional dependencies.
    """
    gen_name = name + "_gen"
    _py_source_gen(
        name = gen_name,
        src = src,
        module_path = module_path,
        **kwargs
    )
    cc_library(
        name = name,
        srcs = [gen_name],
        visibility = visibility,
        deps = deps,
        alwayslink = True,
    )

def gem5_embedded_python_library(name, modules, visibility = None, deps = [],
                                  copts = [], target_compatible_with = None):
    """Batch-embed multiple Python source files.

    Args:
        name: Target name for the aggregate cc_library.
        modules: Dict mapping .py file labels to module paths.
        visibility: Visibility.
        deps: Additional dependencies.
        copts: Compiler flags.
        target_compatible_with: Platform constraints (forwarded to cc_library).
    """
    gen_targets = []
    for src, modpath in modules.items():
        safe = modpath.replace(".", "_").replace("/", "_")
        gen_name = "_{}_{}".format(name, safe)
        _py_source_gen(
            name = gen_name,
            src = src,
            module_path = modpath,
        )
        gen_targets.append(gen_name)

    kwargs = {}
    if target_compatible_with != None:
        kwargs["target_compatible_with"] = target_compatible_with

    cc_library(
        name = name,
        srcs = [":{}".format(t) for t in gen_targets],
        visibility = visibility,
        deps = deps,
        copts = copts,
        alwayslink = True,
        **kwargs
    )
