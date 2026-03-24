"""Rule for embedding Python source files into C++ via marshal.

Compiles .py files to bytecode, marshals, compresses with zlib, and generates
C++ source files that register as EmbeddedPython modules.

IMPORTANT: The marshal tool MUST run with the SAME Python version that gem5py_m5
links against (system Python). Using a different version (e.g., hermetic Python
from rules_python) produces incompatible bytecode that crashes at runtime.
"""

load("@rules_cc//cc:defs.bzl", "cc_import", "cc_library")

def _find_marshal_py(marshal_scripts):
    """Find marshal.py in the build tools file list."""
    for f in marshal_scripts:
        if f.basename == "marshal.py":
            return f
    fail("marshal.py not found in _marshal_scripts")

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
    marshal_py = _find_marshal_py(marshal_scripts)

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

def _simobject_embed_gen_impl(ctx):
    """Aggregate-marshal SimObject .py files into C++ embeddings.

    Reads .py files from the source tree by filesystem path (no Bazel
    labels needed), runs marshal on each, and outputs all .cc files
    into a single tree artifact.

    Also supports generated .py files (e.g. from SLICC) passed via
    generated_simobject_py_srcs.  These are resolved by their actual
    sandbox paths rather than relative to the source root.
    """
    output_dir = ctx.actions.declare_directory(ctx.label.name + "_gen")

    marshal_scripts = ctx.files._marshal_scripts
    marshal_py = _find_marshal_py(marshal_scripts)

    files_str = "\\n".join(ctx.attr.simobject_py_files)

    # Collect generated .py file sandbox paths.
    gen_py_files = ctx.files.generated_simobject_py_srcs
    gen_files_str = "\\n".join([f.path for f in gen_py_files])

    script = ctx.actions.declare_file("_embed_simobjects_{}.py".format(ctx.label.name))
    ctx.actions.write(
        output = script,
        content = """\
import sys
import os

output_dir = sys.argv[1]
build_tools_dir = sys.argv[2]
src_root = sys.argv[3]

sys.path.insert(0, build_tools_dir)
from marshal import *
from blob import bytesToCppArray
from code_formatter import code_formatter
import marshal as marshal_mod
import zlib

_FILES = \"\"\"{}\"\"\"
_GEN_FILES = \"\"\"{}\"\"\"

def embed_py(filepath, modpath, output_dir):
    safe_name = modpath.replace(".", "_")
    cc_path = os.path.join(output_dir, safe_name + ".py.cc")
    with open(filepath) as f:
        src = f.read()
    compiled = compile(src, filepath, "exec")
    marshalled = marshal_mod.dumps(compiled)
    compressed = zlib.compress(marshalled)
    code = code_formatter()
    code('#include "python/embedded.hh"')
    code('')
    code('namespace gem5')
    code('{{')
    code('namespace')
    code('{{')
    code('')
    bytesToCppArray(code, "embedded_module_data", compressed)
    code('')
    code('EmbeddedPython embedded_module_info(')
    code(f'    "{{filepath}}",')
    code(f'    "{{modpath}}",')
    code('    embedded_module_data,')
    code(f'    {{len(compressed)}},')
    code(f'    {{len(marshalled)}});')
    code('')
    code('}} // anonymous namespace')
    code('}} // namespace gem5')
    code.write(cc_path)

os.makedirs(output_dir, exist_ok=True)
count = 0

# Source-tree .py files (resolved relative to src_root).
for line in _FILES.strip().split("\\n"):
    relpath = line.strip()
    if not relpath:
        continue
    filepath = os.path.join(src_root, relpath)
    if not os.path.exists(filepath):
        print(f"Warning: {{filepath}} not found, skipping", file=sys.stderr)
        continue
    stem = os.path.splitext(os.path.basename(relpath))[0]
    modpath = f"m5.objects.{{stem}}"
    embed_py(filepath, modpath, output_dir)
    count += 1

# Generated .py files (resolved by absolute sandbox path).
for line in _GEN_FILES.strip().split("\\n"):
    genpath = line.strip()
    if not genpath:
        continue
    if not os.path.exists(genpath):
        print(f"Warning: generated {{genpath}} not found, skipping", file=sys.stderr)
        continue
    stem = os.path.splitext(os.path.basename(genpath))[0]
    modpath = f"m5.objects.{{stem}}"
    embed_py(genpath, modpath, output_dir)
    count += 1

print(f"Embedded {{count}} SimObject .py files as m5.objects modules")
""".format(files_str, gen_files_str),
    )

    build_tools_dir = marshal_py.dirname
    src_root = "/".join(build_tools_dir.split("/")[:-1]) if build_tools_dir else "."

    ctx.actions.run_shell(
        outputs = [output_dir],
        inputs = [script] + marshal_scripts + ctx.files.simobject_py_srcs + gen_py_files,
        command = "python3 {} {} {} {}".format(
            script.path,
            output_dir.path,
            build_tools_dir,
            src_root,
        ),
        mnemonic = "SimObjectEmbed",
        progress_message = "Embedding SimObject .py files as m5.objects modules",
    )

    return [DefaultInfo(files = depset([output_dir]))]

_simobject_embed_gen = rule(
    implementation = _simobject_embed_gen_impl,
    attrs = {
        "simobject_py_files": attr.string_list(default = []),
        "simobject_py_srcs": attr.label_list(
            default = [],
            allow_files = [".py"],
            doc = "Declared .py file inputs for hermetic sandboxed builds.",
        ),
        "generated_simobject_py_srcs": attr.label_list(
            default = [],
            allow_files = [".py"],
            doc = "Generated SimObject .py files (e.g. from SLICC).",
        ),
        "_marshal_scripts": attr.label(
            default = "//build_tools:build_tools_files",
            allow_files = True,
        ),
    },
)

def gem5_simobject_pysources(name, simobject_py_files = [], simobject_py_srcs = [],
                              generated_simobject_py_srcs = [],
                              visibility = None, deps = []):
    """Embed SimObject .py files as m5.objects.* Python modules.

    Takes a list of .py file paths relative to the source root and
    generates C++ embedding registrations in a single aggregate action.
    Uses the same cc_library -> cc_import(alwayslink) pipeline as
    SimObject params to ensure EmbeddedPython static constructors
    survive linker garbage collection.

    Also supports generated SimObject .py files (e.g. from SLICC) via
    generated_simobject_py_srcs.  These are resolved by their sandbox
    paths rather than relative to the source root.

    Args:
        name: Target name (creates {name} cc_import with alwayslink).
        simobject_py_files: List of .py file paths relative to the source
            root (e.g., ["src/sim/Root.py", "src/cpu/BaseCPU.py"]).
        simobject_py_srcs: Bazel labels for the .py files (declared inputs
            for hermetic sandboxed builds and incremental correctness).
        generated_simobject_py_srcs: Bazel labels for generated .py files
            (e.g. from SLICC protocol compilation).
        visibility: Visibility.
        deps: Additional dependencies (should include embedded_hdr).
    """
    gen_name = name + "_embed_gen"
    _simobject_embed_gen(
        name = gen_name,
        simobject_py_files = simobject_py_files,
        simobject_py_srcs = simobject_py_srcs,
        generated_simobject_py_srcs = generated_simobject_py_srcs,
    )

    # Compile the tree artifact. Use alwayslink + disable dynamic linker
    # to get a .lo archive without attempting .so creation.
    cc_library(
        name = name + "_compile",
        srcs = [gen_name],
        deps = deps,
        alwayslink = True,
        features = ["-supports_dynamic_linker"],
    )

    # Copy .lo to .a for cc_import. Same pipeline as sim_object.bzl.
    native.genrule(
        name = name + "_archive",
        srcs = [":" + name + "_compile"],
        outs = ["lib" + name + "_alwayslink.a"],
        cmd = "for f in $(SRCS); do case \"$$f\" in *.pic.lo) ;; *.lo) cp \"$$f\" \"$@\";; esac; done",
    )

    cc_import(
        name = name,
        static_library = ":" + name + "_archive",
        alwayslink = True,
        visibility = visibility,
    )
