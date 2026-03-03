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
      params_dir:   params/{ClassName}.hh (header-only, for #include "params/...")
      enums_dir:    enums/{EnumName}.hh + enums/{EnumName}.cc
      pybind_dir:   python/_m5/param_{ClassName}.cc (pybind11 bindings)

    SimObject .py files scattered across the source tree are dynamically
    loaded from the filesystem and registered with gem5's embedded Python
    importer, enabling full SimObject class discovery without embedding
    all 300+ .py files into the gem5py_m5 binary.
    """
    gem5py_m5 = ctx.executable.gem5py_m5
    use_python = "True" if ctx.attr.use_python else "False"

    # Three separate tree artifacts for explicit output separation.
    params_dir = ctx.actions.declare_directory(ctx.label.name + "_params")
    enums_dir = ctx.actions.declare_directory(ctx.label.name + "_enums")
    pybind_dir = ctx.actions.declare_directory(ctx.label.name + "_pybind")

    # Build the embedded Python list of SimObject .py file paths.
    # These are relative to the source root (e.g., "src/sim/Root.py").
    simobj_files_str = "\\n".join(ctx.attr.simobject_py_files)

    script = ctx.actions.declare_file("_gen_all_simobjects.py")
    ctx.actions.write(
        output = script,
        content = """\
import sys
import os
import importlib

params_dir = sys.argv[1]
enums_dir = sys.argv[2]
pybind_dir = sys.argv[3]
build_tools_dir = sys.argv[4]
use_python = sys.argv[5] == "True"
src_root = sys.argv[6]

sys.path.insert(0, build_tools_dir)

from sim_object_param_struct_hh import write_header_file as write_param_hh
from sim_object_param_struct_cc import write_cc_file as write_param_cc
from enum_hh import write_header_file as write_enum_hh
from enum_cc import write_cc_file as write_enum_cc

# --- Dynamic SimObject .py file loading ---
# SimObject .py files are scattered across the source tree (src/sim/Root.py,
# src/cpu/BaseCPU.py, etc.). They need to be registered as m5.objects.*
# modules with gem5's embedded Python importer before discovery can work.
#
# The file list is passed from the Bazel rule (originally from
# simobject_py_files.bzl which is generated from CMake's
# gem5_add_simobject() calls).

_SIMOBJ_PY_FILES = \"\"\"{}\"\"\"

# Get access to the embedded Python importer instance.
import importer as _importer_mod
_importer_instance = None
for _finder in sys.meta_path:
    if hasattr(_finder, 'modules') and hasattr(_finder, 'add_module'):
        _importer_instance = _finder
        break

# Replace m5.objects.__init__ with a resilient version that uses try/except
# in its auto-import loop. The original version has no error handling, so a
# single module failure (e.g., missing config key) cascades and breaks ALL
# subsequent imports due to allClasses double-registration.
if _importer_instance and 'm5.objects' in _importer_instance.modules:
    _orig_abspath, _ = _importer_instance.modules['m5.objects']
    _resilient_init = compile(
        'for module in __spec__.loader_state:\\n'
        '    if module.startswith("m5.objects.") and module != "m5.objects":\\n'
        '        try:\\n'
        '            exec(f"from {{module}} import *")\\n'
        '        except Exception:\\n'
        '            pass\\n',
        _orig_abspath, 'exec')
    _importer_instance.modules['m5.objects'] = (_orig_abspath, _resilient_init)
    print("Replaced m5.objects.__init__ with resilient auto-import version")

# Register all SimObject .py files from the source tree with the importer.
_loaded = 0
_failed = 0
for line in _SIMOBJ_PY_FILES.strip().split("\\n"):
    relpath = line.strip()
    if not relpath:
        continue
    filepath = os.path.join(src_root, relpath)
    if not os.path.exists(filepath):
        print(f"Warning: SimObject .py file not found: {{filepath}}", file=sys.stderr)
        _failed += 1
        continue
    stem = os.path.splitext(os.path.basename(relpath))[0]
    modpath = f"m5.objects.{{stem}}"
    try:
        with open(filepath) as f:
            source = f.read()
        code = compile(source, filepath, "exec")
        _importer_instance.add_module(filepath, modpath, code)
        _loaded += 1
    except Exception as e:
        print(f"Warning: failed to register {{modpath}} from {{filepath}}: {{e}}",
              file=sys.stderr)
        _failed += 1

print(f"Registered {{_loaded}} SimObject .py files with importer ({{_failed}} failed)")

# Now import all m5.objects.* modules to trigger SimObject metaclass registration.
import m5

_simobj_mod = importlib.import_module('m5.SimObject')
_allClasses = _simobj_mod.allClasses

# Import m5.objects which triggers the resilient auto-import loop.
try:
    importlib.import_module('m5.objects')
except Exception as e:
    print(f"Warning: m5.objects import had issues: {{e}}", file=sys.stderr)

from m5.SimObject import SimObject
from m5.params import Enum

os.makedirs(os.path.join(params_dir, "params"), exist_ok=True)
os.makedirs(os.path.join(enums_dir, "enums"), exist_ok=True)
os.makedirs(os.path.join(pybind_dir, "python", "_m5"), exist_ok=True)

generated_enums = set()

if not _allClasses:
    print("ERROR: No SimObject classes discovered!", file=sys.stderr)
    print("Registered modules:", sorted(_obj_modules), file=sys.stderr)
    sys.exit(1)

for name in sorted(_allClasses.keys()):
    cls = _allClasses[name]
    write_param_hh(cls, os.path.join(params_dir, "params", name + ".hh"))
    write_param_cc(cls, use_python,
                   os.path.join(pybind_dir, "python", "_m5",
                                "param_" + name + ".cc"))

    for param_name, param in sorted(cls._params.local.items()):
        for ptype in getattr(param, "ptypes", []):
            if issubclass(ptype, Enum) and ptype.__name__ not in generated_enums:
                enum_name = ptype.__name__
                generated_enums.add(enum_name)
                write_enum_hh(ptype,
                              os.path.join(enums_dir, "enums",
                                           enum_name + ".hh"))
                write_enum_cc(ptype, use_python,
                              os.path.join(enums_dir, "enums",
                                           enum_name + ".cc"))

# Generate standalone enums from allEnums registry (catches Enum/Flag
# types defined in SimObject .py files that aren't SimObject params,
# like StaticInstFlags and GPUStaticInstFlags).
from m5.params.enum_params import allEnums as _allEnums
for enum_name in sorted(_allEnums.keys()):
    if enum_name not in generated_enums:
        cls = _allEnums[enum_name]
        generated_enums.add(enum_name)
        write_enum_hh(cls,
                      os.path.join(enums_dir, "enums",
                                   enum_name + ".hh"))
        write_enum_cc(cls, use_python,
                      os.path.join(enums_dir, "enums",
                                   enum_name + ".cc"))

print("Generated params for", len(_allClasses), "SimObjects")
print("Generated", len(generated_enums), "enum types")
""".format(simobj_files_str),
    )

    gen_scripts = ctx.files._gen_scripts
    build_tools_dir = gen_scripts[0].dirname if gen_scripts else ""

    # Derive source root from build_tools_dir: if build_tools_dir is
    # "external/gem5_sources/build_tools", then src_root is
    # "external/gem5_sources".
    src_root = "/".join(build_tools_dir.split("/")[:-1]) if build_tools_dir else "."

    ctx.actions.run_shell(
        outputs = [params_dir, enums_dir, pybind_dir],
        inputs = [script] + gen_scripts,
        tools = [gem5py_m5],
        command = "{} {} {} {} {} {} {} {}".format(
            gem5py_m5.path,
            script.path,
            params_dir.path,
            enums_dir.path,
            pybind_dir.path,
            build_tools_dir,
            use_python,
            src_root,
        ),
        mnemonic = "SimObjectAggregate",
        progress_message = "Generating all SimObject params and enums",
        execution_requirements = {"no-sandbox": "1"},
    )

    return [
        DefaultInfo(files = depset([params_dir, enums_dir, pybind_dir])),
        OutputGroupInfo(
            params_hdrs = depset([params_dir]),
            enums = depset([enums_dir]),
            pybind_srcs = depset([pybind_dir]),
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
        "simobject_py_files": attr.string_list(default = []),
        "_gen_scripts": attr.label(
            default = "//build_tools:simobj_gen_scripts",
            allow_files = True,
        ),
    },
)

def gem5_sim_object_aggregate(name, gem5py_m5 = "//src/python:gem5py_m5",
                               use_python = True, simobject_py_files = [],
                               visibility = None, deps = [],
                               compiled_deps = []):
    """Generate all SimObject params and enums in one pass.

    Discovers all SimObject subclasses by importing m5.objects via gem5py_m5,
    then generates parameter struct headers/sources and enum headers/sources
    for all discovered classes.

    SimObject .py files from across the source tree are dynamically loaded
    and registered with the embedded Python importer at generation time.
    The file list comes from simobject_py_files.bzl (derived from CMake's
    gem5_add_simobject() registrations).

    Output is split into three tree artifacts:
      {name}_gen_params/params/{ClassName}.hh    -- param struct headers
      {name}_gen_enums/enums/{EnumName}.hh|.cc   -- enum type definitions
      {name}_gen_pybind/python/_m5/param_*.cc    -- pybind11 bindings

    Creates TWO cc_library targets:
      {name}:       Header-only library exporting params/*.hh and enums/*.hh
                    on the include path. No .cc compilation. This target is
                    safe for gem5_hdrs to depend on without pulling in debug
                    flag or subsystem deps.
      {name}_srcs:  Compiled sources library (enums/*.cc and pybind/*.cc).
                    Requires the full transitive header graph (debug flags,
                    all source headers) since generated pybind .cc files
                    include sim/sim_object.hh and other deep headers.

    Args:
        name: Target name (creates {name} and {name}_srcs).
        gem5py_m5: Label of the gem5py_m5 executable.
        use_python: Whether to generate pybind11 bindings.
        simobject_py_files: List of SimObject .py file paths relative to
            the source root (e.g., ["src/sim/Root.py", "src/cpu/BaseCPU.py"]).
        visibility: Visibility.
        deps: Dependencies for the header-only library.
        compiled_deps: Dependencies for compiling the .cc sources. Should
            include all debug flag targets and the full header graph.
    """
    gen_name = name + "_gen"
    _sim_object_aggregate_gen(
        name = gen_name,
        gem5py_m5 = gem5py_m5,
        use_python = use_python,
        simobject_py_files = simobject_py_files,
    )

    # Header-only library: exports params/*.hh and enums/*.hh via include
    # path. The tree artifacts are placed in hdrs so dependents can
    # #include "params/Root.hh" and #include "enums/MemoryMode.hh".
    # No .cc files are compiled here.
    cc_library(
        name = name,
        hdrs = [gen_name],
        deps = deps,
        includes = [gen_name + "_params", gen_name + "_enums"],
        visibility = visibility,
    )

    # Compiled sources: enums/*.cc (serialization/pybind) and
    # python/_m5/param_*.cc (pybind11 bindings). These include
    # sim/sim_object.hh and other headers that transitively pull in
    # debug/*.hh, so compiled_deps must include all debug flag targets.
    cc_library(
        name = name + "_srcs",
        srcs = [gen_name],
        deps = compiled_deps + [":" + name],
        visibility = visibility,
        alwayslink = True,
    )
