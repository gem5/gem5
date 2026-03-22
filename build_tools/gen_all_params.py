#!/usr/bin/env python3
"""Generate all SimObject params and enum headers/sources for gem5.

This script replaces the gem5py_m5 + individual build_tools script pipeline.
It uses system Python (no C++ embedding needed) to:
1. Set up the m5 Python infrastructure without C++ embedding
2. Discover and load SimObject .py files (with retry for dependency ordering)
3. Generate params/*.hh and python/_m5/param_*.cc for each SimObject class
4. Generate enums/*.hh and enums/*.cc for each Enum type

Usable by both CMake and Bazel build systems.

Usage:
    python3 gen_all_params.py --src-root <path> --output-dir <path> \
        [--simobj-files file1.py file2.py ...] [--use-python true|false] \
        [--output-type all|params_hdrs|params_srcs|enums_hdrs|enums_srcs] \
        [--file-filter 'pattern1,pattern2'] [--file-exclude 'pattern1,pattern2']
"""

import argparse
import fnmatch
import importlib
import importlib.util
import os
import re
import sys
import types

# Maps m5.objects.<Name> to filesystem path for on-demand loading.
_simobj_file_map = {}
# Guard against re-entrant loading (circular imports)
_loading_in_progress = set()


class _SimObjectFinder:
    """Custom import hook that resolves m5.objects.<Name> on demand.

    When a SimObject .py file does 'from m5.objects.Foo import Foo', Python
    needs to find the m5.objects.Foo module. In the real gem5 build, the C++
    CodeImporter handles this. For standalone param generation, this finder
    maps module names to file paths and loads them lazily.
    """

    def find_module(self, fullname, path=None):
        if not fullname.startswith("m5.objects."):
            return None
        # Already loaded or being loaded
        if fullname in sys.modules:
            return self
        name = fullname[len("m5.objects.") :]
        if name in _simobj_file_map and fullname not in _loading_in_progress:
            return self
        return None

    def load_module(self, fullname):
        if fullname in sys.modules:
            return sys.modules[fullname]
        name = fullname[len("m5.objects.") :]
        filepath = _simobj_file_map.get(name)
        if filepath is None:
            raise ImportError(f"No SimObject file registered for {fullname}")
        _loading_in_progress.add(fullname)
        try:
            module = load_simobject_file(filepath, fullname)
        finally:
            _loading_in_progress.discard(fullname)
        if module is None:
            raise ImportError(f"Failed to load {fullname} from {filepath}")
        return module


def register_simobject_files(files):
    """Register SimObject .py files for on-demand import resolution."""
    for filepath in files:
        basename = os.path.splitext(os.path.basename(filepath))[0]
        _simobj_file_map[basename] = filepath


def setup_python_env(src_root, build_tools_dir):
    """Set up Python path for m5 imports without C++ embedding."""
    sys.path.insert(0, os.path.join(src_root, "src", "python"))
    sys.path.insert(0, build_tools_dir)

    # Pre-create m5.objects as an empty package to avoid the __init__.py
    # that requires the C++ CodeImporter (loader_state).
    import m5

    m5_objects = types.ModuleType("m5.objects")
    m5_objects.__path__ = []
    m5_objects.__package__ = "m5.objects"
    sys.modules["m5.objects"] = m5_objects

    # Install custom finder for m5.objects.* lazy loading
    sys.meta_path.insert(0, _SimObjectFinder())

    # Create m5.defines mock module with common build config values.
    # All ISAs enabled so all SimObject classes can be loaded for generation.
    m5_defines = types.ModuleType("m5.defines")
    m5_defines.__package__ = "m5"
    m5_defines.buildEnv = {
        "USE_ARM_ISA": True,
        "USE_MIPS_ISA": True,
        "USE_POWER_ISA": True,
        "USE_RISCV_ISA": True,
        "USE_SPARC_ISA": True,
        "USE_X86_ISA": True,
        "BUILD_GPU": True,
        "USE_KVM": False,
        "HAVE_PROTOBUF": False,
        "HAVE_TUNTAP": True,
        "USE_SYSTEMC": False,
        "PROTOCOL": "MESI_Two_Level",
        "RUBY": True,
        "TARGET_ISA": "x86",
        "BUILD_ISA": "x86",
    }
    sys.modules["m5.defines"] = m5_defines

    # Alias m5.citations as top-level "citations" module for SimObject .py
    # files that use bare "from citations import ...".
    import m5.citations

    sys.modules["citations"] = m5.citations


def load_simobject_file(filepath, module_name=None):
    """Load a Python file containing SimObject definitions.

    Returns the loaded module, or None on failure (with rollback).
    """
    basename = os.path.splitext(os.path.basename(filepath))[0]
    if module_name is None:
        module_name = f"m5.objects.{basename}"

    if module_name in sys.modules:
        old = sys.modules[module_name]
        if hasattr(old, "__file__") and old.__file__ == filepath:
            return old

    # Snapshot allClasses so we can roll back partially-defined classes
    from m5.SimObject import allClasses

    classes_before = set(allClasses.keys())

    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except (Exception, SystemExit):
        if module_name in sys.modules:
            del sys.modules[module_name]
        new_classes = set(allClasses.keys()) - classes_before
        for cls_name in new_classes:
            del allClasses[cls_name]
        return None

    # Make public names accessible via m5.objects (replicating __init__.py)
    m5_objects = sys.modules.get("m5.objects")
    if m5_objects is not None:
        for attr_name in dir(module):
            if not attr_name.startswith("_"):
                setattr(m5_objects, attr_name, getattr(module, attr_name))

    return module


def find_simobject_files(src_root):
    """Find all .py files that likely define SimObject classes.

    Scans .py files outside src/python/ for imports from m5.SimObject,
    m5.objects, or m5.params -- these indicate SimObject definitions.
    Files inside src/python/ are part of the m5 infrastructure and are
    not SimObject definition files.
    """
    simobj_files = []
    src_dir = os.path.join(src_root, "src")
    python_dir = os.path.join(src_dir, "python")

    # Match files that import SimObject infrastructure
    import_re = re.compile(
        r"from\s+m5\.(SimObject|objects|params)\s+import\s+"
        r"|import\s+m5\.(SimObject|objects|params)"
    )

    # Directories that contain m5 infrastructure or non-SimObject code
    skip_prefixes = [
        python_dir,
        os.path.join(src_dir, "mem", "slicc"),  # SLICC compiler
        os.path.join(src_dir, "systemc", "tests"),  # SystemC tests
    ]

    for root, dirs, files in os.walk(src_dir):
        if any(root.startswith(p) for p in skip_prefixes):
            continue
        dirs[:] = [
            d
            for d in dirs
            if d not in ("__pycache__", ".git", "python", "tests")
        ]
        for fname in sorted(files):
            if not fname.endswith(".py"):
                continue
            fpath = os.path.join(root, fname)
            try:
                with open(fpath) as f:
                    if import_re.search(f.read()):
                        simobj_files.append(fpath)
            except Exception:
                pass

    return simobj_files


def load_files_with_retry(files, max_rounds=10):
    """Load SimObject files with retry for dependency ordering."""
    pending = list(files)
    loaded = []

    for round_num in range(max_rounds):
        still_pending = []
        for filepath in pending:
            # Check if already loaded by the import hook
            basename = os.path.splitext(os.path.basename(filepath))[0]
            modname = f"m5.objects.{basename}"
            if modname in sys.modules:
                loaded.append(filepath)
                continue
            module = load_simobject_file(filepath)
            if module is not None:
                loaded.append(filepath)
            else:
                still_pending.append(filepath)

        if not still_pending:
            break

        pending = still_pending
        if round_num < max_rounds - 1:
            print(
                f"  load pass {round_num + 1}: {len(still_pending)} files "
                f"still pending, retrying...",
                file=sys.stderr,
            )

    if pending:
        print(
            f"Warning: {len(pending)} files could not be loaded after "
            f"{max_rounds} passes (may need KVM, FastModel, or other "
            f"optional dependencies):",
            file=sys.stderr,
        )
        for f in pending:
            print(f"  - {f}", file=sys.stderr)

    return loaded


def generate_params_header(sim_object, output_path):
    """Generate a params/*.hh header for a SimObject class."""
    from sim_object_param_struct_hh import write_header_file

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        write_header_file(sim_object, output_path)
        return True
    except Exception as e:
        print(
            f"ERROR: Failed to generate params header for "
            f"{sim_object.__name__}: {e}",
            file=sys.stderr,
        )
        return False


def generate_params_cc(sim_object, use_python, output_path):
    """Generate a python/_m5/param_*.cc source for a SimObject class."""
    from sim_object_param_struct_cc import write_cc_file

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        write_cc_file(sim_object, use_python, output_path)
        return True
    except Exception as e:
        print(
            f"ERROR: Failed to generate params cc for "
            f"{sim_object.__name__}: {e}",
            file=sys.stderr,
        )
        return False


def generate_enum_header(enum_cls, output_path):
    """Generate an enums/*.hh header for an Enum class."""
    from enum_hh import write_header_file

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        write_header_file(enum_cls, output_path)
        return True
    except Exception as e:
        print(
            f"ERROR: Failed to generate enum header for "
            f"{enum_cls.__name__}: {e}",
            file=sys.stderr,
        )
        return False


def generate_enum_cc(enum_cls, use_python, output_path):
    """Generate an enums/*.cc source for an Enum class."""
    from enum_cc import write_cc_file

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        write_cc_file(enum_cls, use_python, output_path)
        return True
    except Exception as e:
        print(
            f"ERROR: Failed to generate enum cc for "
            f"{enum_cls.__name__}: {e}",
            file=sys.stderr,
        )
        return False


def generate_cxx_config_header(sim_object, output_path):
    """Generate a cxx_config/*.hh header for a SimObject class."""
    from cxx_config_hh import write_header_file

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        write_header_file(sim_object, output_path)
        return True
    except Exception as e:
        print(
            f"ERROR: Failed to generate cxx_config header for "
            f"{sim_object.__name__}: {e}",
            file=sys.stderr,
        )
        return False


def generate_cxx_config_cc(sim_object, output_path):
    """Generate a cxx_config/*.cc source for a SimObject class."""
    from cxx_config_cc import write_cc_file

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    try:
        write_cc_file(sim_object, output_path)
        return True
    except Exception as e:
        print(
            f"ERROR: Failed to generate cxx_config cc for "
            f"{sim_object.__name__}: {e}",
            file=sys.stderr,
        )
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Generate gem5 SimObject params and enum headers/sources"
    )
    parser.add_argument(
        "--src-root",
        required=True,
        help="Path to gem5 source root",
    )
    parser.add_argument(
        "--output-dir",
        required=True,
        help="Output directory for generated files",
    )
    parser.add_argument(
        "--build-tools-dir",
        default=None,
        help="Path to build_tools/ directory (default: <src-root>/build_tools)",
    )
    parser.add_argument(
        "--simobj-files",
        nargs="*",
        default=None,
        help="Specific SimObject .py files to process (default: auto-discover)",
    )
    parser.add_argument(
        "--use-python",
        default="true",
        help="Whether Python support is enabled (true or false)",
    )
    parser.add_argument(
        "--list-only",
        action="store_true",
        help="Only list SimObject classes, don't generate",
    )
    parser.add_argument(
        "--headers-only",
        action="store_true",
        help="Only generate .hh headers, skip .cc sources",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Fail on any generation error (default: warn for SimObjects "
        "whose parent classes could not be loaded)",
    )
    parser.add_argument(
        "--with-cxx-config",
        action="store_true",
        help="Also generate cxx_config/ headers and sources",
    )
    parser.add_argument(
        "--output-type",
        default="all",
        choices=[
            "all",
            "params_hdrs",
            "params_srcs",
            "enums_hdrs",
            "enums_srcs",
            "cxx_config_hdrs",
            "cxx_config_srcs",
        ],
        help="Type of output to generate (default: all)",
    )
    parser.add_argument(
        "--file-filter",
        default=None,
        help="Comma-separated filename patterns to include for .cc source "
        "generation. Supports fnmatch wildcards.",
    )
    parser.add_argument(
        "--file-exclude",
        default=None,
        help="Comma-separated filename patterns to exclude from .cc source "
        "generation. Supports fnmatch wildcards.",
    )
    args = parser.parse_args()

    src_root = os.path.abspath(args.src_root)
    output_dir = os.path.abspath(args.output_dir)
    build_tools_dir = (
        os.path.abspath(args.build_tools_dir)
        if args.build_tools_dir
        else os.path.join(src_root, "build_tools")
    )
    use_python = args.use_python.lower() in ("true", "yes", "1")

    # Set up Python environment for m5 imports
    setup_python_env(src_root, build_tools_dir)

    # Load the base SimObject class first
    simobj_py = os.path.join(
        src_root, "src", "python", "m5", "objects", "SimObject.py"
    )
    load_simobject_file(simobj_py, "m5.objects.SimObject")

    # Discover or use provided SimObject files
    if args.simobj_files:
        simobj_files = [os.path.abspath(f) for f in args.simobj_files]
    else:
        simobj_files = find_simobject_files(src_root)

    print(f"Found {len(simobj_files)} SimObject files", file=sys.stderr)

    # Register all files for on-demand import resolution. This allows
    # "from m5.objects.Foo import Foo" to work when loading Bar.py that
    # depends on Foo, even if Foo hasn't been explicitly loaded yet.
    register_simobject_files(simobj_files)

    # Load all SimObject files (retry handles dependency ordering)
    load_files_with_retry(simobj_files)

    from m5.SimObject import (
        SimObject,
        allClasses,
    )

    classes = sorted(allClasses.values(), key=lambda c: c.__name__)
    print(f"Discovered {len(classes)} SimObject classes", file=sys.stderr)

    if args.list_only:
        for cls in classes:
            cxx_cls = cls._value_dict.get("cxx_class", "???")
            cxx_hdr = cls._value_dict.get("cxx_header", "???")
            print(f"{cls.__name__}: {cxx_cls} ({cxx_hdr})")
        return

    # Determine what to generate
    output_type = args.output_type
    do_params_hh = output_type in ("all", "params_hdrs")
    do_params_cc = (
        output_type in ("all", "params_srcs") and not args.headers_only
    )
    do_enums_hh = output_type in ("all", "enums_hdrs")
    do_enums_cc = (
        output_type in ("all", "enums_srcs") and not args.headers_only
    )

    # File filtering for .cc source generation
    filter_patterns = args.file_filter.split(",") if args.file_filter else None
    exclude_patterns = (
        args.file_exclude.split(",") if args.file_exclude else None
    )

    def should_generate_file(filename):
        """Check if a .cc file passes filter/exclude criteria."""
        if exclude_patterns:
            if any(fnmatch.fnmatch(filename, p) for p in exclude_patterns):
                return False
        if filter_patterns:
            if not any(fnmatch.fnmatch(filename, p) for p in filter_patterns):
                return False
        return True

    total_failed = 0

    # Generate params headers and/or sources
    if do_params_hh or do_params_cc:
        all_param_classes = [SimObject] + [
            c for c in classes if c.__name__ != "SimObject"
        ]

        hh_ok, hh_fail, cc_ok, cc_fail = 0, 0, 0, 0

        for cls in all_param_classes:
            name = cls.__name__

            if do_params_hh:
                out_hh = os.path.join(output_dir, "params", f"{name}.hh")
                if generate_params_header(cls, out_hh):
                    hh_ok += 1
                else:
                    hh_fail += 1

            if do_params_cc:
                cc_file = f"param_{name}.cc"
                if should_generate_file(cc_file):
                    # Output to python/_m5/ to match CMake/Bazel convention
                    out_cc = os.path.join(output_dir, "python", "_m5", cc_file)
                    if generate_params_cc(cls, use_python, out_cc):
                        cc_ok += 1
                    else:
                        cc_fail += 1

        if do_params_hh:
            print(
                f"Params headers: {hh_ok} OK, {hh_fail} failed",
                file=sys.stderr,
            )
            total_failed += hh_fail
        if do_params_cc:
            print(
                f"Params sources: {cc_ok} OK, {cc_fail} failed",
                file=sys.stderr,
            )
            total_failed += cc_fail

    # Generate enum headers and/or sources
    if do_enums_hh or do_enums_cc:
        from m5.params.enum_params import allEnums

        ehh_ok, ehh_fail, ecc_ok, ecc_fail = 0, 0, 0, 0

        for enum_name, enum_cls in sorted(allEnums.items()):
            if not hasattr(enum_cls, "vals") or not hasattr(enum_cls, "map"):
                continue
            if enum_name in ("Enum", "ScopedEnum"):
                continue

            if do_enums_hh:
                out_hh = os.path.join(output_dir, "enums", f"{enum_name}.hh")
                if generate_enum_header(enum_cls, out_hh):
                    ehh_ok += 1
                else:
                    ehh_fail += 1

            if do_enums_cc:
                cc_file = f"{enum_name}.cc"
                if should_generate_file(cc_file):
                    out_cc = os.path.join(output_dir, "enums", cc_file)
                    if generate_enum_cc(enum_cls, use_python, out_cc):
                        ecc_ok += 1
                    else:
                        ecc_fail += 1

        if do_enums_hh:
            print(
                f"Enum headers: {ehh_ok} OK, {ehh_fail} failed",
                file=sys.stderr,
            )
            total_failed += ehh_fail
        if do_enums_cc:
            print(
                f"Enum sources: {ecc_ok} OK, {ecc_fail} failed",
                file=sys.stderr,
            )
            total_failed += ecc_fail

    # Generate cxx_config (optional)
    do_cxx_hh = output_type == "cxx_config_hdrs" or (
        args.with_cxx_config and output_type == "all"
    )
    do_cxx_cc = (
        output_type == "cxx_config_srcs"
        or (args.with_cxx_config and output_type == "all")
    ) and not args.headers_only

    if do_cxx_hh or do_cxx_cc:
        chh_ok, chh_fail, ccc_ok, ccc_fail = 0, 0, 0, 0
        all_cls = [SimObject] + [
            c for c in classes if c.__name__ != "SimObject"
        ]

        for cls in all_cls:
            name = cls.__name__

            if do_cxx_hh:
                out = os.path.join(output_dir, "cxx_config", f"{name}.hh")
                if generate_cxx_config_header(cls, out):
                    chh_ok += 1
                else:
                    chh_fail += 1

            if do_cxx_cc:
                cc_file = f"{name}.cc"
                if should_generate_file(cc_file):
                    out = os.path.join(output_dir, "cxx_config", cc_file)
                    if generate_cxx_config_cc(cls, out):
                        ccc_ok += 1
                    else:
                        ccc_fail += 1

        if do_cxx_hh:
            print(
                f"CxxConfig headers: {chh_ok} OK, {chh_fail} failed",
                file=sys.stderr,
            )
            total_failed += chh_fail
        if do_cxx_cc:
            print(
                f"CxxConfig sources: {ccc_ok} OK, {ccc_fail} failed",
                file=sys.stderr,
            )
            total_failed += ccc_fail

    if total_failed > 0:
        if args.strict:
            print(
                f"FATAL: {total_failed} file(s) failed to generate "
                f"(--strict mode)",
                file=sys.stderr,
            )
            sys.exit(1)
        else:
            print(
                f"Warning: {total_failed} file(s) failed to generate "
                f"(likely due to unloaded optional dependencies). "
                f"Use --strict to treat as error.",
                file=sys.stderr,
            )
    else:
        print("All files generated successfully.", file=sys.stderr)


if __name__ == "__main__":
    main()
