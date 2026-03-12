"""Starlark rules for gem5 builds."""

load("@pybind11_bazel//:build_defs.bzl", "pybind_extension", "pybind_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_cc//cc/common:cc_info.bzl", "CcInfo")
load("@rules_python//python:defs.bzl", "PyInfo", "py_binary", "py_library")

_DEFAULT_VISIBILITY = ["//visibility:public"]

def _transform_gem5_deps(deps):
    """Transforms dependencies from `*_sim_object` to `param_*_hh`.

    This is actually a hack to allow the `param_*_hh` target to be used as a dependency for
    cc_library targets. Ideally we should just use the `*_sim_object` target directly in all cases.
    In reality, this creates a circular dependency issue because the `*_sim_object` target depends on
    the C++ library header for the SimObject, which in turn depends on `*_sim_object` if we were to
    actually replace `param_*_hh` with it.

    TODO: We can at least check if `param_*_hh` exists for each `*_sim_object` using
    `native.existing_rule()` and tell the user that `gem5_sim_object` is what they need to add if
    the corresponding `param_*_hh` target does not exist.

    Args:
        deps: A list of dependency labels.

    Returns:
        A list of labels with transformed dependencies.
    """
    if type(deps) != "list":
        return deps

    new_deps = []
    for dep in deps:
        if type(dep) == "string" and dep.endswith("_sim_object"):
            if ":" in dep:
                pkg, target = dep.rsplit(":", 1)
                new_deps.append(pkg + ":param_" + target[:-11] + "_hh")
            else:
                new_deps.append("param_" + dep[:-11] + "_hh")
        else:
            new_deps.append(dep)
    return new_deps

def to_snake_case(s):
    """Converts CamelCase to snake_case.

    TODO: This is more like a `utils.bzl` function than a `rules.bzl` function. We may also
    make this private to the file, but it's also used by `slicc.bzl`. Consider moving this to a
    `utils.bzl`.

    Args:
        s: The string to convert.

    Returns:
        The snake_case version of the string.
    """
    result = ""
    for i in range(len(s)):
        char = s[i]
        if i > 0 and char.isupper() and s[i - 1] != "_":
            if s[i - 1].islower() or s[i - 1].isdigit():
                result += "_"
            elif i < len(s) - 1 and s[i + 1].islower():
                result += "_"
        result += char.lower()
    return result

def get_gem5_tracing_copts():
    return select({
        "//bazel:tracing_on": ["-DTRACING_ON=1"],
        "//conditions:default": ["-DTRACING_ON=0"],
    })

def get_gem5_include_path_copts():
    return [
        "-Isrc",
    ]

def get_default_gem5_copts():
    return [
        "-DNUMBER_BITS_PER_SET=128",  # For Ruby
    ] + get_gem5_tracing_copts() + get_gem5_include_path_copts()

def gem5_cc_library(name, srcs = [], hdrs = [], deps = [], copts = [], visibility = _DEFAULT_VISIBILITY, **kwargs):
    """Generates a cc_library target for gem5 C++ code.
    """

    cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = _transform_gem5_deps(deps) + ["//:src_include"],
        copts = copts + get_default_gem5_copts(),
        visibility = visibility,
        **kwargs
    )

def _gem5_sim_object_aggregator_impl(ctx):
    """Aggregates providers from its dependencies.

    The target created by this rule provides a single, public-facing target for each SimObject,
    simplifying BUILD file dependencies. Other targets can depend on the aggregated target and let
    the build system select the required artifacts via standard providers (CcInfo, PyInfo), rather
    than needing to specify intermediate targets like `MyObj_py` or `param_MyObj_hh` or
    `param_MyObj_cc` (if any).

    Expected dependencies:
        - A py_library for the SimObject definition (provides PyInfo and PythonStrictDependencyInfo).
        - A pybind_library for the generated SimObject code (e.g., param_*_py), which provides
          CcInfo, PyInfo, and PythonStrictDependencyInfo.
        - An (optional) cc_library which provides CcInfo.

    Rule Attributes:
        deps: A label_list of targets to aggregate providers from.

    Returns:
        A list containing CcInfo, PyInfo, and PythonStrictDependencyInfo providers merged from
        the dependencies.
    """

    # Info objects for each dependency
    cc_infos = []
    py_infos = []

    # There's no "merge" for PythonStrictDependencyInfo, so we have to do it manually for each info
    # params
    exported_modules = []
    exported_module_specs = []

    for dep in ctx.attr.deps:
        if CcInfo in dep:
            cc_infos.append(dep[CcInfo])
        if PyInfo in dep:
            py_infos.append(dep[PyInfo])

    # Collect runfiles from all dependencies
    runfiles = ctx.runfiles()
    for dep in ctx.attr.deps:
        runfiles = runfiles.merge(dep[DefaultInfo].default_runfiles)
    if ctx.attr.cc_dep:
        runfiles = runfiles.merge(ctx.attr.cc_dep[DefaultInfo].default_runfiles)
    if ctx.attr.py_dep:
        runfiles = runfiles.merge(ctx.attr.py_dep[DefaultInfo].default_runfiles)

    # cc_info and py_info are merged from cc_dep and py_dep
    cc_info = ctx.attr.cc_dep[CcInfo] if ctx.attr.cc_dep else None
    py_info = ctx.attr.py_dep[PyInfo] if ctx.attr.py_dep else None

    providers = [DefaultInfo(runfiles = runfiles)]
    if cc_info:
        providers.append(cc_info)
    if py_info:
        providers.append(py_info)
    return providers

def _gem5_param_gen_rule_impl(ctx):
    staged_sources = []
    all_dirs = {}
    dirs_with_real_init = {}
    staged_paths_map = {}

    # Sandbox root relative to package to ensure each target has its own
    sandbox_root = "sandbox_" + ctx.label.name

    def stage_file(src):
        # Canonicalize path: src/python/m5/ -> m5/
        m5_prefix = "src/python/m5/"
        found = src.short_path.find(m5_prefix)
        if found != -1:
            rel_path = src.short_path[found + len("src/python/"):]
        else:
            rel_path = src.short_path

        staged_path = sandbox_root + "/" + rel_path
        if staged_path in staged_paths_map:
            return

        staged_file = ctx.actions.declare_file(staged_path)
        ctx.actions.symlink(output = staged_file, target_file = src)
        staged_sources.append(staged_file)
        staged_paths_map[staged_path] = True

        # Track directory and if it has a real __init__.py
        path_parts = staged_path.split("/")[:-1]
        for i in range(len(path_parts)):
            p = "/".join(path_parts[:i + 1])
            all_dirs[p] = True

        if src.basename == "__init__.py":
            dirs_with_real_init["/".join(staged_path.split("/")[:-1])] = True

    # Stage all transitive sources from py_lib, extra_tool_deps, and mandatory deps
    py_libs = [ctx.attr.py_lib] + ctx.attr.extra_tool_deps + ctx.attr._mandatory_tool_deps
    for lib in py_libs:
        if PyInfo in lib:
            for src in lib[PyInfo].transitive_sources.to_list():
                stage_file(src)

    # Create missing __init__.py files
    for d in all_dirs:
        if d not in dirs_with_real_init:
            init_path = d + "/__init__.py"
            if init_path not in staged_paths_map:
                it_file = ctx.actions.declare_file(init_path)
                ctx.actions.write(output = it_file, content = "")
                staged_sources.append(it_file)
                staged_paths_map[init_path] = True

    # Include the tool themselves and any other tools
    all_inputs = staged_sources + ctx.files.tool_cc + ctx.files.tool_hh + ctx.files.extra_tool_deps

    # PYTHONPATH should be the sandbox root
    # Note: ctx.outputs.output_cc.dirname is the package root relative to execroot
    python_path = ctx.outputs.output_cc.dirname + "/" + sandbox_root

    # Run the generic param tools
    # ... rest is the same ...
    # CC generation
    ctx.actions.run(
        outputs = [ctx.outputs.output_cc],
        inputs = all_inputs,
        executable = ctx.executable.tool_cc,
        arguments = [
            ctx.attr.module_path,
            ctx.outputs.output_cc.path,
            "True",
        ] + ctx.attr.extra_args,
        env = {"PYTHONPATH": python_path},
        mnemonic = "Gem5ParamGenCC",
        progress_message = "Generating CC parameters for %s" % ctx.attr.module_path,
    )

    # HH generation
    ctx.actions.run(
        outputs = [ctx.outputs.output_hh],
        inputs = all_inputs,
        executable = ctx.executable.tool_hh,
        arguments = [
            ctx.attr.module_path,
            ctx.outputs.output_hh.path,
        ],
        env = {"PYTHONPATH": python_path},
        mnemonic = "Gem5ParamGenHH",
        progress_message = "Generating HH parameters for %s" % ctx.attr.module_path,
    )

_gem5_param_gen_rule = rule(
    implementation = _gem5_param_gen_rule_impl,
    attrs = {
        "module_path": attr.string(mandatory = True),
        "py_lib": attr.label(mandatory = True, providers = [PyInfo]),
        "tool_cc": attr.label(mandatory = True, executable = True, cfg = "exec"),
        "tool_hh": attr.label(mandatory = True, executable = True, cfg = "exec"),
        "output_cc": attr.output(mandatory = True),
        "output_hh": attr.output(mandatory = True),
        "extra_args": attr.string_list(),
        "extra_tool_deps": attr.label_list(),
        "_mandatory_tool_deps": attr.label_list(default = [
            "//src/python/m5:SimObject",
            "//src/python/m5/objects:SimObject",
        ]),
    },
)

_gem5_sim_object_aggregator_rule = rule(
    implementation = _gem5_sim_object_aggregator_impl,
    attrs = {
        "deps": attr.label_list(),
        "cc_dep": attr.label(providers = [CcInfo]),
        "py_dep": attr.label(providers = [PyInfo]),
    },
)

def gem5_sim_object_aggregator(name, deps, visibility = _DEFAULT_VISIBILITY):
    cc_agg_name = "_" + name + "_cc_agg"
    py_agg_name = "_" + name + "_py_agg"

    # cc_library will merge CcInfo from its deps.
    # We use a hack: only include deps that likely provide CcInfo.
    # Actually, we can just include all and let it error if it's strictly enforced,
    # but gem5_sim_object knows exactly what it's passing.
    cc_library(
        name = cc_agg_name,
        # Exclude python-related libraries and extensions from C++ aggregation
        deps = [d for d in deps if not d.endswith("_py_lib") and not d.endswith("_py") and "_param_" not in d],
        visibility = ["//visibility:private"],
    )

    py_library(
        name = py_agg_name,
        deps = [d for d in deps if "_m5_param_" not in d],
        data = [d for d in deps if "_m5_param_" in d],
        imports = ["."],
        visibility = ["//visibility:private"],
    )

    _gem5_sim_object_aggregator_rule(
        name = name,
        cc_dep = ":" + cc_agg_name,
        py_dep = ":" + py_agg_name,
        visibility = visibility,
    )

def gem5_pybind_library(name, srcs = [], hdrs = [], deps = [], visibility = _DEFAULT_VISIBILITY, **kwargs):
    """Generates a pybind_library target for gem5 Python code.
    """

    pybind_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        deps = _transform_gem5_deps(deps) + ["//:src_include"],
        copts = get_default_gem5_copts(),
        visibility = visibility,
        **kwargs
    )

def gem5_sim_object(name, sim_object_module, sim_object_name, sim_object_py_lib, sim_object_header = None, extra_deps = [], extra_param_deps = [], extra_tool_deps = [], visibility = _DEFAULT_VISIBILITY):
    """Generates code and libraries for sim object parameter objects.

    From this macro you will get the following targets:
        - param_SimObject_py: A pybind_library target for the SimObject.
          - This is the python code for the SimObject parameters. When you instantiate this object
            in python it enables the creation of the underlying C++ object.
        - param_SimObject_hh: A cc_library target for the SimObject.
          - This can be used for any other cc_library targets which need the SimObject parameters.
            This includes the SimObject itself and any subclasses that depend on the parameters.
        - SimObject_param_gen: A genrule target for the SimObject.
          - This results in two files, a param_SimObject.cc and params/SimObject.hh.
        - _SimObject_param_tool: An intermediate binary target for the SimObject. This creates the
          files needed for the param_gen

    Args:
        name: The name of the rule.
        sim_object_module: The Python import path to the module containing the SimObject.
        sim_object_name: The class name of the SimObject. Note that a module may have multiple
          SimObjects.
        sim_object_py_lib: The py_library target for the SimObject. This is the source for the
          sim_object_module.
        sim_object_header: The header file of the SimObject's C++ code.
        extra_deps: List of extra dependencies for the generated pybind_library and the header.
          These dependencies are the header files that the SimObject header depends on.
        extra_param_deps: List of extra param_xxx_hh dependencies for the param_xxx_hh target.
          These are usually the SimObjects that are used as parameters(???) and parent classes.
        extra_tool_deps: List of extra Python dependencies for the generation tool. This is the
          python dependencies for the py file. This is any parent classes and any SimObjects that
          are used as parameters.
        visibility: The visibility of the generated targets.
    Hints:
        - If the error is in the param_SimObject.cc compilation, the likely missing dependency is
          in the extra_deps list.
    """
    if not name.endswith("_sim_object"):
        fail("gem5_sim_object 'name' must end with '_sim_object'. Got: " + name)
    if name != to_snake_case(sim_object_name) + "_sim_object":
        fail("Target name '{}' must be '{}_sim_object' (snake case of sim_object_name '{}' + _sim_object)".format(
            name,
            to_snake_case(sim_object_name),
            sim_object_name,
        ))

    output_cc = "param_" + sim_object_name + ".cc"
    output_hh = "params/" + sim_object_name + ".hh"

    base_name = to_snake_case(sim_object_name)

    # Use custom rule to handle transitive python deps.
    _gem5_param_gen_rule(
        name = base_name + "_param_gen",
        module_path = sim_object_module,
        py_lib = sim_object_py_lib,
        tool_cc = "//build_tools:sim_object_param_struct_cc",
        tool_hh = "//build_tools:sim_object_param_struct_hh",
        output_cc = output_cc,
        output_hh = output_hh,
        extra_tool_deps = extra_tool_deps,
    )

    # TODO: [Long term] I would like to see this be a pybind_extension. The reason it
    # can't right now is that the code adds it to a _m5 module. Instead, we need to add it to its
    # own module. The assumption in all of the gem5 code is that all of these param python objects
    # are in the _m5 module.
    common_deps = [
        "//src/base:types",
        "//src/base:compiler",
        "//src/sim:init",
    ]
    py_hdrs = [output_hh]
    if sim_object_header:
        if sim_object_header.startswith(":") or sim_object_header.startswith("//"):
            common_deps.append(sim_object_header)
        else:
            py_hdrs.append(sim_object_header)

    pybind_library(
        name = "param_" + base_name + "_py",
        srcs = [output_cc],
        hdrs = py_hdrs,
        visibility = visibility,
        copts = get_default_gem5_copts(),
        alwayslink = True,
        deps = common_deps + extra_deps + _transform_gem5_deps(extra_param_deps),
    )

    cc_library(
        name = "param_" + base_name + "_hh",
        hdrs = [output_hh],
        # This tells targets depending on this one to add the current package
        # to the include search path. This allows anyone to use `#include params/<SimObject>.hh`
        # and not have to worry about the full path.
        includes = ["."],
        visibility = visibility,
        copts = get_default_gem5_copts(),
        deps = [
            "//src/base:types",
        ] + _transform_gem5_deps(extra_param_deps),
    )

    output_ext_cc = "param_" + sim_object_name + "_ext.cc"

    # Extension generation needs another tool run or another rule.
    # For now, keep it as genrule or use another _gem5_param_gen_rule instance with extra_args
    _gem5_param_gen_rule(
        name = name + "_param_ext_cc",
        module_path = sim_object_module,
        py_lib = sim_object_py_lib,
        tool_cc = "//build_tools:sim_object_param_struct_cc",
        tool_hh = "//build_tools:sim_object_param_struct_hh",  # We don't really need a new HH but rule requires it
        output_cc = output_ext_cc,
        output_hh = output_ext_cc + ".hh_dummy",  # Dummy out
        extra_args = ["--extension", "--name", sim_object_name],
        extra_tool_deps = extra_tool_deps,
    )

    pybind_extension(
        name = "_m5_param_" + sim_object_name,
        srcs = [output_ext_cc, output_hh],
        visibility = visibility,
        copts = get_default_gem5_copts(),
        deps = common_deps + extra_deps + _transform_gem5_deps(extra_param_deps),
    )

    gem5_sim_object_aggregator(
        name = name,
        deps = [
            sim_object_py_lib,
            ":param_" + to_snake_case(sim_object_name) + "_py",
            ":_m5_param_" + sim_object_name,
        ],
        visibility = visibility,
    )

def gem5_enum_generator(name, enum_module, enum_name, enum_py_lib, extra_tool_deps = [], visibility = _DEFAULT_VISIBILITY):
    """Generates enum_<name>.cc and enum/<name>.hh for a gem5 Enum.

    From this macro you will get the following targets:
        - name_py: A pybind_library target for the Enum.
          - This is the python code for the Enum. This python object is used to communicate
            parameter values to the underlying C++ code.
        - name + "_hh": A cc_library target for the Enum.
          - This can be used for any other cc_library targets which need the Enum. This includes
            the Enum itself and any subclasses that depend on the Enum.

    Args:
        name: The name of the rule. The convention is enum_SimObject.
        enum_module: The Python import path to the module containing the Enum.
        enum_name: The class name of the Enum.
        enum_py_lib: The py_library target for the Enum.
        extra_tool_deps: List of extra Python dependencies for the generation tool. These are the
          python dependencies for the py file.
    """
    output_cc = "enum_" + enum_name + ".cc"
    output_hh = "enums/" + enum_name + ".hh"

    if enum_py_lib == "//src/python/m5/params":
        param_py_lib = []
    else:
        param_py_lib = ["//src/python/m5/params"]

    _gem5_param_gen_rule(
        name = "_" + name + "_gen",
        module_path = enum_module,
        py_lib = enum_py_lib,
        tool_cc = "//build_tools:enum_cc",
        tool_hh = "//build_tools:enum_hh",
        output_cc = output_cc,
        output_hh = output_hh,
        extra_tool_deps = extra_tool_deps,
    )

    header_name = name + "_hh"

    cc_library(
        name = header_name,
        srcs = [output_cc],
        hdrs = [output_hh],
        # This tells targets depending on this one to add the current package
        # to the include search path.
        includes = ["."],
        copts = get_default_gem5_copts(),
        visibility = visibility,
        deps = [
            "//src/base:compiler",
        ],
    )

    pybind_library(
        name = name + "_py",
        srcs = [output_cc],
        copts = get_default_gem5_copts(),
        local_defines = ["GEM5_ENUM_PYBIND"],
        visibility = _DEFAULT_VISIBILITY,
        alwayslink = True,
        deps = [
            header_name,
            "//src/base:compiler",
            "//src/sim:init",
        ],
    )

def _debug_flag_cc_name(flag):
    return flag + ".cc"

def _debug_flag_hh_name(flag):
    return "debug/" + flag + ".hh"

def _debug_flag_codegen(name, flag, desc, fmt, components):
    output_cc = _debug_flag_cc_name(flag)
    output_hh = _debug_flag_hh_name(flag)

    genrule_name = name + "_codegen"
    escaped_desc = desc.replace("'", "\\'")
    component_names = [c.split(":")[-1].replace("debug_", "") for c in components]

    native.genrule(
        name = genrule_name,
        outs = [
            output_cc,
            output_hh,
        ],
        cmd = """
            $(location //build_tools:debugflagcc) $(location {output_cc}) {flag}
            $(location //build_tools:debugflaghh) $(location {output_hh}) {flag} '{desc}' {fmt} '{components_str}'
        """.format(
            flag = flag,
            desc = desc.replace("'", "'\\''"),
            fmt = "True" if fmt else "False",
            components_str = ":".join(component_names),
            output_cc = output_cc,
            output_hh = output_hh,
        ),
        tools = [
            "//build_tools:debugflagcc",
            "//build_tools:debugflaghh",
        ],
    )
    return output_cc, output_hh

# TODO(hchsiao): Consider splitting this macro into `gem5_compound_debug_flag`, and `gem5_debug_flag` to simplify the logic.
def gem5_debug_flag(name, flag, desc = "", fmt = False, components = [], included_by = [], visibility = _DEFAULT_VISIBILITY):
    """Generates <name>.cc and debug/<name>.hh for a gem5 debug flag.

    Implementation-wise, this macro creates one of three types of flags:
    1. A compound flag (`components` is non-empty, which implies `included_by` is empty).
    2. A component flag (otherwise).

    Note: the `included_by` relationship might seem to be redundant, as it can be logically derived
    as "compound flags that include this flag **for all** flags in the repo". However, such
    **for all** logic is not supported by Bazel, as it would require implicit dependencies on a
    variable number of targets. Therefore, the declaration of `included_by` is essential for the
    implementation of the `gem5_debug_flag` macro in order to link the compound flag as long as one
    of its components is used.

    Args:
        name: The target name. Must start with 'debug_'.
        flag: The actual flag string used in gem5 code (e.g., 'Protocol').
        desc: A description of the debug flag.
        fmt: Boolean, True if it's a format flag.
        components: List of local labels (:debug_X) that this compound flag enables.
        included_by: List containing the local label of the parent compound flag.
        visibility: Target visibility.
    """

    # Enforce a consistent naming convention for all gem5 debug targets.
    if not name.startswith("debug_"):
        fail("The target name '%s' must start with 'debug_' to maintain consistency for compound flags." % name)

    # A flag cannot be both a compound flag and a component of another flag.
    if components and included_by:
        fail("components and included_by cannot both be set")

    if included_by:
        _, hh = _debug_flag_codegen(name, flag, desc, fmt, [])

        # If this flag is part of a compound flag, it creates a header-only cc_library depends on
        # all the containing compound flags (which, in turn, bundle all component flags's .cc files).
        # This ensures that enabling this flag also enables the compound flags' logic.
        # The `hh` provided here allows sim objects to include this flag directly and has nothing to
        # do with the include from the compound flag's .cc files.
        cc_library(
            name = name,
            srcs = [],
            hdrs = [hh],
            # This tells targets depending on this one to add the current package
            # to the include search path.
            includes = ["."],
            copts = get_default_gem5_copts(),
            visibility = visibility,
            deps = included_by + [
                "//src/base:debug",
                "//src/base:compiler",
            ],
        )

        # The difference between this and the above target is that this one does not have any
        # deps. This prevents circular dependencies among compound flags and their components.
        # The `hh` provided here allows compound flag's .cc files to include this flag directly and
        # has nothing to do with the include from sim object's .cc files.
        cc_library(
            name = name + "_hh",
            srcs = [],
            hdrs = [hh],
            # This tells targets depending on this one to add the current package
            # to the include search path.
            includes = ["."],
            copts = get_default_gem5_copts(),
            visibility = visibility,
            deps = [
                "//src/base:debug",
                "//src/base:compiler",
            ],
        )
    else:
        # Generate the compound flag
        cc, hh = _debug_flag_codegen(name, flag, desc, fmt, components)

        # Derive all component flag files that are being bundled. Headers are specified via `deps`
        # as header-only cc_library targets instead of plain files, because we need to include them
        # as `#include "debug/MyFlag.hh"`, which requires the `includes = ["."]` trick.
        component_src_files = []
        component_hdr_deps = []
        for component in components:
            package, component_name = component.split(":debug_")
            component_src_files.append(package + ":" + _debug_flag_cc_name(component_name))
            component_hdr_deps.append(package + ":debug_" + component_name + "_hh")

        # Compound flag and component flag files are bundled together into a single cc_library.
        cc_library(
            name = name,
            srcs = component_src_files + [cc],
            hdrs = [hh],
            includes = ["."],
            copts = get_default_gem5_copts(),
            visibility = visibility,
            deps = component_hdr_deps + [
                "//src/base:debug",
                "//src/base:compiler",
            ],
        )

def gem5_gdbxml_generator(name, xml_file, symbol, isa):
    """Generates <name>.cc and <name>.hh for a gem5 gdbxml file

    Note: The output path is hardcoded to arch/<ISA>/gdb-xml/
    with respect to the current package for includes to work.

    Args:
        name: The name of the rule.
        xml_file: The input .xml file.
        symbol: The symbol name to use for the data.
    """
    output_path = "arch/" + isa + "/gdb-xml/"
    output_cc = output_path + name + ".cc"
    output_hh = output_path + name + ".hh"

    tool_binary = "//bazel/build_tools:create_gdbxml"

    genrule_name = "_" + name + "_gen"
    native.genrule(
        name = genrule_name,
        srcs = [xml_file],
        outs = [
            output_cc,
            output_hh,
        ],
        cmd = """            $(location {tool_binary}) \
                $(location {xml_file}) \
                '{symbol}' \
                '{isa}' \
                $(location {output_cc}) \
                $(location {output_hh})""".format(
            tool_binary = tool_binary,
            xml_file = xml_file,
            symbol = symbol,
            isa = isa,
            output_cc = output_cc,
            output_hh = output_hh,
        ),
        tools = [tool_binary],
    )

    gem5_cc_library(
        name = name,
        srcs = [output_cc],
        hdrs = [output_hh],
        includes = ["."],
    )

def m5_objects_generator(name, simobjects):
    """Creates a py_library shim for each module in the simobjects list.

    Args:
        name: The name of the rule.
        simobjects: A list of dictionaries, where each dictionary has
                    "mod_name", "module_path", "dep", and "simobjects" keys.
    """
    all_module_libs = []
    all_mod_names = []
    for entry in simobjects:
        mod_name = entry["mod_name"]
        module_path = entry["module_path"]
        dep = entry["dep"]
        simobject_names = entry["simobjects"] + entry["other_simobjects"] + entry["enums"]
        shim_file = mod_name + ".py"
        all_mod_names.append(mod_name)

        import_statements = []
        for simobject_name in simobject_names:
            import_statements.append("from {} import {}".format(module_path, simobject_name))

        # Build a command that prints each import statement followed by a newline
        command = " && ".join([
            "echo '{}' >> $(location {})".format(stmt, shim_file)
            for stmt in import_statements
        ])

        native.genrule(
            name = "_" + mod_name + "_module_shim_gen",
            outs = [shim_file],
            cmd = "touch $(location {}) && ".format(shim_file) + command,
        )

        lib_name = mod_name
        py_library(
            name = lib_name,
            srcs = [shim_file],
            visibility = ["//visibility:public"],
            deps = [dep],
        )
        all_module_libs.append(":" + lib_name)

    # Generate a master shim that imports everything
    master_shim_file = "all_objects.py"
    native.genrule(
        name = name + "_master_shim_gen",
        outs = [master_shim_file],
        cmd = "echo '# Auto-generated master shim' > $(location {}) && ".format(master_shim_file) +
              " && ".join(["echo 'from .{} import *' >> $(location {})".format(mod, master_shim_file) for mod in all_mod_names]),
    )

    py_library(
        name = name,
        srcs = [master_shim_file],
        deps = all_module_libs,
        visibility = ["//visibility:public"],
    )

def get_m5_pybind_deps(simobjects):
    """Gets the list of pybind dependencies for the _m5 target.

    Args:
        simobjects: The SIM_OBJECTS list from sim_objects.bzl.

    Returns:
        A list of dependency strings.

    Note: This causes all of the objects in gem5 to be linked into the _m5 target. In the future,
    when we have a way to dynamically discover what objects are used, we can remove this. The main
    impediment to this is all of the "from m5.objects import .." statements.
    """
    deps = []
    for entry in simobjects:
        package = entry["dep"].split(":")[0]
        for simobject_name in entry.get("simobjects", []):
            deps.append(package + ":param_" + to_snake_case(simobject_name) + "_py")
        for enum_name in entry.get("enums", []):
            # Enums are special, their build rules are slightly different
            if enum_name == "ByteOrder":
                deps.append("//src/sim:enum_ByteOrder_py")
            elif enum_name == "PwrState":
                deps.append("//src/sim:enum_PwrState_py")
            elif enum_name == "MemoryMode":
                deps.append("//src/sim:enum_MemoryMode_py")
            elif enum_name == "KernelPanicOopsBehaviour":
                deps.append("//src/sim:enum_KernelPanicOopsBehaviour_py")
            else:
                deps.append(package + ":enum_" + enum_name + "_py")
    return deps

def gem5_isa_cc_library(name, arch, main_isa_file, isa_srcs, exec_splits = 1, constrs_splits = 1, extra_deps = [], extra_hdrs = [], visibility = _DEFAULT_VISIBILITY):
    """Generates C++ library from gem5 ISA description.

    This macro takes a set of ISA description files, typically written in a domain-specific
    language, and uses the build_isa.py script (isa_parser) to generate C++ source code
    that implements the decoder and execution semantics for that ISA. It then compiles
    this generated code into a cc_library.
    Note: The code is put in the directory arch/<arch>/generated with respect to the current
    package (e.g., arch/arm/arch/arm/generated). This is different from upstream gem5.

    Args:
        name: Name of the cc_library to generate.
        arch: Architecture name (e.g., "arm", "riscv"). This is passed to the build_isa.py script.
        main_isa_file: The main .isa file that serves as the entry point for the ISA parser.
        isa_srcs: Filegroup or list of all source .isa files, including any files included
            by the main_isa_file.
        exec_splits: The number of ways the generated execution code (generic_cpu_exec_*.cc)
            is split. This is used to manage compile times and file sizes. This number
            should match the MAX_SPLIT value used within the isa_parser.
        constrs_splits: The number of ways the generated instruction constructor code
            (inst-constrs-*.cc) is split. This is used to manage compile times and file sizes.
            This number should match the MAX_DECODER_SPLIT value used within the isa_parser.
        extra_deps: Dependencies for the main generated library.
        extra_hdrs: Header dependencies for the decoder library.
        visibility: The visibility of the generated cc_library targets.
    """
    tool_name = "//src/arch/isa_parser:isa_parser"

    # Output path relative to the current package
    isa_gen_path = "arch/" + arch + "/generated"

    OUTS = []
    HDRS = []
    SRCS = []
    INCS = []

    def _add_out(filename):
        p = isa_gen_path + "/" + filename
        OUTS.append(p)
        return p

    HDRS.append(_add_out("decoder.hh"))
    SRCS.append(_add_out("decoder.cc"))

    # Generate source file names based on exec_splits
    for i in range(1, exec_splits + 1):
        SRCS.append(_add_out("generic_cpu_exec_{i}.cc".format(i = i)))

    # Generate source file names based on constrs_splits
    for i in range(1, constrs_splits + 1):
        SRCS.append(_add_out("inst-constrs-{i}.cc".format(i = i)))

    inc_files = [
        "decode-method.cc.inc",
        "decoder-g.cc.inc",
        "decoder-g.hh.inc",
        "decoder-ns.cc.inc",
        "decoder-ns.hh.inc",
        "exec-g.cc.inc",
        "exec-ns.cc.inc",
    ]
    for inc in inc_files:
        INCS.append(_add_out(inc))

    genrule_name = "_%s_isa_gen" % name
    native.genrule(
        name = genrule_name,
        srcs = isa_srcs,
        outs = OUTS,
        cmd = """
            set -e
            OUTDIR=$(RULEDIR)/{isa_gen_path}
            mkdir -p $$OUTDIR
            $(location {tool}) \
                $(location {main_isa_file}) \
                $$OUTDIR
        """.format(
            tool = tool_name,
            arch = arch,
            main_isa_file = main_isa_file,
            isa_gen_path = isa_gen_path,
        ),
        tools = [tool_name],
    )

    cc_library(
        name = name + "_decoder",
        hdrs = [h for h in HDRS if h.endswith("decoder.hh")],
        textual_hdrs = [inc for inc in INCS if "decoder" in inc],
        includes = ["."],  # Add current package to search path
        copts = get_default_gem5_copts(),
        visibility = visibility,
        deps = [
            "//src/base:bitfield",
            "//src/mem:packet",
            "//src/sim:faults",
        ] + extra_hdrs,
    )

    cc_library(
        name = name,
        srcs = SRCS,
        hdrs = HDRS,
        textual_hdrs = INCS,
        includes = ["."],  # Add current package to search path
        copts = get_default_gem5_copts() + ["-Wno-self-assign"],
        visibility = visibility,
        deps = [
            ":" + name + "_decoder",
            "//src/base:compiler",
            "//src/cpu:exec_context",
        ] + extra_deps,
    )

def gem5_cc_test(name, srcs, deps = [], **kwargs):
    """Generates a cc_test target for a gem5 C++ test.
    """
    cc_test(
        name = name,
        srcs = srcs,
        deps = _transform_gem5_deps(deps),
        copts = get_default_gem5_copts(),
        **kwargs
    )
