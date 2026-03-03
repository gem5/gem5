"""Rules for generating gem5 debug flag header and source files.

Debug flags are generated entirely in Starlark (no external scripts).
Each flag produces debug/{name}.hh and debug/{name}.cc as a cc_library.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _debug_flag_hh(name, desc, fmt, components):
    """Generate the content of debug/{name}.hh."""
    guard = "DEBUG_{}_HH".format(name.upper())
    lines = [
        "#ifndef {}".format(guard),
        "#define {}".format(guard),
        "",
        '#include "base/debug.hh"',
        "",
        "namespace gem5",
        "{",
        "",
        "namespace debug",
        "{",
        "",
    ]

    if components:
        lines.append("class {} : public CompoundFlag".format(name))
        lines.append("{")
        lines.append("  public:")
        lines.append("    {}();".format(name))
        lines.append("};")
    elif fmt:
        lines.append("class {} : public SimpleFlag".format(name))
        lines.append("{")
        lines.append("  public:")
        lines.append("    {}();".format(name))
        lines.append("};")
    else:
        lines.append("class {} : public SimpleFlag".format(name))
        lines.append("{")
        lines.append("  public:")
        lines.append("    {}();".format(name))
        lines.append("};")

    lines += [
        "",
        "} // namespace debug",
        "",
        "} // namespace gem5",
        "",
        "#endif // {}".format(guard),
        "",
    ]
    return "\n".join(lines)

def _debug_flag_cc(name, desc, fmt, components):
    """Generate the content of debug/{name}.cc."""
    lines = [
        '#include "debug/{}.hh"'.format(name),
        "",
    ]

    if components:
        for comp in components:
            lines.append('#include "debug/{}.hh"'.format(comp))
        lines.append("")

    lines += [
        "namespace gem5",
        "{",
        "",
        "namespace debug",
        "{",
        "",
    ]

    if components:
        lines.append("{}::{}()".format(name, name))
        lines.append('    : CompoundFlag("{}", "{}", {{'.format(name, desc))
        for comp in components:
            lines.append("        &{},".format(comp))
        lines.append("    })")
        lines.append("{}")
    elif fmt:
        lines.append("{}::{}()".format(name, name))
        lines.append('    : SimpleFlag("{}", "{}", true)'.format(name, desc))
        lines.append("{}")
    else:
        lines.append("{}::{}()".format(name, name))
        lines.append('    : SimpleFlag("{}", "{}", false)'.format(name, desc))
        lines.append("{}")

    lines += [
        "",
        "} // namespace debug",
        "",
        "} // namespace gem5",
        "",
    ]
    return "\n".join(lines)

def _debug_flag_gen_impl(ctx):
    hh_file = ctx.actions.declare_file("debug/{}.hh".format(ctx.attr.flag_name))
    cc_file = ctx.actions.declare_file("debug/{}.cc".format(ctx.attr.flag_name))

    hh_content = _debug_flag_hh(
        ctx.attr.flag_name,
        ctx.attr.desc,
        ctx.attr.fmt,
        ctx.attr.components,
    )
    cc_content = _debug_flag_cc(
        ctx.attr.flag_name,
        ctx.attr.desc,
        ctx.attr.fmt,
        ctx.attr.components,
    )

    ctx.actions.write(hh_file, hh_content)
    ctx.actions.write(cc_file, cc_content)

    return [DefaultInfo(files = depset([hh_file, cc_file]))]

_debug_flag_gen = rule(
    implementation = _debug_flag_gen_impl,
    attrs = {
        "flag_name": attr.string(mandatory = True),
        "desc": attr.string(default = ""),
        "fmt": attr.bool(default = False),
        "components": attr.string_list(default = []),
    },
)

def gem5_debug_flag(name, desc = "", visibility = None, **kwargs):
    """Generate a simple debug flag.

    Creates debug/{name}.hh and debug/{name}.cc as a cc_library.
    """
    gen_name = name + "_gen"
    _debug_flag_gen(
        name = gen_name,
        flag_name = name,
        desc = desc,
        **kwargs
    )
    cc_library(
        name = name,
        srcs = [gen_name],
        hdrs = [gen_name],
        deps = ["//src/base:debug_hdrs"],
        includes = ["."],
        visibility = visibility,
    )

def gem5_compound_flag(name, desc = "", components = [], visibility = None, **kwargs):
    """Generate a compound debug flag that combines other flags."""
    gen_name = name + "_gen"
    _debug_flag_gen(
        name = gen_name,
        flag_name = name,
        desc = desc,
        components = components,
        **kwargs
    )
    component_deps = [":" + c for c in components]
    cc_library(
        name = name,
        srcs = [gen_name],
        hdrs = [gen_name],
        deps = ["//src/base:debug_hdrs"] + component_deps,
        includes = ["."],
        visibility = visibility,
    )

def gem5_debug_format_flag(name, desc = "", visibility = None, **kwargs):
    """Generate a format-only debug flag."""
    gen_name = name + "_gen"
    _debug_flag_gen(
        name = gen_name,
        flag_name = name,
        desc = desc,
        fmt = True,
        **kwargs
    )
    cc_library(
        name = name,
        srcs = [gen_name],
        hdrs = [gen_name],
        deps = ["//src/base:debug_hdrs"],
        includes = ["."],
        visibility = visibility,
    )
