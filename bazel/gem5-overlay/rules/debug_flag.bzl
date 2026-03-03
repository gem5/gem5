"""Rules for generating gem5 debug flag header and source files.

Debug flags are generated entirely in Starlark (no external scripts).
Each flag produces debug/{name}.hh and debug/{name}.cc as a cc_library.

The generated code uses the inline-union pattern from the official gem5
debugflaghh.py / debugflagcc.py scripts: each flag is wrapped in an
inline union that prevents destruction, and an inline constexpr reference
aliases the union member for global access.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _debug_flag_hh(name, desc, fmt, components):
    """Generate the content of debug/{name}.hh matching official pattern."""
    guard = "__DEBUG_{}_HH__".format(name.upper())
    lines = [
        "#ifndef {}".format(guard),
        "#define {}".format(guard),
        "",
        '#include "base/compiler.hh"',
        '#include "base/debug.hh"',
    ]

    for comp in components:
        lines.append('#include "debug/{}.hh"'.format(comp))

    lines += [
        "",
        "namespace gem5",
        "{",
        "",
        "namespace debug",
        "{",
        "",
        "namespace unions",
        "{",
        "",
    ]

    if components:
        comp_refs = ",\n            ".join(
            ["(Flag *)&::gem5::debug::{}".format(c) for c in components],
        )
        lines += [
            "inline union {}".format(name),
            "{",
            "    ~{}() {{}}".format(name),
            "",
            "    CompoundFlag flag{};".format(name),
            "",
            '    {}() : flag{}("{}", "{}",'.format(name, name, name, desc),
            "        {",
            "            {}".format(comp_refs),
            "        }) {}",
            "",
            "}} instance{};".format(name),
        ]
    else:
        fmt_str = "true" if fmt else "false"
        lines += [
            "inline union {}".format(name),
            "{",
            "    ~{}() {{}}".format(name),
            "    SimpleFlag flag{};".format(name),
            "",
            '    {}() : flag{}("{}", "{}", {}) {{}}'.format(name, name, name, desc, fmt_str),
            "",
            "}} instance{};".format(name),
        ]

    lines += [
        "",
        "} // namespace unions",
        "",
        "inline constexpr const auto& {} =".format(name),
        "    ::gem5::debug::unions::instance{}.flag{};".format(name, name),
        "",
        "} // namespace debug",
        "} // namespace gem5",
        "",
        "#endif // {}".format(guard),
        "",
    ]
    return "\n".join(lines)

def _debug_flag_cc(name):
    """Generate the content of debug/{name}.cc (just includes the header)."""
    return '#include "debug/{}.hh"\n'.format(name)

def _debug_flag_gen_impl(ctx):
    hh_file = ctx.actions.declare_file("debug/{}.hh".format(ctx.attr.flag_name))
    cc_file = ctx.actions.declare_file("debug/{}.cc".format(ctx.attr.flag_name))

    hh_content = _debug_flag_hh(
        ctx.attr.flag_name,
        ctx.attr.desc,
        ctx.attr.fmt,
        ctx.attr.components,
    )
    cc_content = _debug_flag_cc(ctx.attr.flag_name)

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
