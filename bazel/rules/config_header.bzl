"""Rules for generating gem5 config headers.

Each config header is a file in config/ with a #define based on build flag values.
Source code includes these as #include "config/have_png.hh" etc.

Three macros:
- gem5_config_header: Simple #define KEY VALUE
- gem5_config_header_string: #define KEY "VALUE" (quoted string)
- gem5_config_header_namespace: #define KEY Namespace (namespace alias)

All value attrs are strings that accept select() for flag-dependent generation.
"""

def _config_header_impl(ctx):
    key = ctx.attr.key
    value = ctx.attr.value
    fname = ctx.attr.filename if ctx.attr.filename else key.lower()
    out = ctx.actions.declare_file("config/{}.hh".format(fname))

    guard = "CONFIG_{}_HH".format(key.upper())
    content = "#ifndef {guard}\n#define {guard}\n\n#define {key} {value}\n\n#endif // {guard}\n".format(
        guard = guard,
        key = key,
        value = value,
    )
    ctx.actions.write(out, content)
    return [DefaultInfo(files = depset([out]))]

_config_header = rule(
    implementation = _config_header_impl,
    attrs = {
        "key": attr.string(mandatory = True),
        "value": attr.string(mandatory = True),
        "filename": attr.string(default = ""),
    },
)

def _config_header_string_impl(ctx):
    key = ctx.attr.key
    value = ctx.attr.value
    fname = ctx.attr.filename if ctx.attr.filename else key.lower()
    out = ctx.actions.declare_file("config/{}.hh".format(fname))

    guard = "CONFIG_{}_HH".format(key.upper())
    content = '#ifndef {guard}\n#define {guard}\n\n#define {key} "{value}"\n\n#endif // {guard}\n'.format(
        guard = guard,
        key = key,
        value = value,
    )
    ctx.actions.write(out, content)
    return [DefaultInfo(files = depset([out]))]

_config_header_string = rule(
    implementation = _config_header_string_impl,
    attrs = {
        "key": attr.string(mandatory = True),
        "value": attr.string(mandatory = True),
        "filename": attr.string(default = ""),
    },
)

def gem5_config_header(name, key, value, filename = None, visibility = None):
    """Generate a config header file with #define KEY VALUE.

    Args:
        name: Target name.
        key: The #define key (e.g., "HAVE_PNG").
        value: The #define value (e.g., "1" or "0"). Supports select().
        filename: Override output filename (without .hh). Default: key.lower().
        visibility: Visibility.
    """
    kwargs = {"name": name, "key": key, "value": value}
    if filename:
        kwargs["filename"] = filename
    if visibility:
        kwargs["visibility"] = visibility
    _config_header(**kwargs)

def gem5_config_header_string(name, key, value, filename = None, visibility = None):
    """Generate a config header file with #define KEY "VALUE" (quoted string).

    Args:
        name: Target name.
        key: The #define key (e.g., "KVM_ISA").
        value: The string value. Supports select().
        filename: Override output filename (without .hh). Default: key.lower().
        visibility: Visibility.
    """
    kwargs = {"name": name, "key": key, "value": value}
    if filename:
        kwargs["filename"] = filename
    if visibility:
        kwargs["visibility"] = visibility
    _config_header_string(**kwargs)

def gem5_config_header_namespace(name, key, value, filename = None, visibility = None):
    """Generate a config header with #define KEY Value (unquoted namespace alias).

    Args:
        name: Target name.
        key: The #define key (e.g., "TheGpuISA").
        value: The namespace name (e.g., "VegaISA"). Supports select().
        filename: Override output filename (without .hh). Default: key.lower().
        visibility: Visibility.
    """
    kwargs = {"name": name, "key": key, "value": value}
    if filename:
        kwargs["filename"] = filename
    if visibility:
        kwargs["visibility"] = visibility
    _config_header(**kwargs)
