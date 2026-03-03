"""Rules for generating gem5 config headers.

Each config header is a file in config/ with a #define based on build flag values.
Source code includes these as #include "config/have_png.hh" etc.

Two rules:
- gem5_config_header: Simple #define KEY VALUE
- gem5_config_header_string: #define KEY "VALUE" (quoted string)
- gem5_config_header_namespace: #define KEY Namespace (namespace alias, e.g., TheGpuISA)
"""

def _config_header_impl(ctx):
    key = ctx.attr.key
    value = ctx.attr.value
    filename = "config/{}.hh".format(key.lower())
    out = ctx.actions.declare_file(filename)

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
    },
)

def _config_header_string_impl(ctx):
    key = ctx.attr.key
    value = ctx.attr.value
    filename = "config/{}.hh".format(key.lower())
    out = ctx.actions.declare_file(filename)

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
    },
)

def gem5_config_header(name, key, value, visibility = None):
    """Generate a config header file with #define KEY VALUE.

    Args:
        name: Target name.
        key: The #define key (e.g., "HAVE_PNG").
        value: The #define value (e.g., "1" or "0"). Supports select().
        visibility: Visibility.
    """
    _config_header(
        name = name,
        key = key,
        value = value,
        visibility = visibility,
    )

def gem5_config_header_string(name, key, value, visibility = None):
    """Generate a config header file with #define KEY "VALUE" (quoted string).

    Args:
        name: Target name.
        key: The #define key (e.g., "KVM_ISA").
        value: The string value. Supports select().
        visibility: Visibility.
    """
    _config_header_string(
        name = name,
        key = key,
        value = value,
        visibility = visibility,
    )

def gem5_config_header_namespace(name, key, value, visibility = None):
    """Generate a config header file with #define KEY Value (unquoted, for namespace aliases).

    Args:
        name: Target name.
        key: The #define key (e.g., "TheGpuISA").
        value: The namespace name (e.g., "VegaISA"). Supports select().
        visibility: Visibility.
    """
    _config_header(
        name = name,
        key = key,
        value = value,
        visibility = visibility,
    )
