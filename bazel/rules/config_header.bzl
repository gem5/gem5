"""Rule for generating gem5 config headers.

Each config header is a simple #define file generated from a build flag value.
Output: config/{key_lower}.hh with #define {KEY} {value}.
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

def gem5_config_header(name, key, value, visibility = None):
    """Generate a config header file.

    Args:
        name: Target name.
        key: The #define key (e.g., "HAVE_PNG").
        value: The #define value (e.g., "1" or "0").
        visibility: Visibility.
    """
    _config_header(
        name = name,
        key = key,
        value = value,
        visibility = visibility,
    )
