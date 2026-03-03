"""Rule for embedding binary blobs as C++ arrays.

Converts arbitrary binary files (GDB XML, device trees) into C++ source
and header files with uint8_t arrays.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _blob_gen_impl(ctx):
    src = ctx.file.src
    symbol = ctx.attr.symbol
    cc_file = ctx.actions.declare_file(symbol + ".cc")
    hh_file = ctx.actions.declare_file(symbol + ".hh")

    ctx.actions.run(
        outputs = [cc_file, hh_file],
        inputs = [src, ctx.executable._tool],
        executable = ctx.executable._tool,
        arguments = [
            src.path,
            cc_file.path,
            hh_file.path,
            symbol,
        ],
        progress_message = "Generating blob for {}".format(symbol),
    )

    return [DefaultInfo(files = depset([cc_file, hh_file]))]

_blob_gen = rule(
    implementation = _blob_gen_impl,
    attrs = {
        "src": attr.label(mandatory = True, allow_single_file = True),
        "symbol": attr.string(mandatory = True),
        "_tool": attr.label(
            default = "//build_tools:blob_gen",
            executable = True,
            cfg = "exec",
        ),
    },
)

def gem5_blob(name, src, symbol = None, visibility = None, **kwargs):
    """Embed a binary file as a C++ array.

    Args:
        name: Target name.
        src: Binary file to embed.
        symbol: C++ symbol name. Defaults to name.
        visibility: Visibility.
    """
    if symbol == None:
        symbol = name

    gen_name = name + "_gen"
    _blob_gen(
        name = gen_name,
        src = src,
        symbol = symbol,
        **kwargs
    )
    cc_library(
        name = name,
        srcs = [gen_name],
        hdrs = [gen_name],
        includes = ["."],
        visibility = visibility,
    )
