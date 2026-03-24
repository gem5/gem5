"""Rule for embedding binary blobs as C++ arrays.

Converts arbitrary binary files (GDB XML, device trees, Python importer)
into C++ source and header files with uint8_t arrays.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _blob_gen_impl(ctx):
    src = ctx.file.src
    symbol = ctx.attr.symbol
    include_prefix = ctx.attr.include_prefix

    # Generate into a subdirectory matching include_prefix if set
    if include_prefix:
        cc_file = ctx.actions.declare_file("{}/{}.cc".format(include_prefix, symbol))
        hh_file = ctx.actions.declare_file("{}/{}.hh".format(include_prefix, symbol))
        include_path = "{}/{}.hh".format(include_prefix, symbol)
    else:
        cc_file = ctx.actions.declare_file(symbol + ".cc")
        hh_file = ctx.actions.declare_file(symbol + ".hh")
        include_path = symbol + ".hh"

    # generate_blob.py expects: SYMBOL INPUT_FILE OUTPUT_CC OUTPUT_HH INCLUDE_PATH
    ctx.actions.run(
        outputs = [cc_file, hh_file],
        inputs = [src],
        executable = ctx.executable._tool,
        arguments = [
            symbol,
            src.path,
            cc_file.path,
            hh_file.path,
            include_path,
        ],
        mnemonic = "BlobGen",
        progress_message = "Generating blob for {}".format(symbol),
    )

    return [DefaultInfo(files = depset([cc_file, hh_file]))]

_blob_gen = rule(
    implementation = _blob_gen_impl,
    attrs = {
        "src": attr.label(mandatory = True, allow_single_file = True),
        "symbol": attr.string(mandatory = True),
        "include_prefix": attr.string(default = ""),
        "_tool": attr.label(
            default = "//build_tools:blob_gen",
            executable = True,
            cfg = "exec",
        ),
    },
)

def gem5_blob(name, src, symbol = None, include_prefix = "",
              visibility = None, deps = [], **kwargs):
    """Embed a binary file as a C++ array.

    Args:
        name: Target name.
        src: Binary file to embed.
        symbol: C++ symbol name. Defaults to name.
        include_prefix: Subdirectory prefix for generated files
            (e.g., "python" produces python/{symbol}.hh).
        visibility: Visibility.
        deps: Additional dependencies.
    """
    if symbol == None:
        symbol = name

    gen_name = name + "_gen"
    _blob_gen(
        name = gen_name,
        src = src,
        symbol = symbol,
        include_prefix = include_prefix,
        **kwargs
    )
    cc_library(
        name = name,
        srcs = [gen_name],
        hdrs = [gen_name],
        deps = deps,
        includes = ["."],
        visibility = visibility,
    )
