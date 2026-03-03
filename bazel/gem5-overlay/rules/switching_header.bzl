"""Rule for generating ISA/GPU switching headers.

Switching headers are simple #include redirects that point to the
ISA-specific version of a header based on the primary_isa flag.

The isa_dir attribute is a string that accepts select(), ensuring
configurable ISA selection is preserved through analysis time.
"""

def _switching_header_impl(ctx):
    prefix = ctx.attr.output_prefix
    header = ctx.attr.header_name
    isa = ctx.attr.isa_dir
    out = ctx.actions.declare_file("{}/{}".format(prefix, header))
    content = '#include "{}/{}/{}"\n'.format(prefix, isa, header)
    ctx.actions.write(out, content)
    return [DefaultInfo(files = depset([out]))]

_switching_header = rule(
    implementation = _switching_header_impl,
    attrs = {
        "header_name": attr.string(mandatory = True),
        "isa_dir": attr.string(mandatory = True),
        "output_prefix": attr.string(default = "arch"),
    },
)

def gem5_switching_headers(name, headers, isa_dir, output_prefix = "arch", visibility = None):
    """Generate ISA switching headers.

    For each header, generates a redirect file at {output_prefix}/{header}
    that includes {output_prefix}/{isa_dir}/{header}.

    Args:
        name: Target name.
        headers: List of header filenames to generate redirects for.
        isa_dir: ISA directory name. Accepts select() for configurable ISA.
        output_prefix: Output path prefix (default: "arch").
        visibility: Visibility.
    """
    all_hdrs = []
    for header in headers:
        hdr_name = "_switching_{}_{}".format(name, header.replace("/", "_").replace(".", "_"))
        _switching_header(
            name = hdr_name,
            header_name = header,
            isa_dir = isa_dir,
            output_prefix = output_prefix,
        )
        all_hdrs.append(hdr_name)

    native.filegroup(
        name = name,
        srcs = all_hdrs,
        visibility = visibility,
    )
