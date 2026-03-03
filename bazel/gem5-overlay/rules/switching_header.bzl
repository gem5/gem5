"""Rule for generating ISA/GPU switching headers.

Switching headers are simple #include redirects that point to the
ISA-specific version of a header based on the primary_isa flag.
"""

def _switching_header_impl(ctx):
    out = ctx.actions.declare_file(ctx.attr.output_path)
    content = '#include "{}"\n'.format(ctx.attr.target_header)
    ctx.actions.write(out, content)
    return [DefaultInfo(files = depset([out]))]

_switching_header = rule(
    implementation = _switching_header_impl,
    attrs = {
        "output_path": attr.string(mandatory = True),
        "target_header": attr.string(mandatory = True),
    },
)

def gem5_switching_headers(name, headers, isa_dir, output_prefix = "arch", visibility = None):
    """Generate ISA switching headers.

    For each header, generates a redirect file at {output_prefix}/{header}
    that includes {output_prefix}/{isa_dir}/{header}.

    Args:
        name: Target name.
        headers: List of header filenames to generate redirects for.
        isa_dir: ISA directory name (from select() on primary_isa).
        output_prefix: Output path prefix (default: "arch").
        visibility: Visibility.
    """
    all_hdrs = []
    for header in headers:
        hdr_name = "_switching_{}_{}".format(name, header.replace("/", "_").replace(".", "_"))
        _switching_header(
            name = hdr_name,
            output_path = "{}/{}".format(output_prefix, header),
            target_header = "{}/{}/{}".format(output_prefix, isa_dir, header),
        )
        all_hdrs.append(hdr_name)

    native.filegroup(
        name = name,
        srcs = all_hdrs,
        visibility = visibility,
    )
