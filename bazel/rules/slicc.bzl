"""Rule for SLICC protocol compilation.

Compiles .slicc protocol definitions into C++ controller code.
Uses tree artifacts (declare_directory) since SLICC output files
are dynamic and depend on protocol content.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _slicc_gen_impl(ctx):
    """Run SLICC to generate C++ from protocol definitions."""
    src = ctx.file.src
    protocol = ctx.attr.protocol

    # Use tree artifact for output (SLICC generates dynamic file sets)
    out_dir = ctx.actions.declare_directory("slicc_{}".format(protocol))

    # Find SLICC init for path setup
    slicc_init = None
    for f in ctx.files._slicc_files:
        if f.path.endswith("slicc/__init__.py"):
            slicc_init = f
            break

    ply_init = None
    for f in ctx.files._ply_files:
        if f.path.endswith("ply/__init__.py"):
            ply_init = f
            break

    grammar_file = None
    for f in ctx.files._grammar_files:
        if f.path.endswith("grammar.py"):
            grammar_file = f
            break

    script = ctx.actions.declare_file("_run_slicc_{}.py".format(ctx.label.name))
    ctx.actions.write(
        output = script,
        content = """\
import sys
import os

slicc_file = sys.argv[1]
output_dir = sys.argv[2]
slicc_init = sys.argv[3]
ply_init = sys.argv[4]
grammar_file = sys.argv[5]

# Set up import paths
slicc_parent = os.path.dirname(os.path.dirname(slicc_init))
sys.path.insert(0, slicc_parent)

ply_parent = os.path.dirname(os.path.dirname(ply_init))
sys.path.insert(0, ply_parent)

grammar_dir = os.path.dirname(grammar_file)
sys.path.insert(0, grammar_dir)

from slicc.main import main as slicc_main

os.makedirs(output_dir, exist_ok=True)

protocol_dir = os.path.dirname(slicc_file)

# Build SLICC command. SLICC expects:
# slicc --protocol-dir DIR --output-dir DIR file.slicc
argv = [
    "slicc",
    "--protocol-dir", protocol_dir,
    "--output-dir", output_dir,
    slicc_file,
]

slicc_main(argv)
""",
    )

    all_inputs = [src, script] + ctx.files.sm_sources + \
                 ctx.files._slicc_files + ctx.files._ply_files + \
                 ctx.files._grammar_files

    ctx.actions.run_shell(
        outputs = [out_dir],
        inputs = all_inputs,
        command = "python3 {} {} {} {} {} {}".format(
            script.path,
            src.path,
            out_dir.path,
            slicc_init.path if slicc_init else "",
            ply_init.path if ply_init else "",
            grammar_file.path if grammar_file else "",
        ),
        mnemonic = "SLICC",
        progress_message = "Compiling SLICC protocol {}".format(protocol),
    )

    return [DefaultInfo(files = depset([out_dir]))]

_slicc_gen = rule(
    implementation = _slicc_gen_impl,
    attrs = {
        "src": attr.label(mandatory = True, allow_single_file = [".slicc"]),
        "protocol": attr.string(mandatory = True),
        "sm_sources": attr.label_list(
            allow_files = [".sm", ".slicc"],
            doc = "State machine and protocol files.",
        ),
        "_slicc_files": attr.label(
            default = "//src/mem/slicc:slicc_files",
            allow_files = True,
        ),
        "_ply_files": attr.label(
            default = "@gem5_ext_ply//:ply_files",
            allow_files = True,
        ),
        "_grammar_files": attr.label(
            default = "//build_tools:grammar_files",
            allow_files = True,
        ),
    },
)

def gem5_slicc_protocol(name, src, protocol = None, sm_sources = [],
                        visibility = None, deps = [], copts = []):
    """Compile a SLICC protocol into C++ sources.

    Uses tree artifacts for output since SLICC generates a dynamic set
    of files depending on protocol content.

    Args:
        name: Target name (typically the protocol name).
        src: The main .slicc file.
        protocol: Protocol name. Defaults to name.
        sm_sources: State machine .sm files and included .slicc files.
        visibility: Visibility.
        deps: Dependencies for the generated cc_library.
        copts: Additional compiler flags.
    """
    if protocol == None:
        protocol = name

    gen_name = "_gen_slicc_{}".format(name)

    _slicc_gen(
        name = gen_name,
        src = src,
        protocol = protocol,
        sm_sources = sm_sources,
    )

    cc_library(
        name = name,
        srcs = [":{}".format(gen_name)],
        hdrs = [":{}".format(gen_name)],
        deps = deps + ["//src:gem5_hdrs"],
        copts = copts + ["-Wno-unused-variable"],
        includes = ["."],
        visibility = visibility,
        alwayslink = True,
    )
