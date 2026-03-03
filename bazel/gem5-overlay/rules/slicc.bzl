"""Rule for SLICC protocol compilation.

Compiles .slicc protocol definitions into C++ controller code.
Uses a pre-declared output file list (generated_files attr).
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _slicc_gen_impl(ctx):
    """Run SLICC to generate C++ from protocol definitions."""
    src = ctx.file.src
    protocol = ctx.attr.protocol

    # Declare output files from the pre-declared list
    outputs = []
    gen_dir = "protocol_{}".format(protocol)
    for f in ctx.attr.generated_files:
        outputs.append(ctx.actions.declare_file("{}/{}".format(gen_dir, f)))

    if not outputs:
        fail("No generated_files specified for SLICC protocol {}".format(protocol))

    output_dir = outputs[0].dirname

    # Find SLICC init
    slicc_init = None
    for f in ctx.files._slicc_files:
        if f.path.endswith("slicc/__init__.py"):
            slicc_init = f
            break

    # Find PLY init
    ply_init = None
    for f in ctx.files._ply_files:
        if f.path.endswith("ply/__init__.py"):
            ply_init = f
            break

    # Find grammar.py
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
protocol_dirs = sys.argv[6:]

# Add parent of slicc/ to path
slicc_parent = os.path.dirname(os.path.dirname(slicc_init))
sys.path.insert(0, slicc_parent)

# Add parent of ply/ to path
ply_parent = os.path.dirname(os.path.dirname(ply_init))
sys.path.insert(0, ply_parent)

# Add build_tools/ to path
grammar_dir = os.path.dirname(grammar_file)
sys.path.insert(0, grammar_dir)

from slicc.main import main as slicc_main

os.makedirs(output_dir, exist_ok=True)

# Find RubySlicc_interfaces.slicc
protocol_dir = os.path.dirname(slicc_file)
interfaces_file = os.path.join(protocol_dir, "RubySlicc_interfaces.slicc")
if not os.path.exists(interfaces_file):
    parent_dir = os.path.dirname(protocol_dir)
    interfaces_file = os.path.join(parent_dir, "RubySlicc_interfaces.slicc")

argv = [
    "slicc",
    "--protocol-dir", protocol_dir,
    "--output-dir", output_dir,
    slicc_file,
]
if os.path.exists(interfaces_file):
    argv.extend(["--interfaces", interfaces_file])

slicc_main(argv)
""",
    )

    all_inputs = [src, script] + ctx.files.sm_sources + \
                 ctx.files._slicc_files + ctx.files._ply_files + \
                 ctx.files._grammar_files

    protocol_dir = src.dirname
    ctx.actions.run_shell(
        outputs = outputs,
        inputs = all_inputs,
        command = "python3 {} {} {} {} {} {} {}".format(
            script.path,
            src.path,
            output_dir,
            slicc_init.path if slicc_init else "",
            ply_init.path if ply_init else "",
            grammar_file.path if grammar_file else "",
            protocol_dir,
        ),
        mnemonic = "SLICC",
        progress_message = "Compiling SLICC protocol {}".format(protocol),
    )

    return [DefaultInfo(files = depset(outputs))]

_slicc_gen = rule(
    implementation = _slicc_gen_impl,
    attrs = {
        "src": attr.label(mandatory = True, allow_single_file = [".slicc"]),
        "protocol": attr.string(mandatory = True),
        "sm_sources": attr.label_list(
            allow_files = [".sm", ".slicc"],
            doc = "State machine and protocol files.",
        ),
        "generated_files": attr.string_list(
            mandatory = True,
            doc = "List of expected output file names.",
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
                        generated_files = [], visibility = None,
                        deps = [], copts = []):
    """Compile a SLICC protocol into C++ sources.

    Args:
        name: Target name (typically the protocol name).
        src: The main .slicc file.
        protocol: Protocol name. Defaults to name.
        sm_sources: State machine .sm files.
        generated_files: Expected output file names.
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
        generated_files = generated_files,
    )

    cc_library(
        name = name,
        srcs = [":{}".format(gen_name)],
        hdrs = [":{}".format(gen_name)],
        deps = deps,
        copts = copts + ["-Wno-unused-variable"],
        visibility = visibility,
        alwayslink = True,
    )
