"""Rule for ISA description compilation.

Compiles .isa files into generated C++ decoder, instruction constructor,
and execution code using the ISA parser.
"""

load("@rules_cc//cc:defs.bzl", "cc_library")

def _isa_desc_gen_impl(ctx):
    """Run the ISA parser to generate C++ from .isa description."""
    desc_file = ctx.file.desc
    decoder_splits = ctx.attr.decoder_splits
    exec_splits = ctx.attr.exec_splits

    # Declare all output files
    outputs = []
    gen_dir = "generated"

    # Always-generated include files
    inc_files = [
        "decoder-g.cc.inc",
        "decoder-ns.cc.inc",
        "decode-method.cc.inc",
        "decoder.hh",
        "decoder-g.hh.inc",
        "decoder-ns.hh.inc",
        "exec-g.cc.inc",
        "exec-ns.cc.inc",
    ]
    for f in inc_files:
        outputs.append(ctx.actions.declare_file("{}/{}".format(gen_dir, f)))

    # Source files that get compiled
    decoder_cc = ctx.actions.declare_file("{}/decoder.cc".format(gen_dir))
    outputs.append(decoder_cc)

    if decoder_splits == 1:
        f = ctx.actions.declare_file("{}/inst-constrs.cc".format(gen_dir))
        outputs.append(f)
    else:
        for i in range(1, decoder_splits + 1):
            f = ctx.actions.declare_file("{}/inst-constrs-{}.cc".format(gen_dir, i))
            outputs.append(f)

    if exec_splits == 1:
        f = ctx.actions.declare_file("{}/generic_cpu_exec.cc".format(gen_dir))
        outputs.append(f)
    else:
        for i in range(1, exec_splits + 1):
            f = ctx.actions.declare_file("{}/generic_cpu_exec_{}.cc".format(gen_dir, i))
            outputs.append(f)

    # Collect all input files
    parser_inputs = ctx.files._parser_files + ctx.files._ply_files + \
                    [ctx.file._micro_asm] + ctx.files._grammar_files

    output_dir = outputs[0].dirname

    # Find key files for import path setup
    parser_init = None
    for f in ctx.files._parser_files:
        if f.path.endswith("isa_parser/__init__.py"):
            parser_init = f
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

    script = ctx.actions.declare_file("_run_isa_parser_{}.py".format(ctx.label.name))
    ctx.actions.write(
        output = script,
        content = """\
import sys
import os

input_file = sys.argv[1]
output_dir = sys.argv[2]
parser_init = sys.argv[3]
ply_init = sys.argv[4]
grammar_file = sys.argv[5]

arch_dir = os.path.dirname(os.path.dirname(parser_init))
sys.path.insert(0, arch_dir)

ply_parent = os.path.dirname(os.path.dirname(ply_init))
sys.path.insert(0, ply_parent)

grammar_dir = os.path.dirname(grammar_file)
sys.path.insert(0, grammar_dir)

from isa_parser import ISAParser

os.makedirs(output_dir, exist_ok=True)
parser = ISAParser(output_dir)
parser.parse_isa_desc(input_file)
""",
    )

    ctx.actions.run_shell(
        outputs = outputs,
        inputs = [desc_file, script] + ctx.files.isa_sources + parser_inputs,
        command = "python3 {} {} {} {} {} {}".format(
            script.path,
            desc_file.path,
            output_dir,
            parser_init.path if parser_init else "",
            ply_init.path if ply_init else "",
            grammar_file.path if grammar_file else "",
        ),
        mnemonic = "ISADesc",
        progress_message = "Compiling ISA description {}".format(desc_file.short_path),
    )

    return [DefaultInfo(files = depset(outputs))]

_isa_desc_gen = rule(
    implementation = _isa_desc_gen_impl,
    attrs = {
        "desc": attr.label(mandatory = True, allow_single_file = [".isa"]),
        "decoder_splits": attr.int(default = 1),
        "exec_splits": attr.int(default = 1),
        "isa_sources": attr.label_list(
            allow_files = [".isa", ".py"],
            doc = "Additional .isa and .py files included by the main description.",
        ),
        "_parser_files": attr.label(
            default = "//src/arch/isa_parser:isa_parser_files",
            allow_files = True,
        ),
        "_micro_asm": attr.label(
            default = "//src/arch:micro_asm.py",
            allow_single_file = True,
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

def gem5_isa_desc(name, desc, decoder_splits = 1, exec_splits = 1,
                  isa_sources = [], visibility = None, deps = [], copts = []):
    """Compile an ISA description file into C++ sources.

    Args:
        name: Target name.
        desc: The main .isa description file.
        decoder_splits: Number of inst-constrs output files.
        exec_splits: Number of generic_cpu_exec output files.
        isa_sources: Additional .isa files that are included.
        visibility: Visibility.
        deps: Dependencies for the generated cc_library.
        copts: Additional compiler flags for generated code.
    """
    gen_name = "_gen_isa_{}".format(name)

    _isa_desc_gen(
        name = gen_name,
        desc = desc,
        decoder_splits = decoder_splits,
        exec_splits = exec_splits,
        isa_sources = isa_sources,
    )

    cc_library(
        name = name,
        srcs = [":{}".format(gen_name)],
        hdrs = [":{}".format(gen_name)],
        deps = deps,
        copts = copts + ["-Wno-self-assign"],
        visibility = visibility,
    )
