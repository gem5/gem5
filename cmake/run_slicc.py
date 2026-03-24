#!/usr/bin/env python3
# cmake/run_slicc.py
#
# Standalone CLI wrapper to invoke the SLICC protocol compiler for CMake
# build integration.
#
# SLICC (Specification Language for Implementing Cache Coherence) generates
# C++ code and SimObject Python files from .sm (state machine) descriptions.
#
# The SLICC parser lives in src/mem/slicc/ and requires:
#   - build_tools/ on PYTHONPATH for code_formatter and grammar modules
#   - src/mem/ on PYTHONPATH for the slicc package itself
#
# Usage:
#   python3 run_slicc.py <slicc_file> <output_dir> <source_dir>
#       [--includes INC1,INC2,...] [--protocol-base DIR]
#       [--print-files] [--file-manifest FILE]

import argparse
import os
import sys

# Allow importing file_utils from build_tools/ regardless of how this
# script is invoked (direct execution or via sys.path setup below).
_script_dir = os.path.dirname(os.path.abspath(__file__))
_build_tools = os.path.join(os.path.dirname(_script_dir), "build_tools")
if _build_tools not in sys.path:
    sys.path.insert(0, _build_tools)

from file_utils import write_if_changed


def main():
    parser = argparse.ArgumentParser(
        description="Run SLICC protocol compiler for CMake"
    )
    parser.add_argument("slicc_file", help="Path to the .slicc file")
    parser.add_argument(
        "output_dir", help="Output directory for generated code"
    )
    parser.add_argument(
        "source_dir", help="gem5 source root directory (for module path setup)"
    )
    parser.add_argument(
        "--includes",
        default="",
        help="Comma-separated C++ #include paths for generated code "
        "(e.g., mem/ruby/slicc_interface/RubySlicc_includes.hh)",
    )
    parser.add_argument(
        "--protocol-base",
        default=None,
        help="Protocol base directory (default: <source_dir>/src/mem/ruby/protocol)",
    )
    parser.add_argument(
        "--print-files",
        action="store_true",
        help="Print files SLICC will generate (one per line) and exit",
    )
    parser.add_argument(
        "--file-manifest",
        default=None,
        help="Write generated file list to this file (one per line)",
    )
    args = parser.parse_args()

    # Setup sys.path so SLICC can find its dependencies:
    #   - build_tools/ provides code_formatter, grammar, etc.
    #   - src/mem/ provides the slicc package (slicc.parser, slicc.ast, ...)
    #   - ext/ply/ provides the vendored PLY package (ply.lex, ply.yacc)
    build_tools = os.path.join(args.source_dir, "build_tools")
    src_mem = os.path.join(args.source_dir, "src", "mem")
    ext_ply = os.path.join(args.source_dir, "ext", "ply")
    sys.path.insert(0, build_tools)
    sys.path.insert(0, src_mem)
    sys.path.insert(0, ext_ply)

    from slicc.parser import SLICC

    # Determine protocol base directory
    protocol_base = args.protocol_base or os.path.join(
        src_mem, "ruby", "protocol"
    )

    # Parse includes list
    includes = []
    if args.includes:
        includes = [i.strip() for i in args.includes.split(",") if i.strip()]

    # Determine the interfaces slicc file
    interfaces_slicc = os.path.join(
        protocol_base, "RubySlicc_interfaces.slicc"
    )
    interfaces_list = (
        [interfaces_slicc] if os.path.exists(interfaces_slicc) else []
    )

    slicc = SLICC(
        args.slicc_file,
        interfaces_list,
        protocol_base,
    )

    if args.print_files:
        for f in sorted(slicc.files()):
            print(f)
        return

    # Process and generate code
    slicc.process()

    os.makedirs(args.output_dir, exist_ok=True)
    slicc.writeCodeFiles(args.output_dir, includes)

    # Write file manifest if requested
    if args.file_manifest:
        all_files = sorted(slicc.files())
        # Also include the ProtocolInfo header
        if slicc.protocol:
            info_hh = "{proto}/{proto}ProtocolInfo.hh".format(
                proto=slicc.protocol
            )
            if info_hh not in all_files:
                all_files.append(info_hh)

        manifest_content = "".join(name + "\n" for name in all_files)
        write_if_changed(args.file_manifest, manifest_content)


if __name__ == "__main__":
    main()
