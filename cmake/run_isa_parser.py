#!/usr/bin/env python3
"""Wrapper to invoke the gem5 ISA parser from CMake.

Usage: run_isa_parser.py <isa_file> <output_dir>

This script adds the necessary paths to sys.path so that the isa_parser
package and its dependencies (grammar, PLY) can be imported, then runs
the parser.
"""

import os
import sys

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <isa_file> <output_dir>", file=sys.stderr)
        sys.exit(1)

    isa_file = os.path.abspath(sys.argv[1])
    output_dir = os.path.abspath(sys.argv[2])

    # Determine the gem5 source root from the script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    source_root = os.path.dirname(script_dir)

    # Add src/arch to sys.path so 'isa_parser' package can be imported
    arch_dir = os.path.join(source_root, "src", "arch")
    sys.path.insert(0, arch_dir)

    # Add build_tools to sys.path so 'grammar' module can be imported
    build_tools_dir = os.path.join(source_root, "build_tools")
    sys.path.insert(0, build_tools_dir)

    os.makedirs(output_dir, exist_ok=True)

    import isa_parser
    parser = isa_parser.ISAParser(output_dir)
    parser.parse_isa_desc(isa_file)

if __name__ == "__main__":
    main()
