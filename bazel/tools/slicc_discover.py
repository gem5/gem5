"""Discover SLICC output files for manifest generation.

Runs SLICC in discovery mode (-F/--print-files) for each known protocol.
Outputs a Starlark dictionary mapping protocol name -> list of generated files.
Used by configure.bzl at repository-rule time to create configs/slicc_manifests.bzl.

Usage:
    python3 slicc_discover.py <src_root>
"""

import os
import sys


def discover_protocol_files(src_root, protocol_name, slicc_file):
    """Run SLICC parser for a protocol and return expected output files."""
    protocol_dir = os.path.join(src_root, "src", "mem", "ruby", "protocol")
    interfaces = os.path.join(protocol_dir, "RubySlicc_interfaces.slicc")
    slicc_path = os.path.join(protocol_dir, slicc_file)

    if not os.path.exists(slicc_path):
        return []

    from slicc.parser import SLICC

    try:
        slicc = SLICC(slicc_path, [interfaces], protocol_dir)
        return sorted(slicc.files())
    except Exception as e:
        print(
            "WARNING: SLICC discovery failed for {}: {}".format(
                protocol_name, e
            ),
            file=sys.stderr,
        )
        return []


PROTOCOLS = {
    "MESI_Two_Level": "MESI_Two_Level.slicc",
    "MESI_Three_Level": "MESI_Three_Level.slicc",
    "MESI_Three_Level_HTM": "MESI_Three_Level_HTM.slicc",
    "MI_example": "MI_example.slicc",
    "MOESI_AMD_Base": "MOESI_AMD_Base.slicc",
    "MOESI_CMP_directory": "MOESI_CMP_directory.slicc",
    "MOESI_CMP_token": "MOESI_CMP_token.slicc",
    "MOESI_hammer": "MOESI_hammer.slicc",
    "Garnet_standalone": "Garnet_standalone.slicc",
    "GPU_VIPER": "GPU_VIPER.slicc",
    "CHI": "chi/CHI.slicc",
}


def main():
    src_root = sys.argv[1]

    # Set up Python paths for SLICC imports
    sys.path.insert(0, os.path.join(src_root, "src", "mem"))
    sys.path.insert(0, os.path.join(src_root, "ext", "ply"))
    sys.path.insert(0, os.path.join(src_root, "build_tools"))

    print('"""Generated SLICC output manifests. Do not edit."""')
    print()
    print("SLICC_MANIFESTS = {")

    for name in sorted(PROTOCOLS.keys()):
        slicc_file = PROTOCOLS[name]
        files = discover_protocol_files(src_root, name, slicc_file)
        if files:
            print(f'    "{name}": [')
            for f in files:
                print(f'        "{f}",')
            print("    ],")
        else:
            print(f'    "{name}": [],')

    print("}")


if __name__ == "__main__":
    main()
