#!/usr/bin/env python3
#
# Copyright (c) 2026 The Regents of the University of California
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Experimental helper for canonicalizing gem5 text stat names.

This tool does not infer SimObjectVector paths from stats.txt alone. When a
config.json is supplied, it uses the explicit SimObject paths in that file to
map vector elements such as ``system.cpu0`` to ``system.cpu[0]``.
"""

import argparse
import json
import re
import sys
from pathlib import Path

_STAT_LINE_RE = re.compile(
    r"^(?P<indent>\s*)"
    r"(?P<name>\S+)"
    r"(?P<sep>\s+)"
    r"(?P<value>[+-]?(?:(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|"
    r"nan|inf))"
    r"(?P<trailing>(?:\s.*)?)$",
    re.IGNORECASE,
)


class CanonicalizationError(Exception):
    """Raised when names cannot be safely canonicalized."""


def build_path_map(config):
    """Return actual SimObject paths mapped to canonical paths.

    The mapping is conservative: vector indexes are added only when config.json
    exposes a list of child SimObjects. All other child paths retain their
    existing component names.
    """

    path_map = {}

    def child_path(parent_path, child_name):
        if parent_path == "root":
            return child_name
        return f"{parent_path}.{child_name}"

    def walk(node, canonical_path=None):
        if not isinstance(node, dict):
            return

        actual_path = node.get("path")
        if canonical_path is None:
            canonical_path = actual_path

        if actual_path and canonical_path:
            path_map[actual_path] = canonical_path

        if canonical_path is None:
            return

        for name, value in node.items():
            if isinstance(value, list):
                for index, child in enumerate(value):
                    if isinstance(child, dict) and "path" in child:
                        walk(
                            child,
                            child_path(
                                canonical_path,
                                "{}[{}]".format(
                                    name,
                                    index,
                                ),
                            ),
                        )
            elif isinstance(value, dict) and "path" in value:
                walk(value, child_path(canonical_path, name))

    walk(config)
    return path_map


def canonicalize_name(name, path_map):
    """Canonicalize one stat name using a config-derived path map."""

    base_name, separator, subname = name.partition("::")

    for actual_path, canonical_path in sorted(
        path_map.items(),
        key=lambda item: len(item[0]),
        reverse=True,
    ):
        if base_name == actual_path:
            canonical = canonical_path
            break
        if base_name.startswith(actual_path + "."):
            canonical = canonical_path + base_name[len(actual_path) :]
            break
    else:
        canonical = base_name

    if separator:
        return canonical + separator + subname
    return canonical


def canonicalize_lines(lines, path_map=None, allow_collisions=False):
    """Return canonicalized lines and an old-name to canonical-name mapping."""

    if path_map is None:
        path_map = {}

    output = []
    mapping = {}
    canonical_to_old = {}

    for line in lines:
        content = line.rstrip("\r\n")
        line_ending = line[len(content) :]
        match = _STAT_LINE_RE.match(content)
        if not match:
            output.append(line)
            continue

        name = match.group("name")
        canonical = canonicalize_name(name, path_map)
        mapping[name] = canonical
        canonical_to_old.setdefault(canonical, set()).add(name)
        output.append(
            "{indent}{name}{sep}{value}{trailing}".format(
                indent=match.group("indent"),
                name=canonical,
                sep=match.group("sep"),
                value=match.group("value"),
                trailing=match.group("trailing"),
            )
            + line_ending
        )

    collisions = {
        canonical: sorted(old_names)
        for canonical, old_names in canonical_to_old.items()
        if len(old_names) > 1
    }
    if collisions and not allow_collisions:
        details = ", ".join(
            "{} <- {}".format(
                canonical,
                ", ".join(old_names),
            )
            for canonical, old_names in sorted(collisions.items())
        )
        raise CanonicalizationError("canonical name collision: " + details)

    return output, mapping


def canonicalize_file(
    input_path,
    output_path,
    config_json_path=None,
    mapping_output_path=None,
    allow_collisions=False,
):
    """Canonicalize a stats.txt file."""

    path_map = {}
    if config_json_path is not None:
        with open(config_json_path, encoding="utf-8") as config_file:
            path_map = build_path_map(json.load(config_file))

    with open(input_path, encoding="utf-8") as input_file:
        output, mapping = canonicalize_lines(
            input_file,
            path_map=path_map,
            allow_collisions=allow_collisions,
        )

    with open(output_path, "w", encoding="utf-8") as output_file:
        output_file.writelines(output)

    if mapping_output_path is not None:
        with open(mapping_output_path, "w", encoding="utf-8") as mapping_file:
            json.dump(mapping, mapping_file, indent=2, sort_keys=True)
            mapping_file.write("\n")


def _get_args():
    parser = argparse.ArgumentParser(
        description=(
            "Experimentally canonicalize gem5 stats.txt names using "
            "SimObjectVector paths from config.json."
        )
    )
    parser.add_argument("--input", required=True, help="Input stats.txt path.")
    parser.add_argument(
        "--output",
        required=True,
        help="Output stats.txt path for canonicalized names.",
    )
    parser.add_argument(
        "--config-json",
        help="Optional gem5 config.json used to identify SimObjectVectors.",
    )
    parser.add_argument(
        "--mapping-output",
        help="Optional JSON output path for old-name to canonical-name mapping.",
    )
    parser.add_argument(
        "--allow-collisions",
        action="store_true",
        help=(
            "Allow multiple old names to map to the same canonical name. "
            "By default this is an error."
        ),
    )
    return parser.parse_args()


def main():
    args = _get_args()
    try:
        canonicalize_file(
            Path(args.input),
            Path(args.output),
            config_json_path=(
                Path(args.config_json) if args.config_json else None
            ),
            mapping_output_path=(
                Path(args.mapping_output) if args.mapping_output else None
            ),
            allow_collisions=args.allow_collisions,
        )
    except CanonicalizationError as error:
        print(f"error: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
