#!/usr/bin/env python3
# Copyright (c) 2026 The Regents of The University of California
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

"""Use observed coverage to suggest tests and compare compatible collections.

Suggestions are advisory. A coverage record never authorizes skipping CI.
"""

import argparse
import json
import re
import shlex
import subprocess
import sys
from pathlib import Path

try:
    from .schema import source_path
except ImportError:
    from schema import source_path


def load_index(path):
    index = json.loads(Path(path).read_text(encoding="utf-8"))
    if (
        index.get("format") != "gem5-coverage-index"
        or index.get("schema_version") != 1
    ):
        raise ValueError("Unsupported coverage index")
    return index


def inventory(index, language):
    if language not in ("native", "python"):
        raise ValueError("Unknown coverage language")
    return (
        index["files"]
        if language == "native"
        else index.get("python_files", {})
    )


def collection_limits(index, language):
    records = [
        record
        for record in index["invocations"]
        if record.get("language", "native") == language
    ]
    missing = sum(record["collection"] != "complete" for record in records)
    return {
        "language": language,
        "observed_invocations": len(records),
        "incomplete_invocations": missing,
        "failed_invocations": sum(
            record["outcome"] == "failed" for record in records
        ),
        "expected_test_inventory_known": False,
        "caveat": "The index describes collected invocations; absent tests may be missing. "
        "Use the campaign accounting report to assess collection completeness.",
    }


def covering_suites(index, members):
    return sorted(
        {
            index["invocations"][ordinal]["test_uid"]
            for ordinal in index["memberships"][members]
        }
    )


def _diff_path(value):
    value = value.split("\t", 1)[0]
    if value == "/dev/null":
        return None
    if value.startswith('"'):
        try:
            value = json.loads(value)
        except ValueError as error:
            raise ValueError(
                "Unsupported quoted diff path; use git -c core.quotePath=false"
            ) from error
    if not value.startswith(("a/", "b/")):
        raise ValueError("Diff paths must use git a/ and b/ prefixes")
    return source_path(value[2:])


def parse_diff(patch):
    """Read old-side changed coordinates and account for additions separately."""
    changes = []
    current = None
    remaining_old = remaining_new = 0
    old_line = new_line = 0
    in_hunk = False
    for text in patch.splitlines():
        if in_hunk and (remaining_old or remaining_new):
            if text.startswith("\\ No newline at end of file"):
                continue
            if not text or text[0] not in " +-":
                raise ValueError("Malformed or truncated unified diff hunk")
            prefix = text[0]
            if prefix in " -":
                remaining_old -= 1
                if prefix == "-":
                    current["old_lines"].add(old_line)
                old_line += 1
            if prefix in " +":
                remaining_new -= 1
                if prefix == "+":
                    current["added_lines"].append(new_line)
                new_line += 1
            if remaining_old < 0 or remaining_new < 0:
                raise ValueError("Unified diff hunk length mismatch")
            continue
        if text.startswith("diff --git "):
            tokens = shlex.split(text)
            if len(tokens) != 4:
                raise ValueError("Unsupported git diff file header")
            current = {
                "old_path": _diff_path(tokens[2]),
                "new_path": _diff_path(tokens[3]),
                "old_lines": set(),
                "added_lines": [],
                "binary": False,
            }
            changes.append(current)
            in_hunk = False
        elif text.startswith("--- "):
            if current is None:
                raise ValueError("Expected a git unified diff")
            current["old_path"] = _diff_path(text[4:])
        elif text.startswith("+++ "):
            if current is None:
                raise ValueError("Expected a git unified diff")
            current["new_path"] = _diff_path(text[4:])
        elif text.startswith("@@"):
            if current is None:
                raise ValueError("A diff hunk has no file")
            match = re.match(
                r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@", text
            )
            if not match:
                raise ValueError("Unsupported unified diff hunk")
            old_line, old_count, new_line, new_count = match.groups()
            old_line, new_line = int(old_line), int(new_line)
            remaining_old = int(old_count) if old_count is not None else 1
            remaining_new = int(new_count) if new_count is not None else 1
            in_hunk = True
        elif (
            text.startswith(("Binary files ", "GIT binary patch")) and current
        ):
            current["binary"] = True
        elif text.startswith(("+", "-")):
            raise ValueError("Unexpected diff body outside a declared hunk")
    if remaining_old or remaining_new:
        raise ValueError("Truncated unified diff hunk")
    if not changes and patch.strip():
        raise ValueError("No git file changes found in supplied diff")
    return changes


def suggest_tests(
    index,
    files=(),
    patch=None,
    base_revision=None,
    target_revision=None,
    language="native",
):
    """Suggest every observed covering suite, retaining gaps and provenance."""
    tested = index["revision"]
    if patch is not None and base_revision != tested:
        raise ValueError(
            "Diff base must equal the coverage revision; new-side coordinates cannot be used"
        )
    selected = inventory(index, language)
    changes = parse_diff(patch) if patch is not None else []
    for path in files:
        changes.append(
            {
                "old_path": source_path(path),
                "new_path": path,
                "old_lines": set(),
                "added_lines": [],
                "binary": False,
            }
        )
    matches, unresolved = {}, []
    grouped_evidence, visited = {}, set()
    for change in changes:
        path = change["old_path"]
        measured = selected.get(path, {}) if path else {}
        changed = change["old_lines"]
        # Added lines have no old-side coverage. Broaden to the old file's
        # observed suites without claiming those suites exercise new code.
        broaden = bool(change["added_lines"]) or not changed
        lines = set(measured) if broaden else {str(line) for line in changed}
        basis = "file-only" if broaden else "mapped-old-lines"
        found = False
        for number in sorted(lines, key=int):
            hit = measured.get(number)
            if hit is None or not hit[0]:
                continue
            found = True
            location = path, int(number)
            if location in visited:
                continue
            visited.add(location)
            group = grouped_evidence.setdefault(
                (hit[1], basis), {"count": 0, "examples": []}
            )
            group["count"] += 1
            if len(group["examples"]) < 20:
                group["examples"].append(
                    {"path": path, "line": int(number), "basis": basis}
                )
        missing_lines = [
            line
            for line in changed
            if str(line) not in measured or not measured[str(line)][0]
        ]
        if (
            change["added_lines"]
            or not found
            or missing_lines
            or change["binary"]
        ):
            unresolved.append(
                {
                    "path": change["new_path"] or path,
                    "added_lines_without_prior_coordinates": change[
                        "added_lines"
                    ],
                    "old_lines_without_covering_tests": sorted(missing_lines),
                    "no_observed_covering_suite": not found,
                    "binary": change["binary"],
                }
            )
    for (membership, basis), group in grouped_evidence.items():
        for uid in covering_suites(index, membership):
            item = matches.setdefault(
                uid, {"test_uid": uid, "matching_lines": 0, "evidence": []}
            )
            item["matching_lines"] += group["count"]
            item["evidence"].extend(
                group["examples"][: 20 - len(item["evidence"])]
            )
    suggestions = sorted(
        matches.values(),
        key=lambda item: (-item["matching_lines"], item["test_uid"]),
    )
    return {
        "tested_revision": tested,
        "target_revision": target_revision,
        "freshness": (
            "same-revision"
            if target_revision == tested
            else (
                "different-revision"
                if target_revision
                else "target-not-verified"
            )
        ),
        "coordinate_basis": (
            "validated old-side unified diff"
            if patch is not None
            else "file-level observations"
        ),
        "suggested_tests": suggestions,
        "unresolved_changes": unresolved,
        "evidence_examples_per_test_limit": 20,
        "collection": collection_limits(index, language),
        "advisory_only": True,
        "warning": "These are prior observations, not a safe exclusion list. "
        "New code, absent profiles and dependencies may require additional tests; never automatically skip CI.",
    }


def repository_diff(repository, base, target):
    """Resolve both commits and obtain a diff without invoking external drivers."""

    def git(*arguments):
        return subprocess.check_output(
            ["git", "-C", str(repository), *arguments], text=True
        )

    base_sha = git("rev-parse", "--verify", base + "^{commit}").strip()
    target_sha = git("rev-parse", "--verify", target + "^{commit}").strip()
    patch = git(
        "-c",
        "core.quotePath=false",
        "diff",
        "--no-ext-diff",
        "--no-textconv",
        "--no-color",
        "--unified=0",
        base_sha,
        target_sha,
        "--",
    )
    return base_sha, target_sha, patch


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    suggest = commands.add_parser(
        "suggest",
        help="Suggest observed covering suites; never skip CI automatically",
    )
    suggest.add_argument("index", type=Path)
    suggest.add_argument("--file", action="append", default=[])
    suggest.add_argument("--diff", type=Path)
    suggest.add_argument("--base-revision")
    suggest.add_argument("--target-revision")
    suggest.add_argument("--repository", type=Path)
    suggest.add_argument(
        "--language", choices=("native", "python"), default="native"
    )
    args = parser.parse_args(argv)
    try:
        index = load_index(args.index)
        patch = args.diff.read_text() if args.diff else None
        base, target = args.base_revision, args.target_revision
        if args.repository:
            if not target or args.diff:
                raise ValueError(
                    "Repository mapping needs --target-revision and cannot combine --diff"
                )
            base, target, patch = repository_diff(
                args.repository, index["revision"], target
            )
        if not args.file and patch is None:
            raise ValueError(
                "Provide --file, --diff, or --repository with --target-revision"
            )
        result = suggest_tests(
            index, args.file, patch, base, target, args.language
        )
        if patch is not None:
            result["diff_provenance"] = (
                "verified repository commits"
                if args.repository
                else "caller-supplied patch and revision labels"
            )
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        subprocess.CalledProcessError,
    ) as error:
        print(f"coverage analysis: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
