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

"""Build a global line index with bounded, compressed branch graph shards."""

import base64
import gzip
import hashlib
import json
import re
from collections import defaultdict
from pathlib import Path

try:
    from . import index
    from .schema import (
        clear_baseline_cache,
        load_sparse_record,
        profile_paths,
    )
except ImportError:
    import index
    from schema import (
        clear_baseline_cache,
        load_sparse_record,
        profile_paths,
    )


def _groups(directory):
    groups = defaultdict(list)
    owners = {}
    for path in profile_paths(directory):
        with path.open(encoding="utf-8") as stream:
            record = json.load(stream)
        if not isinstance(record, dict) or not isinstance(
            record.get("build"), dict
        ):
            raise ValueError("Invalid profile metadata")
        identity = record.get("invocation_id")
        language = record.get("language", "native")
        build = record["build"].get("build_id", "")
        if not isinstance(identity, str) or not isinstance(build, str):
            raise ValueError("Invalid invocation/build identity")
        if language not in ("native", "python"):
            raise ValueError("Invalid coverage language")
        group = (language, build if language == "native" else "")
        if identity in owners and owners[identity] != group:
            raise ValueError("Conflicting invocation across build graphs")
        owners[identity] = group
        groups[group].append((record.get("baseline_id", ""), path))
    return {
        group: [path for _, path in sorted(paths)]
        for group, paths in sorted(groups.items())
    }, sorted(owners)


def _write_shards(graph, output, core, chunk_size, graph_id):
    for language, files in graph["branches"].items():
        for path, branches in files.items():
            pages = (
                core["branch_pages"]
                .setdefault(language, {})
                .setdefault(path, {})
            )
            descriptors = (
                core["branch_shards"]
                .setdefault(language, {})
                .setdefault(path, [])
            )
            identities = sorted(branches)
            for start in range(0, len(identities), chunk_size):
                selected = identities[start : start + chunk_size]
                chunk = {identity: branches[identity] for identity in selected}
                packed = gzip.compress(
                    json.dumps(
                        chunk, ensure_ascii=True, separators=(",", ":")
                    ).encode(),
                    compresslevel=6,
                    mtime=0,
                )
                digest = hashlib.sha256(packed).hexdigest()
                filename = f"branches/{digest}-0.js"
                # The same gzip payload is parsed by CLI and decoded by the
                # browser. There is no second uncompressed branch artifact.
                payload = base64.b64encode(packed).decode("ascii")
                (output / filename).write_text(
                    "globalThis.gem5CoverageBranchData["
                    + json.dumps(filename)
                    + "]="
                    + json.dumps(payload)
                    + ";\n",
                    encoding="utf-8",
                )
                descriptors.append(
                    {
                        "file": filename,
                        "first": selected[0],
                        "last": selected[-1],
                        "build_id": graph_id,
                    }
                )
                for line in {
                    str(branch["metadata"]["line"])
                    for branch in chunk.values()
                }:
                    pages.setdefault(line, []).append(filename)


def read_branch_shard(root, descriptor):
    """Read a confined data envelope; never execute downloaded JavaScript."""
    filename = descriptor["file"]
    match = re.fullmatch(r"branches/([a-f0-9]{64})-0\.js", filename)
    if not match:
        raise ValueError("Invalid branch shard path")
    root = Path(root).resolve()
    path = (root / filename).resolve()
    if not path.is_relative_to(root):
        raise ValueError("Branch shard escapes index directory")
    content = path.read_text(encoding="utf-8")
    prefix = "globalThis.gem5CoverageBranchData[" + json.dumps(filename) + "]="
    if not content.startswith(prefix) or not content.endswith(";\n"):
        raise ValueError("Invalid branch shard data envelope")
    encoded = json.loads(content[len(prefix) : -2])
    packed = base64.b64decode(encoded, validate=True)
    if hashlib.sha256(packed).hexdigest() != match[1]:
        raise ValueError("Branch shard checksum mismatch")
    try:
        payload = json.loads(gzip.decompress(packed))
    except EOFError as error:
        raise ValueError("Truncated compressed branch shard") from error
    if not isinstance(payload, dict):
        raise ValueError("Invalid branch shard payload")
    return payload


def build_artifact(
    directory, output, repository_url=index.REPOSITORY_URL, chunk_size=4096
):
    """Accumulate global line data while retaining only one branch graph."""
    if type(chunk_size) is not int or chunk_size < 1:
        raise ValueError("Branch chunk size must be positive")
    directory, output = Path(directory), Path(output)
    (output / "branches").mkdir(parents=True, exist_ok=True)
    groups, identities = _groups(directory)
    if not groups:
        raise ValueError("No coverage.json invocation records found")
    ordinals = {
        identity: ordinal for ordinal, identity in enumerate(identities)
    }
    core = {
        "schema_version": 2,
        "format": "gem5-coverage-index",
        "revision": None,
        "repository_url": repository_url.rstrip("/"),
        "invocations": [None] * len(identities),
        "tests": {},
        "files": {},
        "python_files": {},
        "branches": {},
        "memberships": [],
        "branch_pages": {},
        "branch_shards": {},
    }
    members = {}
    totals = {
        language: {"measured_branches": 0, "covered_branches": 0}
        for language in ("native", "python")
    }

    def intern(values):
        key = tuple(sorted(values))
        if key not in members:
            members[key] = len(core["memberships"])
            core["memberships"].append(list(key))
        return members[key]

    for (_, graph_id), paths in groups.items():
        clear_baseline_cache()
        try:
            graph = index.build_index(
                (load_sparse_record(path, root=directory) for path in paths),
                repository_url,
            )
            if (
                core["revision"] is not None
                and core["revision"] != graph["revision"]
            ):
                raise ValueError(
                    "Cannot combine records from different source revisions"
                )
            core["revision"] = graph["revision"]
            remap = [
                ordinals[item["invocation_id"]]
                for item in graph["invocations"]
            ]
            for local, item in enumerate(graph["invocations"]):
                core["invocations"][remap[local]] = item
            mapped = [
                intern(remap[ordinal] for ordinal in group)
                for group in graph["memberships"]
            ]
            masks = [
                sum(
                    1 << ordinal for ordinal in core["memberships"][membership]
                )
                for membership in mapped
            ]
            for key in ("files", "python_files"):
                for path, lines in graph[key].items():
                    target = core[key].setdefault(path, {})
                    for line, (count, membership) in lines.items():
                        hit = target.setdefault(line, [0, 0])
                        hit[0] += count
                        hit[1] |= masks[membership]
            for language, files in graph["branches"].items():
                for branches in files.values():
                    for branch in branches.values():
                        branch["membership"] = mapped[branch["membership"]]
                for key in totals[language]:
                    totals[language][key] += graph["summary"]["languages"][
                        language
                    ][key]
            _write_shards(graph, output, core, chunk_size, graph_id)
            files = branches = branch = None
            # Drop wrappers, source metadata and branch dictionaries before
            # loading the next graph. Only small descriptors remain in core.
            del graph
        finally:
            clear_baseline_cache()
    for key in ("files", "python_files"):
        for lines in core[key].values():
            for hit in lines.values():
                mask = hit[1]
                values = []
                while mask:
                    bit = mask & -mask
                    values.append(bit.bit_length() - 1)
                    mask ^= bit
                hit[1] = intern(values)
    for ordinal, item in enumerate(core["invocations"]):
        core["tests"].setdefault(item["test_uid"], []).append(ordinal)
    languages = {}
    for language, key in (("native", "files"), ("python", "python_files")):
        languages[language] = {
            **totals[language],
            "measured_lines": sum(len(lines) for lines in core[key].values()),
            "covered_lines": sum(
                hit[0] > 0
                for lines in core[key].values()
                for hit in lines.values()
            ),
        }
    core["summary"] = {
        "invocations": len(identities),
        "tests": len(core["tests"]),
        "collection": {
            status: sum(
                item["collection"] == status for item in core["invocations"]
            )
            for status in ("complete", "missing", "error")
        },
        "outcomes": {
            status: sum(
                item["outcome"] == status for item in core["invocations"]
            )
            for status in ("passed", "failed", "interrupted")
        },
        "languages": languages,
        "measured_lines": languages["native"]["measured_lines"],
        "covered_lines": languages["native"]["covered_lines"],
    }
    return core
