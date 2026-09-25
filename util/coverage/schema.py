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

"""Validate portable coverage records and expand shared sparse baselines.

Consumers use load_record(path, root=input_directory) for stored records;
normalize_record accepts v1 records or v2 records with an explicit baseline.
The canonical in-memory representation is v1 with storage_schema_version=2
for expanded records. Profile files and raw archives remain unchanged.
"""

import hashlib
import json
from collections import OrderedDict
from dataclasses import dataclass
from functools import lru_cache
from pathlib import (
    Path,
    PurePosixPath,
)

COLLECTIONS = ("complete", "missing", "error")
OUTCOMES = ("passed", "failed", "interrupted")
PROFILE_NAMES = ("coverage.json", "python-coverage.json")


def canonical_hash(value):
    data = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=True,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def source_path(value):
    """Require a canonical repository-relative path, without traversal."""
    if (
        not isinstance(value, str)
        or not value
        or "\\" in value
        or any(ord(char) < 32 for char in value)
        or PurePosixPath(value).is_absolute()
        or any(part in ("", ".", "..") for part in value.split("/"))
    ):
        raise ValueError(f"Invalid repository-relative path: {value!r}")
    return value


def line_number(number):
    if (
        not isinstance(number, str)
        or not number.isascii()
        or not number.isdecimal()
        or str(int(number)) != number
        or int(number) < 1
    ):
        raise ValueError(f"Invalid source line: {number!r}")
    return number


def count_value(value):
    if type(value) is not int or value < 0:
        raise ValueError(f"Invalid execution count: {value!r}")
    return value


def _files(entries, language):
    if not isinstance(entries, list):
        raise ValueError("files must be a list")
    files = {}
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError("A file entry must be an object")
        path = source_path(entry.get("path"))
        if path in files:
            raise ValueError(f"Record contains duplicate file: {path}")
        if not isinstance(entry.get("lines"), dict):
            raise ValueError(f"lines must be an object: {path}")
        lines = {
            line_number(line): count_value(count)
            for line, count in entry["lines"].items()
        }
        branches = {}
        if not isinstance(entry.get("branches", []), list):
            raise ValueError("branches must be a list")
        for branch in entry.get("branches", []):
            if not isinstance(branch, dict):
                raise ValueError("A branch must be an object")
            identity = branch.get("id")
            if (
                not isinstance(identity, str)
                or not identity
                or any(ord(char) < 32 for char in identity)
            ):
                raise ValueError("Invalid branch identity")
            if identity in branches:
                raise ValueError("Duplicate branch identity")
            if type(branch.get("line")) is not int or branch["line"] < 1:
                raise ValueError("Invalid branch source line")
            count_value(branch.get("count"))
            if language == "python":
                if (
                    type(branch.get("to_line")) is not int
                    or not branch["to_line"]
                ):
                    raise ValueError("Invalid Python branch destination")
            else:
                if "unit" in branch:
                    source_path(branch["unit"])
                if "ordinal" in branch and (
                    type(branch["ordinal"]) is not int or branch["ordinal"] < 0
                ):
                    raise ValueError("Invalid native branch ordinal")
                for flag in ("fallthrough", "throw"):
                    if flag in branch and type(branch[flag]) is not bool:
                        raise ValueError("Invalid native branch flag")
            branches[identity] = dict(branch)
        files[path] = {
            **entry,
            "lines": dict(sorted(lines.items(), key=lambda x: int(x[0]))),
        }
        if "branches" in entry:
            files[path]["branches"] = [
                branches[key] for key in sorted(branches)
            ]
    return [files[path] for path in sorted(files)]


def baseline_identity(record, baseline):
    """Apply the same revision, language and build checks in both readers."""
    if (
        not isinstance(baseline, dict)
        or type(baseline.get("schema_version")) is not int
        or baseline["schema_version"] != 1
        or baseline.get("format") != "gem5-coverage-baseline"
    ):
        raise ValueError("Invalid coverage baseline format")
    if baseline.get("revision") != record.get("revision") or baseline.get(
        "language"
    ) != record.get("language", "native"):
        raise ValueError("Coverage baseline revision/language mismatch")
    identity = baseline.get("build_id")
    if not isinstance(identity, str) or not identity:
        raise ValueError("Missing coverage baseline build identity")
    build = record.get("build")
    if not isinstance(build, dict) or build.get("build_id") != identity:
        raise ValueError(
            "Coverage baseline build identity mismatch or missing"
        )


def expand_baseline(record, baseline):
    """Validate a baseline and overlay positive counts without inventing lines."""
    baseline_identity(record, baseline)
    if canonical_hash(baseline) != record.get("baseline_id"):
        raise ValueError("Coverage baseline hash mismatch")
    base_files = _files(
        baseline.get("files"), record.get("language", "native")
    )
    files = {entry["path"]: entry for entry in base_files}
    for entry in files.values():
        if any(entry["lines"].values()) or any(
            branch["count"] for branch in entry.get("branches", [])
        ):
            raise ValueError("Coverage baseline must contain zero counters")
    if not isinstance(record.get("files"), list):
        raise ValueError("Sparse files must be a list")
    seen = set()
    for sparse in record["files"]:
        if not isinstance(sparse, dict):
            raise ValueError("A sparse file must be an object")
        path = source_path(sparse.get("path"))
        if path in seen or path not in files:
            raise ValueError("Duplicate or unknown sparse source file")
        seen.add(path)
        lines = sparse.get("lines", {})
        if not isinstance(lines, dict):
            raise ValueError("Sparse lines must be an object")
        for number, count in lines.items():
            line_number(number)
            if number not in files[path]["lines"] or count_value(count) <= 0:
                raise ValueError(
                    "Sparse line must match a baseline and have positive hits"
                )
            files[path]["lines"][number] = count
        branches = {
            item["id"]: item for item in files[path].get("branches", [])
        }
        seen_branches = set()
        if not isinstance(sparse.get("branches", []), list):
            raise ValueError("Sparse branches must be a list")
        for branch in sparse.get("branches", []):
            identity = branch.get("id")
            if identity not in branches or identity in seen_branches:
                raise ValueError("Unknown or duplicate sparse branch")
            seen_branches.add(identity)
            if count_value(branch.get("count")) <= 0:
                raise ValueError("Sparse branches must have positive hits")
            branches[identity]["count"] = branch["count"]
    return [files[path] for path in sorted(files)]


def _normalize_record(record, *, baseline=None):
    """Return one canonical record, preserving v1 metadata and empty profiles."""
    if not isinstance(record, dict):
        raise ValueError("A coverage record must be an object")
    version = record.get("schema_version")
    if type(version) is not int or version not in (1, 2):
        raise ValueError("Unsupported coverage record schema_version")
    for name in ("test_uid", "invocation_id", "revision"):
        if not isinstance(record.get(name), str) or not record[name]:
            raise ValueError(f"Missing or invalid {name}")
    if not record["test_uid"].startswith("SuiteUID:"):
        raise ValueError("test_uid must identify a TestLib SuiteUID")
    if not isinstance(record.get("build"), dict):
        raise ValueError("build must be an object")
    if record.get("outcome") not in OUTCOMES:
        raise ValueError("Invalid invocation outcome")
    if record.get("collection") not in COLLECTIONS:
        raise ValueError("Invalid collection status")
    language = record.get("language", "native")
    if language not in ("native", "python"):
        raise ValueError("Unknown coverage language")
    result = {**record, "language": language}
    if version == 2:
        if baseline is None:
            if record["collection"] == "complete" or record.get("files"):
                raise ValueError("Sparse coverage needs its matching baseline")
            files = []
        else:
            files = expand_baseline(record, baseline)
        result.update(schema_version=1, storage_schema_version=2)
    else:
        files = record.get("files")
    result["files"] = _files(files, language)
    return result


def normalize_record(record, *, baseline=None):
    try:
        return _normalize_record(record, baseline=baseline)
    except (TypeError, KeyError, AttributeError) as error:
        raise ValueError(f"Invalid coverage record: {error}") from error


def profile_paths(directory):
    root = Path(directory).resolve()
    return sorted(path for name in PROFILE_NAMES for path in root.rglob(name))


@lru_cache(maxsize=8)
def _baseline_file(path, size, modified):
    # File identity invalidates the cache on mutation; paths come from the
    # confined resolver below. Consumers never modify this shared object.
    baseline = json.loads(Path(path).read_text(encoding="utf-8"))
    identity = canonical_hash(baseline)
    return baseline, identity


def _stored_record(path, root):
    path = Path(path).resolve()
    root = Path(root).resolve() if root is not None else path.parent
    if not path.is_relative_to(root):
        raise ValueError("Profile escapes input directory")
    record = json.loads(path.read_text(encoding="utf-8"))
    baseline = None
    if (
        isinstance(record, dict)
        and record.get("schema_version") == 2
        and record.get("baseline_id")
    ):
        identity = record["baseline_id"]
        if (
            not isinstance(identity, str)
            or len(identity) != 64
            or any(char not in "0123456789abcdef" for char in identity)
        ):
            raise ValueError("Invalid baseline identity")
        parent = path.parent
        while parent.is_relative_to(root):
            candidate = parent / "baselines" / identity / "baseline.json"
            if candidate.is_file():
                if not candidate.resolve().is_relative_to(root):
                    raise ValueError("Baseline escapes input directory")
                stat = candidate.stat()
                baseline, checksum = _baseline_file(
                    str(candidate.resolve()), stat.st_size, stat.st_mtime_ns
                )
                if checksum != identity:
                    raise ValueError("Coverage baseline hash mismatch")
                break
            if parent == root:
                break
            parent = parent.parent
        if baseline is None:
            raise ValueError("Matching coverage baseline not found")
    return record, baseline


def load_record(path, root=None):
    """Load an expanded profile, resolving baselines only inside input root."""
    record, baseline = _stored_record(path, root)
    return normalize_record(record, baseline=baseline)


@dataclass(frozen=True)
class SparseProfile:
    record: dict
    baseline: dict


_CATALOGS = OrderedDict()


def _baseline_catalog(identity, baseline):
    if identity in _CATALOGS:
        _CATALOGS.move_to_end(identity)
        return _CATALOGS[identity]
    if (
        baseline.get("format") != "gem5-coverage-baseline"
        or baseline.get("schema_version") != 1
    ):
        raise ValueError("Invalid coverage baseline format")
    files = _files(baseline.get("files"), baseline.get("language"))
    for entry in files:
        if any(entry["lines"].values()) or any(
            branch["count"] for branch in entry.get("branches", [])
        ):
            raise ValueError("Coverage baseline must contain zero counters")
    canonical = {**baseline, "files": files}
    result = canonical, {entry["path"]: entry for entry in files}
    _CATALOGS[identity] = result
    if len(_CATALOGS) > 8:
        _CATALOGS.popitem(last=False)
    return result


def load_sparse_record(path, root=None):
    """Validate deltas while retaining a shared immutable zero-count baseline.

    Returns SparseProfile for v2 storage and an ordinary normalized dict for
    v1. This avoids expanding the executable-line inventory per invocation.
    """
    record, baseline = _stored_record(path, root)
    if baseline is None:
        return normalize_record(record)
    # Cache normalized catalogs by validated content identity.
    canonical, catalog = _baseline_catalog(record["baseline_id"], baseline)
    baseline_identity(record, canonical)
    sparse_files = []
    seen = set()
    if not isinstance(record.get("files"), list):
        raise ValueError("Sparse files must be a list")
    for entry in record["files"]:
        path = source_path(entry.get("path"))
        if path in seen or path not in catalog:
            raise ValueError("Duplicate or unknown sparse source file")
        seen.add(path)
        base = catalog[path]
        lines = entry.get("lines", {})
        for line, count in lines.items():
            line_number(line)
            if line not in base["lines"] or count_value(count) <= 0:
                raise ValueError(
                    "Sparse line must match a baseline and have positive hits"
                )
        branches = []
        branch_catalog = {
            branch["id"]: branch for branch in base.get("branches", [])
        }
        for branch in entry.get("branches", []):
            identity = branch.get("id")
            if (
                identity not in branch_catalog
                or count_value(branch.get("count")) <= 0
            ):
                raise ValueError("Unknown sparse branch or invalid count")
            branches.append(
                {**branch_catalog[identity], "count": branch["count"]}
            )
        sparse_files.append(
            {"path": path, "lines": lines, "branches": branches}
        )
    normalized = normalize_record(
        {
            **record,
            "schema_version": 1,
            "storage_schema_version": 2,
            "files": sparse_files,
        }
    )
    return SparseProfile(normalized, canonical)


def read_index_records(directory):
    for path in profile_paths(directory):
        try:
            yield load_sparse_record(path, root=directory)
        except (
            OSError,
            ValueError,
            TypeError,
            KeyError,
            AttributeError,
        ) as error:
            raise ValueError(f"{path}: {error}") from error


def iter_records(directory):
    for path in profile_paths(directory):
        try:
            yield path, load_record(path, root=directory)
        except (
            OSError,
            ValueError,
            TypeError,
            KeyError,
            AttributeError,
        ) as error:
            raise ValueError(f"{path}: {error}") from error


def read_records(directory):
    for path, record in iter_records(directory):
        yield record
