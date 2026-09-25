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
#

"""Isolated GCC coverage for one TestLib gem5 invocation.

This module deliberately has no TestLib imports so the collector can also be
validated with small native programs, without building the simulator.
"""

import gzip
import hashlib
import json
import os
import shutil
import subprocess
import tempfile
import threading
import uuid
from pathlib import Path


def _canonical_bytes(value):
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")


def _digest_json(value):
    return hashlib.sha256(_canonical_bytes(value)).hexdigest()


def _file_digest(path):
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _write_record(path, record):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(record, sort_keys=True) + "\n")
    temporary.replace(path)


def _link_metadata(source, destination):
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)


def _read_profiles(inputs, source_root, gcov, output_dir, units):
    """Read GCC JSON, including zero counts from notes without data files."""
    files = {}
    versions = set()
    # Preserve paths to distinguish translation units with identical basenames.
    for start in range(0, len(inputs), 128):
        command = [
            gcov,
            "--json-format",
            "--branch-counts",
            "--branch-probabilities",
            "--preserve-paths",
            "--hash-filenames",
        ]
        command.extend(str(path) for path in inputs[start : start + 128])
        result = subprocess.run(
            command, cwd=output_dir, capture_output=True, text=True
        )
        if result.returncode:
            raise RuntimeError(result.stderr or result.stdout)
        reports = list(output_dir.glob("*.gcov.json.gz"))
        if not reports:
            raise RuntimeError("gcov did not produce JSON coverage")
        for report in reports:
            with gzip.open(report, "rt") as stream:
                data = json.load(stream)
            versions.add(data["gcc_version"])
            working_dir = Path(data["current_working_directory"])
            unit = units[data["data_file"]]
            for source in data["files"]:
                path = Path(source["file"])
                if not path.is_absolute():
                    path = working_dir / path
                try:
                    relative = path.resolve().relative_to(source_root)
                except ValueError:
                    # System headers and other sources outside this checkout.
                    continue
                entry = files.setdefault(
                    relative.as_posix(), {"lines": {}, "branches": []}
                )
                lines = entry["lines"]
                for line in source["lines"]:
                    number, count = line["line_number"], line["count"]
                    if number <= 0:
                        continue
                    if not isinstance(count, int) or count < 0:
                        raise ValueError("gcov returned an invalid line count")
                    key = str(number)
                    lines[key] = lines.get(key, 0) + count
                    for ordinal, branch in enumerate(line.get("branches", [])):
                        hits = branch["count"]
                        if type(hits) is not int or hits < 0:
                            raise ValueError(
                                "gcov returned invalid branch count"
                            )
                        entry["branches"].append(
                            {
                                "line": number,
                                "unit": unit,
                                "function": line.get("function_name", ""),
                                "ordinal": ordinal,
                                "fallthrough": bool(branch["fallthrough"]),
                                "throw": bool(branch["throw"]),
                                "count": hits,
                            }
                        )
            report.unlink()
    for entry in files.values():
        entry["branches"].sort(
            key=lambda b: (b["unit"], b["function"], b["line"], b["ordinal"])
        )
    return files, sorted(versions)


def _branch_id(build_id, branch):
    return _digest_json(
        {
            "build_id": build_id,
            **{
                key: branch[key]
                for key in ("unit", "function", "line", "ordinal")
            },
        }
    )


class CoverageBuild:
    """Snapshot matching notes once, then share immutable notes between runs."""

    def __init__(self, root, target, binary, result_path, gcov="gcov"):
        self.root = Path(root).resolve()
        self.compiled_root = self.root
        self.target = Path(target).resolve()
        self.binary = Path(binary).resolve()
        self.output = Path(result_path).resolve() / "coverage"
        self.gcov = gcov
        self.lock = threading.Lock()
        self.notes = None
        self.units = {}
        self.baseline = {}
        self.baseline_id = None
        self.error = None
        self.manifest = None
        self.build = {
            "build_root": str(self.root),
            "binary": os.path.relpath(self.binary, self.root),
            "target": os.path.relpath(self.target, self.root),
            "gcov_tool": gcov,
        }
        revision = subprocess.run(
            ["git", "-C", str(self.root), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
        )
        self.revision = revision.stdout.strip() or "unknown"

    def _prepare(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            if self.notes is not None:
                return
            try:
                self.output.mkdir(parents=True, exist_ok=True)
                snapshot = Path(
                    tempfile.mkdtemp(prefix=".notes-", dir=self.output)
                )
                manifest_path = self.target / (
                    f"{self.target.name}-{self.binary.name}.json"
                )
                if manifest_path.exists():
                    contents = manifest_path.read_bytes()
                    self.manifest = snapshot / "build.json"
                    self.manifest.write_bytes(contents)
                    manifest = json.loads(contents)
                    if manifest["revision"] != self.revision:
                        raise RuntimeError(
                            "Build and checkout revisions differ"
                        )
                    self.compiled_root = Path(manifest["build_root"]).resolve()
                    self.build.update(
                        build_root=str(self.compiled_root),
                        manifest="build.json",
                        manifest_sha256=hashlib.sha256(contents).hexdigest(),
                        compiler=manifest["compiler"],
                        image=manifest["image"],
                        instrumentation=manifest["instrumentation"],
                    )
                notes = []
                for original in sorted(self.target.rglob("*.gcno")):
                    if original.name.endswith(".py.gcno"):
                        continue
                    # GCOV_PREFIX_STRIP=0 preserves the absolute object path.
                    try:
                        object_path = (
                            self.compiled_root
                            / original.relative_to(self.root)
                        )
                    except ValueError:
                        # External build directories retain their absolute path.
                        object_path = original
                    relative = Path(*object_path.parts[1:])
                    saved = snapshot / relative
                    saved.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(original, saved)
                    notes.append((relative, saved))
                    try:
                        unit = object_path.relative_to(self.compiled_root)
                    except ValueError:
                        unit = Path("external") / original.relative_to(
                            self.target
                        )
                    self.units[relative] = unit.as_posix()
                if not notes:
                    raise RuntimeError("No GCC coverage notes in build target")
                # Keep notes available for retries even if gcov is missing.
                self.notes = notes
                self.build["gcov_version"] = subprocess.check_output(
                    [self.gcov, "--version"], text=True
                ).splitlines()[0]
                with tempfile.TemporaryDirectory() as directory:
                    self.baseline, versions = _read_profiles(
                        [saved for _, saved in notes],
                        self.compiled_root,
                        self.gcov,
                        Path(directory),
                        {
                            str(saved): self.units[relative]
                            for relative, saved in notes
                        },
                    )
                self.build["gcc_versions"] = versions
                files = [
                    {"path": path, **entry}
                    for path, entry in sorted(self.baseline.items())
                ]
                compatibility = {
                    "target": self.build["target"],
                    "variant": self.binary.name,
                    "gcc_versions": versions,
                    "configuration": _file_digest(self.target / ".config"),
                    "instrumentation": "gcc --gcov",
                }
                self.build["compatibility_id"] = _digest_json(compatibility)
                self.build["build_id"] = _digest_json(
                    {
                        "compatibility_id": self.build["compatibility_id"],
                        "files": files,
                        "sources": {
                            path: _file_digest(self.root / path)
                            for path in self.baseline
                        },
                    }
                )
                for entry in files:
                    for branch in entry["branches"]:
                        branch["id"] = _branch_id(
                            self.build["build_id"], branch
                        )
                baseline = {
                    "schema_version": 1,
                    "format": "gem5-coverage-baseline",
                    "revision": self.revision,
                    "language": "native",
                    "build_id": self.build["build_id"],
                    "files": files,
                }
                self.baseline_id = _digest_json(baseline)
                baseline_dir = self.output / "baselines" / self.baseline_id
                baseline_dir.mkdir(parents=True, exist_ok=True)
                baseline_path = baseline_dir / "baseline.json"
                if not baseline_path.exists():
                    # Distinct fixtures may discover an identical baseline.
                    # An atomic replace only publishes complete content.
                    with tempfile.NamedTemporaryFile(
                        dir=baseline_dir, delete=False
                    ) as temporary:
                        temporary.write(_canonical_bytes(baseline))
                    Path(temporary.name).replace(baseline_path)
                # Generated sources need an offline destination: GitHub cannot
                # resolve their build/ paths at the source revision.
                for entry in files:
                    source = self.root / entry["path"]
                    if entry["path"].startswith("build/") and source.is_file():
                        saved = baseline_dir / "sources" / entry["path"]
                        saved.parent.mkdir(parents=True, exist_ok=True)
                        if not saved.exists():
                            shutil.copy2(source, saved)
            except Exception as error:
                self.error = str(error)
                raise

    def invocation(self, test_uid, log):
        return InvocationCoverage(self, test_uid, log)


class InvocationCoverage:
    """Persist a record before execution and never mask a simulator failure."""

    def __init__(self, build, test_uid, log):
        self.build = build
        self.log = log
        self.identifier = str(uuid.uuid4())
        self.directory = build.output / self.identifier
        self.directory.mkdir(parents=True)
        self.path = self.directory / "coverage.json"
        self.raw = self.directory / "raw"
        self.raw.mkdir()
        self.record = {
            "schema_version": 2,
            "language": "native",
            "test_uid": str(test_uid),
            "invocation_id": self.identifier,
            "revision": build.revision,
            "build": dict(build.build),
            "outcome": "interrupted",
            "collection": "missing",
            "files": [],
        }
        _write_record(self.path, self.record)
        self.environment = dict(os.environ)
        self.environment.update(
            GCOV_PREFIX=str(self.raw), GCOV_PREFIX_STRIP="0"
        )

    def __enter__(self):
        try:
            self.build._prepare()
        except Exception as error:
            self.record["collection"] = "error"
            self.record["error"] = str(error)
            self.log.message(f"Coverage preparation failed: {error}")
        try:
            self.record["build"] = dict(self.build.build)
            if self.build.baseline_id is not None:
                self.record["baseline_id"] = self.build.baseline_id
            if self.build.manifest is not None:
                _link_metadata(
                    self.build.manifest, self.directory / "build.json"
                )
            for relative, saved in self.build.notes or []:
                destination = self.raw / relative
                destination.parent.mkdir(parents=True, exist_ok=True)
                _link_metadata(saved, destination)
        except Exception as error:
            self.record["collection"] = "error"
            self.record["error"] = str(error)
            self.log.message(f"Coverage metadata copy failed: {error}")
        _write_record(self.path, self.record)
        return self.environment

    def __exit__(self, exception_type, exception, traceback):
        if exception_type is None:
            self.record["outcome"] = "passed"
        elif issubclass(
            exception_type,
            (KeyboardInterrupt, SystemExit, subprocess.TimeoutExpired),
        ) or (
            isinstance(exception, subprocess.CalledProcessError)
            and exception.returncode < 0
        ):
            self.record["outcome"] = "interrupted"
        else:
            self.record["outcome"] = "failed"
        try:
            if self.record["collection"] != "error":
                files = {}
                counters = sorted(self.raw.rglob("*.gcda"))
                counters = [
                    path
                    for path in counters
                    if not path.name.endswith(".py.gcda")
                ]
                if counters:
                    with tempfile.TemporaryDirectory() as directory:
                        covered, versions = _read_profiles(
                            counters,
                            self.build.compiled_root,
                            self.build.gcov,
                            Path(directory),
                            {
                                str(path): self.build.units[
                                    path.relative_to(self.raw).with_suffix(
                                        ".gcno"
                                    )
                                ]
                                for path in counters
                            },
                        )
                    if versions != self.build.build["gcc_versions"]:
                        raise RuntimeError("Coverage compiler versions differ")
                    for path, entry in covered.items():
                        lines = {
                            line: count
                            for line, count in entry["lines"].items()
                            if count > 0
                        }
                        branches = [
                            {
                                "id": _branch_id(
                                    self.build.build["build_id"], branch
                                ),
                                "count": branch["count"],
                            }
                            for branch in entry["branches"]
                            if branch["count"] > 0
                        ]
                        if lines or branches:
                            files[path] = {
                                "lines": lines,
                                "branches": branches,
                            }
                    self.record["collection"] = "complete"
                self.record["files"] = [
                    {"path": path, **entry}
                    for path, entry in sorted(files.items())
                ]
        except Exception as error:
            self.record["collection"] = "error"
            self.record["error"] = str(error)
            self.log.message(f"Coverage collection failed: {error}")
        finally:
            try:
                _write_record(self.path, self.record)
            except Exception as error:
                self.log.message(f"Cannot save coverage result: {error}")
        return False
