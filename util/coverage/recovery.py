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

"""Retry native extraction from retained files without executing tests."""

import importlib.util
import json
import os
import shutil
import subprocess
import tarfile
import tempfile
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import (
    Path,
    PurePosixPath,
)

from schema import (
    load_record,
    normalize_record,
    profile_paths,
    source_path,
)

MAX_ARCHIVE_BYTES = 100 * 1024**3


def extract_archive(archive_path, output, max_bytes=MAX_ARCHIVE_BYTES):
    """Restore regular files and internal hard links; never follow symlinks."""
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    total, entries = 0, set()
    with tarfile.open(archive_path, "r:gz") as archive:
        for member in archive:
            name = member.name.removeprefix("./").rstrip("/")
            source_path(name)
            if name in entries:
                raise ValueError("Duplicate archive member")
            entries.add(name)
            if len(entries) > 1_000_000:
                raise ValueError("Archive exceeds extraction entry limit")
            target = output / name
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            if not (member.isfile() or member.islnk()):
                raise ValueError("Archive contains a link or special file")
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.exists():
                raise ValueError("Archive member would overwrite a file")
            if member.islnk():
                link = member.linkname.removeprefix("./")
                source_path(link)
                origin = output / link
                if link not in entries or not origin.is_file():
                    raise ValueError(
                        "Archive hard link target was not restored"
                    )
                os.link(origin, target)
            else:
                total += member.size
                if total > max_bytes or member.size < 0:
                    raise ValueError("Archive exceeds extraction byte limit")
                with (
                    archive.extractfile(member) as src,
                    target.open("xb") as dst,
                ):
                    shutil.copyfileobj(src, dst)
    return output


def collector_module():
    path = Path(__file__).resolve().parents[2] / "ext/testlib/coverage.py"
    spec = importlib.util.spec_from_file_location(
        "gem5_coverage_decoder", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def gcov_version(gcov):
    return subprocess.check_output(
        [gcov, "--version"], text=True
    ).splitlines()[0]


def decode(module, notes, compiled_root, gcov):
    units = {}
    compiled_root = Path(compiled_root)
    if not compiled_root.is_absolute():
        raise ValueError("Coverage build root must be absolute")
    for note, relative in notes:
        source_path(relative)
        units[str(note)] = relative
    with tempfile.TemporaryDirectory() as directory:
        return module._read_profiles(
            [note for note, _ in notes],
            compiled_root,
            gcov,
            Path(directory),
            units,
        )


def recover_profile(
    module, raw_path, output_path, revision, gcov, version, root
):
    load_record(output_path, root=root)
    original = json.loads(output_path.read_text())
    raw = json.loads(raw_path.read_text())
    if original != raw or raw["revision"] != revision:
        raise ValueError("Raw invocation and retained record differ")
    if raw.get("language", "native") != "native":
        raise ValueError(
            "Python profiles require their separate tracer output"
        )
    build = raw["build"]
    if build.get("gcov_version") != version:
        raise ValueError(
            "Original gcov version is required for extraction retry"
        )
    compiled_root = Path(build["build_root"])
    object_root = raw_path.parent / "raw" / Path(*compiled_root.parts[1:])
    counters = sorted(object_root.rglob("*.gcda"))
    counters = [p for p in counters if not p.name.endswith(".py.gcda")]
    if not counters:
        raise ValueError(
            "No retained runtime counters; extraction cannot recover an unrecorded run"
        )
    for counter in counters:
        if not counter.with_suffix(".gcno").is_file():
            raise ValueError("Missing matching coverage notes")
    notes = [
        (p, p.with_suffix(".gcno").relative_to(object_root).as_posix())
        for p in counters
    ]
    files, versions = decode(module, notes, compiled_root, gcov)
    if versions != build.get("gcc_versions"):
        raise ValueError(
            "Recovered compiler version differs from recorded compiler"
        )
    recovered = dict(raw, collection="complete")
    recovered.pop("error", None)
    recovered["files"] = []
    for path, entry in sorted(files.items()):
        lines = {
            line: count for line, count in entry["lines"].items() if count > 0
        }
        branches = [
            {
                "id": module._branch_id(build["build_id"], branch),
                "count": branch["count"],
            }
            for branch in entry["branches"]
            if branch["count"] > 0
        ]
        if lines or branches:
            recovered["files"].append(
                {"path": path, "lines": lines, "branches": branches}
            )
    if recovered.get("schema_version") != 2 or not recovered.get(
        "baseline_id"
    ):
        raise ValueError(
            "Invocation extraction retry requires a retained schema 2 baseline"
        )
    baseline_path = raw_path.parent
    while not (
        baseline_path
        / "baselines"
        / recovered["baseline_id"]
        / "baseline.json"
    ).is_file():
        if (
            baseline_path.name == "coverage"
            or baseline_path == baseline_path.parent
        ):
            raise ValueError("Missing retained baseline")
        baseline_path = baseline_path.parent
    baseline = json.loads(
        (
            baseline_path
            / "baselines"
            / recovered["baseline_id"]
            / "baseline.json"
        ).read_text()
    )
    normalize_record(recovered, baseline=baseline)
    output_path.write_text(json.dumps(recovered, sort_keys=True) + "\n")


def aggregate_xml(files, output, timestamp):
    coverage = ET.Element(
        "coverage",
        timestamp=str(timestamp),
        version="gem5-reextraction",
    )
    package = ET.SubElement(
        ET.SubElement(coverage, "packages"), "package", name="native"
    )
    classes = ET.SubElement(package, "classes")
    total = covered = branch_total = branch_covered = 0
    for path, entry in sorted(files.items()):
        source_path(path)
        lines = ET.SubElement(
            ET.SubElement(classes, "class", name=path, filename=path), "lines"
        )
        for number, count in sorted(
            entry["lines"].items(), key=lambda item: int(item[0])
        ):
            line = ET.SubElement(lines, "line", number=number, hits=str(count))
            total += 1
            covered += count > 0
            branches = [
                branch
                for branch in entry.get("branches", [])
                if branch["line"] == int(number)
            ]
            if branches:
                hits = sum(branch["count"] > 0 for branch in branches)
                branch_total += len(branches)
                branch_covered += hits
                line.set("branch", "true")
                line.set(
                    "condition-coverage",
                    f"{100 * hits // len(branches)}% ({hits}/{len(branches)})",
                )
    coverage.set("lines-valid", str(total))
    coverage.set("lines-covered", str(covered))
    coverage.set("line-rate", str(covered / total if total else 0))
    coverage.set("branches-valid", str(branch_total))
    coverage.set("branches-covered", str(branch_covered))
    coverage.set(
        "branch-rate",
        str(branch_covered / branch_total if branch_total else 0),
    )
    ET.ElementTree(coverage).write(
        output, encoding="utf-8", xml_declaration=True
    )


def reextract(
    source, output, revision, gcov="gcov", max_bytes=MAX_ARCHIVE_BYTES
):
    source, output = Path(source).resolve(), Path(output).resolve()
    if (
        output == source
        or output.is_relative_to(source)
        or source.is_relative_to(output)
    ):
        raise ValueError("Extraction retry needs a separate output directory")
    if output.exists():
        raise ValueError("Extraction retry output must not already exist")
    if max_bytes < 1:
        raise ValueError("Extraction byte limit must be positive")
    if any(path.is_symlink() for path in source.rglob("*")):
        raise ValueError("Retained input must not contain symbolic links")
    shutil.copytree(source, output, ignore=shutil.ignore_patterns("*.tar.gz"))
    result = {
        "schema_version": 1,
        "revision": revision,
        "operation": "native-extraction-retry",
        "recovered_invocations": 0,
        "recovered_aggregates": 0,
        "errors": [],
        "scope": "Native schema 2 invocations and aggregate groups only; Python tracer output is retained unchanged.",
    }
    try:
        module, version = collector_module(), gcov_version(gcov)
    except (OSError, subprocess.CalledProcessError) as error:
        result["errors"].append(f"Cannot run original gcov tool: {error}")
        (output / "reextraction.json").write_text(
            json.dumps(result, indent=2) + "\n"
        )
        return result
    for archive in sorted(source.rglob("*.tar.gz")):
        if archive.name not in {"raw-profiles.tar.gz", "raw-gcov.tar.gz"}:
            continue
        destination = output / archive.parent.relative_to(source)
        try:
            with tempfile.TemporaryDirectory() as directory:
                restored = extract_archive(archive, directory, max_bytes)
                if archive.name == "raw-profiles.tar.gz":
                    for record in sorted(
                        (restored / "coverage").rglob("coverage.json")
                    ):
                        target = (
                            destination
                            / "records"
                            / record.relative_to(restored / "coverage")
                        )
                        try:
                            recover_profile(
                                module,
                                record,
                                target,
                                revision,
                                gcov,
                                version,
                                destination / "records",
                            )
                            result["recovered_invocations"] += 1
                        except (
                            ValueError,
                            KeyError,
                            OSError,
                            RuntimeError,
                        ) as error:
                            result["errors"].append(
                                f"{record.parent.name}: {error}"
                            )
                else:
                    metadata = json.loads(
                        (destination / "aggregate.json").read_text()
                    )
                    if (
                        metadata["revision"] != revision
                        or metadata.get("gcov_version") != version
                    ):
                        raise ValueError(
                            "Aggregate revision or gcov version mismatch"
                        )
                    compiled_root = metadata["build_root"]
                    notes = sorted(restored.rglob("*.gcno"))
                    notes = [
                        p for p in notes if not p.name.endswith(".py.gcno")
                    ]
                    if not notes or not list(restored.rglob("*.gcda")):
                        raise ValueError(
                            "Missing native notes or runtime counters"
                        )
                    files, _ = decode(
                        module,
                        [
                            (p, p.relative_to(restored).as_posix())
                            for p in notes
                        ],
                        compiled_root,
                        gcov,
                    )
                    report = destination / "coverage.xml"
                    timestamp = None
                    if report.is_file():
                        try:
                            timestamp = (
                                ET.parse(report).getroot().get("timestamp")
                            )
                        except ET.ParseError:
                            pass
                    if timestamp is None:
                        collected = datetime.fromisoformat(
                            metadata["collected_at"]
                        )
                        if collected.tzinfo is None:
                            raise ValueError(
                                "Original collection timestamp requires timezone"
                            )
                        timestamp = str(int(collected.timestamp()))
                    aggregate_xml(files, report, timestamp)
                    metadata["extraction"] = "complete"
                    (destination / "aggregate.json").write_text(
                        json.dumps(metadata, sort_keys=True) + "\n"
                    )
                    result["recovered_aggregates"] += 1
        except (
            ValueError,
            KeyError,
            OSError,
            RuntimeError,
            tarfile.TarError,
        ) as error:
            result["errors"].append(f"{archive.parent.name}: {error}")
    if (
        not result["recovered_invocations"]
        and not result["recovered_aggregates"]
    ):
        result["errors"].append("No native profiles could be re-extracted")
    (output / "reextraction.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n"
    )
    return result
