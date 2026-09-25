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

"""Account for TestLib coverage and export recoverable, grouped LCOV reports."""

import argparse
import json
import re
import shutil
import subprocess
import sys
import tarfile
import xml.etree.ElementTree as ET
from collections import defaultdict
from datetime import (
    datetime,
    timezone,
)
from pathlib import (
    Path,
    PurePosixPath,
)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def suite_directory(uid):
    # TestLib UIDs retain the repository's tests/ prefix; CLI directory
    # selection and existing Codecov flags are relative to that directory.
    path = PurePosixPath(uid.split(":")[1])
    if path.parts and path.parts[0] == "tests":
        path = PurePosixPath(*path.parts[1:])
    return str(path.parent)


def manifest(listing, revision, length, output):
    tests = sorted(
        {
            line.strip()
            for line in Path(listing).read_text().splitlines()
            if line.startswith("SuiteUID:")
        }
    )
    if not tests:
        raise ValueError("TestLib selection produced no suites")
    excluded = {
        uid: "Known gcov instrumentation crash"
        for uid in tests
        if length == "very-long"
        and suite_directory(uid) == "gem5/x86_boot_tests"
    }
    write_json(
        Path(output),
        {
            "schema_version": 1,
            "revision": revision,
            "length": length,
            "expected": tests,
            "excluded": excluded,
        },
    )


def native_groups(path):
    data = json.loads(Path(path).read_text())
    if data.get("schema_version") != 1:
        raise ValueError("Unsupported native plan schema")
    result = {}
    for item in data["groups"]:
        group = item["group"]
        if not re.fullmatch(r"[a-zA-Z0-9_.-]+", group):
            raise ValueError("Invalid native group")
        if group in result or item["length"] not in {
            "quick",
            "long",
            "very-long",
        }:
            raise ValueError("Duplicate group or invalid native length")
        for field in ("flags", "stages"):
            values = item[field]
            if (
                not isinstance(values, list)
                or not values
                or len(set(values)) != len(values)
                or any(
                    not isinstance(v, str)
                    or not re.fullmatch(r"[a-zA-Z0-9_.-]+", v)
                    for v in values
                )
            ):
                raise ValueError(f"Invalid native {field}")
        result[group] = item
    if not result:
        raise ValueError("Empty native plan")
    return result


def native_plan(config, revision, output):
    write_json(
        Path(output),
        {
            "schema_version": 1,
            "format": "gem5-native-coverage-plan",
            "revision": revision,
            "groups": list(native_groups(config).values()),
        },
    )


def safe_path(value):
    path = PurePosixPath(value)
    if (
        not value
        or path.is_absolute()
        or ".." in path.parts
        or "\\" in value
        or any(ord(c) < 32 for c in value)
    ):
        raise ValueError(f"Unsafe source path: {value!r}")
    return value


def xml_statuses(root, errors):
    results = defaultdict(set)
    for path in root.rglob("results.xml"):
        try:
            tree = ET.parse(path)
        except ET.ParseError as error:
            errors.append(f"Invalid test result XML: {error}")
            continue
        for case in tree.iter("testcase"):
            uid = case.get("classname", "").split(":")
            if len(uid) >= 4 and uid[0] == "TestUID":
                suite = ":".join(["SuiteUID"] + uid[1:-1])
                results[suite].add(case.get("status", "Unknown"))
    return results


def summarize(source, output, revision, campaign=False):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    errors, expected, excluded, lengths = [], {}, {}, set()
    for path in source.rglob("expected.json"):
        try:
            data = json.loads(path.read_text())
            if (
                data.get("schema_version") != 1
                or data.get("revision") != revision
            ):
                raise ValueError("Manifest schema or revision mismatch")
            length = data["length"]
            if length not in {"quick", "long", "very-long"}:
                raise ValueError("Invalid manifest length")
            tests = data["expected"]
            if not isinstance(tests, list) or not tests:
                raise ValueError("Empty or invalid expected suites")
            for uid in tests:
                if not isinstance(uid, str) or not uid.startswith("SuiteUID:"):
                    raise ValueError("Invalid expected suite UID")
                parts = uid.split(":", 2)
                if len(parts) != 3 or not parts[2]:
                    raise ValueError("Invalid expected suite UID")
                safe_path(parts[1])
            omissions = data.get("excluded", {})
            if not isinstance(omissions, dict) or any(
                uid not in tests
                or length != "very-long"
                or suite_directory(uid) != "gem5/x86_boot_tests"
                for uid in omissions
            ):
                raise ValueError("Invalid coverage exclusion")
            lengths.add(length)
            for uid in tests:
                if uid in expected and expected[uid] != length:
                    raise ValueError("Suite appears in conflicting lengths")
                expected[uid] = length
            excluded.update(omissions)
        except (ValueError, KeyError, TypeError, AttributeError) as error:
            errors.append(f"Invalid discovery manifest: {error}")
    if not expected:
        errors.append("No expected TestLib suites were recovered")
    if campaign and lengths != {"quick", "long", "very-long"}:
        errors.append(
            "Missing discovery manifests for: "
            + ", ".join(sorted({"quick", "long", "very-long"} - lengths))
        )
    records, seen = defaultdict(list), set()
    groups = defaultdict(list)
    for path in source.rglob("coverage.json"):
        try:
            record = json.loads(path.read_text())
            if record.get("schema_version") != 1:
                raise ValueError("Unsupported profile schema")
            if record.get("revision") != revision:
                raise ValueError("Profile revision mismatch")
            uid, identity = record["test_uid"], record["invocation_id"]
            if (
                not isinstance(uid, str)
                or not isinstance(identity, str)
                or not identity
            ):
                raise ValueError("Invalid invocation identity")
            if identity in seen:
                raise ValueError("Duplicate invocation identity")
            if uid not in expected:
                raise ValueError("Profile has no expected suite")
            if record.get("collection") not in {
                "complete",
                "missing",
                "error",
            }:
                raise ValueError("Unknown collection status")
            if record.get("outcome") not in {
                "passed",
                "failed",
                "interrupted",
            }:
                raise ValueError("Unknown execution outcome")
            line_count = 0
            for entry in record["files"]:
                safe_path(entry["path"])
                for line, count in entry["lines"].items():
                    if not str(line).isdigit() or int(line) < 1:
                        raise ValueError("Invalid source line")
                    if type(count) is not int or count < 0:
                        raise ValueError("Invalid execution count")
                    line_count += 1
            if record["collection"] == "complete" and not line_count:
                raise ValueError(
                    "Complete profile contains no executable lines"
                )
            seen.add(identity)
            records[uid].append(
                {
                    "outcome": record["outcome"],
                    "collection": record["collection"],
                }
            )
            if record["collection"] == "complete":
                directory = suite_directory(uid)
                group = (expected[uid], directory)
                groups[group].append(path)
        except (ValueError, KeyError, TypeError, AttributeError) as error:
            errors.append(f"Invalid profile {path.parent.name}: {error}")
    statuses = xml_statuses(source, errors)
    suites = []
    for uid, length in sorted(expected.items()):
        profiles = records[uid]
        states = statuses[uid]
        if (
            "Failed" in states
            or "Errored" in states
            or any(item["outcome"] == "failed" for item in profiles)
        ):
            outcome = "failed"
        elif states and states <= {"Skipped"}:
            outcome = "skipped"
        elif (
            states
            and states <= {"Passed", "Skipped"}
            and not any(item["outcome"] == "interrupted" for item in profiles)
        ):
            outcome = "completed"
        else:
            outcome = "unfinished"
        missing = (
            uid not in excluded
            and outcome != "skipped"
            and (
                not profiles
                or any(item["collection"] != "complete" for item in profiles)
            )
        )
        suites.append(
            {
                "test_uid": uid,
                "length": length,
                "outcome": outcome,
                "invocations": len(profiles),
                "missing_profiles": missing,
                "exclusion": excluded.get(uid),
            }
        )
    counts = {
        key: sum(row["outcome"] == key for row in suites)
        for key in ("completed", "failed", "skipped", "unfinished")
    }
    counts.update(
        expected=len(suites),
        excluded=len(excluded),
        missing_profiles=sum(row["missing_profiles"] for row in suites),
        invocations=len(seen),
    )
    uploads = []
    for (length, directory), paths in sorted(groups.items()):
        slug = directory.replace("/", "-")
        if not re.fullmatch(r"[a-zA-Z0-9_.-]+", slug):
            errors.append("Invalid TestLib group directory")
            continue
        filename = f"testlib-{length}-{slug}.info"
        chunks = []
        # Re-read one group at a time rather than retaining every suite's
        # zero-count baseline (hundreds of thousands of lines) in memory.
        by_file = defaultdict(lambda: defaultdict(int))
        for profile_path in paths:
            profile = json.loads(profile_path.read_text())
            for entry in profile["files"]:
                for line, count in entry["lines"].items():
                    by_file[entry["path"]][int(line)] += count
        for path, hits in sorted(by_file.items()):
            chunks.extend(["TN:", f"SF:{path}"])
            chunks.extend(
                f"DA:{line},{count}" for line, count in sorted(hits.items())
            )
            chunks.extend(
                [
                    f"LF:{len(hits)}",
                    f"LH:{sum(v > 0 for v in hits.values())}",
                    "end_of_record",
                ]
            )
        (output / filename).write_text("\n".join(chunks) + "\n")
        uploads.append(
            {
                "file": filename,
                "flags": f"overall-testdir-{slug},overall-length-{length},{slug}-{length}",
            }
        )
    planned_native = {}
    for path in sorted(source.rglob("expected-native.json")):
        try:
            plan = json.loads(path.read_text())
            if plan.get("revision") != revision:
                raise ValueError("Native plan revision mismatch")
            for group, definition in native_groups(path).items():
                if (
                    group in planned_native
                    and planned_native[group] != definition
                ):
                    raise ValueError("Conflicting native plans")
                planned_native[group] = definition
        except (ValueError, KeyError, TypeError, AttributeError) as error:
            errors.append(f"Invalid native plan: {error}")
    if campaign and not planned_native:
        errors.append("Missing aggregate native discovery plan")
    aggregate_groups, aggregate_rows = set(), []
    for path in sorted(source.rglob("aggregate.json")):
        try:
            item = json.loads(path.read_text())
            if (
                item.get("revision") != revision
                or item.get("schema_version") != 2
            ):
                raise ValueError("Aggregate schema or revision mismatch")
            group = item["group"]
            if group not in planned_native:
                raise ValueError("Aggregate group has no native plan")
            if group in aggregate_groups:
                raise ValueError("Duplicate aggregate group")
            aggregate_groups.add(group)
            definition = planned_native[group]
            outcomes = item["outcomes"]
            execution_complete = set(outcomes) == set(
                definition["stages"]
            ) and all(value == "success" for value in outcomes.values())
            counters = item["counters"]
            counters_present = (
                type(counters.get("files")) is int
                and counters["files"] > 0
                and type(counters.get("bytes")) is int
                and counters["bytes"] > 0
            )
            row = {
                "group": group,
                "outcomes": outcomes,
                "execution_complete": execution_complete,
                "counters": counters,
                "report_present": False,
                "collected_at": item.get("collected_at"),
            }
            aggregate_rows.append(row)
            if not execution_complete:
                errors.append(
                    f"Incomplete aggregate build/test stages: {group}"
                )
            if not counters_present:
                errors.append(f"Missing aggregate runtime counters: {group}")
            report = path.parent / "coverage.xml"
            if not report.is_file():
                raise ValueError(f"Missing aggregate report: {group}")
            tree = ET.parse(report)
            if tree.getroot().tag != "coverage" or not list(tree.iter("line")):
                raise ValueError("No executable native lines")
            filename = f"aggregate-{group}.xml"
            (output / filename).write_bytes(report.read_bytes())
            row["report_present"] = True
            uploads.append(
                {"file": filename, "flags": ",".join(definition["flags"])}
            )
        except (
            ValueError,
            KeyError,
            TypeError,
            AttributeError,
            ET.ParseError,
        ) as error:
            errors.append(f"Invalid aggregate report: {error}")
    missing = set(planned_native) - aggregate_groups
    if missing:
        errors.append(
            "Missing aggregate reports: " + ", ".join(sorted(missing))
        )
    complete = (
        not errors
        and not counts["missing_profiles"]
        and not counts["unfinished"]
    )
    summary = {
        "schema_version": 1,
        "revision": revision,
        "complete": complete,
        "scope": "Native TestLib suite invocations; Python is not measured. "
        "GTest and integrations are aggregate groups, not individual tests.",
        "counts": counts,
        "suites": suites,
        "errors": errors,
        "aggregate_groups": sorted(aggregate_groups),
        "aggregates": aggregate_rows,
        "generated_at": datetime.now(timezone.utc).isoformat(),
    }
    write_json(output / "summary.json", summary)
    write_json(output / "uploads.json", uploads)
    message = [
        "## Coverage collection",
        "",
        summary["scope"],
        "",
        "| Expected suites | Completed | Failed | Skipped | Unfinished | Missing profiles | Excluded |",
        "| --- | --- | --- | --- | --- | --- | --- |",
        "| "
        + " | ".join(
            str(counts[k])
            for k in (
                "expected",
                "completed",
                "failed",
                "skipped",
                "unfinished",
                "missing_profiles",
                "excluded",
            )
        )
        + " |",
        "",
        "Collection accounting: "
        + ("complete" if complete else "INCOMPLETE")
        + ".",
        "Completed/failed are execution outcomes; missing profiles is a separate count.",
        "Suites with no gem5 invocation also have no profile and remain visibly missing.",
    ]
    message.extend("- " + item for item in errors)
    for uid, reason in sorted(excluded.items()):
        message.append(f"- Excluded `{uid}`: {reason}")
    (output / "summary.md").write_text("\n".join(message) + "\n")
    return summary


def aggregate(
    output, revision, group, outcomes=None, config=None, record_only=False
):
    output = Path(output)
    metadata = output / "aggregate.json"
    if record_only:
        definition = native_groups(config)[group]
        outcomes = json.loads(outcomes)
        if not isinstance(outcomes, dict) or set(outcomes) != set(
            definition["stages"]
        ):
            raise ValueError(
                "Native outcomes must match planned stages exactly"
            )
        if any(
            value not in {"success", "failure", "cancelled", "skipped"}
            for value in outcomes.values()
        ):
            raise ValueError("Invalid native stage outcome")
        counters = [
            p
            for p in Path("build").rglob("*.gcda")
            if not p.name.endswith(".py.gcda") and p.stat().st_size
        ]
        version = subprocess.run(
            ["gcov", "--version"], check=True, text=True, capture_output=True
        ).stdout
        write_json(
            metadata,
            {
                "schema_version": 2,
                "revision": revision,
                "group": group,
                "outcomes": outcomes,
                "collected_at": datetime.now(timezone.utc).isoformat(),
                "counters": {
                    "files": len(counters),
                    "bytes": sum(p.stat().st_size for p in counters),
                },
                "gcov_version": version.splitlines()[0],
                "extraction": "pending",
            },
        )
        return
    item = json.loads(metadata.read_text())
    if item["revision"] != revision or item["group"] != group:
        raise ValueError("Aggregate extraction identity mismatch")
    # Embedded Python notes are not native machine-code coverage.
    for path in Path("build").rglob("*.py.gcno"):
        path.unlink()
    try:
        subprocess.run(
            [
                sys.executable,
                "-m",
                "gcovr",
                "--root",
                ".",
                "--merge-mode-functions",
                "separate",
                "--xml",
                str(output / "coverage.xml"),
            ],
            check=True,
        )
        item["extraction"] = "complete"
    except (OSError, subprocess.CalledProcessError):
        item["extraction"] = "error"
        raise
    finally:
        write_json(metadata, item)


def package(source, output, revision):
    """Archive raw profiles once, preserving the collector's hard links."""
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    coverage = source / "coverage"
    write_json(
        output / "artifact.json",
        {
            "schema_version": 1,
            "revision": revision,
            "profiles_present": coverage.is_dir(),
        },
    )
    if coverage.is_dir():
        with tarfile.open(
            output / "raw-profiles.tar.gz", "w:gz", dereference=False
        ) as archive:
            archive.add(coverage, arcname="coverage")
        for path in coverage.rglob("coverage.json"):
            target = output / "records" / path.relative_to(coverage)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(path, target)
    results = source / "results.xml"
    if results.is_file():
        shutil.copyfile(results, output / "results.xml")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("manifest")
    plan.add_argument("listing")
    plan.add_argument(
        "--length", required=True, choices=["quick", "long", "very-long"]
    )
    build = commands.add_parser("summarize")
    build.add_argument("source")
    build.add_argument("--campaign", action="store_true")
    pack = commands.add_parser("package")
    pack.add_argument("source")
    native = commands.add_parser("aggregate")
    native.add_argument("--group", required=True)
    native.add_argument("--outcomes")
    native.add_argument("--config", default=".github/coverage-native.json")
    native.add_argument("--record-only", action="store_true")
    native_discovery = commands.add_parser("native-plan")
    native_discovery.add_argument("config")
    for command in (plan, build, native, pack, native_discovery):
        command.add_argument("--revision", required=True)
        command.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.command == "manifest":
        manifest(args.listing, args.revision, args.length, args.output)
    elif args.command == "native-plan":
        native_plan(args.config, args.revision, args.output)
    elif args.command == "package":
        package(args.source, args.output, args.revision)
    elif args.command == "aggregate":
        aggregate(
            args.output,
            args.revision,
            args.group,
            args.outcomes,
            args.config,
            args.record_only,
        )
    else:
        summarize(args.source, args.output, args.revision, args.campaign)


if __name__ == "__main__":
    main()
