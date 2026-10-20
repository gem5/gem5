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

"""Build line coverage reports and query test attribution without a service."""

import argparse
import gzip
import hashlib
import json
import shutil
import sqlite3
import subprocess
import sys
from pathlib import Path


def read_json(path):
    with gzip.open(path, "rt") if path.suffix == ".gz" else path.open() as f:
        return json.load(f)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def source_path(value):
    if (
        not isinstance(value, str)
        or not value
        or "\\" in value
        or any(ord(c) < 32 for c in value)
        or value.startswith("/")
        or any(p in ("", ".", "..") for p in value.split("/"))
    ):
        raise ValueError(f"Invalid source path: {value!r}")
    return value


def counter(value):
    if type(value) is not int or not 0 <= value < 2**63:
        raise ValueError(f"Invalid execution count: {value!r}")
    return value


def line_number(value):
    if (
        not isinstance(value, str)
        or not value.isascii()
        or not value.isdecimal()
        or str(int(value)) != value
        or int(value) < 1
    ):
        raise ValueError(f"Invalid source line: {value!r}")
    return int(value)


def file_lines(files):
    seen = set()
    for entry in files:
        path = source_path(entry["path"])
        if path in seen:
            raise ValueError(f"Duplicate source path: {path}")
        seen.add(path)
        for number, count in entry["lines"].items():
            yield path, line_number(number), counter(count)


class Index:
    """Keep the executable-line inventory separate from sparse invocation hits."""

    def __init__(self, path, root, revision):
        self.root = root.resolve()
        self.revision = revision
        self.db = sqlite3.connect(path)
        self.db.executescript("""
            CREATE TABLE metadata(revision TEXT NOT NULL);
            CREATE TABLE lines(language TEXT, path TEXT, line INTEGER,
                PRIMARY KEY(language, path, line));
            CREATE TABLE invocations(language TEXT, id TEXT, uid TEXT,
                parent TEXT, outcome TEXT, collection TEXT,
                PRIMARY KEY(language, id));
            CREATE TABLE hits(language TEXT, invocation TEXT, path TEXT,
                line INTEGER, count INTEGER NOT NULL,
                PRIMARY KEY(language, invocation, path, line));
            CREATE INDEX hits_by_line ON hits(language, path, line);
            CREATE UNIQUE INDEX python_parent ON invocations(parent)
                WHERE language='python';
            """)
        self.db.execute("INSERT INTO metadata VALUES (?)", (revision,))
        self.db.commit()
        self.baseline = None
        self.baseline_id = None
        self.seeded = set()

    def inventory(self, language, rows):
        self.db.executemany(
            "INSERT OR IGNORE INTO lines VALUES (?, ?, ?)",
            ((language, path, line) for path, line, _ in rows),
        )

    def stored_baseline(self, path, record):
        identity = record.get("baseline_id")
        if (
            not isinstance(identity, str)
            or len(identity) != 64
            or any(c not in "0123456789abcdef" for c in identity)
        ):
            raise ValueError("Invalid baseline identity")
        if self.baseline_id != identity:
            parent = path.resolve().parent
            while parent.is_relative_to(self.root):
                folder = parent / "baselines" / identity
                candidates = [
                    p
                    for p in (
                        folder / "baseline.json",
                        folder / "baseline.json.gz",
                    )
                    if p.is_file()
                ]
                if candidates:
                    if len(candidates) != 1 or not candidates[
                        0
                    ].resolve().is_relative_to(self.root):
                        raise ValueError("Ambiguous or escaping baseline")
                    baseline = read_json(candidates[0])
                    checksum = hashlib.sha256()
                    encoder = json.JSONEncoder(
                        sort_keys=True,
                        separators=(",", ":"),
                        ensure_ascii=True,
                        allow_nan=False,
                    )
                    for chunk in encoder.iterencode(baseline):
                        checksum.update(chunk.encode())
                    if checksum.hexdigest() != identity:
                        raise ValueError("Baseline hash mismatch")
                    if (
                        baseline.get("schema_version") != 1
                        or baseline.get("format") != "gem5-coverage-baseline"
                    ):
                        raise ValueError("Invalid baseline format")
                    self.baseline, self.baseline_id = baseline, identity
                    break
                parent = parent.parent
            else:
                raise ValueError("Missing coverage baseline")
        baseline = self.baseline
        if (
            baseline["revision"] != self.revision
            or baseline["language"] != record.get("language", "native")
            or baseline["build_id"] != record["build"].get("build_id")
        ):
            raise ValueError("Baseline revision, language or build mismatch")
        if identity not in self.seeded:
            if any(count for _, _, count in file_lines(baseline["files"])):
                raise ValueError("Baseline contains nonzero counters")
            self.inventory(
                record.get("language", "native"), file_lines(baseline["files"])
            )
            self.seeded.add(identity)
        return {e["path"]: e["lines"] for e in baseline["files"]}

    def profile(self, path):
        if not path.resolve().is_relative_to(self.root):
            raise ValueError("Profile escapes artifact directory")
        item = read_json(path)
        language = item.get("language", "native")
        if item["revision"] != self.revision:
            raise ValueError("Profile revision mismatch")
        if language not in ("native", "python"):
            raise ValueError("Unknown profile language")
        if item["schema_version"] not in (1, 2):
            raise ValueError("Unsupported profile schema")
        uid, identity = item["test_uid"], item["invocation_id"]
        if not isinstance(uid, str) or not uid.startswith("SuiteUID:"):
            raise ValueError("Profile needs a TestLib SuiteUID")
        if not isinstance(identity, str) or not identity:
            raise ValueError("Missing invocation identity")
        if item["outcome"] not in ("passed", "failed", "interrupted"):
            raise ValueError("Invalid invocation outcome")
        if item["collection"] not in ("complete", "missing", "error"):
            raise ValueError("Invalid collection outcome")
        self.db.execute(
            "INSERT INTO invocations VALUES (?, ?, ?, ?, ?, ?)",
            (
                language,
                identity,
                uid,
                item.get("parent_invocation_id"),
                item["outcome"],
                item["collection"],
            ),
        )
        if item["collection"] != "complete":
            raise ValueError(f"Collection {item['collection']}: {identity}")
        rows = list(file_lines(item["files"]))
        if item["schema_version"] == 2:
            baseline = self.stored_baseline(path, item)
            for name, line, count in rows:
                if count <= 0 or str(line) not in baseline.get(name, {}):
                    raise ValueError("Sparse hits do not match the baseline")
        else:
            self.inventory(language, rows)
        self.db.executemany(
            "INSERT INTO hits VALUES (?, ?, ?, ?, ?)",
            (
                (language, identity, name, line, count)
                for name, line, count in rows
                if count
            ),
        )
        return item["outcome"]

    def native(self, path, group, outcome="passed"):
        item = read_json(path)
        if item.get("gcovr/format_version") != "0.11":
            raise ValueError("Expected gcovr 8.3 JSON format 0.11")
        rows = []
        for entry in item["files"]:
            name = source_path(entry["file"])
            for line in entry["lines"]:
                if line.get("gcovr/noncode", False) or line.get(
                    "gcovr/excluded", False
                ):
                    continue
                rows.append(
                    (
                        name,
                        line_number(str(line["line_number"])),
                        counter(line["count"]),
                    )
                )
        if not rows:
            raise ValueError(f"No native lines collected for {group}")
        identity = f"group:{group}"
        self.db.execute(
            "INSERT INTO invocations VALUES ('native', ?, ?, NULL, ?, 'complete')",
            (identity, identity, outcome),
        )
        self.inventory("native", rows)
        self.db.executemany(
            "INSERT INTO hits VALUES ('native', ?, ?, ?, ?) "
            "ON CONFLICT(language, invocation, path, line) "
            "DO UPDATE SET count=count+excluded.count",
            (
                (identity, name, line, count)
                for name, line, count in rows
                if count
            ),
        )

    def lcov(self, output, language):
        totals = {"lines": 0, "covered": 0}
        with gzip.open(output, "wt") as stream:
            current = None
            cursor = self.db.execute(
                """SELECT l.path, l.line, COALESCE(SUM(h.count), 0)
                FROM lines l LEFT JOIN hits h USING(language, path, line)
                WHERE l.language=? GROUP BY l.path, l.line
                ORDER BY l.path, l.line""",
                (language,),
            )
            for path, line, count in cursor:
                if path != current:
                    if current is not None:
                        stream.write("end_of_record\n")
                    stream.write(f"TN:\nSF:{path}\n")
                    current = path
                stream.write(f"DA:{line},{count}\n")
                totals["lines"] += 1
                totals["covered"] += count > 0
            if current is not None:
                stream.write("end_of_record\n")
        return totals


def profile_order(path):
    # Keep one large baseline graph resident while consuming all its profiles.
    try:
        identity = read_json(path).get("baseline_id", "")
        return identity if isinstance(identity, str) else "", str(path)
    except (ValueError, AttributeError, OSError):
        # Invalid inputs are diagnosed during ingestion, after creating outputs.
        return "", str(path)


def build(source, output, revision):
    source, output = Path(source), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    database = output / "index.sqlite3"
    database.unlink(missing_ok=True)
    index = Index(database, source, revision)
    errors, seen_tasks, seen_groups = [], set(), set()
    plans = list(source.rglob("plan.json"))
    plan = {}
    try:
        if len(plans) != 1:
            raise ValueError("Expected exactly one campaign plan.json")
        plan = read_json(plans[0])
        if plan["revision"] != revision:
            raise ValueError("Plan revision mismatch")
        tasks, groups = plan["tests"], plan["native"]
        if not tasks or not groups:
            raise ValueError("Campaign plan has no tests or native groups")
        if len({t["id"] for t in tasks}) != len(tasks) or len(
            {g["group"] for g in groups}
        ) != len(groups):
            raise ValueError("Duplicate planned workload")
        if any(
            not t["suites"]
            or any(
                not isinstance(uid, str) or not uid.startswith("SuiteUID:")
                for uid in t["suites"]
            )
            for t in tasks
        ):
            raise ValueError("Planned task needs TestLib suites")
    except (ValueError, KeyError, TypeError, AttributeError, OSError) as error:
        errors.append(str(error))
        plan = {}
    for path in sorted(source.rglob("status.json")):
        try:
            status = read_json(path)
            if status["revision"] != revision:
                raise ValueError("Status revision mismatch")
            stages = status["stages"]
            if not stages or any(s != "success" for s in stages.values()):
                errors.append(
                    f"{path.relative_to(source)}: Workload stages did not all succeed"
                )
            if "task_id" in status:
                if status["task_id"] in seen_tasks:
                    raise ValueError("Duplicate task status")
                seen_tasks.add(status["task_id"])
            else:
                group = status["group"]
                if status.get("extraction") != "complete":
                    raise ValueError("Native extraction did not complete")
                if group in seen_groups:
                    raise ValueError("Duplicate native status")
                candidates = [
                    file
                    for file in (
                        path.parent / "native.json",
                        path.parent / "native.json.gz",
                    )
                    if file.is_file()
                ]
                if len(candidates) != 1:
                    raise ValueError("Missing or ambiguous native report")
                with index.db:
                    index.native(
                        candidates[0],
                        group,
                        (
                            "passed"
                            if all(s == "success" for s in stages.values())
                            else "failed"
                        ),
                    )
                seen_groups.add(group)
        except (
            ValueError,
            KeyError,
            TypeError,
            AttributeError,
            OSError,
            sqlite3.DatabaseError,
        ) as error:
            errors.append(f"{path.relative_to(source)}: {error}")
    for name in ("coverage.json", "python-coverage.json"):
        paths = list(source.rglob(name)) + list(source.rglob(name + ".gz"))
        for path in sorted(paths, key=profile_order):
            try:
                with index.db:
                    outcome = index.profile(path)
                if outcome != "passed":
                    errors.append(
                        f"{path.relative_to(source)}: Invocation {outcome}"
                    )
            except (
                ValueError,
                KeyError,
                TypeError,
                AttributeError,
                OSError,
                sqlite3.DatabaseError,
            ) as error:
                errors.append(f"{path.relative_to(source)}: {error}")
                index.seeded.clear()
    expected_tasks = {item["id"] for item in plan.get("tests", [])}
    expected_groups = {item["group"] for item in plan.get("native", [])}
    expected_uids = {
        uid for item in plan.get("tests", []) for uid in item["suites"]
    }
    native_uids = {
        r[0]
        for r in index.db.execute(
            "SELECT DISTINCT uid FROM invocations WHERE language='native' "
            "AND collection='complete' AND uid LIKE 'SuiteUID:%'"
        )
    }
    missing = {
        "tasks": sorted(expected_tasks - seen_tasks),
        "groups": sorted(expected_groups - seen_groups),
        "suites": sorted(expected_uids - native_uids),
    }
    if seen_tasks - expected_tasks or seen_groups - expected_groups:
        errors.append("Collected workload not present in the campaign plan")
    if native_uids - expected_uids:
        errors.append("Collected suite not present in the campaign plan")
    mismatched = index.db.execute(
        """SELECT p.id FROM invocations p LEFT JOIN invocations n
        ON n.language='native' AND n.id=p.parent AND n.uid=p.uid
        WHERE p.language='python' AND n.id IS NULL"""
    ).fetchall()
    missing_python = index.db.execute(
        """SELECT n.id FROM invocations n LEFT JOIN invocations p
        ON p.language='python' AND p.parent=n.id AND p.uid=n.uid
        WHERE n.language='native' AND n.uid LIKE 'SuiteUID:%'
        AND p.id IS NULL"""
    ).fetchall()
    if mismatched or missing_python:
        errors.append("Native/Python invocation pairing is incomplete")
    summary = {
        "revision": revision,
        "complete": bool(plan) and not errors and not any(missing.values()),
        "errors": errors,
        "missing": missing,
        "unpaired_invocations": {
            "python": [row[0] for row in mismatched],
            "native": [row[0] for row in missing_python],
        },
        "exclusions": plan.get("exclusions", []),
        "coverage": {
            language: index.lcov(
                output / f"coverage-{language}.info.gz", language
            )
            for language in ("native", "python")
        },
    }
    index.db.commit()
    index.db.close()
    write_json(output / "summary.json", summary)
    return summary


def native(group, status_path, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    status = read_json(Path(status_path))
    if status["group"] != group:
        raise ValueError("Native status group mismatch")
    status["extraction"] = "error"
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
                "--gcov-exclude",
                r".*\.py\.gc(no|da)$",
                "--json",
                str(output / "native.json"),
            ],
            check=True,
        )
        path = output / "native.json"
        with (
            path.open("rb") as source,
            gzip.open(str(path) + ".gz", "wb", compresslevel=3) as compressed,
        ):
            shutil.copyfileobj(source, compressed)
        path.unlink()
        status["extraction"] = "complete"
    finally:
        write_json(output / "status.json", status)


def query(database, uid=None, location=None, language=None):
    with sqlite3.connect(
        Path(database).resolve().as_uri() + "?mode=ro", uri=True
    ) as db:
        db.row_factory = sqlite3.Row
        if uid is not None:
            sql = """SELECT h.language, h.path, h.line, SUM(h.count) count,
                json_group_array(h.invocation) invocations FROM hits h
                JOIN invocations i ON i.language=h.language AND i.id=h.invocation
                WHERE i.uid=? GROUP BY h.language, h.path, h.line
                ORDER BY h.language, h.path, h.line"""
            rows = db.execute(sql, (uid,))
        else:
            path, line = location.rsplit(":", 1)
            source_path(path)
            number = line_number(line)
            sql = """SELECT i.uid, h.language, SUM(h.count) count,
                json_group_array(h.invocation) invocations FROM hits h
                JOIN invocations i ON i.language=h.language AND i.id=h.invocation
                WHERE h.path=? AND h.line=? AND (? IS NULL OR h.language=?)
                GROUP BY i.uid, h.language ORDER BY i.uid, h.language"""
            rows = db.execute(sql, (path, number, language, language))
        return [
            {**dict(row), "invocations": json.loads(row["invocations"])}
            for row in rows
        ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    collection = commands.add_parser("native")
    collection.add_argument("--group", required=True)
    collection.add_argument("--status", required=True)
    collection.add_argument("--output", required=True)
    report = commands.add_parser("build")
    report.add_argument("artifacts")
    report.add_argument("--output", required=True)
    report.add_argument("--revision", required=True)
    tests = commands.add_parser("tests")
    tests.add_argument("database")
    tests.add_argument("location", help="Repository-relative PATH:LINE")
    tests.add_argument("--language", choices=("native", "python"))
    lines = commands.add_parser("lines")
    lines.add_argument("database")
    lines.add_argument("uid")
    args = parser.parse_args()
    if args.command == "native":
        native(args.group, args.status, args.output)
    elif args.command == "build":
        summary = build(args.artifacts, args.output, args.revision)
        print(json.dumps(summary, indent=2))
        return 0 if summary["complete"] else 1
    else:
        result = (
            query(args.database, uid=args.uid)
            if args.command == "lines"
            else query(
                args.database, location=args.location, language=args.language
            )
        )
        print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
