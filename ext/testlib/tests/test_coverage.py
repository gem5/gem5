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

"""Native collector checks; run explicitly, outside gem5's PyUnit suite."""

import concurrent.futures
import copy
import gzip
import importlib.util
import json
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

MODULE = Path(__file__).resolve().parents[1] / "coverage.py"
spec = importlib.util.spec_from_file_location("testlib_coverage", MODULE)
coverage = importlib.util.module_from_spec(spec)
spec.loader.exec_module(coverage)


class Log:
    def message(self, message):
        pass


class BranchStorageTest(unittest.TestCase):
    def test_compaction_preserves_identity_queries_and_lcov(self):
        sys.path.insert(0, str(MODULE.parents[2] / "util" / "coverage"))
        try:
            from util.coverage import (
                index,
                report,
            )
        finally:
            sys.path.pop(0)

        branches = [
            {
                "unit": "build/ALL/very_long_translation_unit.gcno",
                "function": "_ZN" + "long_template_argument_" * 100,
                "line": 1,
                "ordinal": ordinal,
                "count": ordinal % 2,
                "fallthrough": ordinal % 2 == 0,
                "throw": False,
            }
            for ordinal in range(200)
        ]
        expanded = [
            {"path": "src/example.cc", "lines": {"1": 1}, "branches": branches}
        ]
        compact = copy.deepcopy(expanded)
        for branch in branches:
            branch["id"] = coverage._branch_id("build-identity", branch)
        coverage._compact_branches(compact, "build-identity")
        self.assertEqual(
            [b["id"] for b in branches],
            [b["id"] for b in compact[0]["branches"]],
        )
        self.assertLess(
            len(coverage._canonical_bytes(compact)),
            len(coverage._canonical_bytes(expanded)) / 10,
        )
        record = {
            "schema_version": 1,
            "revision": "a" * 40,
            "test_uid": "SuiteUID:tests/gem5/example/test.py:one",
            "invocation_id": "one",
            "build": {},
            "collection": "complete",
            "outcome": "passed",
        }
        indexes = [
            index.build_index([{**record, "files": files}])
            for files in (expanded, compact)
        ]
        for branch in branches:
            queries = [
                index.tests_for_branch(data, "src/example.cc", branch["id"])
                for data in indexes
            ]
            for query in queries:
                query["metadata"].pop("unit", None)
                query["metadata"].pop("function", None)
            self.assertEqual(*queries)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            exports = []
            for number, files in enumerate((expanded, compact)):
                source = root / str(number) / "input"
                source.mkdir(parents=True)
                output = source.parent / "output"
                report.write_json(
                    source / "coverage.json", {**record, "files": files}
                )
                report.write_json(
                    source / "expected.json",
                    {
                        "schema_version": 1,
                        "revision": record["revision"],
                        "length": "quick",
                        "expected": [record["test_uid"]],
                        "excluded": {},
                    },
                )
                report.summarize(source, output, record["revision"])
                exports.append(
                    [
                        gzip.decompress(path.read_bytes()).decode()
                        for path in output.glob("*.info.gz")
                    ]
                )
            self.assertTrue(exports[0])
            self.assertEqual(*exports)


class CoverageTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which("gcc") or not shutil.which("gcov"):
            raise unittest.SkipTest("GCC and gcov are required")
        version = subprocess.check_output(["gcc", "--version"], text=True)
        if "Free Software Foundation" not in version:
            raise unittest.SkipTest("GNU GCC, not a Clang alias, is required")

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.target = self.root / "build"
        self.target.mkdir()
        self.source = self.root / "main.c"
        self.source.write_text(
            "#include <stdlib.h>\n"
            "int main(int argc, char **argv) {\n"
            "    if (argv[1][0] == 'a') {\n"
            "        return 0;\n"
            "    }\n"
            "    if (argv[1][0] == 'b') {\n"
            "        return 0;\n"
            "    }\n"
            "    exit(7);\n"
            "}\n"
        )
        # A translation unit which emits notes but never emits counters.
        unused = self.root / "unused.c"
        unused.write_text("int unused(void) { return 42; }\n")
        for source in (self.source, unused):
            subprocess.run(
                [
                    "gcc",
                    "--coverage",
                    "-O0",
                    "-c",
                    str(source),
                    "-o",
                    str(self.target / (source.stem + ".o")),
                ],
                check=True,
            )
        self.binary = self.target / "program"
        subprocess.run(
            [
                "gcc",
                "--coverage",
                str(self.target / "main.o"),
                "-o",
                str(self.binary),
            ],
            check=True,
        )
        self.build = coverage.CoverageBuild(
            self.root, self.target, self.binary, self.root / "results"
        )

    def run_program(self, argument, build=None):
        invocation = (build or self.build).invocation(
            "SuiteUID:gem5/example/test.py:case-" + argument, Log()
        )
        with invocation as environment:
            subprocess.run(
                [str(self.binary), argument], env=environment, check=True
            )
        return json.loads(invocation.path.read_text())

    def python_config(self, argument="a", relocated=False, missing=False):
        if not missing and importlib.util.find_spec("coverage") is None:
            self.skipTest("coverage.py is required for Python collection")
        stub = self.root / "gem5_stub.py"
        stub.write_text(
            "import os,sys,types\n"
            "sys.modules['m5']=types.SimpleNamespace(options=types.SimpleNamespace(P=False))\n"
            "sys.argv=sys.argv[1:]\n"
            "sys.path.insert(0,os.path.dirname(sys.argv[0]))\n"
            "exec(compile(open(sys.argv[0]).read(),sys.argv[0],'exec'),"
            "{'__name__':'__m5_main__','__file__':sys.argv[0]})\n"
        )
        script = self.root / "config.py"
        helper = self.root / "helper.py"
        helper.write_text("value = 42\n")
        script.write_text(
            "import sys,helper\n"
            "assert __name__ == '__m5_main__'\n"
            "assert sys.argv == [__file__, sys.argv[1], 'space argument']\n"
            "assert helper.value == 42\n"
            "if sys.argv[1] == 'a':\n"
            "    selected = 1\n"
            "else:\n"
            "    selected = 2\n"
            "if sys.argv[1] == 'fail':\n"
            "    sys.exit(7)\n"
        )
        if relocated:
            embedded = self.root / "embedded.py"
            embedded.write_text("embedded_value = 3\n")
            script.write_text(
                script.read_text()
                + "exec(compile('embedded_value = 3\\n', "
                + repr("/absent-producer-root/embedded.py")
                + ", 'exec'), {})\n"
            )
        build = coverage.CoverageBuild(
            self.root,
            self.target,
            self.binary,
            self.root / "results",
            python_coverage=True,
        )
        if relocated:
            build.compiled_root = Path("/absent-producer-root")
            # Avoid changing native gcov paths in this wrapper-specific fixture.
            build._prepare = lambda: None
        invocation = build.invocation(
            "SuiteUID:gem5/example/test.py:python", Log()
        )
        early = json.loads(invocation.python_path.read_text())
        self.assertEqual(
            (early["outcome"], early["collection"]), ("interrupted", "missing")
        )
        options = ["-S"] if missing else []
        command = [
            sys.executable,
            *options,
            str(stub),
            "config.py",
            argument,
            "space argument",
        ]
        result = None
        try:
            with invocation as environment:
                command = [
                    sys.executable,
                    *options,
                    *invocation.python_command(command[1 + len(options) :], 1),
                ]
                result = subprocess.run(
                    command,
                    env=environment,
                    cwd=self.root,
                    text=True,
                    capture_output=True,
                )
                result.check_returncode()
        except subprocess.CalledProcessError:
            if argument != "fail":
                self.fail(result.stderr)
        record = json.loads(invocation.python_path.read_text())
        return record, invocation, result

    def test_python_config_identity_and_failure_are_preserved(self):
        for argument in ("a", "b", "fail"):
            record, invocation, result = self.python_config(argument)
            self.assertEqual(result.returncode, 7 if argument == "fail" else 0)
            self.assertEqual(record["collection"], "complete", record)
            self.assertEqual(
                record["outcome"], "failed" if argument == "fail" else "passed"
            )
            files = {entry["path"]: entry for entry in record["files"]}
            self.assertEqual(
                files["config.py"]["lines"]["6"], int(argument == "a")
            )
            self.assertEqual(
                files["config.py"]["lines"]["8"], int(argument != "a")
            )
            self.assertEqual(files["helper.py"]["lines"], {"1": 1})
            self.assertTrue(files["config.py"]["branches"])
            self.assertEqual(
                record["parent_invocation_id"], invocation.identifier
            )
            self.assertEqual(
                record["invocation_id"], invocation.identifier + "-python"
            )
            self.assertTrue((invocation.directory / ".coverage").exists())
            contexts = json.loads(
                (invocation.directory / "python-contexts.json").read_text()
            )
            self.assertIn(
                record["test_uid"] + "|" + invocation.identifier, str(contexts)
            )

    def test_python_relocated_embedded_source_is_mapped(self):
        record, _, _ = self.python_config(relocated=True)
        self.assertEqual(record["collection"], "complete", record)
        files = {entry["path"]: entry for entry in record["files"]}
        self.assertEqual(files["embedded.py"]["lines"], {"1": 1})

    def test_missing_python_dependency_preserves_config_failure(self):
        record, _, result = self.python_config("fail", missing=True)
        self.assertEqual(result.returncode, 7)
        self.assertEqual(record["outcome"], "failed")
        self.assertEqual(record["collection"], "error")
        self.assertIn("coverage", record["error"])

    def test_serial_and_parallel_profiles_remain_distinct(self):
        serial = {argument: self.run_program(argument) for argument in "ab"}
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            arguments = list("abababab")
            records = list(pool.map(self.run_program, arguments))
        self.assertEqual(len({r["invocation_id"] for r in records}), 8)
        for argument, record in zip(arguments, records):
            self.assertEqual(record["collection"], "complete")
            self.assertEqual(record["outcome"], "passed")
            self.assertEqual(record["files"], serial[argument]["files"])
            files = {f["path"]: f["lines"] for f in record["files"]}
            self.assertEqual(files["main.c"].get("4", 0), int(argument == "a"))
            self.assertEqual(files["main.c"].get("7", 0), int(argument == "b"))
            baseline_path = (
                self.build.output
                / "baselines"
                / record["baseline_id"]
                / "baseline.json"
            )
            baseline = json.loads(baseline_path.read_text())
            baseline_files = {f["path"]: f["lines"] for f in baseline["files"]}
            self.assertEqual(baseline_files["unused.c"]["1"], 0)
            self.assertNotIn("unused.c", files)
            self.assertTrue(
                all(
                    count > 0
                    for values in files.values()
                    for count in values.values()
                )
            )
            directory = self.build.output / record["invocation_id"]
            counters = list(directory.rglob("*.gcda"))
            self.assertTrue(counters)
            self.assertTrue(
                all(p.with_suffix(".gcno").exists() for p in counters)
            )
        self.assertFalse(list(self.target.rglob("*.gcda")))
        self.assertIn("gcc_versions", records[0]["build"])

    def test_branch_identity_distinguishes_opposite_paths(self):
        records = [self.run_program(argument) for argument in "ab"]
        baseline = json.loads(
            (
                self.build.output
                / "baselines"
                / records[0]["baseline_id"]
                / "baseline.json"
            ).read_text()
        )
        branches = {
            branch["id"]: branch
            for entry in baseline["files"]
            for branch in entry["branches"]
        }
        first = {
            branch["id"]
            for entry in records[0]["files"]
            for branch in entry["branches"]
        }
        second = {
            branch["id"]
            for entry in records[1]["files"]
            for branch in entry["branches"]
        }
        self.assertEqual(len(branches), 4)
        self.assertTrue(first - second)
        self.assertTrue(second - first)
        self.assertTrue(first | second <= branches.keys())
        self.assertTrue(
            all(branch["count"] == 0 for branch in branches.values())
        )
        self.assertEqual({branches[key]["line"] for key in first}, {3})
        self.assertEqual({branches[key]["line"] for key in second}, {3, 6})
        self.assertTrue(
            all("unit" not in branch for branch in branches.values())
        )
        self.assertTrue(
            all("function" not in branch for branch in branches.values())
        )

    def test_failed_process_retains_counters_and_failure(self):
        invocation = self.build.invocation("SuiteUID:test.py:failure", Log())
        with self.assertRaises(subprocess.CalledProcessError) as raised:
            with invocation as environment:
                subprocess.run(
                    [str(self.binary), "f"], env=environment, check=True
                )
        self.assertEqual(raised.exception.returncode, 7)
        record = json.loads(invocation.path.read_text())
        self.assertEqual(record["outcome"], "failed")
        self.assertEqual(record["collection"], "complete")
        files = {f["path"]: f["lines"] for f in record["files"]}
        self.assertEqual(files["main.c"]["9"], 1)

    def test_early_record_and_missing_profile(self):
        invocation = self.build.invocation("SuiteUID:test.py:missing", Log())
        record = json.loads(invocation.path.read_text())
        self.assertEqual(record["outcome"], "interrupted")
        self.assertEqual(record["collection"], "missing")
        with invocation:
            pass
        record = json.loads(invocation.path.read_text())
        self.assertEqual(record["collection"], "missing")
        self.assertEqual(record["files"], [])
        self.assertIn("baseline_id", record)

    def test_collection_error_does_not_mask_simulator_failure(self):
        build = coverage.CoverageBuild(
            self.root,
            self.target,
            self.binary,
            self.root / "bad-results",
            gcov="/does/not/exist",
        )
        invocation = build.invocation("SuiteUID:test.py:bad-tool", Log())
        with self.assertRaises(subprocess.CalledProcessError) as raised:
            with invocation as environment:
                subprocess.run(
                    [str(self.binary), "f"], env=environment, check=True
                )
        self.assertEqual(raised.exception.returncode, 7)
        record = json.loads(invocation.path.read_text())
        self.assertEqual(record["outcome"], "failed")
        self.assertEqual(record["collection"], "error")
        self.assertTrue(list(invocation.raw.rglob("*.gcda")))

    def test_extraction_error_preserves_failed_process(self):
        invocation = self.build.invocation("SuiteUID:test.py:extract", Log())
        with self.assertRaises(subprocess.CalledProcessError) as raised:
            with invocation as environment:
                self.build.gcov = "/does/not/exist"
                subprocess.run(
                    [str(self.binary), "f"], env=environment, check=True
                )
        self.assertEqual(raised.exception.returncode, 7)
        record = json.loads(invocation.path.read_text())
        self.assertEqual(record["outcome"], "failed")
        self.assertEqual(record["collection"], "error")
        counters = list(invocation.raw.rglob("*.gcda"))
        self.assertTrue(counters)
        self.assertTrue(all(p.with_suffix(".gcno").exists() for p in counters))

    def test_signal_exit_is_interrupted_and_missing(self):
        invocation = self.build.invocation("SuiteUID:test.py:signal", Log())
        with self.assertRaises(subprocess.CalledProcessError):
            with invocation as environment:
                subprocess.run(
                    ["sh", "-c", "kill -TERM $$"], env=environment, check=True
                )
        record = json.loads(invocation.path.read_text())
        self.assertEqual(record["outcome"], "interrupted")
        self.assertEqual(record["collection"], "missing")

    def test_relocated_build_uses_producer_paths(self):
        expected = self.run_program("b")["files"]
        relocated = self.root / "relocated-checkout"
        target = relocated / "build"
        shutil.copytree(self.target, target)
        shutil.copy2(self.source, relocated / "main.c")
        shutil.copy2(self.root / "unused.c", relocated / "unused.c")
        manifest = {
            "schema_version": 1,
            "revision": self.build.revision,
            "build_root": str(self.root),
            "compiler": "GCC native test",
            "image": "native test fixture",
            "instrumentation": "gcc --gcov",
        }
        (target / "build-program.json").write_text(json.dumps(manifest))
        build = coverage.CoverageBuild(
            relocated, target, target / "program", relocated / "results"
        )
        invocation = build.invocation("SuiteUID:test.py:relocated", Log())
        with invocation as environment:
            subprocess.run(
                [str(target / "program"), "b"], env=environment, check=True
            )
        record = json.loads(invocation.path.read_text())
        self.assertEqual(record["collection"], "complete", record)
        self.assertEqual(record["files"], expected)
        self.assertEqual(
            record["build"]["build_id"], self.build.build["build_id"]
        )
        counters = list(invocation.raw.rglob("*.gcda"))
        self.assertTrue(counters)
        self.assertTrue(all(p.with_suffix(".gcno").exists() for p in counters))
        self.assertEqual(
            json.loads((invocation.directory / "build.json").read_text()),
            manifest,
        )

        # Large manifest checksum maps are shared just like the notes.
        second = build.invocation("SuiteUID:test.py:second", Log())
        with second as environment:
            subprocess.run(
                [str(target / "program"), "a"], env=environment, check=True
            )
        first_manifest = invocation.directory / "build.json"
        second_manifest = second.directory / "build.json"
        self.assertEqual(
            first_manifest.stat().st_ino, second_manifest.stat().st_ino
        )
        archive_path = self.root / "metadata.tar"
        with tarfile.open(archive_path, "w") as archive:
            archive.add(first_manifest, arcname="first.json")
            archive.add(second_manifest, arcname="second.json")
        with tarfile.open(archive_path) as archive:
            self.assertTrue(archive.getmember("second.json").islnk())

    def test_sparse_records_share_verified_zero_baseline(self):
        unused = self.root / "unused.c"
        unused.write_text(
            "".join(
                f"int unused_{i}(void) {{ return {i}; }}\n"
                for i in range(1000)
            )
        )
        subprocess.run(
            [
                "gcc",
                "--coverage",
                "-O0",
                "-c",
                str(unused),
                "-o",
                str(self.target / "unused.o"),
            ],
            check=True,
        )
        first, second = self.run_program("a"), self.run_program("b")
        self.assertEqual(first["schema_version"], 2)
        self.assertEqual(first["baseline_id"], second["baseline_id"])
        baselines = list(
            (self.build.output / "baselines").glob("*/baseline.json")
        )
        self.assertEqual(len(baselines), 1)
        baseline = json.loads(baselines[0].read_text())
        self.assertEqual(coverage._digest_json(baseline), first["baseline_id"])
        native_lines = sum(len(item["lines"]) for item in baseline["files"])
        self.assertGreaterEqual(native_lines, 1000)
        record_path = (
            self.build.output / first["invocation_id"] / "coverage.json"
        )
        self.assertLess(
            record_path.stat().st_size, baselines[0].stat().st_size
        )

    def test_multiple_variants_are_rejected_before_running(self):
        repository = MODULE.parents[2]
        process = subprocess.run(
            [
                sys.executable,
                str(repository / "tests/main.py"),
                "list",
                "gem5/pyunit",
                "--gcov=per-test",
                "--variant=opt,debug",
                "--suites",
                "-q",
            ],
            cwd=repository / "tests",
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(process.returncode, 0)
        self.assertIn("requires one variant per run", process.stderr)

    def test_python_coverage_requires_native_invocation_identity(self):
        repository = MODULE.parents[2]
        process = subprocess.run(
            [
                sys.executable,
                str(repository / "tests/main.py"),
                "list",
                "gem5/pyunit",
                "--python-coverage",
                "--suites",
                "-q",
            ],
            cwd=repository / "tests",
            capture_output=True,
            text=True,
        )
        self.assertNotEqual(process.returncode, 0)
        self.assertIn(
            "--python-coverage requires --gcov=per-test", process.stderr
        )

    def test_real_testlib_harness_records_each_invocation(self):
        repository = MODULE.parents[2]
        subprocess.run(["git", "init", "-q", str(self.root)], check=True)
        subprocess.run(
            [
                "git",
                "-C",
                str(self.root),
                "-c",
                "user.name=Coverage test",
                "-c",
                "user.email=coverage@example.invalid",
                "commit",
                "--allow-empty",
                "-qm",
                "Test fixture",
            ],
            check=True,
        )
        target = self.root / "harness-build" / "ALL"
        target.mkdir(parents=True)
        source = self.root / "harness.c"
        source.write_text(
            "int main(int argc, char **argv) {\n"
            "    return argv[argc - 1][0] == 'f' ? 7 : 0;\n"
            "}\n"
        )
        subprocess.run(
            [
                "gcc",
                "--coverage",
                "-O0",
                "-c",
                str(source),
                "-o",
                str(target / "harness.o"),
            ],
            check=True,
        )
        subprocess.run(
            [
                "gcc",
                "--coverage",
                str(target / "harness.o"),
                "-o",
                str(target / "gem5.opt"),
            ],
            check=True,
        )
        suites = self.root / "suites"
        suites.mkdir()
        (suites / "test.py").write_text(
            "from testlib import gem5_verify_config, Fixture\n"
            "from testlib.fixture import SkipException\n"
            "class Skip(Fixture):\n"
            "    def setup(self, testitem):\n"
            "        raise SkipException(self, testitem.metadata)\n"
            "for name in ('passed', 'failed', 'skipped'):\n"
            "    gem5_verify_config(name=name, config=name, config_args=[],\n"
            "        verifiers=(), valid_isas=('ALL',),\n"
            "        valid_variants=('opt',), valid_hosts=('x86_64',),\n"
            "        fixtures=[Skip()] if name == 'skipped' else [])\n"
        )
        result_path = self.root / "testing-results"
        process = subprocess.run(
            [
                sys.executable,
                str(repository / "tests" / "main.py"),
                "run",
                str(suites),
                "--skip-build",
                "--gcov=per-test",
                "--python-coverage",
                "-t",
                "2",
                "--base-dir",
                str(self.root),
                "--build-dir",
                str(target.parent),
                "--host",
                "x86_64",
                "--isa",
                "ALL",
                "--variant",
                "opt",
            ],
            cwd=self.root,
            capture_output=True,
            text=True,
        )
        self.assertEqual(
            process.returncode, 1, process.stdout + process.stderr
        )
        records = [
            json.loads(path.read_text())
            for path in (result_path / "coverage").glob("*/coverage.json")
        ]
        self.assertEqual(len(records), 2, process.stdout + process.stderr)
        self.assertEqual({r["outcome"] for r in records}, {"passed", "failed"})
        self.assertTrue(
            all(r["test_uid"].startswith("SuiteUID:") for r in records)
        )
        self.assertTrue(all(r["collection"] == "complete" for r in records))
        self.assertTrue(all(r["revision"] != "unknown" for r in records))
        python_records = [
            json.loads(path.read_text())
            for path in (result_path / "coverage").glob(
                "*/python-coverage.json"
            )
        ]
        self.assertEqual(len(python_records), 2)
        self.assertEqual(
            {r["outcome"] for r in python_records}, {"passed", "failed"}
        )
        # This C stand-in never executes Python: early records must survive
        # with missing collection instead of reporting measured zero.
        self.assertTrue(
            all(r["collection"] == "missing" for r in python_records)
        )
        self.assertEqual(
            {r["parent_invocation_id"] for r in python_records},
            {r["invocation_id"] for r in records},
        )
        results = ET.parse(result_path / "results.xml")
        cases = list(results.iter("testcase"))
        self.assertEqual(
            {case.get("status") for case in cases},
            {"Passed", "Failed", "Skipped"},
        )
        self.assertEqual(results.getroot().get("skipped"), "1")
        skipped = next(
            case for case in cases if case.get("status") == "Skipped"
        )
        self.assertEqual(skipped.get("time"), "0")


if __name__ == "__main__":
    unittest.main()
