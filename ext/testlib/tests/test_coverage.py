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
            self.assertEqual(files["main.c"]["4"], int(argument == "a"))
            self.assertEqual(files["main.c"]["7"], int(argument == "b"))
            self.assertEqual(files["unused.c"]["1"], 0)
            directory = self.build.output / record["invocation_id"]
            counters = list(directory.rglob("*.gcda"))
            self.assertTrue(counters)
            self.assertTrue(
                all(p.with_suffix(".gcno").exists() for p in counters)
            )
        self.assertFalse(list(self.target.rglob("*.gcda")))
        self.assertIn("gcc_versions", records[0]["build"])

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
        self.assertTrue(record["files"])

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
