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

"""Check gem5 module semantics and fork isolation without a gem5 build."""

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

MODULE = Path(__file__).resolve().parents[1] / "coverage.py"
spec = importlib.util.spec_from_file_location(
    "testlib_module_coverage", MODULE
)
collector = importlib.util.module_from_spec(spec)
spec.loader.exec_module(collector)
ROOT = MODULE.parents[2]


class PythonModuleCoverageTest(unittest.TestCase):
    def setUp(self):
        if importlib.util.find_spec("coverage") is None:
            self.skipTest("coverage.py is required for Python tracing tests")
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.stub = self.root / "gem5_stub.py"
        self.stub.write_text(
            "import importlib.util,json,os,runpy,sys,types\n"
            f"root={str(ROOT)!r}\n"
            "def load(name):\n"
            "    spec=importlib.util.spec_from_file_location('_fixture_'+name,root+'/src/python/m5/'+name+'.py')\n"
            "    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module\n"
            "options_module=load('options');main=load('main')\n"
            "sys.modules['_fixture']=types.ModuleType('_fixture')\n"
            "sys.modules['_fixture.options']=options_module\n"
            "main.__package__='_fixture'\n"
            "options,args=main.parse_options()\n"
            "sys.modules['m5']=types.SimpleNamespace(options=options)\n"
            "sys.path[0:0]=options.path\n"
            "if options.m:\n"
            "    sys.argv=[options.m[0]]+options.m[1]\n"
            "    runpy.run_module(options.m[0],run_name='__m5_main__')\n"
            "else:\n"
            "    sys.argv=args\n"
            "    if not options.P: sys.path.insert(0,os.path.dirname(args[0]))\n"
            "    exec(compile(open(args[0]).read(),args[0],'exec'),{'__name__':'__m5_main__','__file__':args[0]})\n"
        )
        package = self.root / "package"
        package.mkdir()
        (package / "__init__.py").write_text("")
        (package / "sibling.py").write_text("value=42\n")
        (package / "__main__.py").write_text(
            "import json,m5,sys\nfrom .sibling import value\n"
            "print(json.dumps({'argv':sys.argv,'path':sys.path,'options_m':m5.options.m,"
            "'name':__name__,'package':__package__,'spec':__spec__.name,"
            "'file':__file__,'cached':__cached__,'sibling':value,"
            "'outdir':m5.options.outdir,'quiet':m5.options.quiet,"
            "'registered':'__m5_main__' in sys.modules}))\n"
            "if 'fail' in sys.argv: raise SystemExit(7)\n"
        )

    def invocation(self):
        build = collector.CoverageBuild(
            self.root,
            self.root / "build/NULL",
            self.root / "build/NULL/gem5.opt",
            self.root / "results",
            python_coverage=True,
        )
        return build.invocation(
            "SuiteUID:tests/gem5/multisim/test.py:module", None
        )

    def test_module_globals_argv_search_path_and_failure_match_gem5(self):
        for safe in (False, True):
            for argument in ("success", "fail"):
                with self.subTest(safe=safe, argument=argument):
                    prefix = [sys.executable, str(self.stub)] + (
                        ["-P"] if safe else []
                    )
                    command = prefix + [
                        "-m",
                        "package",
                        "config.py",
                        argument,
                        "space argument",
                        "-i",
                    ]
                    normal = subprocess.run(
                        command, cwd=self.root, text=True, capture_output=True
                    )
                    invocation = self.invocation()
                    wrapped = [
                        sys.executable,
                        *invocation.python_command(
                            command[1:], len(prefix) + 1
                        ),
                    ]
                    measured = subprocess.run(
                        wrapped, cwd=self.root, text=True, capture_output=True
                    )
                    self.assertEqual(
                        measured.returncode, normal.returncode, measured.stderr
                    )
                    self.assertEqual(
                        json.loads(measured.stdout), json.loads(normal.stdout)
                    )
                    record = json.loads(invocation.python_path.read_text())
                    self.assertEqual(record["collection"], "complete", record)
                    self.assertEqual(
                        record["outcome"],
                        "failed" if argument == "fail" else "passed",
                    )
                    self.assertEqual(
                        record["build"]["scope"], "module-process"
                    )
                    self.assertTrue(record["exclusions"])
                    self.assertIn(
                        "package/__main__.py",
                        {entry["path"] for entry in record["files"]},
                    )

    def test_attached_grouped_and_option_value_boundaries_match_real_parser(
        self,
    ):
        for options in (
            ["-mpackage"],
            ["-Pm", "package"],
            ["-Pmpackage"],
            ["-dmyout", "-mpackage"],
            ["-d-module-directory", "-mpackage"],
            ["--outdir", "-module-directory", "-mpackage"],
            ["--outdir=-module-directory", "-Pm", "package"],
            ["-p", str(self.root), "-qPm", "package"],
        ):
            with self.subTest(options=options):
                command = [
                    str(self.stub),
                    *options,
                    "config.py",
                    "-i",
                    "space argument",
                ]
                normal = subprocess.run(
                    [sys.executable, *command],
                    cwd=self.root,
                    text=True,
                    capture_output=True,
                )
                invocation = self.invocation()
                wrapped = invocation.python_command(command, 1 + len(options))
                measured = subprocess.run(
                    [sys.executable, *wrapped],
                    cwd=self.root,
                    text=True,
                    capture_output=True,
                )
                self.assertEqual(
                    measured.returncode, normal.returncode, measured.stderr
                )
                self.assertEqual(normal.returncode, 0, normal.stderr)
                self.assertEqual(
                    json.loads(measured.stdout), json.loads(normal.stdout)
                )
                data = json.loads(invocation.python_path.read_text())
                self.assertEqual(data["collection"], "complete", data)
                self.assertEqual(data["entry_mode"], "module")

    def test_normalization_never_executes_help_or_stats_callbacks(self):
        for option in ("--help", "--stats-help"):
            prefix, arguments, mode = collector._python_entry(
                ["gem5", option, "config.py"]
            )
            self.assertEqual(prefix, ["gem5", option])
            self.assertEqual(arguments, ["config.py"])
            self.assertEqual(mode, "file")
        prefix, arguments, mode = collector._python_entry(
            ["gem5", "--outdir", "-m", "config.py", "-mpackage"]
        )
        self.assertEqual(prefix, ["gem5", "--outdir", "-m"])
        self.assertEqual(arguments, ["config.py", "-mpackage"])
        self.assertEqual(mode, "file")

    def test_file_config_behavior_is_unchanged(self):
        script = self.root / "config.py"
        script.write_text(
            "import json,sys\n"
            "print(json.dumps({'name':__name__, 'file':__file__, 'argv':sys.argv, "
            "'path':sys.path, 'has_spec':'__spec__' in globals()}))\n"
        )
        for safe in (False, True):
            with self.subTest(safe=safe):
                prefix = [sys.executable, str(self.stub)] + (
                    ["-P"] if safe else []
                )
                command = prefix + [str(script), "argument"]
                normal = subprocess.run(
                    command,
                    cwd=self.root,
                    text=True,
                    capture_output=True,
                    check=True,
                )
                invocation = self.invocation()
                wrapped = [
                    sys.executable,
                    *invocation.python_command(command[1:], len(prefix) - 1),
                ]
                measured = subprocess.run(
                    wrapped,
                    cwd=self.root,
                    text=True,
                    capture_output=True,
                    check=True,
                )
                self.assertEqual(
                    json.loads(measured.stdout), json.loads(normal.stdout)
                )
                self.assertEqual(
                    json.loads(invocation.python_path.read_text())[
                        "collection"
                    ],
                    "complete",
                )

    @unittest.skipUnless(hasattr(os, "fork"), "fork isolation needs POSIX")
    def test_forked_module_cannot_replace_parent_record_or_database(self):
        (self.root / "package/__main__.py").write_text(
            "import os,sys,time\n"
            "child=os.fork()\n"
            "if child==0:\n"
            "    child_only=123\n"
            "    raise SystemExit(9)\n"
            "os.waitpid(child,0)\n"
            "parent_only=456\n"
        )
        invocation = self.invocation()
        command = [
            sys.executable,
            str(self.stub),
            "-m",
            "package",
            "config.py",
        ]
        measured = subprocess.run(
            [sys.executable, *invocation.python_command(command[1:], 3)],
            cwd=self.root,
            text=True,
            capture_output=True,
        )
        self.assertEqual(measured.returncode, 0, measured.stderr)
        record = json.loads(invocation.python_path.read_text())
        self.assertEqual(record["outcome"], "passed")
        self.assertEqual(record["collection"], "complete", record)
        entry = next(
            entry
            for entry in record["files"]
            if entry["path"] == "package/__main__.py"
        )
        self.assertEqual(entry["lines"]["4"], 0)
        self.assertEqual(entry["lines"]["7"], 1)

    def test_real_testlib_module_suites_remain_selected(self):
        process = subprocess.run(
            [
                sys.executable,
                str(ROOT / "tests/main.py"),
                "list",
                "gem5/multisim",
                "--length=quick",
                "--isa=ALL",
                "--variant=opt",
                "-q",
                "--suites",
                "--gcov=per-test",
                "--python-coverage",
            ],
            cwd=ROOT / "tests",
            text=True,
            capture_output=True,
        )
        self.assertEqual(process.returncode, 0, process.stderr)
        suites = [
            line
            for line in process.stdout.splitlines()
            if line.startswith("SuiteUID:")
        ]
        self.assertGreaterEqual(len(suites), 3, process.stdout)
        ordinary = subprocess.run(
            [
                sys.executable,
                str(ROOT / "tests/main.py"),
                "list",
                "gem5/multisim",
                "--length=quick",
                "--isa=ALL",
                "--variant=opt",
                "-q",
                "--suites",
            ],
            cwd=ROOT / "tests",
            text=True,
            capture_output=True,
            check=True,
        )
        self.assertEqual(
            suites,
            [
                line
                for line in ordinary.stdout.splitlines()
                if line.startswith("SuiteUID:")
            ],
        )


if __name__ == "__main__":
    unittest.main()
