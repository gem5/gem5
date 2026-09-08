#!/usr/bin/env python3
# Copyright (c) 2023 The Regents of the University of California
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

"""Controller failure-path tests; no Docker or GitHub writes are performed."""

import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


class ControllerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        shutil.copy(Path(__file__).with_name("action-run.sh"), self.root)
        (self.root / "runner-state").mkdir()
        (self.root / "bin").mkdir()
        self.script("bin/curl", "echo curl >> calls; echo unused")
        self.script("bin/sleep", "true")
        self.script("bin/jq", "cat >/dev/null; echo test-token")
        self.script("config.sh", "echo config >> calls")
        self.script("run.sh", "echo run >> calls; touch runner-state/drain")
        self.script("runner-health.sh", "echo health >> calls")
        self.script("runner-cleanup.py", "echo cleanup >> calls")

    def script(self, name, body):
        path = self.root / name
        path.write_text("#!/bin/bash\nset -eu\n" + body + "\n")
        path.chmod(0o755)

    def run_controller(self):
        env = dict(
            os.environ,
            PERSONAL_ACCESS_TOKEN="dummy",
            GITHUB_ORG="test",
            PATH=f"{self.root}/bin:" + os.environ["PATH"],
        )
        return subprocess.run(
            ["bash", str(self.root / "action-run.sh")],
            env=env,
            capture_output=True,
            text=True,
            timeout=10,
        )

    def calls(self):
        path = self.root / "calls"
        return path.read_text().splitlines() if path.exists() else []

    def test_failed_health_never_registers(self):
        self.script("runner-health.sh", "exit 1")
        result = self.run_controller()
        self.assertEqual(result.returncode, 78, result.stderr)
        self.assertEqual(self.calls(), [])
        self.assertTrue((self.root / "runner-state/quarantine").exists())

    def test_drain_completes_job_then_cleanup(self):
        result = self.run_controller()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(
            self.calls(),
            ["health", "curl", "config", "run", "cleanup", "health"],
        )
        self.assertIn(
            "drained", (self.root / "runner-state/status").read_text()
        )

    def test_cleanup_failure_quarantines(self):
        self.script("runner-cleanup.py", "echo cleanup >> calls; exit 1")
        result = self.run_controller()
        self.assertEqual(result.returncode, 78, result.stderr)
        self.assertEqual(self.calls().count("config"), 1)
        self.assertEqual(self.calls().count("health"), 1)

    def test_post_cleanup_health_failure_quarantines(self):
        self.script("runner-health.sh", "test ! -e ran")
        self.script("run.sh", "touch ran runner-state/drain")
        result = self.run_controller()
        self.assertEqual(result.returncode, 78, result.stderr)
        self.assertEqual(self.calls().count("config"), 1)

    def test_failed_configuration_never_runs(self):
        self.script("config.sh", "exit 1")
        result = self.run_controller()
        self.assertEqual(result.returncode, 78, result.stderr)
        self.assertNotIn("run", self.calls())

    def test_existing_quarantine_stays_offline(self):
        (self.root / "runner-state/quarantine").touch()
        self.assertEqual(self.run_controller().returncode, 78)
        self.assertEqual(self.calls(), [])

    def test_existing_drain_stays_offline(self):
        (self.root / "runner-state/drain").touch()
        self.assertEqual(self.run_controller().returncode, 0)
        self.assertEqual(self.calls(), [])

    def test_pat_is_not_in_job_environment(self):
        self.script(
            "run.sh",
            'test -z "${PERSONAL_ACCESS_TOKEN:-}"; '
            "touch runner-state/drain",
        )
        self.assertEqual(self.run_controller().returncode, 0)

    def test_lock_prevents_a_second_controller(self):
        import fcntl

        with (self.root / "runner-state/lock").open("w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertEqual(self.run_controller().returncode, 0)
        self.assertEqual(self.calls(), [])


class CleanupTest(unittest.TestCase):
    def setUp(self):
        import importlib.util
        from unittest.mock import patch

        spec = importlib.util.spec_from_file_location(
            "runner_cleanup", Path(__file__).with_name("runner-cleanup.py")
        )
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.images = [
            {
                "Id": "health",
                "RepoTags": ["ubuntu:24.04"],
                "Size": 1,
                "Created": "2026-01-01",
            },
            {
                "Id": "gem5",
                "RepoTags": ["ghcr.io/gem5/test:latest"],
                "Size": 2 * 1024**3,
                "Created": "2026-02-01",
            },
            {
                "Id": "other",
                "RepoTags": ["unrelated:latest"],
                "Size": 1,
                "Created": "2026-03-01",
            },
        ]
        self.commands = []
        self.worker = patch.object(self.module.subprocess, "run")
        self.run = self.worker.start()
        self.run.return_value.returncode = 1
        self.addCleanup(self.worker.stop)
        for obj, attr, value in [
            (self.module.os, "chdir", lambda *args: None),
            (self.module, "docker", self.docker),
        ]:
            mocker = patch.object(obj, attr, value)
            mocker.start()
            self.addCleanup(mocker.stop)
        self.disk = patch.object(self.module.shutil, "disk_usage")
        self.usage = self.disk.start()
        self.usage.return_value.free = 100 * 1024**3
        self.addCleanup(self.disk.stop)
        self.env = patch.dict(
            os.environ, RUNNER_IMAGE_CACHE_GIB="32", RUNNER_MIN_FREE_GIB="20"
        )
        self.env.start()
        self.addCleanup(self.env.stop)
        self.logs = patch.object(self.module.Path, "glob", return_value=[])
        self.logs.start()
        self.addCleanup(self.logs.stop)

    def docker(self, *args):
        import json

        self.commands.append(args)
        if args == ("ps", "-aq"):
            return "container"
        if args == ("image", "ls", "-aq"):
            return "health gem5 other"
        if args[:3] == ("image", "inspect", "--format"):
            return "health"
        if args[:2] == ("image", "inspect"):
            return json.dumps(self.images)
        return ""

    def removed(self):
        return [cmd[-1] for cmd in self.commands if cmd[:2] == ("image", "rm")]

    def test_active_worker_prevents_every_mutation(self):
        self.run.return_value.returncode = 0
        with self.assertRaises(RuntimeError):
            self.module.cleanup()
        self.assertEqual(self.commands, [])
        self.assertEqual(self.run.call_count, 1)
        self.assertEqual(self.run.call_args.args[0][0], "pgrep")

    def test_retains_common_and_health_images(self):
        self.module.cleanup()
        self.assertEqual(self.removed(), ["other"])
        self.assertIn(("rm", "-f", "container"), self.commands)
        self.assertIn(("volume", "prune", "--all", "--force"), self.commands)

    def test_unavailable_worker_state_prevents_cleanup(self):
        self.run.return_value.returncode = 2
        with self.assertRaises(RuntimeError):
            self.module.cleanup()
        self.assertEqual(self.commands, [])

    def test_byte_cap_evicts_common_image_but_keeps_health(self):
        os.environ["RUNNER_IMAGE_CACHE_GIB"] = "1"
        self.module.cleanup()
        self.assertEqual(self.removed(), ["other", "gem5"])

    def test_free_space_floor_evicts_images(self):
        self.usage.return_value.free = 1024**3
        self.module.cleanup()
        self.assertEqual(self.removed(), ["other", "gem5"])

    def test_cleanup_error_does_not_continue_to_images(self):
        self.run.side_effect = [
            type("Result", (), {"returncode": 1})(),
            subprocess.CalledProcessError(1, "rm"),
        ]
        with self.assertRaises(subprocess.CalledProcessError):
            self.module.cleanup()
        self.assertEqual(self.commands, [])


class MigrationTest(unittest.TestCase):
    def setUp(self):
        import importlib.util
        from unittest.mock import patch

        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.home = Path(self.temp.name) / "home"
        self.stage = Path(self.temp.name) / "stage"
        self.home.mkdir()
        self.stage.mkdir()
        spec = importlib.util.spec_from_file_location(
            "migrate_runner", Path(__file__).with_name("migrate-runner.py")
        )
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.module.__file__ = str(self.stage / "migrate-runner.py")
        for name in self.module.FILES:
            (self.stage / name).write_text("new")
            (self.home / name).write_text("old")
        self.args = ["/bin/bash", "./action-run.sh", "dummy", "test"]
        self.parent = (1234, self.args, 1, "T")
        self.child = (1235, ["Runner.Worker"], 1234, "S")
        self.mockers = []
        for obj, attr in [
            (self.module, "processes"),
            (self.module.subprocess, "run"),
            (self.module.os, "kill"),
            (self.module.time, "sleep"),
            (self.module.signal, "signal"),
            (self.module.Path, "home"),
        ]:
            mocker = patch.object(obj, attr)
            setattr(self, attr, mocker.start())
            self.addCleanup(mocker.stop)
        self.home.return_value = Path(self.temp.name) / "home"
        self.run.return_value.returncode = 1
        self.processes.side_effect = [
            [self.parent],
            [self.parent, self.child],
            [self.parent],
        ]

    def test_waits_for_job_then_cleans_before_starting(self):
        self.module.main()
        self.sleep.assert_called_once_with(10)
        signals = [call.args for call in self.kill.call_args_list]
        self.assertEqual(
            signals,
            [
                (1234, self.module.signal.SIGSTOP),
                (1234, self.module.signal.SIGKILL),
            ],
        )
        commands = [call.args[0] for call in self.run.call_args_list]
        self.assertTrue(commands[-3][-1].endswith("runner-cleanup.py"))
        self.assertTrue(commands[-2][-1].endswith("runner-health.sh"))
        self.assertTrue(commands[-1][-1].endswith("install-runner-service.sh"))

    def test_interrupted_drain_resumes_parent(self):
        self.sleep.side_effect = InterruptedError("test interruption")
        with self.assertRaises(InterruptedError):
            self.module.main()
        self.kill.assert_any_call(1234, self.module.signal.SIGCONT)
        self.assertNotIn(
            (1234, self.module.signal.SIGKILL),
            [call.args for call in self.kill.call_args_list],
        )
        self.assertEqual(self.run.call_count, 1)

    def test_changed_parent_is_not_killed_or_resumed(self):
        other = (1234, ["unrelated"], 1, "S")
        self.processes.side_effect = [[self.parent], [other], [other]]
        with self.assertRaises(RuntimeError):
            self.module.main()
        self.kill.assert_called_once_with(1234, self.module.signal.SIGSTOP)

    def test_ambiguous_legacy_loops_are_left_untouched(self):
        self.processes.side_effect = [[self.parent, self.parent]]
        with self.assertRaises(RuntimeError):
            self.module.main()
        self.kill.assert_not_called()


if __name__ == "__main__":
    unittest.main()
