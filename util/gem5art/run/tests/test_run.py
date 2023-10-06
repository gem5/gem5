# Copyright (c) 2019, 2021 The Regents of the University of California
# All Rights Reserved.
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
"""Tests for gem5Run object"""
import hashlib
import os
import unittest
from pathlib import Path
from uuid import uuid4

from gem5art.artifact import artifact
from gem5art.run import gem5Run


class TestSERun(unittest.TestCase):
    def setUp(self):
        self.gem5art = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "test-gem5",
                "type": "test-binary",
                "documentation": "This is a description of gem5 artifact",
                "command": "scons build/X86/gem5.opt",
                "path": "gem5/build/X86/gem5.opt",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            },
        )

        self.gem5gitart = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "test-gem5-git",
                "type": "test-git",
                "documentation": "This is a description of gem5 git artifact",
                "command": "git clone something",
                "path": "/",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            },
        )

        self.runscptart = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "test-runscript",
                "type": "test-git",
                "documentation": "This is a description of runscript aritfact",
                "command": "git clone something",
                "path": "/",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            },
        )

        self.run = gem5Run.createSERun(
            "test SE run",
            "configs-tests/run_test.py",
            "results/run_test/out",
            self.gem5art,
            self.gem5gitart,
            self.runscptart,
            "extra",
            "params",
        )

    def test_out_dir(self):
        relative_outdir = "results/run_test/out"
        self.assertEqual(
            self.run.outdir.relative_to(Path(".").resolve()),
            Path(relative_outdir),
        )

        self.assertTrue(
            self.run.outdir.is_absolute(),
            "outdir should be absolute directory",
        )

    def test_command(self):
        self.assertEqual(
            self.run.command,
            [
                "gem5/build/X86/gem5.opt",
                "-re",
                f"--outdir={os.path.abspath('results/run_test/out')}",
                "configs-tests/run_test.py",
                "extra",
                "params",
            ],
        )


if __name__ == "__main__":
    unittest.main()
