# Copyright (c) 2021 The Regents of the University of California
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

"""Tests for ArtifactFileDB"""


import json
import os
from pathlib import Path
import unittest
from uuid import UUID

from gem5art.artifact import Artifact
from gem5art.artifact._artifactdb import getDBConnection


class TestArtifactFileDB(unittest.TestCase):
    def setUp(self):
        _db = getDBConnection("file://test.json")

        with open("test-file.txt", "w") as f:
            f.write("This is a test file.")

        self.artifact = Artifact.registerArtifact(
            name=f"test-artifact",
            typ="text",
            path=f"test-file.txt",
            cwd="./",
            command='echo "This is a test file" > test-file.txt',
            inputs=[],
            documentation=f"This artifact is made for testing.",
        )

    def tearDown(self):
        os.remove("test-file.txt")
        os.remove("test.json")

    def test_init_function(self):
        self.assertTrue(Path("test.json").exists())

    def test_json_content(self):
        with open("test.json") as f:
            artifacts = json.load(f)
        self.assertTrue(len(artifacts) == 1)
        artifact = artifacts[0]
        self.assertTrue(artifact["hash"] == self.artifact.hash)
        self.assertTrue(UUID(artifact["_id"]) == self.artifact._id)
