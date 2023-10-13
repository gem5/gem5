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

"""Tests for the Artifact object and associated functions"""

import hashlib
from pathlib import Path
import unittest
from uuid import uuid4, UUID
import sys
import io

from gem5art import artifact
from gem5art.artifact._artifactdb import ArtifactDB, getDBConnection


class MockDB(ArtifactDB):
    """
    This is a Mock DB,
    used to run unit tests
    """

    def __init__(self, uri=""):
        self.db = {}
        self.hashes = {}

    def put(self, key, metadata):
        self.db[key] = metadata
        self.hashes[metadata["hash"]] = key

    def __contains__(self, key):
        if isinstance(key, UUID):
            return key in self.db.keys()
        else:
            # This is a hash
            return key in self.hashes

    def get(self, key):
        if isinstance(key, UUID):
            return self.db[key]
        else:
            # This is a hash
            return self.db[self.hashes[key]]

    def upload(self, key, path):
        pass

    def downloadFile(self, key, path):
        pass


# Add the MockDB as a scheme
artifact._artifactdb._db_schemes["mockdb"] = MockDB

# This needs to be a global variable so
# that this getDBConnection is the first
# call to create a DB connection
_db = getDBConnection("mockdb://")


class TestGit(unittest.TestCase):
    def test_keys(self):
        git = artifact.artifact.getGit(Path("."))
        self.assertSetEqual(
            set(git.keys()), {"origin", "hash", "name"}, "git keys wrong"
        )

    def test_origin(self):
        git = artifact.artifact.getGit(Path("."))
        self.assertTrue(
            git["origin"].endswith("gem5"), "Origin should end with gem5art"
        )


class TestArtifact(unittest.TestCase):
    def setUp(self):
        self.artifact = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "test-name",
                "type": "test-type",
                "documentation": (
                    "This is a long test documentation that has "
                    "lots of words"
                ),
                "command": ["ls", "-l"],
                "path": "/",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            }
        )

    def test_dirs(self):
        self.assertTrue(self.artifact.cwd.exists())
        self.assertTrue(self.artifact.path.exists())


class TestArtifactSimilarity(unittest.TestCase):
    def setUp(self):
        self.artifactA = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "artifact-A",
                "type": "type-A",
                "documentation": "This is a description of artifact A",
                "command": ["ls", "-l"],
                "path": "/",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            }
        )

        self.artifactB = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "artifact-B",
                "type": "type-B",
                "documentation": "This is a description of artifact B",
                "command": ["ls", "-l"],
                "path": "/",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            }
        )

        self.artifactC = artifact.Artifact(
            {
                "_id": self.artifactA._id,
                "name": "artifact-A",
                "type": "type-A",
                "documentation": "This is a description of artifact A",
                "command": ["ls", "-l"],
                "path": "/",
                "hash": self.artifactA.hash,
                "git": artifact.artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            }
        )

        self.artifactD = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "artifact-A",
                "type": "type-A",
                "documentation": "This is a description of artifact A",
                "command": ["ls", "-l"],
                "path": "/",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            }
        )

    def test_not_equal(self):
        self.assertTrue(self.artifactA != self.artifactB)

    def test_equal(self):
        self.assertTrue(self.artifactA == self.artifactC)

    def test_not_similar(self):
        capturedOutput = io.StringIO()
        sys.stdout = capturedOutput
        self.artifactA._checkSimilar(self.artifactB)
        sys.stdout = sys.__stdout__
        self.assertTrue("WARNING:" in capturedOutput.getvalue())

    def test_similar(self):
        capturedOutput = io.StringIO()
        sys.stdout = capturedOutput
        self.artifactA._checkSimilar(self.artifactD)
        sys.stdout = sys.__stdout__
        self.assertFalse("WARNING:" in capturedOutput.getvalue())


class TestRegisterArtifact(unittest.TestCase):
    def setUp(self):
        # Create and register an artifact
        self.testArtifactA = artifact.Artifact.registerArtifact(
            name="artifact-A",
            typ="type-A",
            documentation="This is a description of artifact A",
            command="ls -l",
            path="./",
            cwd="./",
        )

        # Create an artifact without pushing it to the database
        self.testArtifactB = artifact.Artifact(
            {
                "_id": uuid4(),
                "name": "artifact-B",
                "type": "type-B",
                "documentation": "This is a description of artifact B",
                "command": ["vim test_artifact.py"],
                "path": "./tests/test_artifact.py",
                "hash": hashlib.md5().hexdigest(),
                "git": artifact.artifact.getGit(Path(".")),
                "cwd": "/",
                "inputs": [],
            }
        )

    # test to see if an artifact is in the database
    def test_in_database(self):
        self.assertTrue(self.testArtifactA.hash in _db)
        self.assertFalse(self.testArtifactB.hash in _db)


if __name__ == "__main__":
    unittest.main()
