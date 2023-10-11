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

import unittest
import tempfile
import os
from typing import Dict
import json

from gem5.resources.client_api.jsonclient import JSONClient


class JSONClientTestSuite(unittest.TestCase):
    """Test cases for gem5.resources.client_api.jsonclient"""

    @classmethod
    def setUpClass(cls) -> str:
        """
        This creates a simple resources collection for testing
        """
        file_contents = [
            {
                "category": "binary",
                "id": "this-is-a-test-resource",
                "description": "This is a test resource",
                "architecture": "X86",
                "size": 13816,
                "tags": ["asmtest", "testing", "riscv", "testing"],
                "is_zipped": False,
                "md5sum": "4e70a98b6976969deffff91eed17fba1",
                "source": "src/asmtest",
                "url": "http://dist.gem5.org/dist/develop/test-progs/asmtest/bin/rv64mi-p-sbreak",
                "code_examples": [],
                "license": " BSD-3-Clause",
                "author": [],
                "source_url": "https://github.com/gem5/gem5-resources/tree/develop/src/asmtest",
                "resource_version": "1.0.0",
                "gem5_versions": ["23.0"],
                "example_usage": 'get_resource(resource_name="rv64mi-p-sbreak")',
            },
            {
                "category": "binary",
                "id": "this-is-a-test-resource",
                "description": "This is a test resource but double newer",
                "architecture": "X86",
                "size": 13816,
                "tags": ["asmtest"],
                "is_zipped": False,
                "md5sum": "4e70a98b6976969deffff91eed17fba1",
                "source": "src/asmtest",
                "url": "http://dist.gem5.org/dist/develop/test-progs/asmtest/bin/rv64mi-p-sbreak",
                "code_examples": [],
                "license": " BSD-3-Clause",
                "author": [],
                "source_url": "https://github.com/gem5/gem5-resources/tree/develop/src/asmtest",
                "resource_version": "2.0.0",
                "gem5_versions": ["23.1"],
                "example_usage": 'get_resource(resource_name="rv64mi-p-sbreak")',
            },
            {
                "category": "simpoint",
                "id": "test-version",
                "description": "Simpoints for running the 'x86-print-this' resource with the parameters `\"print this\" 15000`. This is encapsulated in the 'x86-print-this-15000-with-simpoints' workload.",
                "architecture": "X86",
                "size": 10240,
                "tags": [],
                "is_zipped": False,
                "md5sum": "3fcffe3956c8a95e3fb82e232e2b41fb",
                "is_tar_archive": True,
                "url": "http://dist.gem5.org/dist/develop/simpoints/x86-print-this-15000-simpoints-20221013.tar",
                "simpoint_interval": 1000000,
                "warmup_interval": 1000000,
                "code_examples": [],
                "license": "",
                "author": [],
                "source_url": "",
                "resource_version": "1.0.0",
                "gem5_versions": ["23.0"],
                "workload_name": "x86-print-this-15000-with-simpoints",
                "example_usage": 'get_resource(resource_name="x86-print-this-1500-simpoints")',
                "workloads": [
                    "x86-print-this-15000-with-simpoints",
                    "x86-print-this-15000-with-simpoints-and-checkpoint",
                ],
            },
            {
                "category": "file",
                "id": "test-version",
                "description": "Simpoints for running the 'x86-print-this' resource with the parameters `\"print this\" 15000`. This is encapsulated in the 'x86-print-this-15000-with-simpoints' workload.",
                "architecture": "X86",
                "size": 10240,
                "tags": [],
                "is_zipped": False,
                "md5sum": "3fcffe3956c8a95e3fb82e232e2b41fb",
                "is_tar_archive": True,
                "url": "http://dist.gem5.org/dist/develop/simpoints/x86-print-this-15000-simpoints-20221013.tar",
                "simpoint_interval": 1000000,
                "warmup_interval": 1000000,
                "code_examples": [],
                "license": "",
                "author": [],
                "source_url": "",
                "resource_version": "0.2.0",
                "gem5_versions": ["23.0"],
                "workload_name": "x86-print-this-15000-with-simpoints",
                "example_usage": 'get_resource(resource_name="x86-print-this-1500-simpoints")',
                "workloads": [
                    "x86-print-this-15000-with-simpoints",
                    "x86-print-this-15000-with-simpoints-and-checkpoint",
                ],
            },
        ]
        file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        file.write(json.dumps(file_contents))
        file.close()
        cls.file_path = file.name

    @classmethod
    def tearDownClass(cls) -> None:
        os.remove(cls.file_path)

    def verify_json(self, json: Dict) -> None:
        """
        This verifies the JSON file created in created in
        "create_temp_resources_json" has been loaded correctly into a Python
        dictionary.
        """
        self.assertEqual(4, len(json))
        self.assertTrue("id" in json[0])
        self.assertEqual("this-is-a-test-resource", json[0]["id"])
        self.assertEqual("binary", json[0]["category"])
        self.assertTrue("id" in json[1])
        self.assertEqual("this-is-a-test-resource", json[1]["id"])
        self.assertTrue("id" in json[2])
        self.assertEqual("test-version", json[2]["id"])
        self.assertTrue("id" in json[3])
        self.assertEqual("test-version", json[3]["id"])

    def test_get_resources_json_at_path(self) -> None:
        # Tests JSONClient.get_resources_json()

        client = JSONClient(path=self.file_path)
        json = client.get_resources_json()
        self.verify_json(json=json)

    def test_get_resources_json_invalid_url(self) -> None:
        # Tests the JSONClient.get_resources_json() function in case where an
        # invalid url is passed as the URL/PATH of the resources JSON file.

        path = "NotAURLorFilePath"
        with self.assertRaises(Exception) as context:
            client = JSONClient(path=path)
            json = client.get_resources_json()

        self.assertTrue(
            f"Resources location '{path}' is not a valid path or URL."
            in str(context.exception)
        )
