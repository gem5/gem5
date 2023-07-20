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
import os
import io
import contextlib
from pathlib import Path

from gem5.resources.resource import obtain_resource, BinaryResource

from gem5.isas import ISA

from _m5 import core

from gem5.resources.client_api.client_wrapper import ClientWrapper
from unittest.mock import patch

mock_json_path = Path(__file__).parent / "refs/obtain-resource.json"

mock_config_json = {
    "sources": {
        "baba": {
            "url": mock_json_path,
            "isMongo": False,
        }
    },
}


@patch(
    "gem5.resources.client.clientwrapper",
    new=ClientWrapper(mock_config_json),
)
class TestObtainResourcesCheck(unittest.TestCase):
    def get_resource_dir(cls) -> str:
        """To ensure the resources are cached to the same directory as all
        other tests, this function returns the location of the testing
        directories "resources" directory.
        """
        return os.path.join(
            os.path.realpath(os.path.dirname(__file__)),
            os.pardir,
            os.pardir,
            os.pardir,
            "gem5",
            "resources",
        )

    def test_obtain_resources_no_version(self):
        """Test that the resource loader returns latest version compatible with that version of gem5 when no version is specified."""
        gem5Version = core.gem5Version
        resource = obtain_resource(
            resource_id="test-binary-resource",
            resource_directory=self.get_resource_dir(),
            gem5_version="develop",
        )
        self.assertEquals("1.7.0", resource.get_resource_version())
        self.assertIsInstance(resource, BinaryResource)
        self.assertEquals(
            "test description v1.7.0", resource.get_description()
        )
        self.assertEquals("src/test-source", resource.get_source())
        self.assertEquals(ISA.ARM, resource.get_architecture())

    def test_obtain_resources_with_version_compatible(self):
        resource = obtain_resource(
            resource_id="test-binary-resource",
            resource_directory=self.get_resource_dir(),
            resource_version="1.5.0",
            gem5_version="develop",
        )
        self.assertEquals("1.5.0", resource.get_resource_version())
        self.assertIsInstance(resource, BinaryResource)
        self.assertEquals(
            "test description for 1.5.0", resource.get_description()
        )
        self.assertEquals("src/test-source", resource.get_source())
        self.assertEquals(ISA.ARM, resource.get_architecture())

    def test_obtain_resources_with_version_incompatible(self):
        resource = None
        f = io.StringIO()
        with contextlib.redirect_stderr(f):
            resource = obtain_resource(
                resource_id="test-binary-resource",
                resource_directory=self.get_resource_dir(),
                resource_version="1.5.0",
            )

        resource = obtain_resource(
            resource_id="test-binary-resource",
            resource_directory=self.get_resource_dir(),
            resource_version="1.5.0",
            gem5_version="develop",
        )
        self.assertEquals("1.5.0", resource.get_resource_version())
        self.assertIsInstance(resource, BinaryResource)
        self.assertEquals(
            "test description for 1.5.0", resource.get_description()
        )
        self.assertEquals("src/test-source", resource.get_source())
        self.assertEquals(ISA.ARM, resource.get_architecture())

    def test_obtain_resources_no_version_invalid_id(self):
        with self.assertRaises(Exception) as context:
            obtain_resource(
                resource_id="invalid-id",
                resource_directory=self.get_resource_dir(),
                gem5_version="develop",
            )
        self.assertTrue(
            "Resource with ID 'invalid-id' not found."
            in str(context.exception)
        )

    def test_obtain_resources_with_version_invalid_id(self):
        with self.assertRaises(Exception) as context:
            obtain_resource(
                resource_id="invalid-id",
                resource_directory=self.get_resource_dir(),
                resource_version="1.7.0",
                gem5_version="develop",
            )
        self.assertTrue(
            "Resource with ID 'invalid-id' not found."
            in str(context.exception)
        )

    def test_obtain_resources_with_version_invalid_version(self):
        with self.assertRaises(Exception) as context:
            obtain_resource(
                resource_id="test-binary-resource",
                resource_directory=self.get_resource_dir(),
                resource_version="3.0.0",
            )
        self.assertTrue(
            f"Resource test-binary-resource with version '3.0.0'"
            " not found.\nResource versions can be found at: "
            f"https://resources.gem5.org/resources/test-binary-resource/versions"
            in str(context.exception)
        )
