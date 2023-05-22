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

from gem5.resources.downloader import (
    get_resources_json_obj,
)


class ResourceDownloadTestSuite(unittest.TestCase):
    """Test cases for gem5.resources.downloader"""

    @classmethod
    def setUpClass(cls) -> str:
        pass

    def get_resource_json_by_id(self) -> None:
        """Get a resource by its id"""
        resources = get_resources_json_obj("test-version")
        self.assertEqual(resources["id"], "test-version")
        self.assertEqual(resources["resource_version"], "2.0.0")

    def get_resource_json_invalid_id(self) -> None:
        """Should throw an exception when trying to get a resource that doesn't exist"""
        with self.assertRaises(Exception) as context:
            get_resources_json_obj("this-resource-doesnt-exist")
        self.assertTrue(
            f"Error: Resource with name 'this-resource-doesnt-exist' does not exist"
            in str(context.exception)
        )

    def get_resource_json_by_id_and_version(self) -> None:
        """Get a resource by its id and version"""
        resources = get_resources_json_obj("test-version", "1.0.0")
        self.assertEqual(resources["id"], "test-version")
        self.assertEqual(resources["resource_version"], "1.0.0")

    def get_resource_json_by_id_and_invalid_version(self) -> None:
        """Get a resource by its id and an invalid version (does not exist)"""
        with self.assertRaises(Exception) as context:
            get_resources_json_obj("test-version", "3.0.0")
        self.assertTrue(
            f"Specified Version 3.0.0 does not exist for the resource 'test-version'."
            in str(context.exception)
        )
