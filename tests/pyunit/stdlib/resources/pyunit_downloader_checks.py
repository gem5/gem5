# Copyright (c) 2022 The Regents of the University of California
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
    _get_resources_json_at_path,
    _get_resources_json,
    _resources_json_version_required,
)


class ResourceDownloaderTestSuite(unittest.TestCase):
    """Test cases for gem5.resources.downloader"""

    @classmethod
    def setUpClass(cls) -> str:
        """
        This creates a simple resource.json temp file for testing purposes.
        """

        file_contents = (
            "{"
            + f'"version" : "{_resources_json_version_required()}",'
            + """
    "url_base" : "http://dist.gem5.org/dist/v21-2",
    "previous-versions" : {},
    "resources": [
        {
            "type": "resource",
            "name" : "riscv-disk-img",
            "documentation" : "A simple RISCV disk image based on busybox.",
            "architecture": "RISCV",
            "is_zipped" : true,
            "md5sum" : "d6126db9f6bed7774518ae25aa35f153",
            "url": "{url_base}/images/riscv/busybox/riscv-disk.img.gz",
            "source" : "src/riscv-fs",
            "additional_metadata" : {
                "root_partition": null
            }
        },
        {
            "type": "resource",
            "name" : "riscv-lupio-busybox-img",
            "documentation" : "A RISCV disk image, based on busybox, to ...",
            "architecture": "RISCV",
            "is_zipped" : true,
            "md5sum" : "e5bee8a31f45f4803f87c0d781553ccc",
            "url": "{url_base}/images/riscv/busybox/riscv-lupio-busybox.img",
            "source" : "src/lupv",
            "additional_metadata" : {
                "root_partition": "1"
            }
        }
    ]
}
        """
        )
        file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        file.write(file_contents)
        file.close()
        cls.file_path = file.name

        os.environ["GEM5_RESOURCE_JSON"] = cls.file_path

    @classmethod
    def tearDownClass(cls) -> None:
        os.remove(cls.file_path)
        del os.environ["GEM5_RESOURCE_JSON"]

    def verify_json(self, json: Dict) -> None:
        """
        This verifies the JSON file created in created in
        "create_temp_resources_json" has been loaded correctly into a Python
        dictionary.
        """
        self.assertTrue("resources" in json)
        self.assertEquals(2, len(json["resources"]))
        self.assertTrue("name" in json["resources"][0])
        self.assertEquals("riscv-disk-img", json["resources"][0]["name"])
        self.assertTrue("name" in json["resources"][1])
        self.assertEquals(
            "riscv-lupio-busybox-img", json["resources"][1]["name"]
        )

    def test_get_resources_json_at_path(self) -> None:
        # Tests the gem5.resources.downloader._get_resources_json_at_path()
        # function.

        json = _get_resources_json_at_path(path=self.file_path)
        self.verify_json(json=json)

    def test_get_resources_json(self) -> None:
        # Tests the gem5.resources.downloader._get_resources_json() function.

        json = _get_resources_json()
        self.verify_json(json=json)

    def test_get_resources_json_invalid_url(self) -> None:
        # Tests the gem5.resources.downloader._get_resources_json() function in
        # case where an invalid url is passed as the URL/PATH of the
        # resources.json file.

        path = "NotAURLorFilePath"
        os.environ["GEM5_RESOURCE_JSON"] = path
        with self.assertRaises(Exception) as context:
            _get_resources_json()

        self.assertTrue(
            f"Resources location '{path}' is not a valid path or URL."
            in str(context.exception)
        )

        # Set back to the old path
        os.environ["GEM5_RESOURCE_JSON"] = self.file_path
