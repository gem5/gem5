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

from gem5.resources.workload import Workload, CustomWorkload
from gem5.resources.resource import (
    BinaryResource,
    DiskImageResource,
    obtain_resource,
)
from gem5.resources.downloader import _resources_json_version_required

from typing import Dict


class CustomWorkloadTestSuite(unittest.TestCase):
    """
    Tests the `gem5.resources.workload.CustomWorkload` class.
    """

    @classmethod
    def setUpClass(cls) -> None:
        file_contents = (
            "{"
            + f'"version" : "{_resources_json_version_required()}",'
            + """
        "url_base" : "http://dist.gem5.org/dist/v22-0",
        "previous-versions" : {},
        "resources": [
        {
            "type" : "binary",
            "name" : "x86-hello64-static",
            "documentation" : "A 'Hello World!' binary.",
            "architecture" : "X86",
            "is_zipped" :  false,
            "md5sum" : "dbf120338b37153e3334603970cebd8c",
            "url" : "{url_base}/test-progs/hello/bin/x86/linux/hello64-static",
            "source" : "src/simple"
        }
    ]
}
        """
        )
        file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        file.write(file_contents)
        file.close()

        cls.test_json = file.name
        os.environ["GEM5_RESOURCE_JSON"] = cls.test_json

        cls.custom_workload = CustomWorkload(
            function="set_se_binary_workload",
            parameters={
                "binary": obtain_resource("x86-hello64-static"),
                "arguments": ["hello", 6],
            },
        )

    @classmethod
    def tearDownClass(cls):
        # Remove the test json file and unset the environment variable so this
        # test does not interfere with others.
        os.remove(cls.test_json)
        os.environ["GEM5_RESOURCE_JSON"]

    def test_get_function_str(self) -> None:
        # Tests `CustomResource.get_function_str`

        self.assertEqual(
            "set_se_binary_workload", self.custom_workload.get_function_str()
        )

    def test_get_parameters(self) -> None:
        # Tests `CustomResource.get_parameter`

        parameters = self.custom_workload.get_parameters()
        self.assertTrue(isinstance(parameters, Dict))
        self.assertEquals(2, len(parameters))

        self.assertTrue("binary" in parameters)
        self.assertTrue(isinstance(parameters["binary"], BinaryResource))

        self.assertTrue("arguments" in parameters)
        self.assertTrue(isinstance(parameters["arguments"], list))
        self.assertEquals(2, len(parameters["arguments"]))
        self.assertEquals("hello", parameters["arguments"][0])
        self.assertEquals(6, parameters["arguments"][1])

    def test_add_parameters(self) -> None:
        # Tests `CustomResource.set_parameter` for the case where we add a new
        # parameter value.

        self.custom_workload.set_parameter("test_param", 10)

        self.assertTrue("test_param" in self.custom_workload.get_parameters())
        self.assertEquals(
            10, self.custom_workload.get_parameters()["test_param"]
        )

        # Cleanup
        del self.custom_workload.get_parameters()["test_param"]

    def test_override_parameter(self) -> None:
        # Tests `CustomResource.set_parameter` for the case where we override
        # a parameter's value.

        old_value = self.custom_workload.get_parameters()["binary"]

        self.custom_workload.set_parameter("binary", "test")
        self.assertTrue("binary" in self.custom_workload.get_parameters())
        self.assertEquals(
            "test", self.custom_workload.get_parameters()["binary"]
        )

        # We set the overridden parameter back to it's old value.
        self.custom_workload.set_parameter("binary", old_value)


class WorkloadTestSuite(unittest.TestCase):
    """
    Tests the `gem5.resources.workload.Workload` class.
    """

    @classmethod
    def setUpClass(cls):
        # In this constructor we create a json file to load then create a test
        # workload.

        file_contents = (
            "{"
            + f'"version" : "{_resources_json_version_required()}",'
            + """
        "url_base" : "http://dist.gem5.org/dist/v22-0",
        "previous-versions" : {},
        "resources": [
        {
            "type" : "kernel",
            "name" : "x86-linux-kernel-5.2.3",
            "documentation" : "The linux kernel (v5.2.3), compiled to X86.",
            "architecture" : "X86",
            "is_zipped" : false,
            "md5sum" : "4838c99b77d33c8307b939c16624e4ac",
            "url" : "{url_base}/kernels/x86/static/vmlinux-5.2.3",
            "source" : "src/linux-kernel"
        },
        {
            "type" : "disk-image",
            "name" : "x86-ubuntu-18.04-img",
            "documentation" : "A disk image containing Ubuntu 18.04 for x86..",
            "architecture" : "X86",
            "is_zipped" : true,
            "md5sum" : "90e363abf0ddf22eefa2c7c5c9391c49",
            "url" : "{url_base}/images/x86/ubuntu-18-04/x86-ubuntu.img.gz",
            "source" : "src/x86-ubuntu",
            "root_partition": "1"
        },
        {
            "type" : "workload",
            "name" : "simple-boot",
            "documentation" : "Description of workload here",
            "function" : "set_kernel_disk_workload",
            "resources" : {
                "kernel" : "x86-linux-kernel-5.2.3",
                "disk_image" : "x86-ubuntu-18.04-img"
            },
            "additional_params" : {
                "readfile_contents" : "echo 'Boot successful'; m5 exit"
            }
        }
    ]
}
        """
        )
        file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        file.write(file_contents)
        file.close()

        cls.test_json = file.name
        os.environ["GEM5_RESOURCE_JSON"] = cls.test_json
        cls.workload = Workload("simple-boot")

    @classmethod
    def tearDownClass(cls):
        # Remove the test json file and unset the environment variable so this
        # test does not interfere with others.
        os.remove(cls.test_json)
        os.environ["GEM5_RESOURCE_JSON"]

    def test_get_function_str(self) -> None:
        # Tests `Resource.get_function_str`

        self.assertEquals(
            "set_kernel_disk_workload", self.workload.get_function_str()
        )

    def test_get_parameters(self) -> None:
        # Tests `Resource.get_parameters`

        parameters = self.workload.get_parameters()

        self.assertTrue(isinstance(parameters, Dict))
        self.assertEqual(3, len(parameters))

        self.assertTrue("kernel" in parameters)
        self.assertTrue(isinstance(parameters["kernel"], BinaryResource))

        self.assertTrue("disk_image" in parameters)
        self.assertTrue(
            isinstance(parameters["disk_image"], DiskImageResource)
        )

        self.assertTrue("readfile_contents" in parameters)
        self.assertTrue(
            "echo 'Boot successful'; m5 exit", parameters["readfile_contents"]
        )

    def test_add_parameters(self) -> None:
        # Tests `Resource.set_parameter` for the case where we add a new
        # parameter value.

        self.workload.set_parameter("test_param", 10)

        self.assertTrue("test_param" in self.workload.get_parameters())
        self.assertEquals(10, self.workload.get_parameters()["test_param"])

        # Cleanup
        del self.workload.get_parameters()["test_param"]

    def test_override_parameter(self) -> None:
        # Tests `Resource.set_parameter` for the case where we override
        # a parameter's value.

        old_value = self.workload.get_parameters()["readfile_contents"]

        self.workload.set_parameter("readfile_contents", "test")
        self.assertTrue("readfile_contents" in self.workload.get_parameters())
        self.assertEquals(
            "test", self.workload.get_parameters()["readfile_contents"]
        )

        # We set the overridden parameter back to it's old value.
        self.workload.set_parameter("readfile_contents", old_value)
