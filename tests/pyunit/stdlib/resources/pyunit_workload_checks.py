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

from gem5.resources.workload import Workload, CustomWorkload
from gem5.resources.resource import (
    BinaryResource,
    DiskImageResource,
    obtain_resource,
    WorkloadResource,
)

from typing import Dict

from gem5.resources.client_api.client_wrapper import ClientWrapper
from unittest.mock import patch
from pathlib import Path

mock_config_json = {
    "sources": {
        "baba": {
            "url": Path(__file__).parent / "refs/workload-checks.json",
            "isMongo": False,
        }
    },
}


class CustomWorkloadTestSuite(unittest.TestCase):
    """
    Tests the `gem5.resources.workload.CustomWorkload` class.
    """

    @classmethod
    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def setUpClass(cls) -> None:
        cls.custom_workload = WorkloadResource(
            function="set_se_binary_workload",
            parameters={
                "binary": obtain_resource(
                    "x86-hello64-static-example", gem5_version="develop"
                ),
                "arguments": ["hello", 6],
            },
        )

    def test_get_function_str(self) -> None:
        # Tests `CustomWorkload.get_function_str`

        self.assertEqual(
            "set_se_binary_workload", self.custom_workload.get_function_str()
        )

    def test_get_parameters(self) -> None:
        # Tests `CustomWorkload.get_parameter`

        parameters = self.custom_workload.get_parameters()
        self.assertTrue(isinstance(parameters, Dict))
        self.assertEqual(2, len(parameters))

        self.assertTrue("binary" in parameters)
        self.assertTrue(isinstance(parameters["binary"], BinaryResource))

        self.assertTrue("arguments" in parameters)
        self.assertTrue(isinstance(parameters["arguments"], list))
        self.assertEqual(2, len(parameters["arguments"]))
        self.assertEqual("hello", parameters["arguments"][0])
        self.assertEqual(6, parameters["arguments"][1])

    def test_add_parameters(self) -> None:
        # Tests `CustomWorkload.set_parameter` for the case where we add a new
        # parameter value.

        self.custom_workload.set_parameter("test_param", 10)

        self.assertTrue("test_param" in self.custom_workload.get_parameters())
        self.assertEqual(
            10, self.custom_workload.get_parameters()["test_param"]
        )

        # Cleanup
        del self.custom_workload.get_parameters()["test_param"]

    def test_override_parameter(self) -> None:
        # Tests `CustomWorkload.set_parameter` for the case where we override
        # a parameter's value.

        old_value = self.custom_workload.get_parameters()["binary"]

        self.custom_workload.set_parameter("binary", "test")
        self.assertTrue("binary" in self.custom_workload.get_parameters())
        self.assertEqual(
            "test", self.custom_workload.get_parameters()["binary"]
        )

        # We set the overridden parameter back to it's old value
        self.custom_workload.set_parameter("binary", old_value)


class WorkloadTestSuite(unittest.TestCase):
    """
    Tests the `gem5.resources.workload.Workload` class.
    """

    @classmethod
    @patch(
        "gem5.resources.client.clientwrapper",
        ClientWrapper(mock_config_json),
    )
    def setUpClass(cls):
        cls.workload = obtain_resource("simple-boot", gem5_version="develop")

    def test_get_function_str(self) -> None:
        # Tests `Resource.get_function_str`

        self.assertEqual(
            "set_kernel_disk_workload", self.workload.get_function_str()
        )

    def test_get_parameters(self) -> None:
        # Tests `Resource.get_parameters`

        parameters = self.workload.get_parameters()

        self.assertTrue(isinstance(parameters, Dict))
        self.assertEqual(3, len(parameters))

        self.assertTrue("kernel" in parameters)
        self.assertTrue(isinstance(parameters["kernel"], BinaryResource))

        self.assertTrue("disk-image" in parameters)
        self.assertTrue(
            isinstance(parameters["disk-image"], DiskImageResource)
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
        self.assertEqual(10, self.workload.get_parameters()["test_param"])

        # Cleanup
        del self.workload.get_parameters()["test_param"]

    def test_override_parameter(self) -> None:
        # Tests `Resource.set_parameter` for the case where we override
        # a parameter's value.

        old_value = self.workload.get_parameters()["readfile_contents"]

        self.workload.set_parameter("readfile_contents", "test")
        self.assertTrue("readfile_contents" in self.workload.get_parameters())
        self.assertEqual(
            "test", self.workload.get_parameters()["readfile_contents"]
        )

        # We set the overridden parameter back to it's old value.
        self.workload.set_parameter("readfile_contents", old_value)
