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

import contextlib
import io
import unittest
import tempfile
import os
import shutil
from pathlib import Path
from gem5.resources.resource import (
    obtain_resource,
    SuiteResource,
    WorkloadResource,
)
from gem5.resources.client_api.client_wrapper import ClientWrapper
from unittest.mock import patch

mock_config_json = {
    "sources": {
        "baba": {
            "url": Path(__file__).parent / "refs/suite-checks.json",
            "isMongo": False,
        }
    },
}


class CustomSuiteResourceTestSuite(unittest.TestCase):
    @classmethod
    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def setUpClass(cls):
        cls.workload1 = obtain_resource("simple-workload-1")
        cls.workload2 = obtain_resource("simple-workload-2")
        cls.SuiteResource = SuiteResource(
            workloads={cls.workload1: set(), cls.workload2: set()}
        )

    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def test_with_input_group(self) -> None:
        """
        Tests the `with_input_group` function.
        """
        # test if an input group can return a single workload in a suite resource

        with self.assertRaises(Exception) as context:
            filtered_suite = self.SuiteResource.with_input_group("testtag2")
            self.assertIsInstance(filtered_suite, SuiteResource)
            self.assertEqual(len(filtered_suite), 0)
            self.assertTrue(
                f"Input group invalid not found in Suite.\n"
                f"Available input groups are {filtered_suite.get_input_groups()}"
                in str(context.exception)
            )

    def test_get_input_groups(self):
        """
        Tests the `list_input_groups` function.
        """
        self.assertEqual(self.SuiteResource.get_input_groups(), set())


class SuiteResourceTestSuite(unittest.TestCase):
    @classmethod
    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def setUpClass(cls):
        cls.suite = obtain_resource("suite-example", gem5_version="develop")

    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def test_with_input_group(self) -> None:
        """
        Tests the `with_input_group` function.
        """
        # test if an input group can return a single workload in a suite resource
        filtered_suite = self.suite.with_input_group("testtag2")
        self.assertIsInstance(filtered_suite, SuiteResource)
        self.assertEqual(len(filtered_suite), 1)
        for workload in filtered_suite:
            self.assertIsInstance(workload, WorkloadResource)

    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def test_with_input_group_multiple(self) -> None:
        # test if an input group can return multiple workloads in a suite resource
        filtered_suite = self.suite.with_input_group("testtag1")
        self.assertIsInstance(filtered_suite, SuiteResource)
        self.assertEqual(len(filtered_suite), 2)
        for workload in filtered_suite:
            self.assertIsInstance(workload, WorkloadResource)

    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def test_with_input_group_invalid(self) -> None:
        """
        Tests the `with_input_group` function with an invalid input group.
        """
        with self.assertRaises(Exception) as context:
            filtered_suite = self.suite.with_input_group("invalid")
            # check if exception is raised
            self.assertTrue(
                f"Input group invalid not found in Suite.\n"
                f"Available input groups are {filtered_suite.get_input_groups()}"
                in str(context.exception)
            )

    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def test_get_input_groups(self) -> None:
        """
        Tests the `list_input_groups` function.
        """
        expected_input_groups = {"testtag1", "testtag2", "testtag3"}
        self.assertEqual(self.suite.get_input_groups(), expected_input_groups)

    @patch(
        "gem5.resources.client.clientwrapper",
        new=ClientWrapper(mock_config_json),
    )
    def test_get_input_groups_not_found(self) -> None:
        """
        Tests the `list_input_groups` function with an invalid input group.
        """
        with self.assertRaises(Exception) as context:
            self.suite.get_input_groups("invalid")
            self.assertTrue(
                f"Input group invalid not found in Suite.\n"
                f"Available input groups are {self.suite.get_input_groups()}"
                in str(context.exception)
            )
