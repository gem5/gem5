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
from pathlib import Path

from gem5.resources.downloader import _file_uri_to_path


class LocalPathTestSuite(unittest.TestCase):
    def test_local_path_exists_single_slash(self):
        # Test that a local path is returned as-is
        path = "file:/test/test/file"
        expected_path = Path("/test/test/file")
        self.assertEqual(_file_uri_to_path(path), expected_path)

    def test_non_localhost_exception(self):
        # Test that a local path with different netloc throws an exception
        path = "file://test/test/file"
        # should raise Exception because netloc is not '' or 'localhost'
        with self.assertRaises(Exception) as exception:
            _file_uri_to_path(path)
        self.assertEqual(
            str(exception.exception),
            f"File URI '{path}' specifies host 'test'. "
            "Only localhost is permitted.",
        )

    def test_localhost_accepted(self):
        path = "file://localhost/test/test/file"
        # should work as expected because netloc is 'localhost'
        expected_path = Path("/test/test/file")
        self.assertEqual(_file_uri_to_path(path), expected_path)

    def test_local_path_exists_triple_slash(self):
        # Test that a local path is returned as-is
        path = "file:///test/test/file"
        expected_path = Path("/test/test/file")
        self.assertEqual(_file_uri_to_path(path), expected_path)

    def test_local_path_exists_quadruple_slash(self):
        # Test that a local path is returned as-is
        path = "file:////test/test/file"
        expected_path = Path("//test/test/file")
        self.assertEqual(_file_uri_to_path(path), expected_path)

    def test_uri_not_file(self):
        # Test that a URL returns None
        path = "http://test/test/file"
        self.assertIsNone(_file_uri_to_path(path))
