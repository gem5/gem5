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
import os
import shutil
import tempfile
import unittest
from pathlib import Path

from gem5.resources.md5_utils import md5_dir
from gem5.resources.md5_utils import md5_file


class MD5FileTestSuite(unittest.TestCase):
    """Test cases for gem5.resources.md5_utils.md5_file()"""

    def test_md5FileConsistency(self) -> None:
        # This test ensures the md5 algorithm we use does not change the md5
        # value over time.

        file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        file.write("This is a test string, to be put in a temp file")
        file.close()
        md5 = md5_file(Path(file.name))
        os.remove(file.name)

        self.assertEquals("b113b29fce251f2023066c3fda2ec9dd", md5)

    def test_identicalFilesIdenticalMd5(self) -> None:
        # This test ensures that two files with exactly the same contents have
        # the same md5 value.

        test_str = "This is a test"

        file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        file.write(test_str)
        file.close()
        first_file_md5 = md5_file(Path(file.name))

        os.remove(file.name)

        file = tempfile.NamedTemporaryFile(mode="w", delete=False)
        file.write(test_str)
        file.close()
        second_file_md5 = md5_file(Path(file.name))

        os.remove(file.name)

        self.assertEquals(first_file_md5, second_file_md5)


class MD5DirTestSuite(unittest.TestCase):
    """Test cases for gem5.resources.md5_utils.md5_dir()"""

    def _create_temp_directory(self) -> Path:

        dir = tempfile.mkdtemp()

        with open(os.path.join(dir, "file1"), "w") as f:
            f.write("Some test data here")

        with open(os.path.join(dir, "file2"), "w") as f:
            f.write("Some more test data")

        os.mkdir(os.path.join(dir, "dir2"))

        with open(os.path.join(dir, "dir2", "file1"), "w") as f:
            f.write("Yet more data")

        return Path(dir)

    def test_md5DirConsistency(self) -> None:
        # This test ensures the md5 algorithm we use does not change the value
        # given for directories over time.

        dir = self._create_temp_directory()
        md5 = md5_dir(dir)
        shutil.rmtree(dir)

        self.assertEquals("ad5ac785de44c9fc2fe2798cab2d7b1a", md5)

    def test_identicalDirsIdenticalMd5(self) -> None:
        # This test ensures that two directories with exactly the same contents
        # have the same md5 value.

        dir1 = self._create_temp_directory()
        first_md5 = md5_dir(dir1)
        shutil.rmtree(dir1)

        dir2 = self._create_temp_directory()
        second_md5 = md5_dir(dir2)
        shutil.rmtree(dir2)

        self.assertEquals(first_md5, second_md5)
