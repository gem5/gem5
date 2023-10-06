#!/usr/bin/env python3
#
# Copyright (c) 2020 Arm Limited
# All rights reserved
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
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

from ..maintainers import *

YAML_VALID = r"""
maintained:
  status: maintained
  maintainers:
    - John Doe <john.doe@test.gem5.org>
    - Jane Doe <jane.doe@test.gem5.org>

# Test that we can handle a subsystem without maintainers
orphaned:
  desc: Abandoned
  status: orphaned
"""

YAML_MISSING_STATUS = r"""
key:
  maintainers:
    - John Doe <john.doe@test.gem5.org>
"""

YAML_INVALID_STATUS = r"""
key:
  status: invalid_status_name
  maintainers:
    - John Doe <john.doe@test.gem5.org>
"""

YAML_MAINTAINERS_NOT_LIST = r"""
key:
  status: maintained
  maintainers:
"""


class StatusTestSuite(unittest.TestCase):
    """Test cases for maintainers.Status"""

    def test_str_conv(self):
        pairs = [
            ("maintained", Status.MAINTAINED),
            ("orphaned", Status.ORPHANED),
        ]

        for name, value in pairs:
            assert value == Status.from_str(name)
            assert str(value) == name


class MaintainersTestSuite(unittest.TestCase):
    """Test cases for Maintainers"""

    def test_parser_valid(self):
        maint = Maintainers.from_yaml(YAML_VALID)

        subsys = maint["maintained"]
        self.assertEqual(subsys.status, Status.MAINTAINED)
        self.assertEqual(subsys.description, "")
        self.assertEqual(
            subsys.maintainers,
            [
                ("John Doe", "john.doe@test.gem5.org"),
                ("Jane Doe", "jane.doe@test.gem5.org"),
            ],
        )

        subsys = maint["orphaned"]
        self.assertEqual(subsys.status, Status.ORPHANED)
        self.assertEqual(subsys.description, "Abandoned")
        self.assertEqual(subsys.maintainers, [])

    def test_parser_invalid(self):
        with self.assertRaises(MissingFieldException):
            Maintainers.from_yaml(YAML_MISSING_STATUS)

        with self.assertRaises(IllegalValueException):
            Maintainers.from_yaml(YAML_INVALID_STATUS)

        with self.assertRaises(IllegalValueException):
            Maintainers.from_yaml(YAML_MAINTAINERS_NOT_LIST)


if __name__ == "__main__":
    unittest.main()
