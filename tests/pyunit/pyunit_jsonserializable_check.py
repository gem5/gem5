# Copyright (c) 2022 The Regents of The University of California
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
from m5.ext.pystats.serializable_stat import SerializableStat


class MockSerializable(SerializableStat):
    def __init__(self):
        self.child_1 = MockSerializableChild()
        self.child_1.stat1 = 2
        self.child_1.stat2 = "3"
        self.child_list = []

        child_list_1 = MockSerializableChild()
        child_list_1.stat1 = "hello"
        self.child_list.append(child_list_1)
        child_list_2 = MockSerializableChild()
        child_list_2.list_stat2 = ["1", 2, "3", 4, 5.2, None]
        self.child_list.append(child_list_2)


class MockSerializableChild(SerializableStat):
    def __init__(self):
        pass


class JsonSerializableTestSuite(unittest.TestCase):
    def test_to_json(self):
        obj = MockSerializable()
        obj_json = obj.to_json()
        self.assertTrue("child_1" in obj_json)
        self.assertTrue("stat1" in obj_json["child_1"])
        self.assertEqual(2, obj_json["child_1"]["stat1"])
        self.assertTrue("stat2" in obj_json["child_1"])
        self.assertEqual("3", obj_json["child_1"]["stat2"])
        self.assertTrue("child_list" in obj_json)
        self.assertEqual(2, len(obj_json["child_list"]))
        self.assertTrue("stat1" in obj_json["child_list"][0])
        self.assertEqual("hello", obj_json["child_list"][0]["stat1"])
        self.assertTrue("list_stat2" in obj_json["child_list"][1])
        self.assertEqual(6, len(obj_json["child_list"][1]["list_stat2"]))
        self.assertEqual("1", obj_json["child_list"][1]["list_stat2"][0])
        self.assertEqual(2, obj_json["child_list"][1]["list_stat2"][1])
        self.assertEqual("3", obj_json["child_list"][1]["list_stat2"][2])
        self.assertEqual(4, obj_json["child_list"][1]["list_stat2"][3])
        self.assertEqual(5.2, obj_json["child_list"][1]["list_stat2"][4])
        self.assertEqual(None, obj_json["child_list"][1]["list_stat2"][5])
