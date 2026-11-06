# Copyright (c) 2026 The Regents of the University of California
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

from m5.objects import (
    PowerModel,
    SimpleMemory,
    SubSystem,
)
from m5.params import NULL


class TestFindAllVectorParams(unittest.TestCase):
    def test_empty_vector(self):
        memory = SimpleMemory()
        self.assertEqual(memory.find_all(PowerModel), ([], True))

    def test_nonempty_vector_is_flattened(self):
        first = PowerModel()
        second = PowerModel()
        memory = SimpleMemory(power_model=[first, second])
        matches, done = memory.find_all(PowerModel)
        self.assertTrue(done)
        self.assertEqual(matches, [first, second])

    def test_null_vector_entry_is_ignored(self):
        model = PowerModel()
        memory = SimpleMemory(power_model=[NULL, model])
        self.assertEqual(memory.find_all(PowerModel), ([model], True))

    def test_empty_and_nonempty_child_vectors(self):
        root = SubSystem()
        root.empty = SimpleMemory()
        root.populated = SimpleMemory(power_model=[PowerModel()])
        model = root.populated.power_model[0]
        self.assertEqual(root.find_all(PowerModel), ([model], True))
