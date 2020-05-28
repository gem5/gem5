# Copyright (c) 2017 Mark D. Hill and David A. Wood
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
#
# Authors: Sean Wilson

import os
import itertools

import testlib.configuration as configuration

class UID(object):
    sep = ':'
    type_idx, path_idx = range(2)

    def __init__(self, path, *args):
        self.path = self._shorten_path(path)
        self.attributes = args

    @staticmethod
    def _shorten_path(path):
        return os.path.relpath(path,
                os.path.commonprefix((configuration.constants.testing_base,
                                      path)))

    @staticmethod
    def _full_path(short_path):
        return os.path.join(configuration.constants.testing_base, short_path)

    @classmethod
    def uid_to_path(cls, uid):
        split_path = str(uid).split(cls.sep)[cls.path_idx]
        return cls._full_path(split_path)

    @classmethod
    def uid_to_class(cls, uid):
        return globals()[uid.split(cls.sep)[cls.type_idx]]

    @classmethod
    def from_suite(self, suite, filepath):
        return SuiteUID(filepath, suite.name)

    @classmethod
    def from_test(self, test, filepath):
        return TestUID(filepath, test.name, test.parent_suite.name)

    @classmethod
    def from_uid(cls, uid):
        args = uid.split(cls.sep)
        del args[cls.type_idx]
        return cls.uid_to_class(uid)(*args)

    def __str__(self):
        common_opts = {
            self.path_idx: self.path,
            self.type_idx: self.__class__.__name__
        }
        return self.sep.join(itertools.chain(
            [common_opts[0], common_opts[1]],
            self.attributes))

    def __hash__(self):
        return hash(str(self))

    def __eq__(self, other):
        return type(self) == type(other) and str(self) == str(other)


class TestUID(UID):
    def __init__(self, filename, test_name, suite_name):
        UID.__init__(self, filename, suite_name, test_name)

    @property
    def test(self):
        return self.attributes[1]

    @property
    def suite(self):
        return self.attributes[0]


class SuiteUID(UID):
    def __init__(self, filename, suite_name):
        UID.__init__(self, filename, suite_name)

    @property
    def suite(self):
        return self.attributes[0]
