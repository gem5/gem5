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

from typing import Optional

import testlib.helper as helper
from testlib.configuration import constants


class SkipException(Exception):
    def __init__(self, fixture, testitem):
        self.msg = 'Fixture "{}" raised SkipException for "{}".'.format(
            fixture.name,
            testitem.name,
        )
        super().__init__(self.msg)


class Fixture:
    """
    Base Class for a test Fixture.

    Fixtures are items which possibly require setup and/or tearing down after
    a TestCase, TestSuite, or the Library has completed.

    Fixtures are the prefered method of carrying incremental results or
    variables between TestCases in TestSuites. (Rather than using globals.)
    Using fixtures rather than globals ensures that state will be maintained
    when executing tests in parallel.

    .. note:: In order for Fixtures to be enumerated by the test system this
        class' :code:`__new__` method must be called.
    """

    collector = helper.InstanceCollector()

    def __new__(klass, *args, **kwargs):
        obj = super().__new__(klass)
        Fixture.collector.collect(obj)
        return obj

    def __init__(self, name=None, **kwargs):
        if name is None:
            name = self.__class__.__name__
        self.name = name
        self._is_global = False

    def skip(self, testitem):
        raise SkipException(self.name, testitem.metadata)

    def setup(self, testitem):
        pass

    def post_test_procedure(self, testitem):
        pass

    def teardown(self, testitem):
        pass

    def get_get_build_info(self) -> Optional[dict]:
        # If this is a gem5 build it will return the target gem5 build path
        # and any additional build information. E.g.:
        #
        # /path/to/gem5/build/NULL/gem5.opt--default=NULL PROTOCOL=MI_example
        #
        # In this example this may be passed to scons to build gem5 in
        # accordance to the test's build requirements.
        #
        # If this fixtures is not a build of gem5, None is returned.
        return None

    def __str__(self):
        return f"{self.name} fixture"

    def set_global(self):
        self._is_global = True

    def is_global(self):
        return self._is_global
