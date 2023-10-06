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
import testlib.helper as helper
import testlib.runner as runner_mod


class TestCase(object):
    """
    Base class for all tests.

    ..note::
        The :func:`__new__` method enables collection of test cases, it must
        be called in order for test cases to be collected.
    """

    fixtures = []

    # TODO, remove explicit dependency. Use the loader to set the
    # default runner
    runner = runner_mod.TestRunner
    collector = helper.InstanceCollector()

    def __new__(cls, *args, **kwargs):
        obj = super(TestCase, cls).__new__(cls)
        TestCase.collector.collect(obj)
        return obj

    def __init__(self, name=None, fixtures=tuple(), **kwargs):
        self.fixtures = self.fixtures + list(fixtures)
        if name is None:
            name = self.__class__.__name__
        self.name = name


class TestFunction(TestCase):
    """
    TestCase implementation which uses a callable object as a test.
    """

    def __init__(self, function, name=None, **kwargs):
        self.test_function = function
        if name is None:
            name = function.__name__
        TestCase.__init__(self, name=name, **kwargs)

    def test(self, *args, **kwargs):
        self.test_function(*args, **kwargs)
