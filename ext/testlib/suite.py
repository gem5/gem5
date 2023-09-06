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


class TestSuite(object):
    """
    An object grouping a collection of tests. It provides tags which enable
    filtering during list and run selection. All tests held in the suite must
    have a unique name.

    ..note::
        The :func:`__new__` method enables collection of test cases, it must
        be called in order for test cases to be collected.

    ..note::
        To reduce test definition boilerplate, the :func:`init` method is
        forwarded all `*args` and `**kwargs`. This means derived classes can
        define init without boilerplate super().__init__(*args, **kwargs).
    """

    runner = runner_mod.SuiteRunner
    collector = helper.InstanceCollector()
    fixtures = []
    tests = []
    tags = set()

    def __new__(klass, *args, **kwargs):
        obj = super(TestSuite, klass).__new__(klass)
        TestSuite.collector.collect(obj)
        return obj

    def __init__(
        self,
        name=None,
        fixtures=tuple(),
        tests=tuple(),
        tags=tuple(),
        **kwargs
    ):
        self.fixtures = self.fixtures + list(fixtures)
        self.tags = self.tags | set(tags)
        self.tests = self.tests + list(tests)
        if name is None:
            name = self.__class__.__name__
        self.name = name

    def __iter__(self):
        return iter(self.tests)
