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

import copy
import traceback

import helper
import log

global_fixtures = []

class SkipException(Exception):
    def __init__(self, fixture, testitem):
        self.fixture = fixture
        self.testitem = testitem

        self.msg = 'Fixture "%s" raised SkipException for "%s".' % (
               fixture.name, testitem.name
        )
        super(SkipException, self).__init__(self.msg)


class Fixture(object):
    '''
    Base Class for a test Fixture.

    Fixtures are items which possibly require setup and/or tearing down after
    a TestCase, TestSuite, or the Library has completed.

    Fixtures are the prefered method of carrying incremental results or
    variables between TestCases in TestSuites. (Rather than using globals.)
    Using fixtures rather than globals ensures that state will be maintained
    when executing tests in parallel.

    .. note:: In order for Fixtures to be enumerated by the test system this
        class' :code:`__new__` method must be called.
    '''
    collector = helper.InstanceCollector()

    def __new__(klass, *args, **kwargs):
        obj = super(Fixture, klass).__new__(klass, *args, **kwargs)
        Fixture.collector.collect(obj)
        return obj

    def __init__(self, name=None, **kwargs):
        if name is None:
            name = self.__class__.__name__
        self.name = name

    def skip(self, testitem):
        raise SkipException(self.name, testitem.metadata)

    def schedule_finalized(self, schedule):
        '''
        This method is called once the schedule of for tests is known.
        To enable tests to use the same fixture defintion for each execution
        fixtures must return a copy of themselves in this method.

        :returns: a copy of this fixture which will be setup/torndown
          when the test item this object is tied to is about to execute.
        '''
        return self.copy()

    def init(self, *args, **kwargs):
        pass

    def setup(self, testitem):
        pass

    def teardown(self, testitem):
        pass

    def copy(self):
        return copy.deepcopy(self)


def globalfixture(fixture):
    '''
    Store the given fixture as a global fixture. Its setup() method
    will be called before the first test is executed.
    '''
    global_fixtures.append(fixture)
    return fixture
