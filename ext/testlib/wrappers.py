# Copyright (c) 2019 ARM Limited
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

'''
Module contains wrappers for test items that have been
loaded by the testlib :class:`testlib.loader.Loader`.
'''
import itertools

import log
import uid
from state import Status, Result

class TestCaseMetadata():
    def __init__(self, name, uid, path, result, status, suite_uid):
        self.name = name
        self.uid = uid
        self.path = path
        self.status = status
        self.result = result
        self.suite_uid = suite_uid


class TestSuiteMetadata():
    def __init__(self, name, uid, tags, path, status, result):
        self.name = name
        self.uid = uid
        self.tags = tags
        self.path = path
        self.status = status
        self.result = result


class LibraryMetadata():
    def __init__(self, name, result, status):
        self.name = name
        self.result = result
        self.status = status


class LoadedTestable(object):
    '''
    Base class for loaded test items.

    :property:`result` and :property:`status` setters
    notify testlib via the :func:`log_result` and :func:`log_status`
    of the updated status.
    '''
    def __init__(self, obj):
        self.obj = obj
        self.metadata = self._generate_metadata()

    @property
    def status(self):
        return self.metadata.status

    @status.setter
    def status(self, status):
        self.log_status(status)
        self.metadata.status = status

    @property
    def result(self):
        return self.metadata.result

    @result.setter
    def result(self, result):
        self.log_result(result)
        self.metadata.result = result

    @property
    def uid(self):
        return self.metadata.uid

    @property
    def name(self):
        return self.metadata.name

    @property
    def fixtures(self):
        return self.obj.fixtures

    @fixtures.setter
    def fixtures(self, fixtures):
        self.obj.fixtures = fixtures

    @property
    def runner(self):
        return self.obj.runner

    # TODO Change log to provide status_update, result_update for all types.
    def log_status(self, status):
        log.test_log.status_update(self, status)

    def log_result(self, result):
        log.test_log.result_update(self, result)

    def __iter__(self):
        return iter(())


class LoadedTest(LoadedTestable):
    def __init__(self, test_obj, loaded_suite, path):
        self.parent_suite = loaded_suite
        self._path = path
        LoadedTestable.__init__(self, test_obj)

    def test(self, *args, **kwargs):
        self.obj.test(*args, **kwargs)

    def _generate_metadata(self):
        return TestCaseMetadata( **{
            'name':self.obj.name,
            'path': self._path,
            'uid': uid.TestUID(self._path,
                               self.obj.name,
                               self.parent_suite.name),
            'status': Status.Unscheduled,
            'result': Result(Result.NotRun),
            'suite_uid': self.parent_suite.metadata.uid
        })


class LoadedSuite(LoadedTestable):
    def __init__(self, suite_obj, path):
        self._path = path
        LoadedTestable.__init__(self, suite_obj)
        self.tests = self._wrap_children(suite_obj)

    def _wrap_children(self, suite_obj):
        return [LoadedTest(test, self, self.metadata.path)
                for test in suite_obj]

    def _generate_metadata(self):
        return TestSuiteMetadata( **{
            'name': self.obj.name,
            'tags':self.obj.tags,
            'path': self._path,
            'uid': uid.SuiteUID(self._path, self.obj.name),
            'status': Status.Unscheduled,
            'result': Result(Result.NotRun)
        })

    def __iter__(self):
        return iter(self.tests)

    @property
    def tags(self):
        return self.metadata.tags


class LoadedLibrary(LoadedTestable):
    '''
    Wraps a collection of all loaded test suites and
    provides utility functions for accessing fixtures.
    '''
    def __init__(self, suites):
        LoadedTestable.__init__(self, suites)

    def _generate_metadata(self):
        return LibraryMetadata( **{
            'name': 'Test Library',
            'status': Status.Unscheduled,
            'result': Result(Result.NotRun)
        })

    def __iter__(self):
        '''
        :returns: an iterator over contained :class:`TestSuite` objects.
        '''
        return iter(self.obj)

    def all_fixtures(self):
        '''
        :returns: an interator overall all global, suite,
          and test fixtures
        '''
        return itertools.chain(itertools.chain(
            *(suite.fixtures for suite in self.obj)),
            *(self.test_fixtures(suite) for suite in self.obj)
        )

    def test_fixtures(self, suite):
        '''
        :returns: an interator over all fixtures of each
          test contained in the given suite
        '''
        return itertools.chain(*(test.fixtures for test in suite))

    @property
    def fixtures(self):
        global_fixtures = []
        for fixture in self.all_fixtures():
            if fixture.is_global():
                global_fixtures.append(fixture)
        return global_fixtures

    @property
    def uid(self):
        return self.name

    @property
    def suites(self):
        return self.obj

    @suites.setter
    def suites(self, suites):
        self.obj = suites
