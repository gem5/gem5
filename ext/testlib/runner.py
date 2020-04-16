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

import multiprocessing.dummy
import threading
import traceback

import testlib.helper as helper
import testlib.state as state
import testlib.log as log
import testlib.sandbox as sandbox

from testlib.state import Status, Result
from testlib.fixture import SkipException

def compute_aggregate_result(iterable):
    '''
    Status of the test suite by default is:
    * Passed if all contained tests passed
    * Errored if any contained tests errored
    * Failed if no tests errored, but one or more failed.
    * Skipped if all contained tests were skipped
    '''
    failed = []
    skipped = []
    for testitem in iterable:
        result = testitem.result

        if result.value == Result.Errored:
            return Result(result.value, result.reason)
        elif result.value == Result.Failed:
            failed.append(result.reason)
        elif result.value == result.Skipped:
            skipped.append(result.reason)
    if failed:
        return Result(Result.Failed, failed)
    elif skipped:
        return Result(Result.Skipped, skipped)
    else:
        return Result(Result.Passed)

class TestParameters(object):
    def __init__(self, test, suite):
        self.test = test
        self.suite = suite
        self.log = log.TestLogWrapper(log.test_log, test, suite)

    @helper.cacheresult
    def _fixtures(self):
        fixtures = {fixture.name:fixture for fixture in self.suite.fixtures}
        for fixture in self.test.fixtures:
            fixtures[fixture.name] = fixture
        return fixtures

    @property
    def fixtures(self):
        return self._fixtures()


class RunnerPattern:
    def __init__(self, loaded_testable):
        self.testable = loaded_testable
        self.builder = FixtureBuilder(self.testable.fixtures)

    def handle_error(self, trace):
        self.testable.result = Result(Result.Errored, trace)
        self.avoid_children(trace)

    def handle_skip(self, trace):
        self.testable.result = Result(Result.Skipped, trace)
        self.avoid_children(trace)

    def avoid_children(self, reason):
        for testable in self.testable:
            testable.result = Result(self.testable.result.value, reason)
            testable.status = Status.Avoided

    def test(self):
        pass

    def run(self):
        avoided = False
        try:
            self.testable.status = Status.Building
            self.builder.setup(self.testable)
        except SkipException:
            self.handle_skip(traceback.format_exc())
            avoided = True
        except BrokenFixtureException:
            self.handle_error(traceback.format_exc())
            avoided = True
        else:
            self.testable.status = Status.Running
            self.test()
        finally:
            self.testable.status = Status.TearingDown
            self.builder.teardown(self.testable)

        if avoided:
            self.testable.status = Status.Avoided
        else:
            self.testable.status = Status.Complete

class TestRunner(RunnerPattern):
    def test(self):
        self.sandbox_test()

    def sandbox_test(self):
        try:
            sandbox.Sandbox(TestParameters(
                    self.testable,
                    self.testable.parent_suite))
        except sandbox.SubprocessException:
            self.testable.result = Result(Result.Failed,
                    traceback.format_exc())
        else:
            self.testable.result = Result(Result.Passed)


class SuiteRunner(RunnerPattern):
    def test(self):
        for test in self.testable:
            test.runner(test).run()
        self.testable.result = compute_aggregate_result(
                iter(self.testable))


class LibraryRunner(SuiteRunner):
    pass


class LibraryParallelRunner(RunnerPattern):
    def set_threads(self, threads):
        self.threads = threads

    def _entrypoint(self, suite):
        suite.runner(suite).run()

    def test(self):
        pool = multiprocessing.dummy.Pool(self.threads)
        pool.map(lambda suite : suite.runner(suite).run(), self.testable)
        self.testable.result = compute_aggregate_result(
                iter(self.testable))


class BrokenFixtureException(Exception):
    def __init__(self, fixture, testitem, trace):
        self.fixture = fixture
        self.testitem = testitem
        self.trace = trace

        self.msg = ('%s\n'
                   'Exception raised building "%s" raised SkipException'
                   ' for "%s".' %
                   (trace, fixture.name, testitem.name)
        )
        super(BrokenFixtureException, self).__init__(self.msg)

class FixtureBuilder(object):
    def __init__(self, fixtures):
        self.fixtures = fixtures
        self.built_fixtures = []

    def setup(self, testitem):
        for fixture in self.fixtures:
            # Mark as built before, so if the build fails
            # we still try to tear it down.
            self.built_fixtures.append(fixture)
            try:
                fixture.setup(testitem)
            except SkipException:
                raise
            except Exception as e:
                exc = traceback.format_exc()
                msg = 'Exception raised while setting up fixture for %s' %\
                        testitem.uid
                log.test_log.warn('%s\n%s' % (exc, msg))

                raise BrokenFixtureException(fixture, testitem,
                        traceback.format_exc())

    def teardown(self, testitem):
        for fixture in self.built_fixtures:
            try:
                fixture.teardown(testitem)
            except Exception:
                # Log exception but keep cleaning up.
                exc = traceback.format_exc()
                msg = 'Exception raised while tearing down fixture for %s' %\
                        testitem.uid
                log.test_log.warn('%s\n%s' % (exc, msg))
