# Copyright (c) 2020 ARM Limited
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
import os
import pickle
import xml.sax.saxutils

import testlib.helper as helper
import testlib.state as state
from testlib.configuration import config


def _create_uid_index(iterable):
    index = {}
    for item in iterable:
        assert item.uid not in index
        index[item.uid] = item
    return index


class _CommonMetadataMixin:
    @property
    def name(self):
        return self._metadata.name

    @property
    def uid(self):
        return self._metadata.uid

    @property
    def result(self):
        return self._metadata.result

    @result.setter
    def result(self, result):
        self._metadata.result = result

    @property
    def unsuccessful(self):
        return self._metadata.result.value != state.Result.Passed

    @property
    def time(self):
        return self._metadata.time


class InternalTestResult(_CommonMetadataMixin):
    def __init__(self, obj, suite, directory):
        self._metadata = obj.metadata
        self.suite = suite

        self.stderr = os.path.join(
            InternalSavedResults.output_path(self.uid, suite.uid),
            "stderr",
        )
        self.stdout = os.path.join(
            InternalSavedResults.output_path(self.uid, suite.uid),
            "stdout",
        )


class InternalSuiteResult(_CommonMetadataMixin):
    def __init__(self, obj, directory):
        self._metadata = obj.metadata
        self.directory = directory
        self._wrap_tests(obj)

    def _wrap_tests(self, obj):
        self._tests = [
            InternalTestResult(test, self, self.directory) for test in obj
        ]
        self._tests_index = _create_uid_index(self._tests)

    def get_test(self, uid):
        return self._tests_index[uid]

    def __iter__(self):
        return iter(self._tests)

    def get_test_result(self, uid):
        return self.get_test(uid)

    def aggregate_test_results(self):
        results = {}
        for test in self:
            helper.append_dictlist(results, test.result.value, test)
        return results


class InternalLibraryResults(_CommonMetadataMixin):
    def __init__(self, obj, directory):
        self.directory = directory
        self._metadata = obj.metadata
        self._wrap_suites(obj)

    def __iter__(self):
        return iter(self._suites)

    def _wrap_suites(self, obj):
        self._suites = [
            InternalSuiteResult(suite, self.directory) for suite in obj
        ]
        self._suites_index = _create_uid_index(self._suites)

    def add_suite(self, suite):
        if suite.uid in self._suites:
            raise ValueError("Cannot have duplicate suite UIDs.")
        self._suites[suite.uid] = suite

    def get_suite_result(self, suite_uid):
        return self._suites_index[suite_uid]

    def get_test_result(self, test_uid, suite_uid):
        return self.get_suite_result(suite_uid).get_test_result(test_uid)

    def aggregate_test_results(self):
        results = {}
        for suite in self._suites:
            for test in suite:
                helper.append_dictlist(results, test.result.value, test)
        return results


class InternalSavedResults:
    @staticmethod
    def output_path(test_uid, suite_uid, base=None):
        """
        Return the path which results for a specific test case should be
        stored.
        """
        if base is None:
            base = config.result_path
        return os.path.join(
            base,
            str(suite_uid).replace(os.path.sep, "-"),
            str(test_uid).replace(os.path.sep, "-"),
        )

    @staticmethod
    def save(results, path, protocol=pickle.HIGHEST_PROTOCOL):
        if not os.path.exists(os.path.dirname(path)):
            try:
                os.makedirs(os.path.dirname(path))
            except OSError as exc:  # Guard against race condition
                if exc.errno != errno.EEXIST:
                    raise

        with open(path, "wb") as f:
            pickle.dump(results, f, protocol)

    @staticmethod
    def load(path):
        with open(path, "rb") as f:
            return pickle.load(f)


class XMLElement:
    def write(self, file_):
        self.begin(file_)
        self.end(file_)

    def begin(self, file_):
        file_.write("<")
        file_.write(self.name)
        if hasattr(self, "attributes"):
            for attr in self.attributes:
                file_.write(" ")
                attr.write(file_)
        file_.write(">")

        self.body(file_)

    def body(self, file_):
        if hasattr(self, "elements"):
            for elem in self.elements:
                file_.write("\n")
                elem.write(file_)
        if hasattr(self, "content"):
            file_.write("\n")
            file_.write(xml.sax.saxutils.escape(self.content))
        file_.write("\n")

    def end(self, file_):
        file_.write("</%s>" % self.name)


class XMLAttribute:
    def __init__(self, name, value):
        self.name = name
        self.value = value

    def write(self, file_):
        file_.write(
            f"{self.name}={xml.sax.saxutils.quoteattr(self.value)}",
        )


class JUnitTestSuites(XMLElement):
    name = "testsuites"
    result_map = {
        state.Result.Errored: "errors",
        state.Result.Failed: "failures",
        state.Result.Passed: "tests",
    }

    def __init__(self, internal_results):
        results = internal_results.aggregate_test_results()

        self.attributes = []
        for result, tests in results.items():
            self.attributes.append(
                self.result_attribute(result, str(len(tests))),
            )

        self.elements = []
        for suite in internal_results:
            self.elements.append(JUnitTestSuite(suite))

    def result_attribute(self, result, count):
        return XMLAttribute(self.result_map[result], count)


class JUnitTestSuite(JUnitTestSuites):
    name = "testsuite"
    result_map = {
        state.Result.Errored: "errors",
        state.Result.Failed: "failures",
        state.Result.Passed: "tests",
        state.Result.Skipped: "skipped",
    }

    def __init__(self, suite_result):
        results = suite_result.aggregate_test_results()

        self.attributes = [XMLAttribute("name", suite_result.name)]
        for result, tests in results.items():
            self.attributes.append(
                self.result_attribute(result, str(len(tests))),
            )

        self.elements = []
        for test in suite_result:
            self.elements.append(JUnitTestCase(test))

    def result_attribute(self, result, count):
        return XMLAttribute(self.result_map[result], count)


class JUnitTestCase(XMLElement):
    name = "testcase"

    def __init__(self, test_result):
        self.attributes = [
            XMLAttribute("name", test_result.name),
            # TODO JUnit expects class of test.. add as test metadata.
            XMLAttribute("classname", str(test_result.uid)),
            XMLAttribute("status", str(test_result.result)),
            XMLAttribute("time", str(test_result.time["user_time"])),
        ]

        # TODO JUnit expects a message for the reason a test was
        # skipped or errored, save this with the test metadata.
        # http://llg.cubic.org/docs/junit/
        self.elements = [
            LargeFileElement("system-err", test_result.stderr),
            LargeFileElement("system-out", test_result.stdout),
        ]

        if str(test_result.result) == "Failed":
            self.elements.append(
                JUnitFailure("Test failed", str(test_result.result.reason)),
            )


class JUnitFailure(XMLElement):
    name = "failure"

    def __init__(self, message, cause):
        self.attributes = [
            XMLAttribute("message", message),
        ]
        cause_element = XMLElement()
        cause_element.name = "cause"
        cause_element.content = cause
        self.elements = [cause_element]


class LargeFileElement(XMLElement):
    def __init__(self, name, filename):
        self.name = name
        self.filename = filename
        self.attributes = []

    def body(self, file_):
        try:
            with open(self.filename) as f:
                for line in f:
                    file_.write(xml.sax.saxutils.escape(line))
        except OSError:
            # TODO Better error logic, this is sometimes O.K.
            # if there was no stdout/stderr captured for the test
            #
            # TODO If that was the case, the file should still be made and it
            # should just be empty instead of not existing.
            pass


class JUnitSavedResults:
    @staticmethod
    def save(results, path):
        """
        Compile the internal results into JUnit format writting it to the
        given file.
        """
        results = JUnitTestSuites(results)
        with open(path, "w") as f:
            results.write(f)
