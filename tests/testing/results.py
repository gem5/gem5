#!/usr/bin/env python
#
# Copyright (c) 2016 ARM Limited
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
# Authors: Andreas Sandberg

from abc import ABCMeta, abstractmethod
import inspect
import pickle
import string
import sys

import xml.etree.cElementTree as ET

class UnitResult(object):
    """Results of a single test unit.

    A test result can be one of:
        - STATE_OK: Test ran successfully.
        - STATE_SKIPPED: The test was skipped.
        - STATE_ERROR: The test failed to run.
        - STATE_FAILED: Test ran, but failed.

    The difference between STATE_ERROR and STATE_FAILED is very
    subtle. In a gem5 context, STATE_ERROR would mean that gem5 failed
    to start or crashed, while STATE_FAILED would mean that a test
    failed (e.g., statistics mismatch).

    """

    STATE_OK = 0
    STATE_SKIPPED = 1
    STATE_ERROR = 2
    STATE_FAILURE = 3

    state_names = {
        STATE_OK : "OK",
        STATE_SKIPPED : "SKIPPED",
        STATE_ERROR : "ERROR",
        STATE_FAILURE : "FAILURE",
    }

    def __init__(self, name, state, message="", stderr="", stdout="",
                 runtime=0.0):
        self.name = name
        self.state = state
        self.message = message
        self.stdout = stdout
        self.stderr = stderr
        self.runtime = runtime

    def skipped(self):
        return self.state == UnitResult.STATE_SKIPPED

    def success(self):
        return self.state == UnitResult.STATE_OK

    def state_name(self):
        return UnitResult.state_names[self.state]

    def __nonzero__(self):
        return self.success() or self.skipped()

    def __str__(self):
        state_name = self.state_name()

        status = "%s: %s" % (state_name, self.message) if self.message else \
                 state_name

        return "%s: %s" % (self.name, status)

class TestResult(object):
    """Results for from a single test consisting of one or more units."""

    def __init__(self, name, run_results=[], verify_results=[]):
        self.name = name
        self.results = run_results + verify_results
        self.run_results = run_results
        self.verify_results = verify_results

    def success(self):
        return self.success_run() and self.success_verify()

    def success_run(self):
        return all([ r.success() for r in self.run_results ])

    def success_verify(self):
        return all([ r.success() for r in self.verify_results ])

    def failed(self):
        return self.failed_run() or self.failed_verify()

    def failed_run(self):
        return any([ not r for r in self.run_results ])

    def failed_verify(self):
        return any([ not r for r in self.verify_results ])

    def skipped(self):
        return all([ r.skipped() for r in self.run_results ])

    def changed(self):
        return self.success_run() and self.failed_verify()

    def runtime(self):
        return sum([ r.runtime for r in self.results ])

    def __nonzero__(self):
        return all([ r for r in self.results ])

class ResultFormatter(object):
    __metaclass__ = ABCMeta

    def __init__(self, fout=sys.stdout, verbose=False):
        self.verbose = verbose
        self.fout = fout

    @abstractmethod
    def dump_suites(self, suites):
        pass

class Pickle(ResultFormatter):
    """Save test results as a binary using Python's pickle
    functionality.

    """

    def __init__(self, **kwargs):
        super(Pickle, self).__init__(**kwargs)

    def dump_suites(self, suites):
        pickle.dump(suites, self.fout, pickle.HIGHEST_PROTOCOL)

class Text(ResultFormatter):
    """Output test results as text."""

    def __init__(self, **kwargs):
        super(Text, self).__init__(**kwargs)

    def dump_suites(self, suites):
        fout = self.fout
        for suite in suites:
            print >> fout, "--- %s ---" % suite.name

            for t in suite.results:
                print >> fout, "*** %s" % t

                if t and not self.verbose:
                    continue

                if t.message:
                    print >> fout, t.message

                if t.stderr:
                    print >> fout, t.stderr
                if t.stdout:
                    print >> fout, t.stdout

class TextSummary(ResultFormatter):
    """Output test results as a text summary"""

    def __init__(self, **kwargs):
        super(TextSummary, self).__init__(**kwargs)

    def test_status(self, suite):
        if suite.skipped():
            return "SKIPPED"
        elif suite.changed():
            return "CHANGED"
        elif suite:
            return "OK"
        else:
            return "FAILED"

    def dump_suites(self, suites):
        fout = self.fout
        for suite in suites:
            status = self.test_status(suite)
            print >> fout, "%s: %s" % (suite.name, status)

class JUnit(ResultFormatter):
    """Output test results as JUnit XML"""

    def __init__(self, translate_names=True, **kwargs):
        super(JUnit, self).__init__(**kwargs)

        if translate_names:
            self.name_table = string.maketrans(
                "/.",
                ".-",
            )
        else:
            self.name_table = string.maketrans("", "")

    def convert_unit(self, x_suite, test):
        x_test = ET.SubElement(x_suite, "testcase",
                               name=test.name,
                               time="%f" % test.runtime)

        x_state = None
        if test.state == UnitResult.STATE_OK:
            pass
        elif test.state == UnitResult.STATE_SKIPPED:
            x_state = ET.SubElement(x_test, "skipped")
        elif test.state == UnitResult.STATE_FAILURE:
            x_state = ET.SubElement(x_test, "failure")
        elif test.state == UnitResult.STATE_ERROR:
            x_state = ET.SubElement(x_test, "error")
        else:
            assert False, "Unknown test state"

        if x_state is not None:
            if test.message:
                x_state.set("message", test.message)

            msg = []
            if test.stderr:
                msg.append("*** Standard Errror: ***")
                msg.append(test.stderr)
            if test.stdout:
                msg.append("*** Standard Out: ***")
                msg.append(test.stdout)

            x_state.text = "\n".join(msg)

        return x_test

    def convert_suite(self, x_suites, suite):
        x_suite = ET.SubElement(x_suites, "testsuite",
                                name=suite.name.translate(self.name_table),
                                time="%f" % suite.runtime())
        errors = 0
        failures = 0
        skipped = 0

        for test in suite.results:
            if test.state != UnitResult.STATE_OK:
                if test.state == UnitResult.STATE_SKIPPED:
                    skipped += 1
                elif test.state == UnitResult.STATE_ERROR:
                    errors += 1
                elif test.state == UnitResult.STATE_FAILURE:
                    failures += 1

            x_test = self.convert_unit(x_suite, test)

        x_suite.set("errors", str(errors))
        x_suite.set("failures", str(failures))
        x_suite.set("skipped", str(skipped))
        x_suite.set("tests", str(len(suite.results)))

        return x_suite

    def convert_suites(self, suites):
        x_root = ET.Element("testsuites")

        for suite in suites:
            self.convert_suite(x_root, suite)

        return x_root

    def dump_suites(self, suites):
        et = ET.ElementTree(self.convert_suites(suites))
        et.write(self.fout, encoding="UTF-8")
