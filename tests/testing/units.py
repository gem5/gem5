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
from datetime import datetime
import difflib
import functools
import os
import re
import subprocess
import sys
import traceback

from results import UnitResult
from helpers import *

_test_base = os.path.join(os.path.dirname(__file__), "..")

class TestUnit(object):
    """Base class for all test units.

    A test unit is a part of a larger test case. Test cases usually
    contain two types of units, run units (run gem5) and verify units
    (diff output files). All unit implementations inherit from this
    class.

    A unit implementation overrides the _run() method. The test runner
    calls the run() method, which wraps _run() to protect against
    exceptions.

    """

    __metaclass__ = ABCMeta

    def __init__(self, name, ref_dir, test_dir, skip=False):
        self.name = name
        self.ref_dir = ref_dir
        self.test_dir = test_dir
        self.force_skip = skip
        self.start_time = None
        self.stop_time = None

    def result(self, state, **kwargs):
        if self.start_time is not None and "runtime" not in kwargs:
            self.stop_time = datetime.utcnow()
            delta = self.stop_time - self.start_time
            kwargs["runtime"] = delta.total_seconds()

        return UnitResult(self.name, state, **kwargs)

    def ok(self, **kwargs):
        return self.result(UnitResult.STATE_OK, **kwargs)

    def skip(self, **kwargs):
        return self.result(UnitResult.STATE_SKIPPED, **kwargs)

    def error(self, message, **kwargs):
        return self.result(UnitResult.STATE_ERROR, message=message, **kwargs)

    def failure(self, message, **kwargs):
        return self.result(UnitResult.STATE_FAILURE, message=message, **kwargs)

    def ref_file(self, fname):
        return os.path.join(self.ref_dir, fname)

    def out_file(self, fname):
        return os.path.join(self.test_dir, fname)

    def _read_output(self, fname, default=""):
        try:
            with open(self.out_file(fname), "r") as f:
                return f.read()
        except IOError:
            return default

    def run(self):
        self.start_time = datetime.utcnow()
        try:
            if self.force_skip:
                return self.skip()
            else:
                return self._run()
        except:
            return self.error("Python exception:\n%s" % traceback.format_exc())

    @abstractmethod
    def _run(self):
        pass

class RunGem5(TestUnit):
    """Test unit representing a gem5 run.

    Possible failure modes:
       - gem5 failed to run -> STATE_ERROR
       - timeout -> STATE_ERROR
       - non-zero exit code -> STATE_ERROR

    Possible non-failure results:
       - exit code == 0 -> STATE_OK
       - exit code == 2 -> STATE_SKIPPED
    """

    def __init__(self, gem5, gem5_args, timeout=0, **kwargs):
        super(RunGem5, self).__init__("gem5", **kwargs)
        self.gem5 = gem5
        self.args = gem5_args
        self.timeout = timeout

    def _run(self):
        gem5_cmd = [
            self.gem5,
            "-d", self.test_dir,
            "-re",
        ] + self.args

        try:
            with ProcessHelper(gem5_cmd, stdout=subprocess.PIPE,
                               stderr=subprocess.PIPE) as p:
                status, gem5_stdout, gem5_stderr = p.call(timeout=self.timeout)
        except CallTimeoutException as te:
            return self.error("Timeout", stdout=te.stdout, stderr=te.stderr)
        except OSError as ose:
            return self.error("Failed to launch gem5: %s" % ose)

        stderr = "\n".join([
            "*** gem5 stderr ***",
            gem5_stderr,
            "",
            "*** m5out/simerr ***",
            self._read_output("simerr"),
        ])

        stdout = "\n".join([
            "*** gem5 stdout ***",
            gem5_stdout,
            "",
            "*** m5out/simout ***",
            self._read_output("simout"),
        ])

        # Signal
        if status < 0:
            return self.error("gem5 terminated by signal %i" % (-status, ),
                              stdout=stdout, stderr=stderr)
        elif status == 2:
            return self.skip(stdout=stdout, stderr=stderr)
        elif status > 0:
            return self.error("gem5 exited with non-zero status: %i" % status,
                              stdout=stdout, stderr=stderr)
        else:
            return self.ok(stdout=stdout, stderr=stderr)

class DiffOutFile(TestUnit):
    """Test unit comparing and output file and a reference file."""

    # regular expressions of lines to ignore when diffing outputs
    diff_ignore_regexes = {
        "simout" : [
            re.compile('^Redirecting (stdout|stderr) to'),
            re.compile('^gem5 compiled '),
            re.compile('^gem5 started '),
            re.compile('^gem5 executing on '),
            re.compile('^command line:'),
            re.compile("^Couldn't import dot_parser,"),
            re.compile("^info: kernel located at:"),
            re.compile("^Couldn't unlink "),
            re.compile("^Using GPU kernel code file\(s\) "),
        ],
        "simerr" : [
            #re.compile('^Simulation complete at'),
        ],
        "config.ini" : [
            re.compile("^(executable|readfile|kernel|image_file)="),
            re.compile("^(cwd|input|codefile)="),
        ],
        "config.json" : [
            re.compile(r'''^\s*"(executable|readfile|kernel|image_file)":'''),
            re.compile(r'''^\s*"(cwd|input|codefile)":'''),
        ],
    }

    def __init__(self, fname, **kwargs):
        super(DiffOutFile, self).__init__("diff[%s]" % fname,
                                          **kwargs)

        self.fname = fname
        self.line_filters = DiffOutFile.diff_ignore_regexes.get(fname, tuple())

    def _filter_file(self, fname):
        def match_line(l):
            for r in self.line_filters:
                if r.match(l):
                    return True
            return False

        with open(fname, "r") as f:
            for l in f:
                if not match_line(l):
                    yield l


    def _run(self):
        fname = self.fname
        ref = self.ref_file(fname)
        out = self.out_file(fname)

        if not os.path.exists(ref):
            return self.error("%s doesn't exist in reference directory" \
                              % fname)

        if not os.path.exists(out):
            return self.error("%s doesn't exist in output directory" % fname)

        diff = difflib.unified_diff(
            tuple(self._filter_file(ref)),
            tuple(self._filter_file(out)),
            fromfile="ref/%s" % fname, tofile="out/%s" % fname)

        diff = list(diff)
        if diff:
            return self.error("ref/%s and out/%s differ" % (fname, fname),
                              stderr="".join(diff))
        else:
            return self.ok(stdout="-- ref/%s and out/%s are identical --" \
                           % (fname, fname))

class DiffStatFile(TestUnit):
    """Test unit comparing two gem5 stat files."""

    def __init__(self, **kwargs):
        super(DiffStatFile, self).__init__("stat_diff", **kwargs)

        self.stat_diff = os.path.join(_test_base, "diff-out")

    def _run(self):
        stats = "stats.txt"

        cmd = [
            self.stat_diff,
            self.ref_file(stats), self.out_file(stats),
        ]
        with ProcessHelper(cmd,
                           stdout=subprocess.PIPE,
                           stderr=subprocess.PIPE) as p:
            status, stdout, stderr = p.call()

        if status == 0:
            return self.ok(stdout=stdout, stderr=stderr)
        if status == 1:
            return self.failure("Statistics mismatch",
                                stdout=stdout, stderr=stderr)
        else:
            return self.error("diff-out returned an error: %i" % status,
                              stdout=stdout, stderr=stderr)
