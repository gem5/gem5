# Copyright (c) 2021 Arm Limited
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
# Copyright (c) 2021 Huawei International
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

"""
Built in test cases that verify particular details about a gem5 run.
"""
import re
import os
import json

from testlib import test_util
from testlib.configuration import constants
from testlib.helper import joinpath, diff_out_file


class Verifier(object):
    def __init__(self, fixtures=tuple()):
        self.fixtures = fixtures

    def _test(self, *args, **kwargs):
        # Use a callback wrapper to make stack
        # traces easier to understand.
        self.test(*args, **kwargs)

    def instantiate_test(self, name_pfx):
        name = "-".join([name_pfx, self.__class__.__name__])
        return test_util.TestFunction(
            self._test, name=name, fixtures=self.fixtures
        )


class CheckH5StatsExist(Verifier):
    def __init__(self, stats_file="stats.h5"):
        super(CheckH5StatsExist, self).__init__()
        self.stats_file = stats_file

    def test(self, params):
        tempdir = params.fixtures[constants.tempdir_fixture_name].path
        h5_file = joinpath(tempdir, self.stats_file)
        if not os.path.isfile(h5_file):
            test_util.fail("Could not find h5 stats file %s", h5_file)


class MatchGoldStandard(Verifier):
    """
    Compares a standard output to the test output and passes if they match,
    fails if they do not.
    """

    def __init__(
        self, standard_filename, ignore_regex=None, test_filename="simout"
    ):
        """
        :param standard_filename: The path of the standard file to compare
        output to.

        :param ignore_regex: A string, compiled regex, or iterable containing
        either which will be ignored in 'standard' and test output files when
        diffing.
        """
        super(MatchGoldStandard, self).__init__()
        self.standard_filename = standard_filename
        self.test_filename = test_filename

        self.ignore_regex = _iterable_regex(ignore_regex)

    def test(self, params):
        # We need a tempdir fixture from our parent verifier suite.
        fixtures = params.fixtures
        # Get the file from the tempdir of the test.
        tempdir = fixtures[constants.tempdir_fixture_name].path
        self.test_filename = joinpath(tempdir, self.test_filename)

        diff = diff_out_file(
            self.standard_filename,
            self.test_filename,
            ignore_regexes=self.ignore_regex,
            logger=params.log,
        )
        if diff is not None:
            test_util.fail(
                f"Stdout did not match:\n{diff}\nSee {tempdir} for full results"
            )

    def _generic_instance_warning(self, kwargs):
        """
        Method for helper classes to tell users to use this more generic class
        if they are going to manually override the test_filename param.
        """
        if "test_filename" in kwargs:
            raise ValueError(
                "If you are setting test_filename use the more"
                " generic %s"
                " instead" % MatchGoldStandard.__name__
            )


class DerivedGoldStandard(MatchGoldStandard):
    __ignore_regex_sentinel = object()
    _file = None
    _default_ignore_regex = []

    def __init__(
        self, standard_filename, ignore_regex=__ignore_regex_sentinel, **kwargs
    ):

        if ignore_regex == self.__ignore_regex_sentinel:
            ignore_regex = self._default_ignore_regex

        self._generic_instance_warning(kwargs)

        super(DerivedGoldStandard, self).__init__(
            standard_filename,
            test_filename=self._file,
            ignore_regex=ignore_regex,
            **kwargs,
        )


class MatchStdout(DerivedGoldStandard):
    _file = constants.gem5_simulation_stdout
    _default_ignore_regex = [
        re.compile("^\s+$"),  # Remove blank lines.
        re.compile("^gem5 Simulator System"),
        re.compile("^gem5 is copyrighted software"),
        re.compile("^Redirecting (stdout|stderr) to"),
        re.compile("^gem5 version "),
        re.compile("^gem5 compiled "),
        re.compile("^gem5 started "),
        re.compile("^gem5 executing on "),
        re.compile("^command line:"),
        re.compile("^Couldn't import dot_parser,"),
        re.compile("^info: kernel located at:"),
        re.compile("^info: Standard input is not a terminal"),
        re.compile("^Couldn't unlink "),
        re.compile("^Using GPU kernel code file\(s\) "),
        re.compile("^.* not found locally\. Downloading"),
        re.compile("^Finished downloading"),
        re.compile("^info: Using default config"),
    ]


class MatchStdoutNoPerf(MatchStdout):
    _file = constants.gem5_simulation_stdout
    _default_ignore_regex = MatchStdout._default_ignore_regex + [
        re.compile("^Exiting @ tick")
    ]


class MatchStderr(DerivedGoldStandard):
    _file = constants.gem5_simulation_stderr
    _default_ignore_regex = []


class MatchStats(DerivedGoldStandard):
    # TODO: Likely will want to change this verifier since we have the weird
    # perl script right now. A simple diff probably isn't going to work.
    _file = constants.gem5_simulation_stats
    _default_ignore_regex = []


class MatchConfigINI(DerivedGoldStandard):
    _file = constants.gem5_simulation_config_ini
    _default_ignore_regex = (
        re.compile("^(executable|readfile|kernel|image_file)="),
        re.compile("^(cwd|input|codefile)="),
    )


class MatchConfigJSON(DerivedGoldStandard):
    _file = constants.gem5_simulation_config_json
    _default_ignore_regex = (
        re.compile(r"""^\s*"(executable|readfile|kernel|image_file)":"""),
        re.compile(r"""^\s*"(cwd|input|codefile)":"""),
    )


class MatchFileRegex(Verifier):
    """
    Looking for a match between a regex pattern and the content of a list
    of files. Verifier will pass as long as the pattern is found in at least
    one of the files.
    """

    def __init__(self, regex, filenames):
        super(MatchFileRegex, self).__init__()
        self.regex = _iterable_regex(regex)
        self.filenames = filenames

    def parse_file(self, fname):
        with open(fname, "r") as file_:
            for line in file_:
                for regex in self.regex:
                    if re.match(regex, line):
                        return True

    def test(self, params):
        fixtures = params.fixtures
        # Get the file from the tempdir of the test.
        tempdir = fixtures[constants.tempdir_fixture_name].path

        for fname in self.filenames:
            if self.parse_file(joinpath(tempdir, fname)):
                return  # Success

        test_util.fail("Could not match regex.")


class MatchRegex(MatchFileRegex):
    """
    Looking for a match between a regex pattern and stdout/stderr.
    """

    def __init__(self, regex, match_stderr=True, match_stdout=True):
        filenames = list()
        if match_stdout:
            filenames.append(constants.gem5_simulation_stdout)
        if match_stderr:
            filenames.append(constants.gem5_simulation_stderr)
        super(MatchRegex, self).__init__(regex, filenames)


class NoMatchRegex(MatchRegex):
    """
    Checks that the given pattern does *not* match
    """

    def __init__(self, regex, match_stderr=True, match_stdout=True):
        super(NoMatchRegex, self).__init__(regex, match_stderr, match_stdout)

    def test(self, params):
        fixtures = params.fixtures
        tempdir = fixtures[constants.tempdir_fixture_name].path

        for fname in self.filenames:
            if self.parse_file(joinpath(tempdir, fname)):
                test_util.fail("Could not match regex.")


class MatchJSONStats(Verifier):
    """
    Verifer to check the correctness of stats reported by gem5. It uses
    gem5stats to store the stastistics as json files and does the comparison.
    """

    def __init__(
        self,
        truth_name: str,
        test_name: str,
        test_name_in_outdir: bool = False,
    ):
        """
        :param truth_dir: The path to the directory including the trusted_stats
        for this test.
        :param test_name_in_m5out: True if the 'test_name' dir is to found in
        the `m5.options.outdir`.
        """
        super(MatchJSONStats, self).__init__()
        self.truth_name = truth_name
        self.test_name = test_name
        self.test_name_in_outdir = test_name_in_outdir

    def _compare_stats(self, trusted_file, test_file):
        trusted_stats = json.load(trusted_file)
        test_stats = json.load(test_file)
        is_subset = trusted_stats.items() <= test_stats.items()
        if is_subset:
            err = (
                "Following differences found between "
                + f"{self.truth_name} and {self.test_name}.\n"
            )
            diffs = set(trusted_stats.items()) - set(test_stats.items())
            for diff in diffs:
                trusted_value = trusted_stats[diff[0]]
                test_value = None
                if diff[0] in test_stats.keys():
                    test_value = test_stats[diff[0]]
                err += f"{diff[0]}:\n"
                err += (
                    f"trusted_value: {trusted_value}, "
                    + f"test_value: {test_value}"
                )
            test_util.fail(err)

    def test(self, params):
        trusted_file = open(self.truth_name, "r")
        if self.test_name_in_outdir:
            fixtures = params.fixtures
            tempdir = fixtures[constants.tempdir_fixture_name].path
            test_file = open(joinpath(tempdir, self.test_name), "r")
        else:
            test_file = open(self.test_name, "r")

        return self._compare_stats(trusted_file, test_file)


_re_type = type(re.compile(""))


def _iterable_regex(regex):
    if not regex:
        return ()  # If no regex we return an empty tuple.
    if isinstance(regex, _re_type) or isinstance(regex, str):
        regex = (regex,)
    return regex
