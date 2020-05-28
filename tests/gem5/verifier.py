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

'''
Built in test cases that verify particular details about a gem5 run.
'''
import re

from testlib import test_util as test
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
        name = '-'.join([name_pfx, self.__class__.__name__])
        return test.TestFunction(self._test,
                name=name, fixtures=self.fixtures)

class MatchGoldStandard(Verifier):
    '''
    Compares a standard output to the test output and passes if they match,
    fails if they do not.
    '''
    def __init__(self, standard_filename, ignore_regex=None,
                 test_filename='simout'):
        '''
        :param standard_filename: The path of the standard file to compare
        output to.

        :param ignore_regex: A string, compiled regex, or iterable containing
        either which will be ignored in 'standard' and test output files when
        diffing.
        '''
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

        diff = diff_out_file(self.standard_filename,
                            self.test_filename,
                            ignore_regexes=self.ignore_regex,
                            logger=params.log)
        if diff is not None:
            test.fail('Stdout did not match:\n%s\nSee %s for full results'
                      % (diff, tempdir))

    def _generic_instance_warning(self, kwargs):
        '''
        Method for helper classes to tell users to use this more generic class
        if they are going to manually override the test_filename param.
        '''
        if 'test_filename' in kwargs:
            raise ValueError('If you are setting test_filename use the more'
                             ' generic %s'
                             ' instead' % MatchGoldStandard.__name__)

class DerivedGoldStandard(MatchGoldStandard):
    __ignore_regex_sentinel = object()
    _file = None
    _default_ignore_regex = []

    def __init__(self, standard_filename,
                 ignore_regex=__ignore_regex_sentinel, **kwargs):

        if ignore_regex == self.__ignore_regex_sentinel:
            ignore_regex = self._default_ignore_regex

        self._generic_instance_warning(kwargs)

        super(DerivedGoldStandard, self).__init__(
            standard_filename,
            test_filename=self._file,
            ignore_regex=ignore_regex,
            **kwargs)

class MatchStdout(DerivedGoldStandard):
    _file = constants.gem5_simulation_stdout
    _default_ignore_regex = [
            re.compile('^Redirecting (stdout|stderr) to'),
            re.compile('^gem5 version '),
            re.compile('^gem5 compiled '),
            re.compile('^gem5 started '),
            re.compile('^gem5 executing on '),
            re.compile('^command line:'),
            re.compile("^Couldn't import dot_parser,"),
            re.compile("^info: kernel located at:"),
            re.compile("^info: Standard input is not a terminal"),
            re.compile("^Couldn't unlink "),
            re.compile("^Using GPU kernel code file\(s\) "),
        ]

class MatchStdoutNoPerf(MatchStdout):
    _file = constants.gem5_simulation_stdout
    _default_ignore_regex = MatchStdout._default_ignore_regex + [
            re.compile('^Exiting @ tick'),
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
            re.compile(r'''^\s*"(executable|readfile|kernel|image_file)":'''),
            re.compile(r'''^\s*"(cwd|input|codefile)":'''),
            )

class MatchRegex(Verifier):
    def __init__(self, regex, match_stderr=True, match_stdout=True):
        super(MatchRegex, self).__init__()
        self.regex = _iterable_regex(regex)
        self.match_stderr = match_stderr
        self.match_stdout = match_stdout

    def test(self, params):
        fixtures = params.fixtures
        # Get the file from the tempdir of the test.
        tempdir = fixtures[constants.tempdir_fixture_name].path

        def parse_file(fname):
            with open(fname, 'r') as file_:
                for line in file_:
                    for regex in self.regex:
                        if re.match(regex, line):
                            return True
        if self.match_stdout:
            if parse_file(joinpath(tempdir,
                                   constants.gem5_simulation_stdout)):
                return # Success
        if self.match_stderr:
            if parse_file(joinpath(tempdir,
                                   constants.gem5_simulation_stderr)):
                return # Success
        test.fail('Could not match regex.')

_re_type = type(re.compile(''))
def _iterable_regex(regex):
    if isinstance(regex, _re_type) or isinstance(regex, str):
        regex = (regex,)
    return regex
