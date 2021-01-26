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
Contains the :class:`Loader` which is responsible for discovering and loading
tests.

Loading typically follows the following stages.

1. Recurse down a given directory looking for tests which match a given regex.

    The default regex used will match any python file (ending in .py) that has
    a name starting or ending in test(s). If there are any additional
    components of the name they must be connected with '-' or '_'. Lastly,
    file names that begin with '.' will be ignored.

    The following names would match:

    - `tests.py`
    - `test.py`
    - `test-this.py`
    - `tests-that.py`
    - `these-test.py`

    These would not match:

    - `.test.py`    - 'hidden' files are ignored.
    - `test`        - Must end in '.py'
    - `test-.py`    - Needs a character after the hypen.
    - `testthis.py` - Needs a hypen or underscore to separate 'test' and 'this'


2. With all files discovered execute each file gathering its test items we
   care about collecting. (`TestCase`, `TestSuite` and `Fixture` objects.)

As a final note, :class:`TestCase` instances which are not put into
a :class:`TestSuite` by the test writer will be placed into
a :class:`TestSuite` named after the module.

.. seealso:: :func:`load_file`
'''

import os
import re
import sys
import traceback

import testlib.log as log
import testlib.suite as suite_mod
import testlib.test_util as test_mod
import testlib.fixture as fixture_mod
import testlib.wrappers as wrappers
import testlib.uid as uid

class DuplicateTestItemException(Exception):
    '''
    Exception indicates multiple test items with the same UID
    were discovered.
    '''
    pass


# Match filenames that either begin or end with 'test' or tests and use
# - or _ to separate additional name components.
default_filepath_regex = re.compile(
            r'(((.+[_])?tests?)|(tests?([-_].+)?))\.py$')

def default_filepath_filter(filepath):
    '''The default filter applied to filepaths to marks as test sources.'''
    filepath = os.path.basename(filepath)
    if default_filepath_regex.match(filepath):
        # Make sure doesn't start with .
        return not filepath.startswith('.')
    return False

def path_as_modulename(filepath):
    '''Return the given filepath as a module name.'''
    # Remove the file extention (.py)
    return os.path.splitext(os.path.basename(filepath))[0]

def path_as_suitename(filepath):
    return os.path.split(os.path.dirname(os.path.abspath((filepath))))[-1]

def _assert_files_in_same_dir(files):
    if __debug__:
        if files:
            directory = os.path.dirname(files[0])
            for f in files:
                assert(os.path.dirname(f) == directory)

class Loader(object):
    '''
    Class for discovering tests.

    Discovered :class:`TestCase` and :class:`TestSuite` objects are wrapped by
    :class:`LoadedTest` and :class:`LoadedSuite` objects respectively.
    These objects provided additional methods and metadata about the loaded
    objects and are the internal representation used by testlib.

    To simply discover and load all tests using the default filter create an
    instance and `load_root`.

    >>> import os
    >>> tl = Loader()
    >>> tl.load_root(os.getcwd())

    .. note:: If tests are not contained in a TestSuite, they will
        automatically be placed into one for the module.

    .. warn:: This class is extremely thread-unsafe.
       It modifies the sys path and global config.
       Use with care.
    '''
    def __init__(self):
        self.suites = []
        self.suite_uids = {}
        self.filepath_filter = default_filepath_filter

        # filepath -> Successful | Failed to load
        self._files = {}

    @property
    def schedule(self):
        return wrappers.LoadedLibrary(self.suites)

    def load_schedule_for_suites(self, *uids):
        files = {uid.UID.uid_to_path(id_) for id_ in uids}
        for file_ in files:
            self.load_file(file_)

        return wrappers.LoadedLibrary(
                [self.suite_uids[id_] for id_ in uids])

    def _verify_no_duplicate_suites(self, new_suites):
        new_suite_uids = self.suite_uids.copy()
        for suite in new_suites:
            if suite.uid in new_suite_uids:
                raise DuplicateTestItemException(
                        "More than one suite with UID '%s' was defined" %\
                                suite.uid)
            new_suite_uids[suite.uid] = suite

    def _verify_no_duplicate_tests_in_suites(self, new_suites):
        for suite in new_suites:
            test_uids = set()
            for test in suite:
                if test.uid in test_uids:
                     raise DuplicateTestItemException(
                            "More than one test with UID '%s' was defined"
                            " in suite '%s'"
                            % (test.uid, suite.uid))
                test_uids.add(test.uid)

    def load_root(self, root):
        '''
        Load files from the given root directory which match
        `self.filepath_filter`.
        '''
        for directory in self._discover_files(root):
            directory = list(directory)
            if directory:
                _assert_files_in_same_dir(directory)
                for f in directory:
                    self.load_file(f)

    def load_file(self, path):
        path = os.path.abspath(path)

        if path in self._files:
            if not self._files[path]:
                raise Exception('Attempted to load a file which already'
                        ' failed to load')
            else:
                log.test_log.debug('Tried to reload: %s' % path)
                return

        # Create a custom dictionary for the loaded module.
        newdict = {
            '__builtins__':__builtins__,
            '__name__': path_as_modulename(path),
            '__file__': path,
        }

        # Add the file's containing directory to the system path. So it can do
        # relative imports naturally.
        old_path = sys.path[:]
        sys.path.insert(0, os.path.dirname(path))
        cwd = os.getcwd()
        os.chdir(os.path.dirname(path))

        new_tests = test_mod.TestCase.collector.create()
        new_suites = suite_mod.TestSuite.collector.create()
        new_fixtures = fixture_mod.Fixture.collector.create()

        try:
            exec(open(path).read(), newdict, newdict)
        except Exception as e:
            log.test_log.debug(traceback.format_exc())
            log.test_log.warn(
                              'Exception thrown while loading "%s"\n'
                              'Ignoring all tests in this file.'
                               % (path))
            # Clean up
            sys.path[:] = old_path
            os.chdir(cwd)
            test_mod.TestCase.collector.remove(new_tests)
            suite_mod.TestSuite.collector.remove(new_suites)
            fixture_mod.Fixture.collector.remove(new_fixtures)
            return

        # Create a module test suite for those not contained in a suite.
        orphan_tests = set(new_tests)
        for suite in new_suites:
            for test in suite:
                # Remove the test if it wasn't already removed.
                # (Suites may contain copies of tests.)
                if test in orphan_tests:
                    orphan_tests.remove(test)
        if orphan_tests:
            orphan_tests = sorted(orphan_tests, key=new_tests.index)
            # FIXME Use the config based default to group all uncollected
            # tests.
            # NOTE: This is automatically collected (we still have the
            # collector active.)
            suite_mod.TestSuite(tests=orphan_tests,
                    name=path_as_suitename(path))

        try:
            loaded_suites = [wrappers.LoadedSuite(suite, path)
                    for suite in new_suites]

            self._verify_no_duplicate_suites(loaded_suites)
            self._verify_no_duplicate_tests_in_suites(loaded_suites)
        except Exception as e:
            log.test_log.warn('%s\n'
                    'Exception thrown while loading "%s"\n'
                    'Ignoring all tests in this file.'
                    % (traceback.format_exc(), path))
        else:
            log.test_log.info('Discovered %d tests and %d suites in %s'
                    '' % (len(new_tests), len(loaded_suites), path))

            self.suites.extend(loaded_suites)
            self.suite_uids.update({suite.uid: suite
                    for suite in loaded_suites})
        # Clean up
        sys.path[:] = old_path
        os.chdir(cwd)
        test_mod.TestCase.collector.remove(new_tests)
        suite_mod.TestSuite.collector.remove(new_suites)
        fixture_mod.Fixture.collector.remove(new_fixtures)

    def _discover_files(self, root):
        '''
        Recurse down from the given root directory returning a list of
        directories which contain a list of files matching
        `self.filepath_filter`.
        '''
        # Will probably want to order this traversal.
        for root, dirnames, filenames in os.walk(root):
            dirnames.sort()
            if filenames:
                filenames.sort()
                filepaths = [os.path.join(root, filename) \
                             for filename in filenames]
                filepaths = filter(self.filepath_filter, filepaths)
                if filepaths:
                    yield filepaths
