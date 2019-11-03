#!/usr/bin/env python2.7
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

from __future__ import print_function

import argparse
import sys
import os
import pickle

from testing.tests import *
import testing.results

class ParagraphHelpFormatter(argparse.HelpFormatter):
    def _fill_text(self, text, width, indent):
        return "\n\n".join([
            super(ParagraphHelpFormatter, self)._fill_text(p, width, indent) \
            for p in text.split("\n\n") ])

formatters = {
    "junit" : testing.results.JUnit,
    "text" : testing.results.Text,
    "summary" : testing.results.TextSummary,
    "pickle" : testing.results.Pickle,
}


def _add_format_args(parser):
    parser.add_argument("--format", choices=formatters, default="text",
                        help="Output format")

    parser.add_argument("--no-junit-xlate-names", action="store_true",
                        help="Don't translate test names to " \
                        "package-like names")

    parser.add_argument("--output", "-o",
                        type=argparse.FileType('w'), default=sys.stdout,
                        help="Test result output file")


def _create_formatter(args):
    formatter = formatters[args.format]
    kwargs = {
        "fout" : args.output,
        "verbose" : args.verbose
    }

    if issubclass(formatter, testing.results.JUnit):
        kwargs.update({
            "translate_names" : not args.no_junit_xlate_names,
        })

    return formatter(**kwargs)


def _list_tests_args(subparsers):
    parser = subparsers.add_parser(
        "list",
        formatter_class=ParagraphHelpFormatter,
        help="List available tests",
        description="List available tests",
        epilog="""
        Generate a list of available tests using a list filter.

        The filter is a string consisting of the target ISA optionally
        followed by the test category and mode separated by
        slashes. The test names emitted by this command can be fed
        into the run command.

        For example, to list all quick arm tests, run the following:
        tests.py list arm/quick

        Non-mandatory parts of the filter string (anything other than
        the ISA) can be left out or replaced with the wildcard
        character. For example, all full-system tests can be listed
        with this command: tests.py list arm/*/fs""")

    parser.add_argument("--ruby-protocol", type=str, default=None,
                        help="Ruby protocol")

    parser.add_argument("--gpu-isa", type=str, default=None,
                        help="GPU ISA")

    parser.add_argument("list_filter", metavar="ISA[/category/mode]",
                        action="append", type=str,
                        help="List available test cases")

def _list_tests(args):
    for isa, categories, modes in \
        ( parse_test_filter(f) for f in args.list_filter ):

        for test in get_tests(isa, categories=categories, modes=modes,
                              ruby_protocol=args.ruby_protocol,
                              gpu_isa=args.gpu_isa):
            print("/".join(test))
    sys.exit(0)

def _run_tests_args(subparsers):
    parser = subparsers.add_parser(
        "run",
        formatter_class=ParagraphHelpFormatter,
        help='Run one or more tests',
        description="Run one or more tests.",
        epilog="""
        Run one or more tests described by a gem5 test tuple.

        The test tuple consists of a test category (quick or long), a
        test mode (fs or se), a workload name, an isa, an operating
        system, and a config name separate by slashes. For example:
        quick/se/00.hello/arm/linux/simple-timing

        Available tests can be listed using the 'list' sub-command
        (e.g., "tests.py list arm/quick" or one of the scons test list
        targets (e.g., "scons build/ARM/tests/opt/quick.list").

        The test results can be stored in multiple different output
        formats. See the help for the show command for more details
        about output formatting.""")

    parser.add_argument("gem5", type=str,
                        help="gem5 binary")

    parser.add_argument("test", type=str, nargs="*",
                        help="List of tests to execute")

    parser.add_argument("--directory", "-d",
                        type=str, default="m5tests",
                        help="Test work directory")

    parser.add_argument("--timeout", "-t",
                        type=int, default="0", metavar="MINUTES",
                        help="Timeout, 0 to disable")

    parser.add_argument("--skip-diff-out", action="store_true",
                        help="Skip output diffing stage")

    parser.add_argument("--skip-diff-stat", action="store_true",
                        help="Skip stat diffing stage")

    _add_format_args(parser)

def _run_tests(args):
    if not os.path.isfile(args.gem5) or not os.access(args.gem5, os.X_OK):
        print("gem5 binary '%s' not an executable file" % args.gem5,
            file=sys.stderr)
        sys.exit(2)

    formatter = _create_formatter(args)

    out_base = os.path.abspath(args.directory)
    if not os.path.exists(out_base):
        os.mkdir(out_base)
    tests = []
    for test_name in args.test:
        config = ClassicConfig(*test_name.split("/"))
        out_dir = os.path.join(out_base, "/".join(config))
        tests.append(
            ClassicTest(args.gem5, out_dir, config,
                        timeout=args.timeout,
                        skip_diff_stat=args.skip_diff_stat,
                        skip_diff_out=args.skip_diff_out))

    all_results = []
    print("Running %i tests" % len(tests))
    for testno, test in enumerate(tests):
        print("%i: Running '%s'..." % (testno, test))

        all_results.append(test.run())

    formatter.dump_suites(all_results)

def _show_args(subparsers):
    parser = subparsers.add_parser(
        "show",
        formatter_class=ParagraphHelpFormatter,
        help='Display pickled test results',
        description='Display pickled test results',
        epilog="""
        Reformat the pickled output from one or more test runs. This
        command is typically used with the output from a single test
        run, but it can also be used to merge the outputs from
        multiple runs.

        The 'text' format is a verbose output format that provides
        information about individual test units and the output from
        failed tests. It's mainly useful for debugging test failures.

        The 'summary' format provides outputs the results of one test
        per line with the test's overall status (OK, SKIPPED, or
        FAILED).

        The 'junit' format is primarily intended for use with CI
        systems. It provides an XML representation of test
        status. Similar to the text format, it includes detailed
        information about test failures. Since many JUnit parser make
        assume that test names look like Java packet strings, the
        JUnit formatter automatically to something the looks like a
        Java class path ('.'->'-', '/'->'.').

        The 'pickle' format stores the raw results in a format that
        can be reformatted using this command. It's typically used
        with the show command to merge multiple test results into one
        pickle file.""")

    _add_format_args(parser)

    parser.add_argument("result", type=argparse.FileType("rb"), nargs="*",
                        help="Pickled test results")

def _show(args):
    def _load(f):
        # Load the pickled status file, sometimes e.g., when a
        # regression is still running the status file might be
        # incomplete.
        try:
            return pickle.load(f)
        except EOFError:
            print('Could not read file %s' % f.name, file=sys.stderr)
            return []

    formatter = _create_formatter(args)
    suites = sum([ _load(f) for f in args.result ], [])
    formatter.dump_suites(suites)

def _test_args(subparsers):
    parser = subparsers.add_parser(
        "test",
        formatter_class=ParagraphHelpFormatter,
        help='Probe test results and set exit code',
        epilog="""

        Load one or more pickled test file and return an exit code
        corresponding to the test outcome. The following exit codes
        can be returned:

        0: All tests were successful or skipped.

        1: General fault in the script such as incorrect parameters or
        failing to parse a pickle file.

        2: At least one test failed to run. This is what the summary
        formatter usually shows as a 'FAILED'.

        3: All tests ran correctly, but at least one failed to
        verify its output. When displaying test output using the
        summary formatter, such a test would show up as 'CHANGED'.
        """)

    parser.add_argument("result", type=argparse.FileType("rb"), nargs="*",
                        help="Pickled test results")

def _test(args):
    try:
        suites = sum([ pickle.load(f) for f in args.result ], [])
    except EOFError:
        print('Could not read all files', file=sys.stderr)
        sys.exit(2)

    if all(s for s in suites):
        sys.exit(0)
    elif any([ s.failed_run() for s in suites ]):
        sys.exit(2)
    elif any([ s.changed() for s in suites ]):
        sys.exit(3)
    else:
        assert False, "Unexpected return status from test"

_commands = {
    "list" : (_list_tests, _list_tests_args),
    "run" : (_run_tests, _run_tests_args),
    "show" : (_show, _show_args),
    "test" : (_test, _test_args),
}

def main():
    parser = argparse.ArgumentParser(
        formatter_class=ParagraphHelpFormatter,
        description="""gem5 testing multi tool.""",
        epilog="""
        This tool provides an interface to gem5's test framework that
        doesn't depend on gem5's build system. It supports test
        listing, running, and output formatting.

        The list sub-command (e.g., "test.py list arm/quick") produces
        a list of tests tuples that can be used by the run command
        (e.g., "tests.py run gem5.opt
        quick/se/00.hello/arm/linux/simple-timing").

        The run command supports several output formats. One of them,
        pickle, contains the raw output from the tests and can be
        re-formatted using the show command (e.g., "tests.py show
        --format summary *.pickle"). Such pickle files are also
        generated by the build system when scons is used to run
        regressions.

        See the usage strings for the individual sub-commands for
        details.""")

    parser.add_argument("--verbose", action="store_true",
                        help="Produce more verbose output")

    subparsers = parser.add_subparsers(dest="command")

    for key, (impl, cmd_parser) in _commands.items():
        cmd_parser(subparsers)

    args = parser.parse_args()
    impl, cmd_parser = _commands[args.command]
    impl(args)

if __name__ == "__main__":
    main()
