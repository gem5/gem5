#!/usr/bin/env python2
#
# Copyright 2018 Google, Inc.
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
# Authors: Gabe Black

from __future__ import print_function

import argparse
import functools
import inspect
import itertools
import json
import logging
import os
import subprocess
import sys

script_path = os.path.abspath(inspect.getfile(inspect.currentframe()))
script_dir = os.path.dirname(script_path)
config_path = os.path.join(script_dir, 'config.py')

systemc_rel_path = 'systemc'
tests_rel_path = os.path.join(systemc_rel_path, 'tests')
json_rel_path = os.path.join(tests_rel_path, 'tests.json')



logging.basicConfig(level=logging.INFO)

def scons(*args):
    args = ['scons'] + list(args)
    subprocess.check_call(args)



class Test(object):
    def __init__(self, target, suffix, build_dir, props):
        self.target = target
        self.suffix = suffix
        self.build_dir = build_dir

        for key, val in props.iteritems():
            setattr(self, key, val)

    def dir(self):
        return os.path.join(self.build_dir, tests_rel_path, self.path)

    def src_dir(self):
        return os.path.join(script_dir, self.path)

    def golden_dir(self):
        return os.path.join(self.src_dir(), 'golden')

    def bin(self):
        return '.'.join([self.name, self.suffix])

    def full_path(self):
        return os.path.join(self.dir(), self.bin())

    def m5out_dir(self):
        return os.path.join(self.dir(), 'm5out.' + self.suffix)



test_phase_classes = {}

class TestPhaseMeta(type):
    def __init__(cls, name, bases, d):
        if not d.pop('abstract', False):
            test_phase_classes[d['name']] = cls

        super(TestPhaseMeta, cls).__init__(name, bases, d)

class TestPhaseBase(object):
    __metaclass__ = TestPhaseMeta
    abstract = True

    def __init__(self, main_args, *args):
        self.main_args = main_args
        self.args = args

    def __lt__(self, other):
        return self.number < other.number

class CompilePhase(TestPhaseBase):
    name = 'compile'
    number = 1

    def run(self, tests):
        targets = list([test.full_path() for test in tests])
        scons_args = list(self.args) + targets
        scons(*scons_args)

class RunPhase(TestPhaseBase):
    name = 'execute'
    number = 2

    def run(self, tests):
        for test in tests:
            if test.compile_only:
                continue
            args = [
                test.full_path(),
                '-red', test.m5out_dir(),
                '--listener-mode=off',
                config_path
            ]
            subprocess.check_call(args)

class VerifyPhase(TestPhaseBase):
    name = 'verify'
    number = 3

    def run(self, tests):
        for test in tests:
            if test.compile_only:
                continue
            logging.info("Would verify %s", test.m5out_dir())



parser = argparse.ArgumentParser(description='SystemC test utility')

parser.add_argument('build_dir', metavar='BUILD_DIR',
                    help='The build directory (ie. build/ARM).')

parser.add_argument('--update-json', action='store_true',
                    help='Update the json manifest of tests.')

parser.add_argument('--flavor', choices=['debug', 'opt', 'fast'],
                    default='opt',
                    help='Flavor of binary to test.')

parser.add_argument('--list', action='store_true',
                    help='List the available tests')

filter_opts = parser.add_mutually_exclusive_group()
filter_opts.add_argument('--filter', default='True',
                         help='Python expression which filters tests based '
                         'on their properties')
filter_opts.add_argument('--filter-file', default=None,
                         type=argparse.FileType('r'),
                         help='Same as --filter, but read from a file')

def collect_phases(args):
    phase_groups = [list(g) for k, g in
                    itertools.groupby(args, lambda x: x != '--phase') if k]
    main_args = parser.parse_args(phase_groups[0][1:])
    phases = []
    names = []
    for group in phase_groups[1:]:
        name = group[0]
        if name in names:
            raise RuntimeException('Phase %s specified more than once' % name)
        phase = test_phase_classes[name]
        phases.append(phase(main_args, *group[1:]))
    phases.sort()
    return main_args, phases

main_args, phases = collect_phases(sys.argv)

if len(phases) == 0:
    phases = [
        CompilePhase(main_args),
        RunPhase(main_args),
        VerifyPhase(main_args)
    ]



json_path = os.path.join(main_args.build_dir, json_rel_path)

if main_args.update_json:
    scons(os.path.join(json_path))

with open(json_path) as f:
    test_data = json.load(f)

    if main_args.filter_file:
        f = main_args.filter_file
        filt = compile(f.read(), f.name, 'eval')
    else:
        filt = compile(main_args.filter, '<string>', 'eval')

    filtered_tests = {
        target: props for (target, props) in
                    test_data.iteritems() if eval(filt, dict(props))
    }

    if main_args.list:
        for target, props in sorted(filtered_tests.iteritems()):
            print('%s.%s' % (target, main_args.flavor))
            for key, val in props.iteritems():
                print('    %s: %s' % (key, val))
        print('Total tests: %d' % len(filtered_tests))
    else:
        tests_to_run = list([
            Test(target, main_args.flavor, main_args.build_dir, props) for
                target, props in sorted(filtered_tests.iteritems())
        ])

        for phase in phases:
            phase.run(tests_to_run)
