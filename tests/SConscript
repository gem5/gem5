# -*- mode:python -*-
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
# Copyright (c) 2004-2006 The Regents of The University of Michigan
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
# Authors: Steve Reinhardt
#          Kevin Lim
#          Andreas Sandberg

from __future__ import print_function

from SCons.Script.SConscript import SConsEnvironment
import os
import pickle
import sys

sys.path.insert(0, Dir(".").srcnode().abspath)
import testing.tests as tests
import testing.results as results
from gem5_scons.util import get_termcap

Import('env')

# get the termcap from the environment
termcap = get_termcap()

# Dict that accumulates lists of tests by category (quick, medium, long)
env.Tests = {}
gpu_isa = env['TARGET_GPU_ISA'] if env['BUILD_GPU'] else None
for cat in tests.all_categories:
    env.Tests[cat] = tuple(
        tests.get_tests(env["TARGET_ISA"],
                        categories=(cat, ),
                        ruby_protocol=env["PROTOCOL"],
                        gpu_isa=gpu_isa))

def color_message(color, msg):
    return color + msg + termcap.Normal

def run_test(target, source, env):
    """Run a test and produce results as a pickle file.

    Targets are as follows:
    target[0] : Pickle file

    Sources are:
    source[0] : gem5 binary
    source[1] : tests/run.py script
    source[2:] : reference files

    """
    tgt_dir = os.path.dirname(str(target[0]))
    config = tests.ClassicConfig(*tgt_dir.split('/')[-6:])
    test = tests.ClassicTest(source[0].abspath, tgt_dir, config,
                             timeout=5*60*60,
                             skip_diff_out=True)

    for ref in test.ref_files():
        out_file = os.path.join(tgt_dir, ref)
        if os.path.exists(out_file):
            env.Execute(Delete(out_file))

    with open(target[0].abspath, "wb") as fout:
        formatter = results.Pickle(fout=fout)
        formatter.dump_suites([ test.run() ])

    return 0

def run_test_string(target, source, env):
    return env.subst("Running test in ${TARGETS[0].dir}.",
                     target=target, source=source)

testAction = env.Action(run_test, run_test_string)

def print_test(target, source, env):
    """Run a test and produce results as a pickle file.

    Targets are as follows:
    target[*] : Dummy targets

    Sources are:
    source[0] : Pickle file

    """
    with open(source[0].abspath, "rb") as fin:
        result = pickle.load(fin)

    assert len(result) == 1
    result = result[0]

    formatter = None
    if result.skipped():
        status = color_message(termcap.Cyan, "skipped.")
    elif result.changed():
        status = color_message(termcap.Yellow, "CHANGED!")
        formatter = results.Text()
    elif result:
        status = color_message(termcap.Green, "passed.")
    else:
        status = color_message(termcap.Red, "FAILED!")
        formatter = results.Text()

    if formatter:
        formatter.dump_suites([result])

    print("***** %s: %s" % (source[0].dir, status))
    return 0

printAction = env.Action(print_test, strfunction=None)

def update_test(target, source, env):
    """Update test reference data

    Targets are as follows:
    target[0] : Dummy file

    Sources are:
    source[0] : Pickle file
    """

    src_dir = os.path.dirname(str(source[0]))
    config = tests.ClassicConfig(*src_dir.split('/')[-6:])
    test = tests.ClassicTest(source[0].abspath, src_dir, config)
    ref_dir = test.ref_dir

    with open(source[0].abspath, "rb") as fin:
        result = pickle.load(fin)

    assert len(result) == 1
    result = result[0]

    if result.skipped():
        print("*** %s: %s: Test skipped, not updating." %
              (source[0].dir, color_message(termcap.Yellow, "WARNING")))
        return 0
    elif result:
        print("*** %s: %s: Test successful, not updating." %
              (source[0].dir, color_message(termcap.Green, "skipped")))
        return 0
    elif result.failed_run():
        print("*** %s: %s: Test failed, not updating." %
              (source[0].dir, color_message(termcap.Red, "ERROR")))
        return 1

    print("** Updating %s" % test)
    test.update_ref()

    return 0

def update_test_string(target, source, env):
    return env.subst("Updating ${SOURCES[0].dir}",
                     target=target, source=source)

updateAction = env.Action(update_test, update_test_string)

def test_builder(test_tuple):
    """Define a test."""

    out_dir = "/".join(test_tuple)
    binary = env.M5Binary.abspath
    test = tests.ClassicTest(binary, out_dir, test_tuple)

    def tgt(name):
        return os.path.join(out_dir, name)

    def ref(name):
        return os.path.join(test.ref_dir, name)

    pickle_file = tgt("status.pickle")
    targets = [
        pickle_file,
    ]

    sources = [
        env.M5Binary,
        "run.py",
    ] + [ ref(f) for f in test.ref_files() ]

    env.Command(targets, sources, testAction)

    # phony target to echo status
    if GetOption('update_ref'):
        p = env.Command(tgt("_update"), [pickle_file], updateAction)
    else:
        p = env.Command(tgt("_print"), [pickle_file], printAction)

    env.AlwaysBuild(p)

def list_tests(target, source, env):
    """Create a list of tests

    Targets are as follows:
    target[0] : List file (e.g., tests/opt/all.list,  tests/opt/quick.list)

    Sources are: -

    """

    tgt_name = os.path.basename(str(target[0]))
    base, ext = os.path.splitext(tgt_name)
    categories = tests.all_categories if base == "all" else (base, )

    with open(target[0].abspath, "w") as fout:
        for cat in categories:
            for test in env.Tests[cat]:
                print("/".join(test), file=fout)

    return 0

testListAction = env.Action(list_tests, strfunction=None)

env.Command("all.list", tuple(), testListAction)
for cat, test_list in env.Tests.items():
    env.Command("%s.list" % cat, tuple(), testListAction)
    for test in test_list:
        test_builder(test)
