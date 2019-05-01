#!/usr/bin/env python2.7
#
# Copyright (c) 2016-2017 ARM Limited
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
import os
from collections import namedtuple
from units import *
from results import TestResult
import shutil

_test_base = os.path.join(os.path.dirname(__file__), "..")

ClassicConfig = namedtuple("ClassicConfig", (
    "category",
    "mode",
    "workload",
    "isa",
    "os",
    "config",
))

# There are currently two "classes" of test
# configurations. Architecture-specific ones and generic ones
# (typically SE mode tests). In both cases, the configuration name
# matches a file in tests/configs/ that will be picked up by the test
# runner (run.py).
#
# Architecture specific configurations are listed in the arch_configs
# dictionary. This is indexed by a (cpu architecture, gpu
# architecture) tuple. GPU architecture is optional and may be None.
#
# Generic configurations are listed in the generic_configs tuple.
#
# When discovering available test cases, this script look uses the
# test list as a list of /candidate/ configurations. A configuration
# is only used if a test has a reference output for that
# configuration. In addition to the base configurations from
# arch_configs and generic_configs, a Ruby configuration may be
# appended to the base name (this is probed /in addition/ to the
# original name. See get_tests() for details.
#
arch_configs = {
    ("alpha", None) : (
        'tsunami-simple-atomic',
        'tsunami-simple-timing',
        'tsunami-simple-atomic-dual',
        'tsunami-simple-timing-dual',
        'twosys-tsunami-simple-atomic',
        'tsunami-o3', 'tsunami-o3-dual',
        'tsunami-minor', 'tsunami-minor-dual',
        'tsunami-switcheroo-full',
    ),

    ("arm", None) : (
        'simple-atomic-dummychecker',
        'o3-timing-checker',
        'realview-simple-atomic',
        'realview-simple-atomic-dual',
        'realview-simple-atomic-checkpoint',
        'realview-simple-timing',
        'realview-simple-timing-dual',
        'realview-o3',
        'realview-o3-checker',
        'realview-o3-dual',
        'realview-minor',
        'realview-minor-dual',
        'realview-switcheroo-atomic',
        'realview-switcheroo-timing',
        'realview-switcheroo-noncaching-timing',
        'realview-switcheroo-o3',
        'realview-switcheroo-full',
        'realview64-simple-atomic',
        'realview64-simple-atomic-checkpoint',
        'realview64-simple-atomic-dual',
        'realview64-simple-timing',
        'realview64-simple-timing-dual',
        'realview64-o3',
        'realview64-o3-checker',
        'realview64-o3-dual',
        'realview64-minor',
        'realview64-minor-dual',
        'realview64-switcheroo-atomic',
        'realview64-switcheroo-timing',
        'realview64-switcheroo-o3',
        'realview64-switcheroo-full',
    ),

    ("sparc", None) : (
        't1000-simple-atomic',
        't1000-simple-x86',
    ),

    ("x86", None) : (
        'pc-simple-atomic',
        'pc-simple-timing',
        'pc-o3-timing',
        'pc-switcheroo-full',
    ),

    ("x86", "hsail") : (
        'gpu',
    ),
}

generic_configs = (
    'simple-atomic',
    'simple-atomic-mp',
    'simple-timing',
    'simple-timing-mp',

    'minor-timing',
    'minor-timing-mp',

    'o3-timing',
    'o3-timing-mt',
    'o3-timing-mp',

    'rubytest',
    'memcheck',
    'memtest',
    'memtest-filter',
    'tgen-simple-mem',
    'tgen-dram-ctrl',
    'dram-lowp',

    'learning-gem5-p1-simple',
    'learning-gem5-p1-two-level',
)

default_ruby_protocol = {
    "arm" : "MOESI_CMP_directory",
}

def get_default_protocol(arch):
    return default_ruby_protocol.get(arch, 'MI_example')

all_categories = ("quick", "long")
all_modes = ("fs", "se")

class Test(object):
    """Test case base class.

    Test cases consists of one or more test units that are run in two
    phases. A run phase (units produced by run_units() and a verify
    phase (units from verify_units()). The verify phase is skipped if
    the run phase fails.

    """

    __metaclass__ = ABCMeta

    def __init__(self, name):
        self.test_name = name

    @abstractmethod
    def ref_files(self):
        """Get a list of reference files used by this test case"""
        pass

    @abstractmethod
    def run_units(self):
        """Units (typically RunGem5 instances) that describe the run phase of
        this test.

        """
        pass

    @abstractmethod
    def verify_units(self):
        """Verify the output from the run phase (see run_units())."""
        pass

    @abstractmethod
    def update_ref(self):
        """Update reference files with files from a test run"""
        pass

    def run(self):
        """Run this test case and return a list of results"""

        run_results = [ u.run() for u in self.run_units() ]
        run_ok = all([not r.skipped() and r for r in run_results ])

        verify_results = [
            u.run() if run_ok else u.skip()
            for u in self.verify_units()
        ]

        return TestResult(self.test_name,
                          run_results=run_results,
                          verify_results=verify_results)

    def __str__(self):
        return self.test_name

class ClassicTest(Test):
    # The diff ignore list contains all files that shouldn't be diffed
    # using DiffOutFile. These files typically use special-purpose
    # diff tools (e.g., DiffStatFile).
    diff_ignore_files = FileIgnoreList(
        names=(
            # Stat files use a special stat differ
            "stats.txt",
        ), rex=(
        ))

    # These files should never be included in the list of
    # reference files. This list should include temporary files
    # and other files that we don't care about.
    ref_ignore_files = FileIgnoreList(
        names=(
            "EMPTY",
        ), rex=(
            # Mercurial sometimes leaves backups when applying MQ patches
            r"\.orig$",
            r"\.rej$",
        ))

    def __init__(self, gem5, output_dir, config_tuple,
                 timeout=None,
                 skip=False, skip_diff_out=False, skip_diff_stat=False):

        super(ClassicTest, self).__init__("/".join(config_tuple))

        ct = config_tuple

        self.gem5 = os.path.abspath(gem5)
        self.script = os.path.join(_test_base, "run.py")
        self.config_tuple = ct
        self.timeout = timeout

        self.output_dir = output_dir
        self.ref_dir = os.path.join(_test_base,
                                    ct.category, ct.mode, ct.workload,
                                    "ref", ct.isa, ct.os, ct.config)
        self.skip_run = skip
        self.skip_diff_out = skip or skip_diff_out
        self.skip_diff_stat = skip or skip_diff_stat

    def ref_files(self):
        ref_dir = os.path.abspath(self.ref_dir)
        for root, dirs, files in os.walk(ref_dir, topdown=False):
            for f in files:
                fpath = os.path.join(root[len(ref_dir) + 1:], f)
                if fpath not in ClassicTest.ref_ignore_files:
                    yield fpath

    def run_units(self):
        args = [
            self.script,
            "/".join(self.config_tuple),
        ]

        return [
            RunGem5(self.gem5, args,
                    ref_dir=self.ref_dir, test_dir=self.output_dir,
                    skip=self.skip_run),
        ]

    def verify_units(self):
        ref_files = set(self.ref_files())
        units = []
        if "stats.txt" in ref_files:
            units.append(
                DiffStatFile(ref_dir=self.ref_dir, test_dir=self.output_dir,
                             skip=self.skip_diff_stat))
        units += [
            DiffOutFile(f,
                        ref_dir=self.ref_dir, test_dir=self.output_dir,
                        skip=self.skip_diff_out)
            for f in ref_files if f not in ClassicTest.diff_ignore_files
        ]

        return units

    def update_ref(self):
        for fname in self.ref_files():
            shutil.copy(
                os.path.join(self.output_dir, fname),
                os.path.join(self.ref_dir, fname))

def parse_test_filter(test_filter):
    wildcards = ("", "*")

    _filter = list(test_filter.split("/"))
    if len(_filter) > 3:
        raise RuntimeError("Illegal test filter string")
    _filter += [ "", ] * (3 - len(_filter))

    isa, cat, mode = _filter

    if isa in wildcards:
        raise RuntimeError("No ISA specified")

    cat = all_categories if cat in wildcards else (cat, )
    mode = all_modes if mode in wildcards else (mode, )

    return isa, cat, mode

def get_tests(isa,
              categories=all_categories, modes=all_modes,
              ruby_protocol=None, gpu_isa=None):

    # Generate a list of candidate configs
    configs = list(arch_configs.get((isa, gpu_isa), []))

    if (isa, gpu_isa) == ("x86", "hsail"):
        if ruby_protocol == "GPU_RfO":
            configs += ['gpu-randomtest']
    else:
        configs += generic_configs

    if ruby_protocol == get_default_protocol(isa):
        if ruby_protocol == 'MI_example':
            configs += [ "%s-ruby" % (c, ) for c in configs ]
        else:
            configs += [ "%s-ruby-%s" % (c, ruby_protocol) for c in configs ]
    elif ruby_protocol is not None:
        # Override generic ISA configs when using Ruby (excluding
        # MI_example which is included in all ISAs by default). This
        # reduces the number of generic tests we re-run for when
        # compiling Ruby targets.
        configs = [ "%s-ruby-%s" % (c, ruby_protocol) for c in configs ]

    # /(quick|long)/(fs|se)/workload/ref/arch/guest/config/
    for conf_script in configs:
        for cat in categories:
            for mode in modes:
                mode_dir = os.path.join(_test_base, cat, mode)
                if not os.path.exists(mode_dir):
                    continue

                for workload in os.listdir(mode_dir):
                    isa_dir = os.path.join(mode_dir, workload, "ref", isa)
                    if not os.path.isdir(isa_dir):
                        continue

                    for _os in os.listdir(isa_dir):
                        test_dir = os.path.join(isa_dir, _os, conf_script)
                        if not os.path.exists(test_dir) or \
                           os.path.exists(os.path.join(test_dir, "skip")):
                            continue

                        yield ClassicConfig(cat, mode, workload, isa, _os,
                                            conf_script)
