#! /usr/bin/env python2.7
# Copyright (c) 2014, 2016 ARM Limited
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
# Copyright (c) 2006 The Regents of The University of Michigan
# Copyright (c) 2007,2011 The Hewlett-Packard Development Company
# Copyright (c) 2016 Advanced Micro Devices, Inc.
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
# Authors: Nathan Binkert
#          Steve Reinhardt
#          Andreas Sandberg

from abc import ABCMeta, abstractmethod
import difflib
import re
import sys

from region import *

tabsize = 8
lead = re.compile(r'^([ \t]+)')
trail = re.compile(r'([ \t]+)$')
any_control = re.compile(r'\b(if|while|for)([ \t]*)\(')


class UserInterface(object):
    __metaclass__ = ABCMeta

    def __init__(self, verbose=False):
        self.verbose = verbose

    def prompt(self, prompt, results, default):
        while True:
            result = self._prompt(prompt, results, default)
            if result in results:
                return result

    @abstractmethod
    def _prompt(self, prompt, results, default):
        pass

    @abstractmethod
    def write(self, string):
        pass

class StdioUI(UserInterface):
    def _prompt(self, prompt, results, default):
        return raw_input(prompt) or default

    def write(self, string):
        sys.stdout.write(string)

class MercurialUI(UserInterface):
    def __init__(self, ui, *args, **kwargs):
        super(MercurialUI, self).__init__(*args, **kwargs)
        self.hg_ui = ui

    def _prompt(self, prompt, results, default):
        return self.hg_ui.prompt(prompt, default=default)

    def write(self, string):
        self.hg_ui.write(string)


def _re_ignore(expr):
    """Helper function to create regular expression ignore file
    matcher functions"""

    rex = re.compile(expr)
    def match_re(fname):
        return rex.match(fname)
    return match_re

# This list contains a list of functions that are called to determine
# if a file should be excluded from the style matching rules or
# not. The functions are called with the file name relative to the
# repository root (without a leading slash) as their argument. A file
# is excluded if any function in the list returns true.
style_ignores = [
    # Ignore external projects as they are unlikely to follow the gem5
    # coding convention.
    _re_ignore("^ext/"),
    # Ignore test data, as they are not code
    _re_ignore("^tests/(?:quick|long)/"),
    # Ignore RISC-V assembly tests as they are maintained in an external
    # project that does not follow the gem5 coding convention
    _re_ignore("tests/test-progs/asmtest/src/riscv/"),
    # Ignore RISC-V assembly dump files
    _re_ignore("tests/test-progs/asmtest/dump/riscv/")
]

def check_ignores(fname):
    """Check if a file name matches any of the ignore rules"""

    for rule in style_ignores:
        if rule(fname):
            return True

    return False


def normalized_len(line):
    """Return a normalized line length with expanded tabs"""

    count = 0
    for c in line:
        if c == '\t':
            count += tabsize - count % tabsize
        else:
            count += 1

    return count

def modified_regions(old, new, context=0):
    regions = Regions()
    m = difflib.SequenceMatcher(a=old, b=new, autojunk=False)
    for group in m.get_grouped_opcodes(context):
        first = group[0]
        last = group[-1]

        regions.extend(Region(first[3], last[4] + 1))

    return regions
