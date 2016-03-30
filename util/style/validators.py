#!/usr/bin/env python
#
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
import inspect
import re
import sys

import style

tabsize = 8
lead = re.compile(r'^([ \t]+)')
trail = re.compile(r'([ \t]+)$')
any_control = re.compile(r'\b(if|while|for)([ \t]*)\(')

class Validator(object):
    """Base class for style validators

    Validators analyze source files for common style violations and
    produce source code style violation statistics. Unlike style
    verifiers (see verifiers.py), they do not try to fix any style
    violations violations.

    Deprecation warning: These classes are currently only used by the
    "hg m5format" command and not by any style hooks. New style
    checkers should inherit from Verifier instead of Validator.

    """

    __metaclass__ = ABCMeta

    def __init__(self, file_name, verbose=False, language=None):
        self.file_name = file_name
        self.verbose = verbose
        self.bad = 0
        self.language = language

    def fail_line(self, line_no, line, message):
        print '%s:%d>' % (self.file_name, line_no + 1), message
        if self.verbose:
            print line
        self.bad += 1

    def __nonzero__(self):
        return self.bad == 0

    @classmethod
    def supported_lang(cls, language):
        return True

    @abstractmethod
    def validate_line(self, line_no, line):
        pass

    @abstractmethod
    def dump(self):
        pass

class SimpleValidator(Validator):
    supported_langs = set()

    def __init__(self, fail_message, dump_message, file_name, **kwargs):
        super(SimpleValidator, self).__init__(file_name, **kwargs)

        self.fail_message = fail_message
        self.dump_message = dump_message

    @classmethod
    def supported_lang(cls, language):
        return not cls.cupported_langs or language in cls.supported_langs

    def validate_line(self, line_no, line):
        if not self.simple_validate_line(line):
            self.fail_line(line_no, line, self.fail_message)
            return False
        else:
            return True

    @abstractmethod
    def simple_validate_line(self, line):
        pass

    def dump(self):
        print self.dump_message % {
            "bad" : self.bad
        }

class LineLength(Validator):
    def __init__(self, *args, **kwargs):
        super(LineLength, self).__init__(*args, **kwargs)

        self.toolong80 = 0

    def validate_line(self, line_no, line):
        llen = style.normalized_len(line)
        if llen == 80:
            self.toolong80 += 1

        if llen > 79:
            self.fail_line(line_no, line, 'line too long (%d chars)' % llen)
            return False
        else:
            return True

    def dump(self):
        print "%d violations of lines over 79 chars. " \
            "%d of which are 80 chars exactly." % (self.bad, self.toolong80)

class ControlSpacing(Validator):
    supported_langs = set(('C', 'C++'))

    def validate_line(self, line_no, line):
        match = any_control.search(line)
        if match and match.group(2) != " ":
            stats.badcontrol += 1
            self.fail_line(line_no, line,
                           'improper spacing after %s' % match.group(1))
            return False
        else:
            return True

    def dump(self):
        print "%d bad parens after if/while/for." % (self.bad, )

class CarriageReturn(SimpleValidator):
    def __init__(self, *args, **kwargs):
        super(CarriageReturn, self).__init__(
            "carriage return found",
            "%(bad)d carriage returns found.",
            *args, **kwargs)

    def simple_validate_line(self, line):
        return line.find('\r') == -1

class TabIndent(SimpleValidator):
    lead = re.compile(r'^([ \t]+)')

    def __init__(self, *args, **kwargs):
        super(TabIndent, self).__init__(
            "using tabs to indent",
            "%(bad)d cases of tabs to indent.",
            *args, **kwargs)

    def simple_validate_line(self, line):
        match = TabIndent.lead.search(line)
        return not (match and match.group(1).find('\t') != -1)

class TrailingWhitespace(SimpleValidator):
    trail = re.compile(r'([ \t]+)$')

    def __init__(self, *args, **kwargs):
        super(TrailingWhitespace, self).__init__(
            "trailing whitespace",
            "%(bad)d cases of whitespace at the end of a line.",
            *args, **kwargs)

    def simple_validate_line(self, line):
        return not TrailingWhitespace.trail.search(line)

def is_validator(cls):
    """Determine if a class is a Validator that can be instantiated"""

    return inspect.isclass(cls) and issubclass(cls, Validator) and \
        not inspect.isabstract(cls)

# list of all verifier classes
all_validators = [ v for n, v in \
                  inspect.getmembers(sys.modules[__name__], is_validator) ]

