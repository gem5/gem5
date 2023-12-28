# Copyright (c) 2014, 2016, 2018-2019 ARM Limited
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
# Copyright (c) 2003-2005 The Regents of The University of Michigan
# Copyright (c) 2013,2015 Advanced Micro Devices, Inc.
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

import re

###################
# Utility functions


#
# Indent every line in string 's' by two spaces
# (except preprocessor directives).
# Used to make nested code blocks look pretty.
#
def indent(s):
    return re.sub(r"(?m)^(?!#)", "  ", s)


# Regular expression object to match C++ strings
stringRE = re.compile(r'"([^"\\]|\\.)*"')

# Regular expression object to match C++ comments
# (used in findOperands())
commentRE = re.compile(
    r"(^)?[^\S\n]*/(?:\*(.*?)\*/[^\S\n]*|/[^\n]*)($)?",
    re.DOTALL | re.MULTILINE,
)

# Regular expression object to match assignment statements (used in
# findOperands()).  If the code immediately following the first
# appearance of the operand matches this regex, then the operand
# appears to be on the LHS of an assignment, and is thus a
# destination.  basically we're looking for an '=' that's not '=='.
# The heinous tangle before that handles the case where the operand
# has an array subscript.
assignRE = re.compile(
    r"((\.as<[^>]+>\(\s*\))?\[[^\]]+\])?\s*=(?!=)", re.MULTILINE
)

#
# Munge a somewhat arbitrarily formatted piece of Python code
# (e.g. from a format 'let' block) into something whose indentation
# will get by the Python parser.
#
# The two keys here are that Python will give a syntax error if
# there's any whitespace at the beginning of the first line, and that
# all lines at the same lexical nesting level must have identical
# indentation.  Unfortunately the way code literals work, an entire
# let block tends to have some initial indentation.  Rather than
# trying to figure out what that is and strip it off, we prepend 'if
# 1:' to make the let code the nested block inside the if (and have
# the parser automatically deal with the indentation for us).
#
# We don't want to do this if (1) the code block is empty or (2) the
# first line of the block doesn't have any whitespace at the front.


def fixPythonIndentation(s):
    # get rid of blank lines first
    s = re.sub(r"(?m)^\s*\n", "", s)
    if s != "" and re.match(r"[ \t]", s[0]):
        s = "if 1:\n" + s
    return s


class ISAParserError(Exception):
    """Exception class for parser errors"""

    def __init__(self, first, second=None):
        if second is None:
            self.lineno = 0
            self.string = first
        else:
            self.lineno = first
            self.string = second

    def __str__(self):
        return self.string


def error(*args):
    raise ISAParserError(*args)


def protectNonSubstPercents(s):
    """Protect any non-dict-substitution '%'s in a format string
    (i.e. those not followed by '(')"""

    return re.sub(r"%(?!\()", "%%", s)


##############
# Stack: a simple stack object.  Used for both formats (formatStack)
# and default cases (defaultStack).  Simply wraps a list to give more
# stack-like syntax and enable initialization with an argument list
# (as opposed to an argument that's a list).


class Stack(list):
    def __init__(self, *items):
        list.__init__(self, items)

    def push(self, item):
        self.append(item)

    def top(self):
        return self[-1]


# Format a file include stack backtrace as a string
def backtrace(filename_stack):
    fmt = "In file included from %s:"
    return "\n".join([fmt % f for f in filename_stack])


#######################
#
# LineTracker: track filenames along with line numbers in PLY lineno fields
#     PLY explicitly doesn't do anything with 'lineno' except propagate
#     it.  This class lets us tie filenames with the line numbers with a
#     minimum of disruption to existing increment code.
#


class LineTracker:
    def __init__(self, filename, lineno=1):
        self.filename = filename
        self.lineno = lineno

    # Overload '+=' for increments.  We need to create a new object on
    # each update else every token ends up referencing the same
    # constantly incrementing instance.
    def __iadd__(self, incr):
        return LineTracker(self.filename, self.lineno + incr)

    def __str__(self):
        return "%s:%d" % (self.filename, self.lineno)

    # In case there are places where someone really expects a number
    def __int__(self):
        return self.lineno
