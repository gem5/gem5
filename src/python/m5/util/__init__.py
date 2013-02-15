# Copyright (c) 2008-2009 The Hewlett-Packard Development Company
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
# Authors: Nathan Binkert

import os
import re
import sys

import convert
import jobfile

from attrdict import attrdict, multiattrdict, optiondict
from code_formatter import code_formatter
from multidict import multidict
from orderdict import orderdict
from smartdict import SmartDict
from sorteddict import SortedDict
from region import neg_inf, pos_inf, Region, Regions

# panic() should be called when something happens that should never
# ever happen regardless of what the user does (i.e., an acutal m5
# bug).
def panic(fmt, *args):
    print >>sys.stderr, 'panic:', fmt % args
    sys.exit(1)

# fatal() should be called when the simulation cannot continue due to
# some condition that is the user's fault (bad configuration, invalid
# arguments, etc.) and not a simulator bug.
def fatal(fmt, *args):
    print >>sys.stderr, 'fatal:', fmt % args
    sys.exit(1)

# warn() should be called when the user should be warned about some condition
# that may or may not be the user's fault, but that they should be made aware
# of as it may affect the simulation or results.
def warn(fmt, *args):
    print >>sys.stderr, 'warn:', fmt % args

# inform() should be called when the user should be informed about some
# condition that they may be interested in.
def inform(fmt, *args):
    print >>sys.stdout, 'info:', fmt % args

class Singleton(type):
    def __call__(cls, *args, **kwargs):
        if hasattr(cls, '_instance'):
            return cls._instance

        cls._instance = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instance

def addToPath(path):
    """Prepend given directory to system module search path.  We may not
    need this anymore if we can structure our config library more like a
    Python package."""

    # if it's a relative path and we know what directory the current
    # python script is in, make the path relative to that directory.
    if not os.path.isabs(path) and sys.path[0]:
        path = os.path.join(sys.path[0], path)
    path = os.path.realpath(path)
    # sys.path[0] should always refer to the current script's directory,
    # so place the new dir right after that.
    sys.path.insert(1, path)

# Apply method to object.
# applyMethod(obj, 'meth', <args>) is equivalent to obj.meth(<args>)
def applyMethod(obj, meth, *args, **kwargs):
    return getattr(obj, meth)(*args, **kwargs)

# If the first argument is an (non-sequence) object, apply the named
# method with the given arguments.  If the first argument is a
# sequence, apply the method to each element of the sequence (a la
# 'map').
def applyOrMap(objOrSeq, meth, *args, **kwargs):
    if not isinstance(objOrSeq, (list, tuple)):
        return applyMethod(objOrSeq, meth, *args, **kwargs)
    else:
        return [applyMethod(o, meth, *args, **kwargs) for o in objOrSeq]

def compareVersions(v1, v2):
    """helper function: compare arrays or strings of version numbers.
    E.g., compare_version((1,3,25), (1,4,1)')
    returns -1, 0, 1 if v1 is <, ==, > v2
    """
    def make_version_list(v):
        if isinstance(v, (list,tuple)):
            return v
        elif isinstance(v, str):
            return map(lambda x: int(re.match('\d+', x).group()), v.split('.'))
        else:
            raise TypeError

    v1 = make_version_list(v1)
    v2 = make_version_list(v2)
    # Compare corresponding elements of lists
    for n1,n2 in zip(v1, v2):
        if n1 < n2: return -1
        if n1 > n2: return  1
    # all corresponding values are equal... see if one has extra values
    if len(v1) < len(v2): return -1
    if len(v1) > len(v2): return  1
    return 0

def crossproduct(items):
    if len(items) == 1:
        for i in items[0]:
            yield (i,)
    else:
        for i in items[0]:
            for j in crossproduct(items[1:]):
                yield (i,) + j

def flatten(items):
    while items:
        item = items.pop(0)
        if isinstance(item, (list, tuple)):
            items[0:0] = item
        else:
            yield item

# force scalars to one-element lists for uniformity
def makeList(objOrList):
    if isinstance(objOrList, list):
        return objOrList
    return [objOrList]

def printList(items, indent=4):
    line = ' ' * indent
    for i,item in enumerate(items):
        if len(line) + len(item) > 76:
            print line
            line = ' ' * indent

        if i < len(items) - 1:
            line += '%s, ' % item
        else:
            line += item
            print line

def readCommand(cmd, **kwargs):
    """run the command cmd, read the results and return them
    this is sorta like `cmd` in shell"""
    from subprocess import Popen, PIPE, STDOUT

    if isinstance(cmd, str):
        cmd = cmd.split()

    no_exception = 'exception' in kwargs
    exception = kwargs.pop('exception', None)
    
    kwargs.setdefault('shell', False)
    kwargs.setdefault('stdout', PIPE)
    kwargs.setdefault('stderr', STDOUT)
    kwargs.setdefault('close_fds', True)
    try:
        subp = Popen(cmd, **kwargs)
    except Exception, e:
        if no_exception:
            return exception
        raise

    return subp.communicate()[0]

def makeDir(path):
    """Make a directory if it doesn't exist.  If the path does exist,
    ensure that it is a directory"""
    if os.path.exists(path):
        if not os.path.isdir(path):
            raise AttributeError, "%s exists but is not directory" % path
    else:
        os.mkdir(path)
