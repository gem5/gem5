# Copyright (c) 2016, 2020-2021 Arm Limited
# All rights reserved.
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
import os
import re
import sys
from functools import wraps

from . import convert
from .attrdict import attrdict
from .attrdict import multiattrdict
from .attrdict import optiondict
from .multidict import multidict

# panic() should be called when something happens that should never
# ever happen regardless of what the user does (i.e., an acutal m5
# bug).
def panic(fmt, *args):
    print("panic:", fmt % args, file=sys.stderr)
    sys.exit(1)


# fatal() should be called when the simulation cannot continue due to
# some condition that is the user's fault (bad configuration, invalid
# arguments, etc.) and not a simulator bug.
def fatal(fmt, *args):
    print("fatal:", fmt % args, file=sys.stderr)
    sys.exit(1)


# warn() should be called when the user should be warned about some condition
# that may or may not be the user's fault, but that they should be made aware
# of as it may affect the simulation or results.
def warn(fmt, *args):
    print("warn:", fmt % args, file=sys.stderr)


# inform() should be called when the user should be informed about some
# condition that they may be interested in.
def inform(fmt, *args):
    print("info:", fmt % args, file=sys.stdout)


def callOnce(func):
    """Decorator that enables to run a given function only once. Subsequent
    calls are discarded."""

    @wraps(func)
    def wrapper(*args, **kwargs):
        if not wrapper.has_run:
            wrapper.has_run = True
            return func(*args, **kwargs)

    wrapper.has_run = False
    return wrapper


def deprecated(replacement=None, logger=warn):
    """This decorator warns the user about a deprecated function."""

    def decorator(func):
        @callOnce
        def notifyDeprecation():
            try:
                func_name = lambda f: f.__module__ + "." + f.__qualname__
                message = f"Function {func_name(func)} is deprecated."
                if replacement:
                    message += f" Prefer {func_name(replacement)} instead."
            except AttributeError:
                message = f"Function {func} is deprecated."
                if replacement:
                    message += f" Prefer {replacement} instead."
            logger(message)

        @wraps(func)
        def wrapper(*args, **kwargs):
            notifyDeprecation()
            return func(*args, **kwargs)

        return wrapper

    return decorator


class Singleton(type):
    def __call__(cls, *args, **kwargs):
        if hasattr(cls, "_instance"):
            return cls._instance

        cls._instance = super().__call__(*args, **kwargs)
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


def repoPath():
    """
    Return the abspath of the gem5 repository.
    This is relying on the following structure:

    <gem5-repo>/build/<ISA>/gem5.[opt,debug...]
    """
    return os.path.dirname(os.path.dirname(os.path.dirname(sys.executable)))


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
    line = " " * indent
    for i, item in enumerate(items):
        if len(line) + len(item) > 76:
            print(line)
            line = " " * indent

        if i < len(items) - 1:
            line += f"{item}, "
        else:
            line += item
            print(line)


def isInteractive():
    """Check if the simulator is run interactively or in a batch environment"""

    return sys.__stdin__.isatty()
