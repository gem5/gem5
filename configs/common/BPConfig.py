# Copyright (c) 2018 Metempsy Technology Consulting
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
# Authors: Pau Cabre

# This file is a copy of MemConfig.py / CpuConfig.py, but modified to
# hanle branch predictors instead of memory controllers / CPUs

from __future__ import print_function
from __future__ import absolute_import

from m5 import fatal
import m5.objects
import inspect
import sys
from textwrap import TextWrapper

# Dictionary of mapping names of real branch predictor models to classes.
_bp_classes = {}


def is_bp_class(cls):
    """Determine if a class is a branch predictor that can be instantiated"""

    # We can't use the normal inspect.isclass because the ParamFactory
    # and ProxyFactory classes have a tendency to confuse it.
    try:
        return issubclass(cls, m5.objects.BranchPredictor) and \
            not cls.abstract
    except (TypeError, AttributeError):
        return False

def get(name):
    """Get a BP class from a user provided class name or alias."""

    try:
        bp_class = _bp_classes[name]
        return bp_class
    except KeyError:
        print("%s is not a valid BP model." % (name,))
        sys.exit(1)

def print_bp_list():
    """Print a list of available BP classes."""

    print("Available BranchPredictor classes:")
    doc_wrapper = TextWrapper(initial_indent="\t\t", subsequent_indent="\t\t")
    for name, cls in _bp_classes.items():
        print("\t%s" % name)

        # Try to extract the class documentation from the class help
        # string.
        doc = inspect.getdoc(cls)
        if doc:
            for line in doc_wrapper.wrap(doc):
                print(line)

def bp_names():
    """Return a list of valid Branch Predictor names."""
    return list(_bp_classes.keys())

# Add all BPs in the object hierarchy.
for name, cls in inspect.getmembers(m5.objects, is_bp_class):
    _bp_classes[name] = cls


# The same for indirect branch predictors...
# Dictionary of mapping names of real branch predictor models to classes.
_indirect_bp_classes = {}


def is_indirect_bp_class(cls):
    """Determine if a class is an indirect branch predictor that can be
    instantiated"""

    # We can't use the normal inspect.isclass because the ParamFactory
    # and ProxyFactory classes have a tendency to confuse it.
    try:
        return issubclass(cls, m5.objects.IndirectPredictor) and \
            not cls.abstract
    except (TypeError, AttributeError):
        return False

def get_indirect(name):
    """Get an Indirect BP class from a user provided class name or alias."""

    try:
        indirect_bp_class = _indirect_bp_classes[name]
        return indirect_bp_class
    except KeyError:
        print("%s is not a valid indirect BP model." % (name,))
        sys.exit(1)

def print_indirect_bp_list():
    """Print a list of available indirect BP classes."""

    print("Available Indirect BranchPredictor classes:")
    doc_wrapper = TextWrapper(initial_indent="\t\t", subsequent_indent="\t\t")
    for name, cls in _indirect_bp_classes.items():
        print("\t%s" % name)

        # Try to extract the class documentation from the class help
        # string.
        doc = inspect.getdoc(cls)
        if doc:
            for line in doc_wrapper.wrap(doc):
                print(line)

def indirect_bp_names():
    """Return a list of valid Indirect Branch Predictor names."""
    return _indirect_bp_classes.keys()

# Add all indirect BPs in the object hierarchy.
for name, cls in inspect.getmembers(m5.objects, is_indirect_bp_class):
    _indirect_bp_classes[name] = cls

