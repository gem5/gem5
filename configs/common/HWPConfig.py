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
_hwp_classes = {}


def is_hwp_class(cls):
    """Determine if a class is a prefetcher that can be instantiated"""

    # We can't use the normal inspect.isclass because the ParamFactory
    # and ProxyFactory classes have a tendency to confuse it.
    try:
        return issubclass(cls, m5.objects.BasePrefetcher) and \
            not cls.abstract
    except (TypeError, AttributeError):
        return False

def get(name):
    """Get a HWP class from a user provided class name or alias."""

    try:
        hwp_class = _hwp_classes[name]
        return hwp_class
    except KeyError:
        print("%s is not a valid HWP model." % (name,))
        sys.exit(1)

def print_hwp_list():
    """Print a list of available HWP classes."""

    print("Available Hardware Prefetcher classes:")
    doc_wrapper = TextWrapper(initial_indent="\t\t", subsequent_indent="\t\t")
    for name, cls in _hwp_classes.items():
        print("\t%s" % name)

        # Try to extract the class documentation from the class help
        # string.
        doc = inspect.getdoc(cls)
        if doc:
            for line in doc_wrapper.wrap(doc):
                print(line)

def hwp_names():
    """Return a list of valid Hardware Prefetcher names."""
    return list(_hwp_classes.keys())

# Add all HWPs in the object hierarchy.
for name, cls in inspect.getmembers(m5.objects, is_hwp_class):
    _hwp_classes[name] = cls

