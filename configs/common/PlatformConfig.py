# Copyright (c) 2012, 2015 ARM Limited
# All rights reserved.
#
# Copyright (c) 2017, Centre National de la Recherche Scientifique (CNRS)
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
#          Pierre-Yves Peneau

import m5.objects
import inspect
import sys
from m5.util import fatal
from textwrap import TextWrapper

# Dictionary of mapping names of real CPU models to classes.
_platform_classes = {}

# Platform aliases. The platforms listed here might not be compiled,
# we make sure they exist before we add them to the platform list.
_platform_aliases_all = [
    ("RealView_EB", "RealViewEB"),
    ("RealView_PBX", "RealViewPBX"),
    ("VExpress_GEM5", "VExpress_GEM5_V1"),
    ]

# Filtered list of aliases. Only aliases for existing platforms exist
# in this list.
_platform_aliases = {}

def is_platform_class(cls):
    """Determine if a class is a Platform that can be instantiated"""

    # We can't use the normal inspect.isclass because the ParamFactory
    # and ProxyFactory classes have a tendency to confuse it.
    try:
        return issubclass(cls, m5.objects.Platform) and \
            not cls.abstract
    except (TypeError, AttributeError):
        return False

def get(name):
    """Get a platform class from a user provided class name."""

    real_name = _platform_aliases.get(name, name)

    try:
        return _platform_classes[real_name]
    except KeyError:
        fatal("%s is not a valid Platform model." % (name,))

def print_platform_list():
    """Print a list of available Platform classes including their aliases."""

    print "Available Platform classes:"
    doc_wrapper = TextWrapper(initial_indent="\t\t", subsequent_indent="\t\t")
    for name, cls in _platform_classes.items():
        print "\t%s" % name

        # Try to extract the class documentation from the class help
        # string.
        doc = inspect.getdoc(cls)
        if doc:
            for line in doc_wrapper.wrap(doc):
                print line

    if _platform_aliases:
        print "\Platform aliases:"
        for alias, target in _platform_aliases.items():
            print "\t%s => %s" % (alias, target)

def platform_names():
    """Return a list of valid Platform names."""
    return _platform_classes.keys() + _platform_aliases.keys()

# Add all Platforms in the object hierarchy.
for name, cls in inspect.getmembers(m5.objects, is_platform_class):
    _platform_classes[name] = cls

for alias, target in _platform_aliases_all:
    if target in _platform_classes:
        _platform_aliases[alias] = target
