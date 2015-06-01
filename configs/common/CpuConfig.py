# Copyright (c) 2012 ARM Limited
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

import m5.objects
import inspect
import sys
from textwrap import  TextWrapper

# Dictionary of mapping names of real CPU models to classes.
_cpu_classes = {}

# CPU aliases. The CPUs listed here might not be compiled, we make
# sure they exist before we add them to the CPU list. A target may be
# specified as a tuple, in which case the first available CPU model in
# the tuple will be used as the target.
_cpu_aliases_all = [
    ("timing", "TimingSimpleCPU"),
    ("atomic", "AtomicSimpleCPU"),
    ("minor", "MinorCPU"),
    ("detailed", "DerivO3CPU"),
    ("kvm", ("ArmKvmCPU", "ArmV8KvmCPU", "X86KvmCPU")),
    ]

# Filtered list of aliases. Only aliases for existing CPUs exist in
# this list.
_cpu_aliases = {}


def is_cpu_class(cls):
    """Determine if a class is a CPU that can be instantiated"""

    # We can't use the normal inspect.isclass because the ParamFactory
    # and ProxyFactory classes have a tendency to confuse it.
    try:
        return issubclass(cls, m5.objects.BaseCPU) and \
            not cls.abstract and \
            not issubclass(cls, m5.objects.CheckerCPU)
    except TypeError:
        return False

def get(name):
    """Get a CPU class from a user provided class name or alias."""

    real_name = _cpu_aliases.get(name, name)

    try:
        cpu_class = _cpu_classes[real_name]
        return cpu_class
    except KeyError:
        print "%s is not a valid CPU model." % (name,)
        sys.exit(1)

def print_cpu_list():
    """Print a list of available CPU classes including their aliases."""

    print "Available CPU classes:"
    doc_wrapper = TextWrapper(initial_indent="\t\t", subsequent_indent="\t\t")
    for name, cls in _cpu_classes.items():
        print "\t%s" % name

        # Try to extract the class documentation from the class help
        # string.
        doc = inspect.getdoc(cls)
        if doc:
            for line in doc_wrapper.wrap(doc):
                print line

    if _cpu_aliases:
        print "\nCPU aliases:"
        for alias, target in _cpu_aliases.items():
            print "\t%s => %s" % (alias, target)

def cpu_names():
    """Return a list of valid CPU names."""
    return _cpu_classes.keys() + _cpu_aliases.keys()

# The ARM detailed CPU is special in the sense that it doesn't exist
# in the normal object hierarchy, so we have to add it manually.
try:
    from O3_ARM_v7a import O3_ARM_v7a_3
    _cpu_classes["arm_detailed"] = O3_ARM_v7a_3
except:
    pass

# Add all CPUs in the object hierarchy.
for name, cls in inspect.getmembers(m5.objects, is_cpu_class):
    _cpu_classes[name] = cls

for alias, target in _cpu_aliases_all:
    if isinstance(target, tuple):
        # Some aliases contain a list of CPU model sorted in priority
        # order. Use the first target that's available.
        for t in target:
            if t in _cpu_classes:
                _cpu_aliases[alias] = t
                break
    elif target in _cpu_classes:
        # Normal alias
        _cpu_aliases[alias] = target
