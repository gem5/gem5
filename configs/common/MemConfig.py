# Copyright (c) 2013 ARM Limited
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
#          Andreas Hansson

import m5.objects
import inspect
import sys
from textwrap import  TextWrapper

# Dictionary of mapping names of real memory controller models to
# classes.
_mem_classes = {}

# Memory aliases. We make sure they exist before we add them to the
# fina; list. A target may be specified as a tuple, in which case the
# first available memory controller model in the tuple will be used.
_mem_aliases_all = [
    ("simple_mem", "SimpleMemory"),
    ("ddr3_1600_x64", "DDR3_1600_x64"),
    ("lpddr2_s4_1066_x32", "LPDDR2_S4_1066_x32"),
    ("lpddr3_1600_x32", "LPDDR3_1600_x32"),
    ("wio_200_x128", "WideIO_200_x128"),
    ("dramsim2", "DRAMSim2")
    ]

# Filtered list of aliases. Only aliases for existing memory
# controllers exist in this list.
_mem_aliases = {}


def is_mem_class(cls):
    """Determine if a class is a memory controller that can be instantiated"""

    # We can't use the normal inspect.isclass because the ParamFactory
    # and ProxyFactory classes have a tendency to confuse it.
    try:
        return issubclass(cls, m5.objects.AbstractMemory) and \
            not cls.abstract
    except TypeError:
        return False

def get(name):
    """Get a memory class from a user provided class name or alias."""

    real_name = _mem_aliases.get(name, name)

    try:
        mem_class = _mem_classes[real_name]
        return mem_class
    except KeyError:
        print "%s is not a valid memory controller." % (name,)
        sys.exit(1)

def print_mem_list():
    """Print a list of available memory classes including their aliases."""

    print "Available memory classes:"
    doc_wrapper = TextWrapper(initial_indent="\t\t", subsequent_indent="\t\t")
    for name, cls in _mem_classes.items():
        print "\t%s" % name

        # Try to extract the class documentation from the class help
        # string.
        doc = inspect.getdoc(cls)
        if doc:
            for line in doc_wrapper.wrap(doc):
                print line

    if _mem_aliases:
        print "\nMemory aliases:"
        for alias, target in _mem_aliases.items():
            print "\t%s => %s" % (alias, target)

def mem_names():
    """Return a list of valid memory names."""
    return _mem_classes.keys() + _mem_aliases.keys()

# Add all memory controllers in the object hierarchy.
for name, cls in inspect.getmembers(m5.objects, is_mem_class):
    _mem_classes[name] = cls

for alias, target in _mem_aliases_all:
    if isinstance(target, tuple):
        # Some aliases contain a list of memory controller models
        # sorted in priority order. Use the first target that's
        # available.
        for t in target:
            if t in _mem_classes:
                _mem_aliases[alias] = t
                break
    elif target in _mem_classes:
        # Normal alias
        _mem_aliases[alias] = target

def config_mem(options, system):
    """
    Create the memory controllers based on the options and attach them.

    If requested, we make a multi-channel configuration of the
    selected memory controller class by creating multiple instances of
    the specific class. The individual controllers have their
    parameters set such that the address range is interleaved between
    them.
    """

    nbr_mem_ctrls = options.mem_channels
    import math
    from m5.util import fatal
    intlv_bits = int(math.log(nbr_mem_ctrls, 2))
    if 2 ** intlv_bits != nbr_mem_ctrls:
        fatal("Number of memory channels must be a power of 2")
    cls = get(options.mem_type)
    mem_ctrls = []

    # The default behaviour is to interleave on cache line granularity
    cache_line_bit = int(math.log(system.cache_line_size.value, 2)) - 1
    intlv_low_bit = cache_line_bit

    # For every range (most systems will only have one), create an
    # array of controllers and set their parameters to match their
    # address mapping in the case of a DRAM
    for r in system.mem_ranges:
        for i in xrange(nbr_mem_ctrls):
            # Create an instance so we can figure out the address
            # mapping and row-buffer size
            ctrl = cls()

            # Only do this for DRAMs
            if issubclass(cls, m5.objects.SimpleDRAM):
                # Inform each controller how many channels to account
                # for
                ctrl.channels = nbr_mem_ctrls

                # If the channel bits are appearing after the column
                # bits, we need to add the appropriate number of bits
                # for the row buffer size
                if ctrl.addr_mapping.value == 'RaBaChCo':
                    # This computation only really needs to happen
                    # once, but as we rely on having an instance we
                    # end up having to repeat it for each and every
                    # one
                    rowbuffer_size = ctrl.device_rowbuffer_size.value * \
                        ctrl.devices_per_rank.value

                    intlv_low_bit = int(math.log(rowbuffer_size, 2)) - 1

            # We got all we need to configure the appropriate address
            # range
            ctrl.range = m5.objects.AddrRange(r.start, size = r.size(),
                                              intlvHighBit = \
                                                  intlv_low_bit + intlv_bits,
                                              intlvBits = intlv_bits,
                                              intlvMatch = i)
            mem_ctrls.append(ctrl)

    system.mem_ctrls = mem_ctrls

    # Connect the controllers to the membus
    for i in xrange(len(system.mem_ctrls)):
        system.mem_ctrls[i].port = system.membus.master
