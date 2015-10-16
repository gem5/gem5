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
    """Get a memory class from a user provided class name."""

    try:
        mem_class = _mem_classes[name]
        return mem_class
    except KeyError:
        print "%s is not a valid memory controller." % (name,)
        sys.exit(1)

def print_mem_list():
    """Print a list of available memory classes."""

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

def mem_names():
    """Return a list of valid memory names."""
    return _mem_classes.keys()

# Add all memory controllers in the object hierarchy.
for name, cls in inspect.getmembers(m5.objects, is_mem_class):
    _mem_classes[name] = cls

def create_mem_ctrl(cls, r, i, nbr_mem_ctrls, intlv_bits, intlv_size):
    """
    Helper function for creating a single memoy controller from the given
    options.  This function is invoked multiple times in config_mem function
    to create an array of controllers.
    """

    import math
    intlv_low_bit = int(math.log(intlv_size, 2))

    # Use basic hashing for the channel selection, and preferably use
    # the lower tag bits from the last level cache. As we do not know
    # the details of the caches here, make an educated guess. 4 MByte
    # 4-way associative with 64 byte cache lines is 6 offset bits and
    # 14 index bits.
    xor_low_bit = 20

    # Create an instance so we can figure out the address
    # mapping and row-buffer size
    ctrl = cls()

    # Only do this for DRAMs
    if issubclass(cls, m5.objects.DRAMCtrl):
        # Inform each controller how many channels to account
        # for
        ctrl.channels = nbr_mem_ctrls

        # If the channel bits are appearing after the column
        # bits, we need to add the appropriate number of bits
        # for the row buffer size
        if ctrl.addr_mapping.value == 'RoRaBaChCo':
            # This computation only really needs to happen
            # once, but as we rely on having an instance we
            # end up having to repeat it for each and every
            # one
            rowbuffer_size = ctrl.device_rowbuffer_size.value * \
                ctrl.devices_per_rank.value

            intlv_low_bit = int(math.log(rowbuffer_size, 2))

    # We got all we need to configure the appropriate address
    # range
    ctrl.range = m5.objects.AddrRange(r.start, size = r.size(),
                                      intlvHighBit = \
                                          intlv_low_bit + intlv_bits - 1,
                                      xorHighBit = \
                                          xor_low_bit + intlv_bits - 1,
                                      intlvBits = intlv_bits,
                                      intlvMatch = i)
    return ctrl

def config_mem(options, system):
    """
    Create the memory controllers based on the options and attach them.

    If requested, we make a multi-channel configuration of the
    selected memory controller class by creating multiple instances of
    the specific class. The individual controllers have their
    parameters set such that the address range is interleaved between
    them.
    """

    if options.tlm_memory:
        system.external_memory = m5.objects.ExternalSlave(
            port_type="tlm",
            port_data=options.tlm_memory,
            port=system.membus.master,
            addr_ranges=system.mem_ranges)
        system.kernel_addr_check = False
        return

    if options.external_memory_system:
        system.external_memory = m5.objects.ExternalSlave(
            port_type=options.external_memory_system,
            port_data="init_mem0", port=system.membus.master,
            addr_ranges=system.mem_ranges)
        system.kernel_addr_check = False
        return

    nbr_mem_ctrls = options.mem_channels
    import math
    from m5.util import fatal
    intlv_bits = int(math.log(nbr_mem_ctrls, 2))
    if 2 ** intlv_bits != nbr_mem_ctrls:
        fatal("Number of memory channels must be a power of 2")

    cls = get(options.mem_type)
    mem_ctrls = []

    # The default behaviour is to interleave memory channels on 128
    # byte granularity, or cache line granularity if larger than 128
    # byte. This value is based on the locality seen across a large
    # range of workloads.
    intlv_size = max(128, system.cache_line_size.value)

    # For every range (most systems will only have one), create an
    # array of controllers and set their parameters to match their
    # address mapping in the case of a DRAM
    for r in system.mem_ranges:
        for i in xrange(nbr_mem_ctrls):
            mem_ctrl = create_mem_ctrl(cls, r, i, nbr_mem_ctrls, intlv_bits,
                                       intlv_size)
            # Set the number of ranks based on the command-line
            # options if it was explicitly set
            if issubclass(cls, m5.objects.DRAMCtrl) and \
                    options.mem_ranks:
                mem_ctrl.ranks_per_channel = options.mem_ranks

            mem_ctrls.append(mem_ctrl)

    system.mem_ctrls = mem_ctrls

    # Connect the controllers to the membus
    for i in xrange(len(system.mem_ctrls)):
        system.mem_ctrls[i].port = system.membus.master
