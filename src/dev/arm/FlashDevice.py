# Copyright (c) 2013-2015 ARM Limited
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
# Authors: Rene de Jong
#

from m5.params import *
from m5.proxy import *
from AbstractNVM import *

#Distribution of the data.
#sequential: sequential (address n+1 is likely to be on the same plane as n)
#Random: @TODO Not yet implemented
#stripe: striping over all the planes
class DataDistribution(Enum): vals = ['sequential', 'stripe']

class FlashDevice(AbstractNVM):
    type = 'FlashDevice'
    cxx_header = "dev/arm/flash_device.hh"
    # default blocksize is 128 kB.This seems to be the most common size in
    # mobile devices (not the image blocksize)
    blk_size = Param.MemorySize("128kB", "Size of one disk block")
    # disk page size is 2 kB. This is the most commonly used page size in
    # flash devices
    page_size = Param.MemorySize("2kB", "Size of one disk page")
    # There are many GC flavours. It is impossible to cover them all; this
    # parameter enables the approximation of different GC algorithms
    GC_active = Param.Percent(50, "Percentage of the time (in whole numbers) \
        that the GC is activated if a block is full")
    # Access latencies. Different devices will have different latencies, but
    # the latencies will be around the default values.
    read_lat = Param.Latency("25us", "Read Latency")
    write_lat = Param.Latency("200us", "Write Latency")
    erase_lat = Param.Latency("1500us", "Erase Delay")
    # Number of planes ought to be a power of two according to ONFI standard
    num_planes = Param.UInt32(1, "Number of planes per die")
    # Data distribution. Default is none. It is adviced to switch to stripe
    # when more than one plane is used.
    data_distribution = Param.DataDistribution('sequential', "Distribution \
        of the data in the adress table; Stripe needed for multiple\
        planes; otherwise use: sequential")

