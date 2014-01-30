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
# Copyright (c) 2005 The Regents of The University of Michigan
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
# Authors: Ron Dreslinski

from ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *

class BasePrefetcher(ClockedObject):
    type = 'BasePrefetcher'
    abstract = True
    cxx_header = "mem/cache/prefetch/base.hh"
    size = Param.Int(100,
         "Number of entries in the hardware prefetch queue")
    cross_pages = Param.Bool(False,
         "Allow prefetches to cross virtual page boundaries")
    serial_squash = Param.Bool(False,
         "Squash prefetches with a later time on a subsequent miss")
    degree = Param.Int(1,
         "Degree of the prefetch depth")
    latency = Param.Cycles('1', "Latency of the prefetcher")
    use_master_id = Param.Bool(True,
         "Use the master id to separate calculations of prefetches")
    data_accesses_only = Param.Bool(False,
         "Only prefetch on data not on instruction accesses")
    on_miss_only = Param.Bool(False,
         "Only prefetch on miss (as opposed to always)")
    on_read_only = Param.Bool(False,
         "Only prefetch on read requests (write requests ignored)")
    on_prefetch = Param.Bool(True,
         "Let lower cache prefetcher train on prefetch requests")
    sys = Param.System(Parent.any, "System this device belongs to")

class GHBPrefetcher(BasePrefetcher):
    type = 'GHBPrefetcher'
    cxx_class = 'GHBPrefetcher'
    cxx_header = "mem/cache/prefetch/ghb.hh"

class StridePrefetcher(BasePrefetcher):
    type = 'StridePrefetcher'
    cxx_class = 'StridePrefetcher'
    cxx_header = "mem/cache/prefetch/stride.hh"

class TaggedPrefetcher(BasePrefetcher):
    type = 'TaggedPrefetcher'
    cxx_class = 'TaggedPrefetcher'
    cxx_header = "mem/cache/prefetch/tagged.hh"




