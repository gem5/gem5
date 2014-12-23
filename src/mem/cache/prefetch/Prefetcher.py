# Copyright (c) 2012, 2014 ARM Limited
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
#          Mitch Hayenga

from ClockedObject import ClockedObject
from m5.params import *
from m5.proxy import *

class BasePrefetcher(ClockedObject):
    type = 'BasePrefetcher'
    abstract = True
    cxx_header = "mem/cache/prefetch/base.hh"
    sys = Param.System(Parent.any, "System this prefetcher belongs to")

    on_miss = Param.Bool(False, "Only notify prefetcher on misses")
    on_read = Param.Bool(True, "Notify prefetcher on reads")
    on_write = Param.Bool(True, "Notify prefetcher on writes")
    on_data  = Param.Bool(True, "Notify prefetcher on data accesses")
    on_inst  = Param.Bool(True, "Notify prefetcher on instruction accesses")

class QueuedPrefetcher(BasePrefetcher):
    type = "QueuedPrefetcher"
    abstract = True
    cxx_class = "QueuedPrefetcher"
    cxx_header = "mem/cache/prefetch/queued.hh"
    latency = Param.Int(1, "Latency for generated prefetches")
    queue_size = Param.Int(32, "Maximum number of queued prefetches")
    queue_squash = Param.Bool(True, "Squash queued prefetch on demand access")
    queue_filter = Param.Bool(True, "Don't queue redundant prefetches")
    cache_snoop = Param.Bool(False, "Snoop cache to eliminate redundant request")

    tag_prefetch = Param.Bool(True, "Tag prefetch with PC of generating access")

class StridePrefetcher(QueuedPrefetcher):
    type = 'StridePrefetcher'
    cxx_class = 'StridePrefetcher'
    cxx_header = "mem/cache/prefetch/stride.hh"

    max_conf = Param.Int(7, "Maximum confidence level")
    thresh_conf = Param.Int(4, "Threshold confidence level")
    min_conf = Param.Int(0, "Minimum confidence level")
    start_conf = Param.Int(4, "Starting confidence for new entries")

    table_sets = Param.Int(16, "Number of sets in PC lookup table")
    table_assoc = Param.Int(4, "Associativity of PC lookup table")
    use_master_id = Param.Bool(True, "Use master id based history")

    degree = Param.Int(4, "Number of prefetches to generate")

class TaggedPrefetcher(QueuedPrefetcher):
    type = 'TaggedPrefetcher'
    cxx_class = 'TaggedPrefetcher'
    cxx_header = "mem/cache/prefetch/tagged.hh"

    degree = Param.Int(2, "Number of prefetches to generate")
