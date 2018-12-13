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
from IndexingPolicies import *
from m5.SimObject import *
from m5.params import *
from m5.proxy import *
from ReplacementPolicies import *

class HWPProbeEvent(object):
    def __init__(self, prefetcher, obj, *listOfNames):
        self.obj = obj
        self.prefetcher = prefetcher
        self.names = listOfNames

    def register(self):
        if self.obj:
            for name in self.names:
                self.prefetcher.getCCObject().addEventProbe(
                    self.obj.getCCObject(), name)

class BasePrefetcher(ClockedObject):
    type = 'BasePrefetcher'
    abstract = True
    cxx_header = "mem/cache/prefetch/base.hh"
    cxx_exports = [
        PyBindMethod("addEventProbe"),
    ]
    sys = Param.System(Parent.any, "System this prefetcher belongs to")

    # Get the block size from the parent (system)
    block_size = Param.Int(Parent.cache_line_size, "Block size in bytes")

    on_miss = Param.Bool(False, "Only notify prefetcher on misses")
    on_read = Param.Bool(True, "Notify prefetcher on reads")
    on_write = Param.Bool(True, "Notify prefetcher on writes")
    on_data  = Param.Bool(True, "Notify prefetcher on data accesses")
    on_inst  = Param.Bool(True, "Notify prefetcher on instruction accesses")
    prefetch_on_access = Param.Bool(Parent.prefetch_on_access,
        "Notify the hardware prefetcher on every access (not just misses)")
    use_virtual_addresses = Param.Bool(False,
        "Use virtual addresses for prefetching")

    _events = []
    def addEvent(self, newObject):
        self._events.append(newObject)

    # Override the normal SimObject::regProbeListeners method and
    # register deferred event handlers.
    def regProbeListeners(self):
        for event in self._events:
           event.register()
        self.getCCObject().regProbeListeners()

    def listenFromProbe(self, simObj, *probeNames):
        if not isinstance(simObj, SimObject):
            raise TypeError("argument must be of SimObject type")
        if len(probeNames) <= 0:
            raise TypeError("probeNames must have at least one element")
        self.addEvent(HWPProbeEvent(self, simObj, *probeNames))

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

    # Do not consult stride prefetcher on instruction accesses
    on_inst = False

    max_conf = Param.Int(7, "Maximum confidence level")
    thresh_conf = Param.Int(4, "Threshold confidence level")
    min_conf = Param.Int(0, "Minimum confidence level")
    start_conf = Param.Int(4, "Starting confidence for new entries")

    table_sets = Param.Int(16, "Number of sets in PC lookup table")
    table_assoc = Param.Int(4, "Associativity of PC lookup table")
    use_master_id = Param.Bool(True, "Use master id based history")

    degree = Param.Int(4, "Number of prefetches to generate")

    # Get replacement policy
    replacement_policy = Param.BaseReplacementPolicy(RandomRP(),
        "Replacement policy")

class TaggedPrefetcher(QueuedPrefetcher):
    type = 'TaggedPrefetcher'
    cxx_class = 'TaggedPrefetcher'
    cxx_header = "mem/cache/prefetch/tagged.hh"

    degree = Param.Int(2, "Number of prefetches to generate")

class SignaturePathPrefetcher(QueuedPrefetcher):
    type = 'SignaturePathPrefetcher'
    cxx_class = 'SignaturePathPrefetcher'
    cxx_header = "mem/cache/prefetch/signature_path.hh"

    signature_shift = Param.UInt8(3,
        "Number of bits to shift when calculating a new signature");
    signature_bits = Param.UInt16(12,
        "Size of the signature, in bits");
    signature_table_entries = Param.MemorySize("1024",
        "Number of entries of the signature table")
    signature_table_assoc = Param.Unsigned(2,
        "Associativity of the signature table")
    signature_table_indexing_policy = Param.BaseIndexingPolicy(
        SetAssociative(entry_size = 1, assoc = Parent.signature_table_assoc,
        size = Parent.signature_table_entries),
        "Indexing policy of the signature table")
    signature_table_replacement_policy = Param.BaseReplacementPolicy(LRURP(),
        "Replacement policy of the signature table")

    max_counter_value = Param.UInt8(7, "Maximum pattern counter value")
    pattern_table_entries = Param.MemorySize("4096",
        "Number of entries of the pattern table")
    pattern_table_assoc = Param.Unsigned(1,
        "Associativity of the pattern table")
    strides_per_pattern_entry = Param.Unsigned(4,
        "Number of strides stored in each pattern entry")
    pattern_table_indexing_policy = Param.BaseIndexingPolicy(
        SetAssociative(entry_size = 1, assoc = Parent.pattern_table_assoc,
        size = Parent.pattern_table_entries),
        "Indexing policy of the pattern table")
    pattern_table_replacement_policy = Param.BaseReplacementPolicy(LRURP(),
        "Replacement policy of the pattern table")

    prefetch_confidence_threshold = Param.Float(0.5,
        "Minimum confidence to issue prefetches")
    lookahead_confidence_threshold = Param.Float(0.75,
        "Minimum confidence to continue exploring lookahead entries")

class AccessMapPatternMatchingPrefetcher(QueuedPrefetcher):
    type = 'AccessMapPatternMatchingPrefetcher'
    cxx_class = 'AccessMapPatternMatchingPrefetcher'
    cxx_header = "mem/cache/prefetch/access_map_pattern_matching.hh"

    start_degree = Param.Unsigned(4,
        "Initial degree (Maximum number of prefetches generated")
    hot_zone_size = Param.MemorySize("2kB", "Memory covered by a hot zone")
    access_map_table_entries = Param.MemorySize("256",
        "Number of entries in the access map table")
    access_map_table_assoc = Param.Unsigned(8,
        "Associativity of the access map table")
    access_map_table_indexing_policy = Param.BaseIndexingPolicy(
        SetAssociative(entry_size = 1, assoc = Parent.access_map_table_assoc,
        size = Parent.access_map_table_entries),
        "Indexing policy of the access map table")
    access_map_table_replacement_policy = Param.BaseReplacementPolicy(LRURP(),
        "Replacement policy of the access map table")
    high_coverage_threshold = Param.Float(0.25,
        "A prefetch coverage factor bigger than this is considered high")
    low_coverage_threshold = Param.Float(0.125,
        "A prefetch coverage factor smaller than this is considered low")
    high_accuracy_threshold = Param.Float(0.5,
        "A prefetch accuracy factor bigger than this is considered high")
    low_accuracy_threshold = Param.Float(0.25,
        "A prefetch accuracy factor smaller than this is considered low")
    high_cache_hit_threshold = Param.Float(0.875,
        "A cache hit ratio bigger than this is considered high")
    low_cache_hit_threshold = Param.Float(0.75,
        "A cache hit ratio smaller than this is considered low")
    epoch_cycles = Param.Cycles(256000, "Cycles in an epoch period")
    offchip_memory_latency = Param.Latency("30ns",
        "Memory latency used to compute the required memory bandwidth")
