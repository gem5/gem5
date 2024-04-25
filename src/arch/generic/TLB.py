from m5.objects.IndexingPolicies import SetAssociative
from m5.objects.ReplacementPolicies import LRURP
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class NewTLB(SimObject):
    type = "NewTLB"
    cxx_header = "arch/generic/new_tlb.hh"
    cxx_class = "gem5::NewTLB"

    num_entries = Param.Int("Number of TLB Entries")
    assoc = Param.Int("Associativity of the TLB")
    size = Param.MemorySize("Size of the TLB in Bytes")
    entry_size = Param.Int("Entry Size")
    replacement_policy = Param.BaseReplacementPolicy(
        LRURP(), "Replacement Policy of the TLB"
    )
    indexing_policy = Param.BaseIndexingPolicy(
        SetAssociative(), "Indexing Policy of the TLB"
    )
