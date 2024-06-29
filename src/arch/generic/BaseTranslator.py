from m5.objects.IndexingPolicies import SetAssociative
from m5.objects.ReplacementPolicies import LRURP
from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject

class Translator(SimObject):
    type = "Translator"
    cxx_class = "gem5::Translator"
    cxx_header = "arch/generic/translator.hh"

    num_entries = Param.Int("Number of address translation cache entries")
    associativity = Param.Int("Associativity of the cache")
    size = Param.MemorySize("Size of the cache in Bytes")
    entry_size = Param.Int("Entry Size")
    repl_policy = Param.BaseReplacementPolicy(
        LRURP(), "Replacement Policy of the cache"
    )
    indexing_policy = Param.BaseIndexingPolicy(
        SetAssociative(), "Indexing Policy of the cache"
    )
