from m5.params import *
from m5.SimObject import SimObject
from Controller import RubyController

class RubyCache(SimObject):
    type = 'RubyCache'
    cxx_class = 'CacheMemory'
    size = Param.Int("");
    latency = Param.Int("");
    assoc = Param.Int("");
    replacement_policy = Param.String("PSEUDO_LRU", "");
