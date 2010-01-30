from m5.params import *
from MemObject import MemObject

class RubyPort(MemObject):
    type = 'RubyPort'
    abstract = True
    port = VectorPort("M5 port")
    controller = Param.RubyController("")
    version = Param.Int(0, "")

class RubySequencer(RubyPort):
    type = 'RubySequencer'
    cxx_class = 'Sequencer'
    icache = Param.RubyCache("")
    dcache = Param.RubyCache("")
    max_outstanding_requests = Param.Int(16,
        "max requests (incl. prefetches) outstanding")
    deadlock_threshold = Param.Int(500000,
        "max outstanding cycles for a request before deadlock/livelock declared")
    funcmem_port = Port("port to functional memory")

class DMASequencer(RubyPort):
    type = 'DMASequencer'
