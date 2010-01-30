from m5.params import *
from m5.proxy import *
from MemObject import MemObject

class RubyPort(MemObject):
    type = 'RubyPort'
    abstract = True
    port = VectorPort("M5 port")
    version = Param.Int(0, "")
    pio_port = Port("Ruby_pio_port")
    physmem = Param.PhysicalMemory("")
    physMemPort = Port("port to physical memory")

class RubySequencer(RubyPort):
    type = 'RubySequencer'
    cxx_class = 'Sequencer'
    icache = Param.RubyCache("")
    dcache = Param.RubyCache("")
    max_outstanding_requests = Param.Int(16,
        "max requests (incl. prefetches) outstanding")
    deadlock_threshold = Param.Int(500000,
        "max outstanding cycles for a request before deadlock/livelock declared")

class DMASequencer(RubyPort):
    type = 'DMASequencer'
