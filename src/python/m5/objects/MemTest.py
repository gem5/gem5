from m5.SimObject import SimObject
from m5.params import *
from m5.proxy import *
from m5 import build_env

class MemTest(SimObject):
    type = 'MemTest'
    max_loads = Param.Counter("number of loads to execute")
    atomic = Param.Bool(False, "Execute tester in atomic mode? (or timing)\n")
    memory_size = Param.Int(65536, "memory size")
    percent_dest_unaligned = Param.Percent(50,
        "percent of copy dest address that are unaligned")
    percent_reads = Param.Percent(65, "target read percentage")
    percent_source_unaligned = Param.Percent(50,
        "percent of copy source address that are unaligned")
    percent_uncacheable = Param.Percent(10,
        "target uncacheable percentage")
    progress_interval = Param.Counter(1000000,
        "progress report interval (in accesses)")
    trace_addr = Param.Addr(0, "address to trace")

    test = Port("Port to the memory system to test")
    functional = Port("Port to the functional memory used for verification")
