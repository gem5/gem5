from m5.params import *
from m5.SimObject import SimObject

class RubyProfiler(SimObject):
    type = 'RubyProfiler'
    cxx_class = 'Profiler'
    hot_lines = Param.Bool(False, "")
    all_instructions = Param.Bool(False, "")
    num_of_sequencers = Param.Int("")
    mem_cntrl_count = Param.Int(0, "")
    banks_per_rank = Param.Int("")
    ranks_per_dimm = Param.Int("")
    dimms_per_channel = Param.Int("")

class CacheProfiler(SimObject):
    type = 'CacheProfiler'
    cxx_class = 'CacheProfiler'
    description = Param.String("")
