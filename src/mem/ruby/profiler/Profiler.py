from m5.params import *
from m5.SimObject import SimObject

class RubyProfiler(SimObject):
    type = 'RubyProfiler'
    cxx_class = 'Profiler'
    hot_lines = Param.Bool(False, "")
    all_instructions = Param.Bool(False, "")
    num_of_sequencers = Param.Int("")
