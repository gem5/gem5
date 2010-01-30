from m5.params import *
from m5.SimObject import SimObject

class RubyProfiler(SimObject):
    type = 'RubyProfiler'
    cxx_class = 'Profiler'
    hot_lines = Param.Bool(False, "")
    all_instructions = Param.Bool(False, "")

class CacheProfiler(SimObject):
    type = 'CacheProfiler'
    cxx_class = 'CacheProfiler'
    description = Param.String("")
